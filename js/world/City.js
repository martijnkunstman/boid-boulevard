/**
 * City - Manages the flat 2D road network, intersections, interactive obstacles,
 * spatial road queries, and sensor raycasts against road borders.
 */
class City {
    // Signed distance field grid: cell size in px, and the clamp value stored far away from any road
    static SDF_CELL_SIZE = 2;
    static SDF_FAR_DISTANCE = 8;

    // Traffic & crash heatmaps: cell size in px, and how much activity is kept at each generation end
    static HEAT_CELL_SIZE = 10;
    static HEAT_DECAY_PER_GENERATION = 0.5;

    // Spatial grid cell size for boid neighbor queries (px); measured fastest for the ~140 px
    // sensor query radius (smaller cells cost more in per-cell overhead than they save)
    static BOID_GRID_CELL_SIZE = 128;

    // Reusable scratch projection structs for zero-allocation road state queries
    static _projScratch = { x: 0, y: 0, t: 0, distSq: 0, dist: 0, signedSide: 1 };
    static _bestProjScratch = { x: 0, y: 0, t: 0, distSq: 0, dist: 0, signedSide: 1 };

    /**
     * @param {number} width
     * @param {number} height
     * @param {object} bounds
     * @param {RoadNode[]} nodes
     * @param {Road[]} roads
     * @param {string} seed
     * @param {object} options - { trackMode, recordVisuals, obstacles }
     */
    constructor(width, height, bounds, nodes, roads, seed = '104729', options = {}) {
        this.width = width;
        this.height = height;
        this.bounds = bounds;
        this.nodes = nodes;
        this.roads = roads;
        this.seed = String(seed);
        this.trackMode = options.trackMode || 'loop';
        this.recordVisuals = options.recordVisuals ?? true;

        // Interactive user-placed roadblock obstacles: { x, y, radius }
        this.obstacles = Array.isArray(options.obstacles)
            ? options.obstacles.map(o => ({ x: o.x, y: o.y, radius: o.radius || 13 }))
            : [];

        this._buildDistanceField();

        // Heatmaps of where boids drive (seconds spent per cell) and where they crash
        this.trafficHeat = new HeatGrid(width, height, City.HEAT_CELL_SIZE);
        this.crashHeat = new HeatGrid(width, height, City.HEAT_CELL_SIZE);

        // Per-step bucketing of boids for fast neighbor queries (see Boid.stepAll)
        this.boidGrid = new SpatialGrid(width, height, City.BOID_GRID_CELL_SIZE);
    }

    /**
     * Places a circular roadblock obstacle at (x, y) if on road, or removes an existing obstacle
     * if (x, y) is clicked within an existing obstacle's radius.
     * @returns {'added'|'removed'|null}
     */
    toggleObstacleAt(x, y, radius = 13) {
        for (let i = 0; i < this.obstacles.length; i++) {
            const obs = this.obstacles[i];
            if (Math.hypot(obs.x - x, obs.y - y) <= obs.radius + 8) {
                this.obstacles.splice(i, 1);
                return 'removed';
            }
        }
        if (this.isPointOnRoad(x, y)) {
            this.obstacles.push({ x: Math.round(x), y: Math.round(y), radius });
            return 'added';
        }
        return null;
    }

    clearObstacles() {
        this.obstacles.length = 0;
    }

    /**
     * Fades both heatmaps at the end of a generation so they reflect recent driving behavior.
     */
    decayHeatmaps() {
        this.trafficHeat.decay(City.HEAT_DECAY_PER_GENERATION);
        this.crashHeat.decay(City.HEAT_DECAY_PER_GENERATION);
    }

    /**
     * Precomputes a grid of signed distances to the nearest road border (negative = on road).
     * The drivable area is the union of road capsules, so each cell stores min(capsuleDist - halfWidth).
     * Only cells near a road are evaluated; all others keep the positive SDF_FAR_DISTANCE clamp,
     * since only the sign matters off-road.
     */
    _buildDistanceField() {
        const cell = City.SDF_CELL_SIZE;
        const far = City.SDF_FAR_DISTANCE;
        const cols = Math.ceil(this.width / cell) + 1;
        const rows = Math.ceil(this.height / cell) + 1;
        const field = new Float32Array(cols * rows).fill(far);

        for (const road of this.roads) {
            const ax = road.nodeA.x;
            const ay = road.nodeA.y;
            const abx = road.nodeB.x - ax;
            const aby = road.nodeB.y - ay;
            const abLenSq = abx * abx + aby * aby;
            const invAbLenSq = abLenSq > 1e-8 ? 1 / abLenSq : 0;
            const hw = road.halfWidth;
            const margin = hw + far;

            const c0 = Math.max(0, Math.floor((Math.min(ax, ax + abx) - margin) / cell));
            const c1 = Math.min(cols - 1, Math.ceil((Math.max(ax, ax + abx) + margin) / cell));
            const r0 = Math.max(0, Math.floor((Math.min(ay, ay + aby) - margin) / cell));
            const r1 = Math.min(rows - 1, Math.ceil((Math.max(ay, ay + aby) + margin) / cell));

            for (let r = r0; r <= r1; r++) {
                const py = r * cell;
                const rowOffset = r * cols;
                const apy = py - ay;
                for (let c = c0; c <= c1; c++) {
                    const px = c * cell;
                    const apx = px - ax;
                    let t = (apx * abx + apy * aby) * invAbLenSq;
                    if (t < 0) t = 0;
                    else if (t > 1) t = 1;
                    const dx = apx - abx * t;
                    const dy = apy - aby * t;
                    const dist = Math.sqrt(dx * dx + dy * dy) - hw;
                    const idx = rowOffset + c;
                    if (dist < field[idx]) {
                        field[idx] = dist;
                    }
                }
            }
        }

        this.sdfCols = cols;
        this.sdfRows = rows;
        this.sdfInvCell = 1 / cell;
        this.sdf = field;
    }

    /**
     * Bilinearly samples the signed distance to the nearest road border at (px, py).
     * Negative inside the drivable road area, positive outside. Points outside the grid count as off-road.
     */
    sampleRoadDistance(px, py) {
        const gx = px * this.sdfInvCell;
        const gy = py * this.sdfInvCell;
        const cols = this.sdfCols;
        if (gx < 0 || gy < 0 || gx >= cols - 1 || gy >= this.sdfRows - 1) {
            return City.SDF_FAR_DISTANCE;
        }
        const c = gx | 0;
        const r = gy | 0;
        const fx = gx - c;
        const fy = gy - r;
        const i = r * cols + c;
        const f = this.sdf;
        const top = f[i] + (f[i + 1] - f[i]) * fx;
        const bottom = f[i + cols] + (f[i + cols + 1] - f[i + cols]) * fx;
        return top + (bottom - top) * fy;
    }

    /**
     * Computes the inward unit normal (-grad SDF) pointing back toward the road interior at (px, py).
     * Written into `out` ({ x, y, dist }) with zero allocations.
     */
    sampleRoadNormal(px, py, out) {
        const eps = 2.0;
        const d = this.sampleRoadDistance(px, py);
        const gx = this.sampleRoadDistance(px + eps, py) - this.sampleRoadDistance(px - eps, py);
        const gy = this.sampleRoadDistance(px, py + eps) - this.sampleRoadDistance(px, py - eps);
        const len = Math.hypot(gx, gy);
        if (len > 1e-5) {
            out.x = -gx / len;
            out.y = -gy / len;
        } else {
            out.x = 0;
            out.y = 0;
        }
        out.dist = d;
        return out;
    }

    update(dt) {
        // Stateless city road network: nothing to update per step
    }

    /**
     * Checks if point (px, py) is inside the drivable road boundaries.
     * Road segments are round-capped capsules of radius road.halfWidth, so intersections
     * are completely flush and smooth without sticking-out bulges.
     * Uses the precomputed signed distance field (O(1) per query).
     * @param {number} px
     * @param {number} py
     * @returns {boolean}
     */
    isPointOnRoad(px, py) {
        return this.sampleRoadDistance(px, py) <= 0;
    }

    /**
     * Raycasts from (ox, oy) along unit vector (dx, dy) up to maxDist to find the road border.
     * Sphere-traces the signed distance field: inside the road, |distance| never exceeds the true
     * distance to the border, so each step can safely advance by it without skipping past an edge.
     * Returns the distance to the border, or exactly maxDist when no border is hit
     * (a hit is always < maxDist). Returns a plain number so sensor updates allocate nothing.
     */
    raycastRoadSide(ox, oy, dx, dy, maxDist) {
        const hitEpsilon = 0.4;
        const minStep = 0.6;
        let t = 0;

        for (let i = 0; i < 48 && t < maxDist; i++) {
            const dist = this.sampleRoadDistance(ox + dx * t, oy + dy * t);
            if (dist > -hitEpsilon) {
                return t;
            }
            t += -dist > minStep ? -dist : minStep;
        }

        return maxDist;
    }

    /**
     * Analyzes a boid's position (x, y) and heading to determine its active road, centerline point,
     * and approaching crossing node. Writes into `outState` if provided (zero allocation).
     */
    queryBoidRoadState(x, y, heading, currentRoad = null, currentTravelSign = null, outState = null, cosH = null, sinH = null) {
        const hx = cosH !== null ? cosH : Math.cos(heading);
        const hy = sinH !== null ? sinH : Math.sin(heading);
        const projScratch = City._projScratch;

        // If currentRoad is valid and the boid is within or near its capsule, keep currentRoad locked
        // so intersections never oscillate back to the old incoming road.
        if (currentRoad) {
            Vector2D.closestPointOnSegmentInto(
                x, y,
                currentRoad.nodeA.x, currentRoad.nodeA.y,
                currentRoad.nodeB.x, currentRoad.nodeB.y,
                projScratch
            );
            if (projScratch.dist <= currentRoad.halfWidth + 10) {
                return this._buildRoadStateForProjection(
                    x, y, heading, hx, hy, currentRoad, projScratch, currentTravelSign, outState
                );
            }
        }

        let bestRoad = null;
        let bestScore = Infinity;
        const bestProj = City._bestProjScratch;

        const roads = this.roads;
        const nRoads = roads.length;
        for (let i = 0; i < nRoads; i++) {
            const road = roads[i];
            Vector2D.closestPointOnSegmentInto(
                x, y,
                road.nodeA.x, road.nodeA.y,
                road.nodeB.x, road.nodeB.y,
                projScratch
            );

            let score = projScratch.dist - road.halfWidth;
            if (road === currentRoad) {
                score -= 10.0;
            }
            const align = Math.abs(hx * road.dir.x + hy * road.dir.y);
            score -= align * 4.0;

            if (score < bestScore) {
                bestScore = score;
                bestRoad = road;
                bestProj.x = projScratch.x;
                bestProj.y = projScratch.y;
                bestProj.t = projScratch.t;
                bestProj.distSq = projScratch.distSq;
                bestProj.dist = projScratch.dist;
                bestProj.signedSide = projScratch.signedSide;
            }
        }

        if (!bestRoad) {
            bestRoad = roads[0];
            Vector2D.closestPointOnSegmentInto(
                x, y,
                bestRoad.nodeA.x, bestRoad.nodeA.y,
                bestRoad.nodeB.x, bestRoad.nodeB.y,
                bestProj
            );
        }

        return this._buildRoadStateForProjection(x, y, heading, hx, hy, bestRoad, bestProj, null, outState);
    }

    _buildRoadStateForProjection(x, y, heading, hx, hy, road, proj, forcedTravelSign = null, outState = null) {
        const dotDir = hx * road.dir.x + hy * road.dir.y;

        let travelSign;
        if (forcedTravelSign === 1 || forcedTravelSign === -1) {
            travelSign = forcedTravelSign;
        } else if (proj.t < 0.12) {
            travelSign = 1;
        } else if (proj.t > 0.88) {
            travelSign = -1;
        } else {
            travelSign = dotDir >= 0 ? 1 : -1;
        }

        // Reuse or allocate state object and its nested point vectors
        const state = outState || {
            onRoad: true,
            road: null,
            t: 0,
            distToCenterline: 0,
            lateralOffset: 0,
            laneError: 0,
            inCrossing: false,
            travelSign: 1,
            travelDir: new Vector2D(),
            desiredAngle: 0,
            laneTarget: new Vector2D(),
            angleDiff: 0,
            headingError: 0,
            approachingNode: null,
            distToNode: 0
        };

        // Lookahead point in the keep-right lane, so boids at intersections are pulled forward into
        // their own lane of the road, never backward into the intersection corner cap.
        const lookaheadT = Math.max(0.14, Math.min(0.86, proj.t + travelSign * 0.18));
        const laneTarget = road.getLanePointInto(lookaheadT, travelSign, state.laneTarget);
        state.travelDir.x = road.dir.x * travelSign;
        state.travelDir.y = road.dir.y * travelSign;

        // Signed lateral position relative to the centerline (+ = right-hand side of travel direction)
        const rightX = road.rightX * travelSign;
        const rightY = road.rightY * travelSign;
        const lateralOffset = (x - proj.x) * rightX + (y - proj.y) * rightY;
        const laneError = lateralOffset - road.laneOffset;

        // Blend road axis angle with angle toward the lookahead lane point when off the lane center
        const roadAxisAngle = travelSign > 0 ? road.forwardAnglePos : road.forwardAngleNeg;
        const toLookaheadAngle = Math.atan2(laneTarget.y - y, laneTarget.x - x);
        // Stronger pull when drifting into the oncoming lane (lateralOffset < 0)
        const maxPull = lateralOffset < 0 ? 0.65 : 0.55;
        const centerPullWeight = Math.min(maxPull, Math.abs(laneError) / Math.max(1, road.laneOffset));
        const axisToLookaheadDiff = Vector2D.normalizeAngle(toLookaheadAngle - roadAxisAngle);
        const desiredAngle = Vector2D.normalizeAngle(roadAxisAngle + axisToLookaheadDiff * centerPullWeight);

        const angleDiff = Vector2D.normalizeAngle(heading - desiredAngle);
        const headingError = Math.max(-1, Math.min(1, angleDiff / (Math.PI * 0.5)));
        const onRoad = this.isPointOnRoad(x, y);

        const approachingNode = travelSign > 0 ? road.nodeB : road.nodeA;
        const distToNode = Math.hypot(x - approachingNode.x, y - approachingNode.y);

        // Inside a crossing, lanes of different roads overlap, so lane rules do not apply there
        const distAlong = proj.t * road.length;
        const inCrossing = distAlong < road.nodeA.radius || road.length - distAlong < road.nodeB.radius;

        state.onRoad = onRoad;
        state.road = road;
        state.t = proj.t;
        state.distToCenterline = proj.dist;
        state.lateralOffset = lateralOffset;
        state.laneError = laneError;
        state.inCrossing = inCrossing;
        state.travelSign = travelSign;
        state.desiredAngle = desiredAngle;
        state.angleDiff = angleDiff;
        state.headingError = headingError;
        state.approachingNode = approachingNode;
        state.distToNode = distToNode;
        return state;
    }

    /**
     * Returns `count` spawn poses spread evenly over the middle 60% of all roads (by length), alternating
     * travel directions, starting at a random offset. Compared with independent random spawns this avoids
     * clumps and empty stretches, so a boid's score depends less on a lucky or unlucky start.
     * @param {number} count
     */
    getSpawnPoses(count) {
        const usable = this.roads.map(road => road.length * 0.6);
        const total = usable.reduce((a, b) => a + b, 0);
        const spacing = total / Math.max(1, count);
        let pos = Random.next() * spacing;
        let roadIndex = 0;
        let roadStart = 0;
        const poses = [];

        for (let i = 0; i < count; i++) {
            while (roadIndex < this.roads.length - 1 && pos > roadStart + usable[roadIndex]) {
                roadStart += usable[roadIndex];
                roadIndex++;
            }
            const road = this.roads[roadIndex];
            const t = Math.min(0.8, 0.2 + (pos - roadStart) / road.length);
            const travelSign = i % 2 === 0 ? 1 : -1;
            const point = road.getLanePoint(t, travelSign);
            poses.push({
                x: point.x,
                y: point.y,
                heading: travelSign > 0 ? road.forwardAnglePos : road.forwardAngleNeg,
                road,
                travelSign
            });
            pos += spacing;
        }
        return poses;
    }

    /**
     * Picks a valid random spawn point in the keep-right lane of a road, well clear of existing boids
     * and any roadblock obstacles.
     */
    getRandomSpawnPose(existingBoids = []) {
        for (let attempt = 0; attempt < 40; attempt++) {
            const road = this.roads[Math.floor(Random.next() * this.roads.length)];
            const travelSign = Random.next() < 0.5 ? 1 : -1;
            const t = 0.25 + Random.next() * 0.5;
            const pos = road.getLanePoint(t, travelSign);
            const heading = travelSign > 0 ? road.forwardAnglePos : road.forwardAngleNeg;

            let clear = true;
            for (let i = 0; i < existingBoids.length; i++) {
                const other = existingBoids[i];
                const dx = other.x - pos.x;
                const dy = other.y - pos.y;
                if (dx * dx + dy * dy < 784) { // 28^2
                    clear = false;
                    break;
                }
            }
            if (clear) {
                for (let i = 0; i < this.obstacles.length; i++) {
                    const obs = this.obstacles[i];
                    if (Math.hypot(obs.x - pos.x, obs.y - pos.y) < obs.radius + 20) {
                        clear = false;
                        break;
                    }
                }
            }

            if (clear || attempt === 39) {
                return {
                    x: pos.x,
                    y: pos.y,
                    heading,
                    road,
                    travelSign
                };
            }
        }
    }

    serialize() {
        return {
            seed: this.seed,
            trackMode: this.trackMode,
            width: this.width,
            height: this.height,
            bounds: this.bounds,
            obstacles: this.obstacles,
            nodes: this.nodes.map(n => ({
                id: n.id,
                x: n.x,
                y: n.y,
                isPerimeter: n.isPerimeter
            })),
            roads: this.roads.map(r => ({
                id: r.id,
                nodeAId: r.nodeA.id,
                nodeBId: r.nodeB.id,
                width: r.width,
                isDiagonal: r.isDiagonal,
                isPerimeter: r.isPerimeter
            }))
        };
    }

    static deserialize(data) {
        if (!data || !Array.isArray(data.nodes) || !Array.isArray(data.roads)) {
            return null;
        }
        const nodeMap = new Map();
        const nodes = data.nodes.map(nd => {
            const node = new RoadNode(nd.id, nd.x, nd.y, nd.isPerimeter);
            nodeMap.set(nd.id, node);
            return node;
        });

        const roads = [];
        for (const rd of data.roads) {
            const nodeA = nodeMap.get(rd.nodeAId);
            const nodeB = nodeMap.get(rd.nodeBId);
            if (!nodeA || !nodeB) continue;
            if (nodeA.roads.length >= 4 || nodeB.roads.length >= 4) continue;
            const road = new Road(rd.id, nodeA, nodeB, {
                // Older saves used narrower single-track roads; widen them to fit two lanes
                width: Math.max(rd.width || 0, Road.DEFAULT_WIDTH),
                isDiagonal: rd.isDiagonal,
                isPerimeter: rd.isPerimeter
            });
            nodeA.addRoad(road);
            nodeB.addRoad(road);
            roads.push(road);
        }

        return new City(data.width, data.height, data.bounds, nodes, roads, data.seed || '104729', {
            trackMode: data.trackMode || 'loop',
            obstacles: data.obstacles || []
        });
    }
}
