/**
 * City - Manages the flat 2D road network, intersections,
 * spatial road queries, and sensor raycasts against road borders.
 */
class City {
    // Signed distance field grid: cell size in px, and the clamp value stored far away from any road
    static SDF_CELL_SIZE = 2;
    static SDF_FAR_DISTANCE = 8;

    /**
     * @param {number} width
     * @param {number} height
     * @param {object} bounds
     * @param {RoadNode[]} nodes
     * @param {Road[]} roads
     * @param {string} seed
     */
    constructor(width, height, bounds, nodes, roads, seed = '104729') {
        this.width = width;
        this.height = height;
        this.bounds = bounds;
        this.nodes = nodes;
        this.roads = roads;
        this.seed = String(seed);

        this._buildDistanceField();
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
            const hw = road.halfWidth;
            const margin = hw + far;

            const c0 = Math.max(0, Math.floor((Math.min(ax, ax + abx) - margin) / cell));
            const c1 = Math.min(cols - 1, Math.ceil((Math.max(ax, ax + abx) + margin) / cell));
            const r0 = Math.max(0, Math.floor((Math.min(ay, ay + aby) - margin) / cell));
            const r1 = Math.min(rows - 1, Math.ceil((Math.max(ay, ay + aby) + margin) / cell));

            for (let r = r0; r <= r1; r++) {
                const py = r * cell;
                for (let c = c0; c <= c1; c++) {
                    const px = c * cell;
                    let t = abLenSq > 1e-8 ? ((px - ax) * abx + (py - ay) * aby) / abLenSq : 0;
                    if (t < 0) t = 0;
                    else if (t > 1) t = 1;
                    const dx = px - ax - abx * t;
                    const dy = py - ay - aby * t;
                    const dist = Math.sqrt(dx * dx + dy * dy) - hw;
                    const idx = r * cols + c;
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
     * Returns { hit: boolean, distance: number, x: number, y: number }.
     */
    raycastRoadSide(ox, oy, dx, dy, maxDist) {
        const hitEpsilon = 0.4;
        const minStep = 0.6;
        let t = 0;

        for (let i = 0; i < 48 && t < maxDist; i++) {
            const dist = this.sampleRoadDistance(ox + dx * t, oy + dy * t);
            if (dist > -hitEpsilon) {
                return {
                    hit: true,
                    distance: t,
                    x: ox + dx * t,
                    y: oy + dy * t
                };
            }
            t += Math.max(-dist, minStep);
        }

        return {
            hit: false,
            distance: maxDist,
            x: ox + dx * maxDist,
            y: oy + dy * maxDist
        };
    }

    /**
     * Analyzes a boid's position (x, y) and heading to determine its active road, centerline point,
     * and approaching crossing node.
     */
    queryBoidRoadState(x, y, heading, currentRoad = null, currentTravelSign = null) {
        // If currentRoad is valid and the boid is within or near its capsule, keep currentRoad locked
        // so intersections never oscillate back to the old incoming road.
        if (currentRoad) {
            const curProj = Vector2D.closestPointOnSegment(
                x, y,
                currentRoad.nodeA.x, currentRoad.nodeA.y,
                currentRoad.nodeB.x, currentRoad.nodeB.y
            );
            if (curProj.dist <= currentRoad.halfWidth + 10) {
                return this._buildRoadStateForProjection(
                    x, y, heading, currentRoad, curProj, currentTravelSign
                );
            }
        }

        let bestRoad = null;
        let bestProj = null;
        let bestScore = Infinity;

        const hx = Math.cos(heading);
        const hy = Math.sin(heading);

        for (let i = 0; i < this.roads.length; i++) {
            const road = this.roads[i];
            const proj = Vector2D.closestPointOnSegment(
                x, y,
                road.nodeA.x, road.nodeA.y,
                road.nodeB.x, road.nodeB.y
            );

            let score = proj.dist - road.halfWidth;
            if (road === currentRoad) {
                score -= 10.0;
            }
            const align = Math.abs(hx * road.dir.x + hy * road.dir.y);
            score -= align * 4.0;

            if (score < bestScore) {
                bestScore = score;
                bestRoad = road;
                bestProj = proj;
            }
        }

        if (!bestRoad) {
            bestRoad = this.roads[0];
            bestProj = Vector2D.closestPointOnSegment(
                x, y,
                bestRoad.nodeA.x, bestRoad.nodeA.y,
                bestRoad.nodeB.x, bestRoad.nodeB.y
            );
        }

        return this._buildRoadStateForProjection(x, y, heading, bestRoad, bestProj, null);
    }

    _buildRoadStateForProjection(x, y, heading, road, proj, forcedTravelSign = null) {
        const hx = Math.cos(heading);
        const hy = Math.sin(heading);
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

        // Lookahead point along the road so boids at intersections are pulled forward into the road,
        // never backward into the intersection corner cap.
        const lookaheadT = Math.max(0.14, Math.min(0.86, proj.t + travelSign * 0.18));
        const roadCenter = road.getPointAt(lookaheadT);
        const travelDir = road.dir.scale(travelSign);

        // Blend road axis angle with angle toward lookahead centerline when off-center
        const roadAxisAngle = Math.atan2(travelDir.y, travelDir.x);
        const toLookaheadAngle = Math.atan2(roadCenter.y - y, roadCenter.x - x);
        const centerPullWeight = Math.min(0.55, proj.dist / Math.max(1, road.halfWidth));
        const axisToLookaheadDiff = Vector2D.normalizeAngle(toLookaheadAngle - roadAxisAngle);
        const desiredAngle = Vector2D.normalizeAngle(roadAxisAngle + axisToLookaheadDiff * centerPullWeight);

        const angleDiff = Vector2D.normalizeAngle(heading - desiredAngle);
        const headingError = Math.max(-1, Math.min(1, angleDiff / (Math.PI * 0.5)));
        const onRoad = this.isPointOnRoad(x, y);

        const approachingNode = travelSign > 0 ? road.nodeB : road.nodeA;
        const distToNode = Math.hypot(x - approachingNode.x, y - approachingNode.y);

        return {
            onRoad,
            road,
            t: proj.t,
            distToCenterline: proj.dist,
            travelSign,
            travelDir,
            desiredAngle,
            roadCenter,
            angleDiff,
            headingError,
            approachingNode,
            distToNode
        };
    }

    /**
     * Picks a valid random spawn point on a road centerline, well clear of existing boids.
     */
    getRandomSpawnPose(existingBoids = []) {
        for (let attempt = 0; attempt < 40; attempt++) {
            const road = this.roads[Math.floor(Random.next() * this.roads.length)];
            const travelSign = Random.next() < 0.5 ? 1 : -1;
            const t = 0.25 + Random.next() * 0.5;
            const pos = road.getPointAt(t);
            const heading = Math.atan2(road.dir.y * travelSign, road.dir.x * travelSign);

            let clear = true;
            for (const other of existingBoids) {
                if (Math.hypot(other.x - pos.x, other.y - pos.y) < 28) {
                    clear = false;
                    break;
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
            width: this.width,
            height: this.height,
            bounds: this.bounds,
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
                width: rd.width,
                isDiagonal: rd.isDiagonal,
                isPerimeter: rd.isPerimeter
            });
            nodeA.addRoad(road);
            nodeB.addRoad(road);
            roads.push(road);
        }

        return new City(data.width, data.height, data.bounds, nodes, roads, data.seed || '104729');
    }
}
