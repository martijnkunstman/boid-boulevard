/**
 * City - Manages the road network, intersections, stoplights, bridges, heightmap,
 * spatial road queries, and sensor raycasts against road sides.
 */
class City {
    /**
     * @param {number} width
     * @param {number} height
     * @param {object} bounds
     * @param {RoadNode[]} nodes
     * @param {Road[]} roads
     * @param {object[]} hills
     */
    constructor(width, height, bounds, nodes, roads, hills = []) {
        this.width = width;
        this.height = height;
        this.bounds = bounds;
        this.nodes = nodes;
        this.roads = roads;
        this.hills = hills;
    }

    update(dt) {
        for (const node of this.nodes) {
            if (node.trafficController) {
                node.trafficController.update(dt);
            }
        }
    }

    /**
     * Evaluates terrain elevation (outside or underneath bridges) at any (x, y).
     */
    getTerrainElevationAt(x, y) {
        let z = 0;
        for (const h of this.hills) {
            const dx = x - h.x;
            const dy = y - h.y;
            const d2 = dx * dx + dy * dy;
            const r2 = h.radius * h.radius;
            if (d2 < r2) {
                const falloff = 1 - d2 / r2;
                z += h.peak * falloff * falloff;
            }
        }
        return Math.min(14, Math.max(0, z));
    }

    /**
     * Checks if point (px, py) is inside the drivable road boundaries, respecting bridge layers.
     * @param {number} px
     * @param {number} py
     * @param {Road|null} contextRoad - The road the boid is currently on
     * @param {number} contextLayer - 0 for ground, 1 for elevated bridge span
     * @returns {boolean}
     */
    isPointOnRoad(px, py, contextRoad = null, contextLayer = 0) {
        // If on an elevated bridge span (layer 1), only the bridge road itself and its end nodes count
        if (contextLayer === 1 && contextRoad && contextRoad.isBridge) {
            const proj = Vector2D.closestPointOnSegment(
                px, py,
                contextRoad.nodeA.x, contextRoad.nodeA.y,
                contextRoad.nodeB.x, contextRoad.nodeB.y
            );
            if (proj.dist <= contextRoad.halfWidth) return true;
            if (Math.hypot(px - contextRoad.nodeA.x, py - contextRoad.nodeA.y) <= contextRoad.nodeA.radius) return true;
            if (Math.hypot(px - contextRoad.nodeB.x, py - contextRoad.nodeB.y) <= contextRoad.nodeB.radius) return true;
            return false;
        }

        // Check intersection nodes first
        for (let i = 0; i < this.nodes.length; i++) {
            const node = this.nodes[i];
            const dx = px - node.x;
            const dy = py - node.y;
            if (dx * dx + dy * dy <= node.radius * node.radius) {
                return true;
            }
        }

        // Check roads compatible with ground/ramp layer
        for (let i = 0; i < this.roads.length; i++) {
            const road = this.roads[i];
            const proj = Vector2D.closestPointOnSegment(
                px, py,
                road.nodeA.x, road.nodeA.y,
                road.nodeB.x, road.nodeB.y
            );
            if (proj.dist <= road.halfWidth) {
                // If this is a bridge road's elevated middle span, only allow if contextRoad is this bridge
                if (road.isBridge && proj.t > 0.18 && proj.t < 0.82 && contextRoad !== road) {
                    continue;
                }
                return true;
            }
        }

        return false;
    }

    /**
     * Raycasts from (ox, oy) along unit vector (dx, dy) up to maxDist to find the side of the road.
     * Returns { hit: boolean, distance: number, x: number, y: number }.
     */
    raycastRoadSide(ox, oy, dx, dy, maxDist, contextRoad = null, contextLayer = 0) {
        const steps = 14;
        const stepDist = maxDist / steps;
        let prevDist = 0;

        for (let s = 1; s <= steps; s++) {
            const curDist = s * stepDist;
            const px = ox + dx * curDist;
            const py = oy + dy * curDist;

            if (!this.isPointOnRoad(px, py, contextRoad, contextLayer)) {
                // Binary search refinement between prevDist (inside) and curDist (outside)
                let lo = prevDist;
                let hi = curDist;
                for (let b = 0; b < 4; b++) {
                    const mid = (lo + hi) * 0.5;
                    const mx = ox + dx * mid;
                    const my = oy + dy * mid;
                    if (this.isPointOnRoad(mx, my, contextRoad, contextLayer)) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                const hitDist = (lo + hi) * 0.5;
                return {
                    hit: true,
                    distance: hitDist,
                    x: ox + dx * hitDist,
                    y: oy + dy * hitDist
                };
            }
            prevDist = curDist;
        }

        return {
            hit: false,
            distance: maxDist,
            x: ox + dx * maxDist,
            y: oy + dy * maxDist
        };
    }

    /**
     * Analyzes a boid's position (x, y) and heading to determine its active road, lane offset,
     * elevation, layer, slope, and any approaching stoplight.
     */
    queryBoidRoadState(x, y, heading, currentRoad = null, currentLayer = 0) {
        // If currently on an elevated bridge deck, stay locked to that bridge until reaching the ramp ends
        if (currentRoad && currentRoad.isBridge && currentLayer === 1) {
            const proj = Vector2D.closestPointOnSegment(
                x, y,
                currentRoad.nodeA.x, currentRoad.nodeA.y,
                currentRoad.nodeB.x, currentRoad.nodeB.y
            );
            if (proj.t > 0.12 && proj.t < 0.88 && proj.dist <= currentRoad.halfWidth + 14) {
                return this._buildRoadStateForProjection(x, y, heading, currentRoad, proj);
            }
        }

        let bestRoad = null;
        let bestProj = null;
        let bestScore = Infinity;

        for (let i = 0; i < this.roads.length; i++) {
            const road = this.roads[i];
            const proj = Vector2D.closestPointOnSegment(
                x, y,
                road.nodeA.x, road.nodeA.y,
                road.nodeB.x, road.nodeB.y
            );

            // Prevent ground traffic from snapping up onto a crossing bridge deck mid-span
            if (road.isBridge && proj.t > 0.22 && proj.t < 0.78 && currentRoad !== road) {
                continue;
            }

            // Continuity bonus for currentRoad and alignment bonus when inside an intersection
            let score = proj.dist - road.halfWidth;
            if (road === currentRoad) {
                score -= 5.0;
            } else if (currentRoad) {
                // Check if road shares a node with currentRoad
                const sharesNode =
                    road.nodeA === currentRoad.nodeA ||
                    road.nodeA === currentRoad.nodeB ||
                    road.nodeB === currentRoad.nodeA ||
                    road.nodeB === currentRoad.nodeB;
                if (!sharesNode && proj.dist > road.halfWidth) {
                    score += 18.0;
                }
            }

            // Heading alignment bonus when inside road/intersection
            const hx = Math.cos(heading);
            const hy = Math.sin(heading);
            const align = Math.abs(hx * road.dir.x + hy * road.dir.y);
            score -= align * 4.5;

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

        return this._buildRoadStateForProjection(x, y, heading, bestRoad, bestProj);
    }

    _buildRoadStateForProjection(x, y, heading, road, proj) {
        const hx = Math.cos(heading);
        const hy = Math.sin(heading);
        const dotDir = hx * road.dir.x + hy * road.dir.y;
        const travelSign = dotDir >= 0 ? 1 : -1;

        // Desired right-hand lane center at parameter proj.t
        const laneCenter = road.getLanePointAt(proj.t, travelSign);
        const travelDir = road.dir.scale(travelSign);
        const rightNorm = new Vector2D(-travelDir.y, travelDir.x);

        // Signed lateral offset from right-hand lane center (+ = too far right toward curb, - = too far left toward oncoming lane)
        const dx = x - laneCenter.x;
        const dy = y - laneCenter.y;
        const lateralOffset = dx * rightNorm.x + dy * rightNorm.y;
        const normalizedLateral = Math.max(-1, Math.min(1, lateralOffset / (road.halfWidth * 0.6)));

        // Heading angle error relative to road forward travel direction
        const desiredAngle = Math.atan2(travelDir.y, travelDir.x);
        const angleDiff = Vector2D.normalizeAngle(heading - desiredAngle);
        const normalizedAngleErr = Math.max(-1, Math.min(1, angleDiff / (Math.PI * 0.5)));

        const laneError = Math.max(-1, Math.min(1, normalizedLateral * 0.55 + normalizedAngleErr * 0.65));

        const onRoad = this.isPointOnRoad(x, y, road, road.getLayerAt(proj.t));
        const elevation = road.getElevationAt(proj.t);
        const layer = road.getLayerAt(proj.t);
        const slope = road.getSlopeAt(proj.t, travelSign);

        // Approaching node & stoplight check
        const approachingNode = travelSign > 0 ? road.nodeB : road.nodeA;
        const distToNode = Math.hypot(x - approachingNode.x, y - approachingNode.y);
        const distToStopLine = Math.max(0, distToNode - (approachingNode.radius + 7));

        let trafficLight = null;
        if (approachingNode.trafficController) {
            trafficLight = approachingNode.trafficController.getLightForRoad(road);
        }

        return {
            onRoad,
            road,
            t: proj.t,
            distToCenterline: proj.dist,
            travelSign,
            travelDir,
            desiredAngle,
            laneCenter,
            lateralOffset,
            angleDiff,
            laneError,
            elevation,
            layer,
            slope,
            approachingNode,
            distToNode,
            distToStopLine,
            trafficLight
        };
    }

    /**
     * Picks a valid random spawn point on a road's right-hand lane, well clear of existing boids.
     */
    getRandomSpawnPose(existingBoids = []) {
        for (let attempt = 0; attempt < 40; attempt++) {
            const road = this.roads[Math.floor(Math.random() * this.roads.length)];
            const travelSign = Math.random() < 0.5 ? 1 : -1;
            const t = 0.25 + Math.random() * 0.5;
            const pos = road.getLanePointAt(t, travelSign);
            const heading = Math.atan2(road.dir.y * travelSign, road.dir.x * travelSign);
            const elevation = road.getElevationAt(t);
            const layer = road.getLayerAt(t);

            let clear = true;
            for (const other of existingBoids) {
                if (Math.abs(other.layer - layer) === 0 && Math.hypot(other.x - pos.x, other.y - pos.y) < 28) {
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
                    layer,
                    elevation
                };
            }
        }
    }

    serialize() {
        return {
            width: this.width,
            height: this.height,
            bounds: this.bounds,
            hills: this.hills,
            nodes: this.nodes.map(n => ({
                id: n.id,
                x: n.x,
                y: n.y,
                elevation: n.elevation,
                isPerimeter: n.isPerimeter
            })),
            roads: this.roads.map(r => ({
                id: r.id,
                nodeAId: r.nodeA.id,
                nodeBId: r.nodeB.id,
                width: r.width,
                isBridge: r.isBridge,
                isDiagonal: r.isDiagonal,
                isPerimeter: r.isPerimeter,
                bridgeDeckHeight: r.bridgeDeckHeight
            }))
        };
    }

    static deserialize(data) {
        if (!data || !Array.isArray(data.nodes) || !Array.isArray(data.roads)) {
            return null;
        }
        const nodeMap = new Map();
        const nodes = data.nodes.map(nd => {
            const node = new RoadNode(nd.id, nd.x, nd.y, nd.elevation, nd.isPerimeter);
            nodeMap.set(nd.id, node);
            return node;
        });

        const roads = [];
        for (const rd of data.roads) {
            const nodeA = nodeMap.get(rd.nodeAId);
            const nodeB = nodeMap.get(rd.nodeBId);
            if (!nodeA || !nodeB) continue;
            const road = new Road(rd.id, nodeA, nodeB, {
                width: rd.width,
                isBridge: rd.isBridge,
                isDiagonal: rd.isDiagonal,
                isPerimeter: rd.isPerimeter,
                bridgeDeckHeight: rd.bridgeDeckHeight
            });
            nodeA.addRoad(road);
            nodeB.addRoad(road);
            roads.push(road);
        }

        for (const node of nodes) {
            if (node.roads.length >= 3) {
                node.trafficController = new IntersectionController(node);
            }
        }

        return new City(data.width, data.height, data.bounds, nodes, roads, data.hills || []);
    }
}
