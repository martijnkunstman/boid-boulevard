/**
 * City - Manages the flat 2D road network, intersections, stoplights,
 * spatial road queries, and sensor raycasts against road sides.
 */
class City {
    /**
     * @param {number} width
     * @param {number} height
     * @param {object} bounds
     * @param {RoadNode[]} nodes
     * @param {Road[]} roads
     */
    constructor(width, height, bounds, nodes, roads) {
        this.width = width;
        this.height = height;
        this.bounds = bounds;
        this.nodes = nodes;
        this.roads = roads;
    }

    update(dt) {
        for (const node of this.nodes) {
            if (node.trafficController) {
                node.trafficController.update(dt);
            }
        }
    }

    /**
     * Checks if point (px, py) is inside the drivable road boundaries.
     * @param {number} px
     * @param {number} py
     * @returns {boolean}
     */
    isPointOnRoad(px, py) {
        // Check intersection nodes first
        for (let i = 0; i < this.nodes.length; i++) {
            const node = this.nodes[i];
            const dx = px - node.x;
            const dy = py - node.y;
            if (dx * dx + dy * dy <= node.radius * node.radius) {
                return true;
            }
        }

        // Check road segments
        for (let i = 0; i < this.roads.length; i++) {
            const road = this.roads[i];
            const proj = Vector2D.closestPointOnSegment(
                px, py,
                road.nodeA.x, road.nodeA.y,
                road.nodeB.x, road.nodeB.y
            );
            if (proj.dist <= road.halfWidth) {
                return true;
            }
        }

        return false;
    }

    /**
     * Raycasts from (ox, oy) along unit vector (dx, dy) up to maxDist to find the side of the road.
     * Returns { hit: boolean, distance: number, x: number, y: number }.
     */
    raycastRoadSide(ox, oy, dx, dy, maxDist) {
        const steps = 14;
        const stepDist = maxDist / steps;
        let prevDist = 0;

        for (let s = 1; s <= steps; s++) {
            const curDist = s * stepDist;
            const px = ox + dx * curDist;
            const py = oy + dy * curDist;

            if (!this.isPointOnRoad(px, py)) {
                // Binary search refinement between prevDist (inside) and curDist (outside)
                let lo = prevDist;
                let hi = curDist;
                for (let b = 0; b < 4; b++) {
                    const mid = (lo + hi) * 0.5;
                    const mx = ox + dx * mid;
                    const my = oy + dy * mid;
                    if (this.isPointOnRoad(mx, my)) {
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
     * and any approaching stoplight.
     */
    queryBoidRoadState(x, y, heading, currentRoad = null) {
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

            // Continuity bonus for currentRoad and alignment bonus when inside an intersection
            let score = proj.dist - road.halfWidth;
            if (road === currentRoad) {
                score -= 5.0;
            } else if (currentRoad) {
                const sharesNode =
                    road.nodeA === currentRoad.nodeA ||
                    road.nodeA === currentRoad.nodeB ||
                    road.nodeB === currentRoad.nodeA ||
                    road.nodeB === currentRoad.nodeB;
                if (!sharesNode && proj.dist > road.halfWidth) {
                    score += 18.0;
                }
            }

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

        // Signed lateral offset from right-hand lane center (+ = too far right toward curb, - = too far left)
        const dx = x - laneCenter.x;
        const dy = y - laneCenter.y;
        const lateralOffset = dx * rightNorm.x + dy * rightNorm.y;
        const normalizedLateral = Math.max(-1, Math.min(1, lateralOffset / (road.halfWidth * 0.6)));

        // Heading angle error relative to road forward travel direction
        const desiredAngle = Math.atan2(travelDir.y, travelDir.x);
        const angleDiff = Vector2D.normalizeAngle(heading - desiredAngle);
        const normalizedAngleErr = Math.max(-1, Math.min(1, angleDiff / (Math.PI * 0.5)));

        const laneError = Math.max(-1, Math.min(1, normalizedLateral * 0.55 + normalizedAngleErr * 0.65));
        const onRoad = this.isPointOnRoad(x, y);

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
                    road
                };
            }
        }
    }

    serialize() {
        return {
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

        for (const node of nodes) {
            if (node.roads.length >= 3) {
                node.trafficController = new IntersectionController(node);
            }
        }

        return new City(data.width, data.height, data.bounds, nodes, roads);
    }
}
