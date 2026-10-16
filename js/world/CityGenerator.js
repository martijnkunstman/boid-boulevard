/**
 * CityGenerator - Randomly generates a clean, flat 2D city road network within a rectangular boundary:
 *  - Rectangular perimeter limited by diagonal corner roads (no roads leading out of the city)
 *  - Clean, less complicated layout with spacious blocks
 *  - Internal diagonal roads and orthogonal streets
 *  - Strictly enforces <= 4 connected roads at every crossing
 *  - Crossings equipped with multi-phase stoplights (TrafficLights)
 */
class CityGenerator {
    static MAX_ROADS_PER_NODE = 4;

    /**
     * @param {number} width - Canvas/world width
     * @param {number} height - Canvas/world height
     * @returns {City}
     */
    static generate(width = 1020, height = 620) {
        const marginX = 74;
        const marginY = 60;
        const rect = {
            left: marginX,
            top: marginY,
            right: width - marginX,
            bottom: height - marginY,
            width: width - marginX * 2,
            height: height - marginY * 2
        };

        // Build a clean 4x4 grid of base positions (omitting the 4 extreme corners)
        const cols = 4;
        const rows = 4;
        const colWeights = CityGenerator._randomPartition(cols - 1, 0.27);
        const rowWeights = CityGenerator._randomPartition(rows - 1, 0.27);

        const colX = [rect.left];
        for (let c = 0; c < colWeights.length; c++) {
            colX.push(colX[c] + colWeights[c] * rect.width);
        }
        colX[cols - 1] = rect.right;

        const rowY = [rect.top];
        for (let r = 0; r < rowWeights.length; r++) {
            rowY.push(rowY[r] + rowWeights[r] * rect.height);
        }
        rowY[rows - 1] = rect.bottom;

        const nodes = [];
        const gridNodes = Array.from({ length: cols }, () => new Array(rows).fill(null));
        let nextNodeId = 1;

        const isCorner = (c, r) =>
            (c === 0 && r === 0) ||
            (c === cols - 1 && r === 0) ||
            (c === 0 && r === rows - 1) ||
            (c === cols - 1 && r === rows - 1);

        // 1. Create nodes (omitting the 4 corners so diagonal roads limit the town corners within the rectangle)
        for (let c = 0; c < cols; c++) {
            for (let r = 0; r < rows; r++) {
                if (isCorner(c, r)) continue;

                const onBoundary = (c === 0 || c === cols - 1 || r === 0 || r === rows - 1);
                let x = colX[c];
                let y = rowY[r];

                // Gentle organic variation for interior intersections
                if (!onBoundary) {
                    x += (Math.random() * 2 - 1) * 18;
                    y += (Math.random() * 2 - 1) * 14;
                }

                const node = new RoadNode(nextNodeId++, x, y, onBoundary);
                nodes.push(node);
                gridNodes[c][r] = node;
            }
        }

        const roads = [];
        let nextRoadId = 1;
        const roadKeySet = new Set();

        /**
         * Validates that adding a road between nodeA and nodeB:
         *  1. Never exceeds 4 roads on either node
         *  2. Does not duplicate an existing road
         *  3. Does not geometrically cross any existing road
         *  4. Maintains a clean minimum angle (>= 30 deg) with existing roads at both nodes
         */
        const canAddRoad = (nodeA, nodeB) => {
            if (!nodeA || !nodeB || nodeA === nodeB) return false;
            if (nodeA.roads.length >= CityGenerator.MAX_ROADS_PER_NODE) return false;
            if (nodeB.roads.length >= CityGenerator.MAX_ROADS_PER_NODE) return false;

            const key = nodeA.id < nodeB.id
                ? `${nodeA.id}-${nodeB.id}`
                : `${nodeB.id}-${nodeA.id}`;
            if (roadKeySet.has(key)) return false;

            // Check minimum angle at nodeA and nodeB so roads never overlap at acute angles
            const minAngleRad = 0.52; // ~30 degrees
            const angleAB = Math.atan2(nodeB.y - nodeA.y, nodeB.x - nodeA.x);
            for (const r of nodeA.roads) {
                const other = r.getOtherNode(nodeA);
                const existingAngle = Math.atan2(other.y - nodeA.y, other.x - nodeA.x);
                if (Math.abs(Vector2D.normalizeAngle(angleAB - existingAngle)) < minAngleRad) {
                    return false;
                }
            }

            const angleBA = Math.atan2(nodeA.y - nodeB.y, nodeA.x - nodeB.x);
            for (const r of nodeB.roads) {
                const other = r.getOtherNode(nodeB);
                const existingAngle = Math.atan2(other.y - nodeB.y, other.x - nodeB.x);
                if (Math.abs(Vector2D.normalizeAngle(angleBA - existingAngle)) < minAngleRad) {
                    return false;
                }
            }

            // Ensure segment AB does not intersect any existing road segment
            for (const existing of roads) {
                if (
                    existing.nodeA === nodeA || existing.nodeA === nodeB ||
                    existing.nodeB === nodeA || existing.nodeB === nodeB
                ) {
                    continue;
                }
                const hit = Vector2D.segmentIntersection(
                    nodeA.x, nodeA.y,
                    nodeB.x, nodeB.y,
                    existing.nodeA.x, existing.nodeA.y,
                    existing.nodeB.x, existing.nodeB.y,
                    0.01
                );
                if (hit) return false;
            }

            // Ensure segment AB does not pass right through another node's radius
            for (const otherNode of nodes) {
                if (otherNode === nodeA || otherNode === nodeB) continue;
                const proj = Vector2D.closestPointOnSegment(
                    otherNode.x, otherNode.y,
                    nodeA.x, nodeA.y,
                    nodeB.x, nodeB.y
                );
                if (proj.t > 0.05 && proj.t < 0.95 && proj.dist < otherNode.radius + 22) {
                    return false;
                }
            }

            return true;
        };

        const addRoad = (nodeA, nodeB, options = {}) => {
            if (!canAddRoad(nodeA, nodeB)) return null;
            const key = nodeA.id < nodeB.id
                ? `${nodeA.id}-${nodeB.id}`
                : `${nodeB.id}-${nodeA.id}`;
            roadKeySet.add(key);

            const road = new Road(nextRoadId++, nodeA, nodeB, options);
            nodeA.addRoad(road);
            nodeB.addRoad(road);
            roads.push(road);
            return road;
        };

        // 2. Connect the outer perimeter ring: straight boundary edges + 4 diagonal corner roads
        // Top & Bottom straight edges
        addRoad(gridNodes[1][0], gridNodes[2][0], { isPerimeter: true });
        addRoad(gridNodes[1][3], gridNodes[2][3], { isPerimeter: true });
        // Left & Right straight edges
        addRoad(gridNodes[0][1], gridNodes[0][2], { isPerimeter: true });
        addRoad(gridNodes[3][1], gridNodes[3][2], { isPerimeter: true });

        // 4 Diagonal corner roads limiting the town within the rectangle
        addRoad(gridNodes[1][0], gridNodes[0][1], { isPerimeter: true, isDiagonal: true });
        addRoad(gridNodes[2][0], gridNodes[3][1], { isPerimeter: true, isDiagonal: true });
        addRoad(gridNodes[0][2], gridNodes[1][3], { isPerimeter: true, isDiagonal: true });
        addRoad(gridNodes[3][2], gridNodes[2][3], { isPerimeter: true, isDiagonal: true });

        // 3. Add internal diagonal roads first so they are featured cleanly in the city
        const n11 = gridNodes[1][1];
        const n21 = gridNodes[2][1];
        const n12 = gridNodes[1][2];
        const n22 = gridNodes[2][2];

        if (Math.random() < 0.5) {
            // Pattern A: Central 4-way diagonal crossing node in the middle of the town
            const cx = (n11.x + n21.x + n12.x + n22.x) * 0.25;
            const cy = (n11.y + n21.y + n12.y + n22.y) * 0.25;
            const centerNode = new RoadNode(nextNodeId++, cx, cy, false);
            nodes.push(centerNode);

            addRoad(n11, centerNode, { isDiagonal: true });
            addRoad(centerNode, n22, { isDiagonal: true });
            addRoad(n21, centerNode, { isDiagonal: true });
            addRoad(centerNode, n12, { isDiagonal: true });
        } else {
            // Pattern B: Diagonal avenue across the center block + 1 outer block diagonal
            if (Math.random() < 0.5) {
                addRoad(n11, n22, { isDiagonal: true });
            } else {
                addRoad(n21, n12, { isDiagonal: true });
            }

            // Pick 1 outer block for an additional diagonal road
            const outerDiagPairs = [
                [gridNodes[1][0], n21],
                [gridNodes[2][0], n11],
                [n12, gridNodes[2][3]],
                [n22, gridNodes[1][3]],
                [gridNodes[0][1], n12],
                [n21, gridNodes[3][2]]
            ];
            CityGenerator._shuffle(outerDiagPairs);
            for (const [a, b] of outerDiagPairs) {
                if (addRoad(a, b, { isDiagonal: true })) {
                    break;
                }
            }
        }

        // 4. Add orthogonal grid streets connecting perimeter to interior and interior nodes together
        // All candidate orthogonal links
        const orthogonalCandidates = [
            // Spokes from perimeter to interior nodes
            [gridNodes[1][0], n11],
            [gridNodes[2][0], n21],
            [gridNodes[0][1], n11],
            [gridNodes[0][2], n12],
            [gridNodes[3][1], n21],
            [gridNodes[3][2], n22],
            [gridNodes[1][3], n12],
            [gridNodes[2][3], n22],
            // Inner box orthogonal streets
            [n11, n21],
            [n12, n22],
            [n11, n12],
            [n21, n22]
        ];

        // Shuffle so different crossings get 3 or 4 roads each time while never exceeding 4 roads
        CityGenerator._shuffle(orthogonalCandidates);
        for (const [a, b] of orthogonalCandidates) {
            addRoad(a, b, {});
        }

        return new City(width, height, rect, nodes, roads);
    }

    static _shuffle(arr) {
        for (let i = arr.length - 1; i > 0; i--) {
            const j = Math.floor(Math.random() * (i + 1));
            [arr[i], arr[j]] = [arr[j], arr[i]];
        }
        return arr;
    }

    static _randomPartition(parts, minShare = 0.25) {
        const raw = [];
        let sum = 0;
        for (let i = 0; i < parts; i++) {
            const v = 0.75 + Math.random() * 0.5;
            raw.push(v);
            sum += v;
        }
        const weights = raw.map(v => Math.max(minShare, v / sum));
        const normSum = weights.reduce((a, b) => a + b, 0);
        return weights.map(w => w / normSum);
    }
}
