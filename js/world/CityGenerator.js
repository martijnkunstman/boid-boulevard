/**
 * CityGenerator - Randomly generates a self-contained city road network within a rectangular boundary:
 *  - Rectangular perimeter limited by diagonal corner roads (no roads leading out of the city)
 *  - Grid streets + internal diagonal avenues
 *  - Height differences (hills/valleys across the city)
 *  - Bridge overpasses where roads cross each other at different heights
 *  - Crossings equipped with multi-phase stoplights (TrafficLights)
 */
class CityGenerator {
    /**
     * @param {number} width - Canvas/world width
     * @param {number} height - Canvas/world height
     * @returns {City}
     */
    static generate(width = 1000, height = 660) {
        const marginX = 64;
        const marginY = 54;
        const rect = {
            left: marginX,
            top: marginY,
            right: width - marginX,
            bottom: height - marginY,
            width: width - marginX * 2,
            height: height - marginY * 2
        };

        // 1. Generate smooth terrain elevation hills/ridges for height differences
        const hills = CityGenerator._generateElevationHills(rect);
        const sampleTerrainElevation = (x, y) => {
            let z = 0;
            for (const h of hills) {
                const dx = x - h.x;
                const dy = y - h.y;
                const d2 = dx * dx + dy * dy;
                const r2 = h.radius * h.radius;
                if (d2 < r2) {
                    const falloff = 1 - d2 / r2;
                    z += h.peak * falloff * falloff;
                }
            }
            return Math.round(Math.min(14, Math.max(0, z)) * 10) / 10;
        };

        // 2. Build a 5x4 grid of candidate coordinates inside the rectangle
        const cols = 5;
        const rows = 4;
        const colWeights = CityGenerator._randomPartition(cols - 1, 0.22);
        const rowWeights = CityGenerator._randomPartition(rows - 1, 0.22);

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

        // Create nodes (omitting the 4 extreme corners so diagonal roads limit the town corners within the rectangle)
        for (let c = 0; c < cols; c++) {
            for (let r = 0; r < rows; r++) {
                if (isCorner(c, r)) continue;

                const onBoundary = (c === 0 || c === cols - 1 || r === 0 || r === rows - 1);
                let x = colX[c];
                let y = rowY[r];

                // Slight organic jitter for interior intersections
                if (!onBoundary) {
                    x += (Math.random() * 2 - 1) * 14;
                    y += (Math.random() * 2 - 1) * 12;
                }

                const elev = sampleTerrainElevation(x, y);
                const node = new RoadNode(nextNodeId++, x, y, elev, onBoundary);
                nodes.push(node);
                gridNodes[c][r] = node;
            }
        }

        const roads = [];
        let nextRoadId = 1;
        const roadKeySet = new Set();

        const addRoad = (nodeA, nodeB, options = {}) => {
            if (!nodeA || !nodeB || nodeA === nodeB) return null;
            const key = nodeA.id < nodeB.id
                ? `${nodeA.id}-${nodeB.id}`
                : `${nodeB.id}-${nodeA.id}`;
            if (roadKeySet.has(key)) return null;
            roadKeySet.add(key);

            const road = new Road(nextRoadId++, nodeA, nodeB, options);
            nodeA.addRoad(road);
            nodeB.addRoad(road);
            roads.push(road);
            return road;
        };

        // 3. Connect outer boundary: rectangular sides + 4 diagonal corner-limiting roads
        // Top boundary (c = 1 .. cols-2, r = 0)
        for (let c = 1; c < cols - 2; c++) {
            addRoad(gridNodes[c][0], gridNodes[c + 1][0], { isPerimeter: true });
        }
        // Bottom boundary (c = 1 .. cols-2, r = rows-1)
        for (let c = 1; c < cols - 2; c++) {
            addRoad(gridNodes[c][rows - 1], gridNodes[c + 1][rows - 1], { isPerimeter: true });
        }
        // Left boundary (c = 0, r = 1 .. rows-2)
        for (let r = 1; r < rows - 2; r++) {
            addRoad(gridNodes[0][r], gridNodes[0][r + 1], { isPerimeter: true });
        }
        // Right boundary (c = cols-1, r = 1 .. rows-2)
        for (let r = 1; r < rows - 2; r++) {
            addRoad(gridNodes[cols - 1][r], gridNodes[cols - 1][r + 1], { isPerimeter: true });
        }

        // 4 Diagonal roads limiting the corners of the town rectangle
        addRoad(gridNodes[1][0], gridNodes[0][1], { isPerimeter: true, isDiagonal: true });
        addRoad(gridNodes[cols - 2][0], gridNodes[cols - 1][1], { isPerimeter: true, isDiagonal: true });
        addRoad(gridNodes[0][rows - 2], gridNodes[1][rows - 1], { isPerimeter: true, isDiagonal: true });
        addRoad(gridNodes[cols - 1][rows - 2], gridNodes[cols - 2][rows - 1], { isPerimeter: true, isDiagonal: true });

        // 4. Connect internal horizontal and vertical streets
        for (let c = 0; c < cols; c++) {
            for (let r = 0; r < rows; r++) {
                const curr = gridNodes[c][r];
                if (!curr) continue;

                // Horizontal interior segments
                if (c + 1 < cols && r > 0 && r < rows - 1 && gridNodes[c + 1][r]) {
                    // Keep nearly all interior grid streets for rich connectivity, occasionally skip 1 for variety
                    addRoad(curr, gridNodes[c + 1][r], {});
                }
                // Vertical interior segments
                if (r + 1 < rows && c > 0 && c < cols - 1 && gridNodes[c][r + 1]) {
                    addRoad(curr, gridNodes[c][r + 1], {});
                }
            }
        }

        // 5. Add internal diagonal roads & Bridge Overpasses across interior city blocks!
        // Candidate interior blocks are (c, r) for c in [1..cols-3], r in [0..rows-2] where all 4 corners exist
        const candidateBlocks = [];
        for (let c = 0; c < cols - 1; c++) {
            for (let r = 0; r < rows - 1; r++) {
                const n00 = gridNodes[c][r];
                const n10 = gridNodes[c + 1][r];
                const n01 = gridNodes[c][r + 1];
                const n11 = gridNodes[c + 1][r + 1];
                if (n00 && n10 && n01 && n11) {
                    candidateBlocks.push({ c, r, n00, n10, n01, n11 });
                }
            }
        }

        // Shuffle candidate blocks randomly
        for (let i = candidateBlocks.length - 1; i > 0; i--) {
            const j = Math.floor(Math.random() * (i + 1));
            [candidateBlocks[i], candidateBlocks[j]] = [candidateBlocks[j], candidateBlocks[i]];
        }

        let bridgeCount = 0;
        for (let i = 0; i < candidateBlocks.length; i++) {
            const blk = candidateBlocks[i];
            if (i < 3) {
                // Create a pair of crossing diagonal roads where ONE is a ground road and ONE is a BRIDGE OVERPASS!
                const firstIsMainDiag = (i % 2 === 0);
                if (firstIsMainDiag) {
                    addRoad(blk.n00, blk.n11, { isDiagonal: true, isBridge: false });
                    addRoad(blk.n10, blk.n01, { isDiagonal: true, isBridge: true, bridgeDeckHeight: 8.5 });
                } else {
                    addRoad(blk.n10, blk.n01, { isDiagonal: true, isBridge: false });
                    addRoad(blk.n00, blk.n11, { isDiagonal: true, isBridge: true, bridgeDeckHeight: 8.5 });
                }
                bridgeCount++;
            } else if (i === 3) {
                // Create a central diagonal X-crossing at ground level WITH a shared stoplight intersection node!
                const ix = (blk.n00.x + blk.n10.x + blk.n01.x + blk.n11.x) * 0.25;
                const iy = (blk.n00.y + blk.n10.y + blk.n01.y + blk.n11.y) * 0.25;
                const iElev = sampleTerrainElevation(ix, iy);
                const centerNode = new RoadNode(nextNodeId++, ix, iy, iElev, false);
                nodes.push(centerNode);
                addRoad(blk.n00, centerNode, { isDiagonal: true });
                addRoad(centerNode, blk.n11, { isDiagonal: true });
                addRoad(blk.n10, centerNode, { isDiagonal: true });
                addRoad(centerNode, blk.n01, { isDiagonal: true });
            } else if (i <= 5) {
                // Single diagonal avenue across the block
                if (Math.random() < 0.5) {
                    addRoad(blk.n00, blk.n11, { isDiagonal: true });
                } else {
                    addRoad(blk.n10, blk.n01, { isDiagonal: true });
                }
            }
        }

        // 6. Initialize stoplights (IntersectionControllers) at all crossings with >= 3 roads
        for (const node of nodes) {
            if (node.roads.length >= 3) {
                node.trafficController = new IntersectionController(node);
            }
        }

        return new City(width, height, rect, nodes, roads, hills);
    }

    static _randomPartition(parts, minShare = 0.2) {
        const raw = [];
        let sum = 0;
        for (let i = 0; i < parts; i++) {
            const v = 0.65 + Math.random() * 0.7;
            raw.push(v);
            sum += v;
        }
        const weights = raw.map(v => Math.max(minShare, v / sum));
        const normSum = weights.reduce((a, b) => a + b, 0);
        return weights.map(w => w / normSum);
    }

    static _generateElevationHills(rect) {
        const hills = [];
        const count = 3;
        for (let i = 0; i < count; i++) {
            hills.push({
                x: rect.left + (0.18 + Math.random() * 0.64) * rect.width,
                y: rect.top + (0.18 + Math.random() * 0.64) * rect.height,
                radius: 170 + Math.random() * 130,
                peak: 5.5 + Math.random() * 6.5
            });
        }
        return hills;
    }
}
