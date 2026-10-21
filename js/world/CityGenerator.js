/**
 * CityGenerator - Procedural track and city road network generator.
 * Supports three selectable track topologies:
 *  1. 'loop' (default): Closed-loop circuit (10-14 corners, at least 3 left and 3 right turns, no splits)
 *  2. 'crosslink': Closed-loop circuit with 1-2 interior cross-link chords (3-way intersections)
 *  3. 'grid': Multi-block city grid with 3-way and 4-way intersections
 */
class CityGenerator {
    static MIN_CORNERS = 10;
    static MAX_CORNERS = 14;
    static MIN_TURNS_EACH_WAY = 3;
    // Sharpest allowed direction change at a corner (~110 deg), so boids can take every turn
    static MAX_TURN_ANGLE = 1.92;
    // Direction change from which a corner counts as a real turn (~14 deg)
    static MIN_COUNTED_TURN = 0.25;
    static MIN_SEGMENT_LENGTH = 110;
    // Extra clearance between road surfaces of non-adjacent segments (px, on top of the road width)
    static SURFACE_CLEARANCE = 24;
    static MAX_ATTEMPTS = 400;

    /**
     * @param {number} width - Canvas/world width
     * @param {number} height - Canvas/world height
     * @param {string|number|null} seed - Optional seed for deterministic city generation
     * @param {string} trackMode - 'loop' | 'crosslink' | 'grid'
     * @returns {City}
     */
    static generate(width = 1020, height = 620, seed = null, trackMode = 'loop') {
        if (seed !== null && seed !== undefined) {
            Random.setSeed(seed);
        } else {
            Random.setSeed(Random.currentSeed);
        }

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

        if (trackMode === 'grid') {
            return CityGenerator._generateGridCity(width, height, rect);
        }

        for (let attempt = 0; attempt < CityGenerator.MAX_ATTEMPTS; attempt++) {
            const points = CityGenerator._randomLoopPoints(rect);
            if (CityGenerator._isValidLoop(points, Road.DEFAULT_WIDTH)) {
                return CityGenerator._buildCity(width, height, rect, points, trackMode);
            }
        }

        // Practically unreachable; a fixed, known-valid shape keeps generation from ever failing
        return CityGenerator._buildCity(width, height, rect, CityGenerator._fallbackLoopPoints(rect), trackMode);
    }

    /**
     * Corner points at increasing angles around the rect center (one point per angle, so the polygon
     * is never self-intersecting). 3-4 non-adjacent points are pulled inward to create opposite turns.
     */
    static _randomLoopPoints(rect) {
        const span = CityGenerator.MAX_CORNERS - CityGenerator.MIN_CORNERS + 1;
        const count = CityGenerator.MIN_CORNERS + Math.floor(Random.next() * span);
        const inward = CityGenerator._pickNonAdjacent(count, 3 + (Random.next() < 0.5 ? 1 : 0));

        const cx = rect.left + rect.width * 0.5;
        const cy = rect.top + rect.height * 0.5;
        const rx = rect.width * 0.5;
        const ry = rect.height * 0.5;
        const step = (Math.PI * 2) / count;
        const startAngle = Random.next() * Math.PI * 2;

        const points = [];
        for (let i = 0; i < count; i++) {
            const angle = startAngle + i * step + (Random.next() - 0.5) * 0.4 * step;
            const scale = inward.has(i)
                ? 0.35 + Random.next() * 0.25
                : 0.78 + Random.next() * 0.22;
            points.push({
                x: cx + Math.cos(angle) * rx * scale,
                y: cy + Math.sin(angle) * ry * scale
            });
        }
        return points;
    }

    /**
     * Picks up to `wanted` indices out of a ring of `count`, no two of them next to each other.
     */
    static _pickNonAdjacent(count, wanted) {
        const order = Array.from({ length: count }, (_, i) => i);
        CityGenerator._shuffle(order);
        const picked = new Set();
        for (const i of order) {
            if (picked.size >= wanted) break;
            const prev = (i - 1 + count) % count;
            const next = (i + 1) % count;
            if (!picked.has(prev) && !picked.has(next)) {
                picked.add(i);
            }
        }
        return picked;
    }

    /**
     * Checks a closed corner loop: segment lengths, turn sharpness, enough turns to each side,
     * and enough space between the road surfaces of non-adjacent segments.
     */
    static _isValidLoop(points, roadWidth) {
        const n = points.length;
        let turnsOneWay = 0;
        let turnsOtherWay = 0;

        for (let i = 0; i < n; i++) {
            const prev = points[(i - 1 + n) % n];
            const curr = points[i];
            const next = points[(i + 1) % n];

            if (Math.hypot(next.x - curr.x, next.y - curr.y) < CityGenerator.MIN_SEGMENT_LENGTH) {
                return false;
            }

            const inAngle = Math.atan2(curr.y - prev.y, curr.x - prev.x);
            const outAngle = Math.atan2(next.y - curr.y, next.x - curr.x);
            const turn = Vector2D.normalizeAngle(outAngle - inAngle);
            if (Math.abs(turn) > CityGenerator.MAX_TURN_ANGLE) return false;
            if (turn >= CityGenerator.MIN_COUNTED_TURN) turnsOneWay++;
            else if (turn <= -CityGenerator.MIN_COUNTED_TURN) turnsOtherWay++;
        }

        if (turnsOneWay < CityGenerator.MIN_TURNS_EACH_WAY || turnsOtherWay < CityGenerator.MIN_TURNS_EACH_WAY) {
            return false;
        }

        // Non-adjacent segments must keep their road surfaces apart
        const minGap = roadWidth + CityGenerator.SURFACE_CLEARANCE;
        for (let i = 0; i < n; i++) {
            const a1 = points[i];
            const a2 = points[(i + 1) % n];
            for (let j = i + 2; j < n; j++) {
                if (i === 0 && j === n - 1) continue; // adjacent through the loop closure
                const b1 = points[j];
                const b2 = points[(j + 1) % n];
                if (CityGenerator._segmentDistance(a1, a2, b1, b2) < minGap) {
                    return false;
                }
            }
        }
        return true;
    }

    static _segmentDistance(a1, a2, b1, b2) {
        if (Vector2D.segmentIntersection(a1.x, a1.y, a2.x, a2.y, b1.x, b1.y, b2.x, b2.y, 0)) {
            return 0;
        }
        return Math.min(
            Vector2D.closestPointOnSegment(a1.x, a1.y, b1.x, b1.y, b2.x, b2.y).dist,
            Vector2D.closestPointOnSegment(a2.x, a2.y, b1.x, b1.y, b2.x, b2.y).dist,
            Vector2D.closestPointOnSegment(b1.x, b1.y, a1.x, a1.y, a2.x, a2.y).dist,
            Vector2D.closestPointOnSegment(b2.x, b2.y, a1.x, a1.y, a2.x, a2.y).dist
        );
    }

    /**
     * Fixed 12-corner loop with three inward corners, used only if no random candidate validates.
     */
    static _fallbackLoopPoints(rect) {
        const cx = rect.left + rect.width * 0.5;
        const cy = rect.top + rect.height * 0.5;
        const scales = [1, 0.95, 0.5, 0.95, 1, 0.95, 0.5, 0.95, 1, 0.95, 0.5, 0.95];
        return scales.map((scale, i) => {
            const angle = (Math.PI * 2 * i) / scales.length;
            return {
                x: cx + Math.cos(angle) * rect.width * 0.5 * scale,
                y: cy + Math.sin(angle) * rect.height * 0.5 * scale
            };
        });
    }

    /**
     * Turns the loop corners into RoadNodes and Roads (corner i connects to corner i + 1).
     * When trackMode === 'crosslink', adds 1-2 interior cross-link roads creating 3-way intersections.
     */
    static _buildCity(width, height, rect, points, trackMode = 'loop') {
        const nodes = points.map((p, i) => new RoadNode(i + 1, Math.round(p.x), Math.round(p.y), false));
        const roads = [];
        for (let i = 0; i < nodes.length; i++) {
            const nodeA = nodes[i];
            const nodeB = nodes[(i + 1) % nodes.length];
            const road = new Road(i + 1, nodeA, nodeB, {});
            nodeA.addRoad(road);
            nodeB.addRoad(road);
            roads.push(road);
        }

        if (trackMode === 'crosslink') {
            CityGenerator._addCrossLinks(nodes, roads);
        }

        return new City(width, height, rect, nodes, roads, Random.currentSeed, { trackMode });
    }

    /**
     * Adds 1 or 2 valid interior chord roads connecting opposite sides of the loop so boids
     * experience real 3-way intersections and route selection.
     */
    static _addCrossLinks(nodes, roads) {
        const n = nodes.length;
        const candidates = [];
        for (let i = 0; i < n; i++) {
            for (let j = i + 3; j < n; j++) {
                const ringDist = Math.min(j - i, n - (j - i));
                if (ringDist < 4) continue;
                const a = nodes[i];
                const b = nodes[j];
                const len = Math.hypot(b.x - a.x, b.y - a.y);
                if (len >= 160 && len <= 440) {
                    candidates.push([a, b, len]);
                }
            }
        }
        CityGenerator._shuffle(candidates);

        let added = 0;
        for (const [nodeA, nodeB] of candidates) {
            if (added >= 2) break;
            if (nodeA.roads.length >= 3 || nodeB.roads.length >= 3) continue;

            // Check that the chord does not cross or pass too close to non-incident roads
            let valid = true;
            for (const r of roads) {
                if (r.nodeA === nodeA || r.nodeB === nodeA || r.nodeA === nodeB || r.nodeB === nodeB) {
                    // Check angle clearance with incident roads
                    const shared = (r.nodeA === nodeA || r.nodeB === nodeA) ? nodeA : nodeB;
                    const otherExisting = r.getOtherNode(shared);
                    const otherNew = shared === nodeA ? nodeB : nodeA;
                    const ang1 = Math.atan2(otherExisting.y - shared.y, otherExisting.x - shared.x);
                    const ang2 = Math.atan2(otherNew.y - shared.y, otherNew.x - shared.x);
                    if (Math.abs(Vector2D.normalizeAngle(ang1 - ang2)) < 0.72) {
                        valid = false;
                        break;
                    }
                    continue;
                }
                const dist = CityGenerator._segmentDistance(nodeA, nodeB, r.nodeA, r.nodeB);
                if (dist < Road.DEFAULT_WIDTH + 18) {
                    valid = false;
                    break;
                }
            }

            if (valid) {
                const road = new Road(roads.length + 1, nodeA, nodeB, {});
                nodeA.addRoad(road);
                nodeB.addRoad(road);
                roads.push(road);
                added++;
            }
        }
    }

    /**
     * Generates a 3x3 intersecting multi-block city grid with organic node jitter,
     * 3-way T-junctions, and 4-way crossings.
     */
    static _generateGridCity(width, height, rect) {
        const cols = 4;
        const rows = 3;
        const nodes = [];
        const grid = [];
        let nodeId = 1;

        for (let r = 0; r < rows; r++) {
            const rowNodes = [];
            for (let c = 0; c < cols; c++) {
                const baseX = rect.left + (c / (cols - 1)) * rect.width;
                const baseY = rect.top + (r / (rows - 1)) * rect.height;
                const isPerim = r === 0 || r === rows - 1 || c === 0 || c === cols - 1;
                const jx = (Random.next() - 0.5) * (isPerim ? 26 : 38);
                const jy = (Random.next() - 0.5) * (isPerim ? 22 : 32);
                const node = new RoadNode(
                    nodeId++,
                    Math.round(Math.max(rect.left, Math.min(rect.right, baseX + jx))),
                    Math.round(Math.max(rect.top, Math.min(rect.bottom, baseY + jy))),
                    isPerim
                );
                nodes.push(node);
                rowNodes.push(node);
            }
            grid.push(rowNodes);
        }

        const roads = [];
        const connect = (a, b, isPerim = false) => {
            if (a.roads.length >= 4 || b.roads.length >= 4) return;
            const road = new Road(roads.length + 1, a, b, { isPerimeter: isPerim });
            a.addRoad(road);
            b.addRoad(road);
            roads.push(road);
        };

        // Horizontal and vertical streets (always include outer ring so there are no dead ends)
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols - 1; c++) {
                const isPerim = r === 0 || r === rows - 1;
                if (isPerim || Random.next() < 0.85) {
                    connect(grid[r][c], grid[r][c + 1], isPerim);
                }
            }
        }
        for (let r = 0; r < rows - 1; r++) {
            for (let c = 0; c < cols; c++) {
                const isPerim = c === 0 || c === cols - 1;
                if (isPerim || Random.next() < 0.85) {
                    connect(grid[r][c], grid[r + 1][c], isPerim);
                }
            }
        }

        // Ensure every interior node has at least 2 connected roads (no dead-end cul-de-sacs)
        for (let r = 1; r < rows - 1; r++) {
            for (let c = 1; c < cols - 1; c++) {
                const node = grid[r][c];
                const neighbors = [grid[r - 1][c], grid[r + 1][c], grid[r][c - 1], grid[r][c + 1]];
                for (const nb of neighbors) {
                    if (node.roads.length >= 2) break;
                    const already = node.roads.some(rd => rd.getOtherNode(node) === nb);
                    if (!already) connect(node, nb, false);
                }
            }
        }

        return new City(width, height, rect, nodes, roads, Random.currentSeed, { trackMode: 'grid' });
    }

    static _shuffle(arr) {
        for (let i = arr.length - 1; i > 0; i--) {
            const j = Math.floor(Random.next() * (i + 1));
            [arr[i], arr[j]] = [arr[j], arr[i]];
        }
        return arr;
    }
}
