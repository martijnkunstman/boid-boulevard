/**
 * CityGenerator - Randomly generates a single closed-loop track (a circuit without any splits):
 *  - Every node joins exactly two roads, so there are no crossings or forks
 *  - Corner points sit around the map center at increasing angles, so the loop never crosses itself
 *  - A few non-adjacent corner points are pulled inward, so the track turns both ways
 *    (at least MIN_TURNS_EACH_WAY turns to each side)
 *  - Every candidate is validated (turn sharpness, segment length, spacing between road surfaces);
 *    invalid candidates are discarded and a new one is drawn from the same seeded generator
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
     * @returns {City}
     */
    static generate(width = 1020, height = 620, seed = null) {
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

        for (let attempt = 0; attempt < CityGenerator.MAX_ATTEMPTS; attempt++) {
            const points = CityGenerator._randomLoopPoints(rect);
            if (CityGenerator._isValidLoop(points, Road.DEFAULT_WIDTH)) {
                return CityGenerator._buildCity(width, height, rect, points);
            }
        }

        // Practically unreachable; a fixed, known-valid shape keeps generation from ever failing
        return CityGenerator._buildCity(width, height, rect, CityGenerator._fallbackLoopPoints(rect));
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
     */
    static _buildCity(width, height, rect, points) {
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
        return new City(width, height, rect, nodes, roads, Random.currentSeed);
    }

    static _shuffle(arr) {
        for (let i = arr.length - 1; i > 0; i--) {
            const j = Math.floor(Random.next() * (i + 1));
            [arr[i], arr[j]] = [arr[j], arr[i]];
        }
        return arr;
    }
}
