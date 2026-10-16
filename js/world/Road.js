/**
 * Road - Represents a two-way city road segment connecting two RoadNodes.
 * Traffic keeps right: each travel direction has its own lane, centered a quarter
 * of the road width to the right of the centerline.
 * Supports orthogonal and diagonal road geometry.
 */
class Road {
    static DEFAULT_WIDTH = 44;

    /**
     * @param {number} id
     * @param {RoadNode} nodeA
     * @param {RoadNode} nodeB
     * @param {object} options
     */
    constructor(id, nodeA, nodeB, options = {}) {
        this.id = id;
        this.nodeA = nodeA;
        this.nodeB = nodeB;
        this.width = options.width || Road.DEFAULT_WIDTH;
        this.halfWidth = this.width * 0.5;
        // Distance from the centerline to the center of each lane
        this.laneOffset = this.width * 0.25;
        this.isDiagonal = Boolean(options.isDiagonal);
        this.isPerimeter = Boolean(options.isPerimeter);

        this.recalculateGeometry();
    }

    recalculateGeometry() {
        const dx = this.nodeB.x - this.nodeA.x;
        const dy = this.nodeB.y - this.nodeA.y;
        this.length = Math.hypot(dx, dy);
        this.dir = this.length > 1e-6
            ? new Vector2D(dx / this.length, dy / this.length)
            : new Vector2D(1, 0);

        const pad = this.halfWidth + 1;
        this.minX = Math.min(this.nodeA.x, this.nodeB.x) - pad;
        this.maxX = Math.max(this.nodeA.x, this.nodeB.x) + pad;
        this.minY = Math.min(this.nodeA.y, this.nodeB.y) - pad;
        this.maxY = Math.max(this.nodeA.y, this.nodeB.y) + pad;

        const angleDeg = Math.abs((Math.atan2(dy, dx) * 180) / Math.PI) % 90;
        if (angleDeg > 14 && angleDeg < 76) {
            this.isDiagonal = true;
        }
    }

    getOtherNode(node) {
        return node === this.nodeA ? this.nodeB : this.nodeA;
    }

    /**
     * Returns centerline point at parameter t in [0, 1].
     */
    getPointAt(t) {
        return new Vector2D(
            this.nodeA.x + (this.nodeB.x - this.nodeA.x) * t,
            this.nodeA.y + (this.nodeB.y - this.nodeA.y) * t
        );
    }

    /**
     * Returns the unit vector pointing to the right-hand side when driving in travelSign direction
     * (+1 = nodeA -> nodeB, -1 = nodeB -> nodeA). Canvas y points down, so right of (x, y) is (-y, x).
     */
    getRightNormal(travelSign) {
        return new Vector2D(-this.dir.y * travelSign, this.dir.x * travelSign);
    }

    /**
     * Returns the center point of the right-hand (keep-right) lane for travelSign at parameter t.
     */
    getLanePoint(t, travelSign) {
        const right = this.getRightNormal(travelSign);
        return new Vector2D(
            this.nodeA.x + (this.nodeB.x - this.nodeA.x) * t + right.x * this.laneOffset,
            this.nodeA.y + (this.nodeB.y - this.nodeA.y) * t + right.y * this.laneOffset
        );
    }
}
