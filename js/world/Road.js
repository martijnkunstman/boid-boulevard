/**
 * Road - Represents a unified single-track city road segment connecting two RoadNodes.
 * Supports orthogonal and diagonal road geometry without left/right lane division.
 */
class Road {
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
        this.width = options.width || 32;
        this.halfWidth = this.width * 0.5;
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
}
