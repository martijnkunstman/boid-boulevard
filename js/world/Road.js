/**
 * Road - Represents a flat, two-way city road segment connecting two RoadNodes.
 * Supports orthogonal and diagonal road geometry.
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
        this.width = options.width || 36;
        this.halfWidth = this.width * 0.5;
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
        // Right-hand normal when travelling from nodeA -> nodeB
        this.rightNormal = new Vector2D(-this.dir.y, this.dir.x);

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
     * Returns right-hand lane center point at parameter t in [0, 1].
     * @param {number} t - Position along road (0 = nodeA, 1 = nodeB)
     * @param {number} travelSign - +1 for nodeA->nodeB, -1 for nodeB->nodeA
     */
    getLanePointAt(t, travelSign = 1) {
        const center = this.getPointAt(t);
        return center.add(this.rightNormal.scale(this.laneOffset * travelSign));
    }
}
