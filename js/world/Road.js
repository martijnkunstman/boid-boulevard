/**
 * Road - Represents a two-way city road segment connecting two RoadNodes.
 * Supports ground-level roads with height differences as well as elevated bridges (overpasses).
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
        this.width = options.width || 34;
        this.halfWidth = this.width * 0.5;
        this.laneOffset = this.width * 0.25;
        this.isBridge = Boolean(options.isBridge);
        this.isDiagonal = Boolean(options.isDiagonal);
        this.isPerimeter = Boolean(options.isPerimeter);
        this.bridgeDeckHeight = options.bridgeDeckHeight || 7.5;

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
     * Bridge ramp profile factor in [0, 1] for parameter t in [0, 1].
     */
    getBridgeProfile(t) {
        if (!this.isBridge) return 0;
        const rampStart = 0.08;
        const deckStart = 0.24;
        const deckEnd = 0.76;
        const rampEnd = 0.92;

        if (t <= rampStart || t >= rampEnd) return 0;
        if (t >= deckStart && t <= deckEnd) return 1;
        if (t < deckStart) {
            const u = (t - rampStart) / (deckStart - rampStart);
            return 0.5 - 0.5 * Math.cos(u * Math.PI);
        }
        const u = (rampEnd - t) / (rampEnd - deckEnd);
        return 0.5 - 0.5 * Math.cos(u * Math.PI);
    }

    /**
     * Returns elevation (in meters) at normalized parameter t in [0, 1] from nodeA to nodeB.
     */
    getElevationAt(t) {
        const clampedT = Math.max(0, Math.min(1, t));
        const baseZ = this.nodeA.elevation + (this.nodeB.elevation - this.nodeA.elevation) * clampedT;
        if (!this.isBridge) return baseZ;
        return baseZ + this.bridgeDeckHeight * this.getBridgeProfile(clampedT);
    }

    /**
     * Returns discrete layer (0 = ground/ramp, 1 = elevated bridge span) at parameter t.
     */
    getLayerAt(t) {
        if (!this.isBridge) return 0;
        return (t > 0.16 && t < 0.84) ? 1 : 0;
    }

    /**
     * Returns road slope (grade) in [-1, 1] at parameter t for travel direction (+1: A->B, -1: B->A).
     */
    getSlopeAt(t, travelSign = 1) {
        const delta = 0.03;
        const t1 = Math.max(0, t - delta);
        const t2 = Math.min(1, t + delta);
        const dz = this.getElevationAt(t2) - this.getElevationAt(t1);
        const ds = Math.max(1, (t2 - t1) * this.length);
        // Scale to roughly [-1, 1] for physics & neural network input
        const grade = (dz / ds) * 8.0 * travelSign;
        return Math.max(-1, Math.min(1, grade));
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
