/**
 * Vector2D - 2D vector math and geometric helper utilities.
 */
class Vector2D {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    clone() {
        return new Vector2D(this.x, this.y);
    }

    set(x, y) {
        this.x = x;
        this.y = y;
        return this;
    }

    add(v) {
        return new Vector2D(this.x + v.x, this.y + v.y);
    }

    sub(v) {
        return new Vector2D(this.x - v.x, this.y - v.y);
    }

     scale(s) {
        return new Vector2D(this.x * s, this.y * s);
    }

    dot(v) {
        return this.x * v.x + this.y * v.y;
    }

    cross(v) {
        return this.x * v.y - this.y * v.x;
    }

    lengthSq() {
        return this.x * this.x + this.y * this.y;
    }

    length() {
        return Math.hypot(this.x, this.y);
    }

    normalize() {
        const len = this.length();
        if (len < 1e-8) return new Vector2D(0, 0);
        return new Vector2D(this.x / len, this.y / len);
    }

    perp() {
        return new Vector2D(-this.y, this.x);
    }

    angle() {
        return Math.atan2(this.y, this.x);
    }

    distanceTo(v) {
        return Math.hypot(this.x - v.x, this.y - v.y);
    }

    distanceSqTo(v) {
        const dx = this.x - v.x;
        const dy = this.y - v.y;
        return dx * dx + dy * dy;
    }

    static fromAngle(angle, length = 1) {
        return new Vector2D(Math.cos(angle) * length, Math.sin(angle) * length);
    }

    static lerp(a, b, t) {
        return new Vector2D(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t
        );
    }

    static normalizeAngle(angle) {
        while (angle > Math.PI) angle -= Math.PI * 2;
        while (angle < -Math.PI) angle += Math.PI * 2;
        return angle;
    }

    /**
     * Closest point on line segment AB to point P.
     * Returns { point: Vector2D, t: number, distSq: number, dist: number, signedSide: number }
     */
    static closestPointOnSegment(px, py, ax, ay, bx, by) {
        const abx = bx - ax;
        const aby = by - ay;
        const apx = px - ax;
        const apy = py - ay;
        const abLenSq = abx * abx + aby * aby;
        let t = 0;
        if (abLenSq > 1e-8) {
            t = (apx * abx + apy * aby) / abLenSq;
            if (t < 0) t = 0;
            else if (t > 1) t = 1;
        }
        const cx = ax + abx * t;
        const cy = ay + aby * t;
        const dx = px - cx;
        const dy = py - cy;
        const distSq = dx * dx + dy * dy;
        const cross = abx * apy - aby * apx;
        return {
            x: cx,
            y: cy,
            t,
            distSq,
            dist: Math.sqrt(distSq),
            signedSide: cross >= 0 ? 1 : -1
        };
    }

    /**
     * Intersection between two open line segments AB and CD.
     * Returns null or { x, y, t, u } where t, u in (0, 1).
     */
    static segmentIntersection(ax, ay, bx, by, cx, cy, dx, dy, margin = 0.02) {
        const rX = bx - ax;
        const rY = by - ay;
        const sX = dx - cx;
        const sY = dy - cy;
        const denom = rX * sY - rY * sX;
        if (Math.abs(denom) < 1e-8) return null;

        const qpx = cx - ax;
        const qpy = cy - ay;
        const t = (qpx * sY - qpy * sX) / denom;
        const u = (qpx * rY - qpy * rX) / denom;

        if (t > margin && t < 1 - margin && u > margin && u < 1 - margin) {
            return {
                x: ax + t * rX,
                y: ay + t * rY,
                t,
                u
            };
        }
        return null;
    }

    /**
     * Ray-circle intersection for sensor vs boid detection.
     * Ray from (ox, oy) in direction (dx, dy) (unit vector) up to maxDist.
     */
    static rayCircleIntersection(ox, oy, dx, dy, cx, cy, radius, maxDist) {
        const fx = ox - cx;
        const fy = oy - cy;
        const b = 2 * (fx * dx + fy * dy);
        const c = (fx * fx + fy * fy) - radius * radius;

        let discriminant = b * b - 4 * c;
        if (discriminant < 0) return null;

        discriminant = Math.sqrt(discriminant);
        const t1 = (-b - discriminant) / 2;
        const t2 = (-b + discriminant) / 2;

        if (t1 >= 0 && t1 <= maxDist) return t1;
        if (t2 >= 0 && t2 <= maxDist) return 0;
        return null;
    }
}
