/**
 * Vector2D - 2D vector math and zero-allocation geometric helper utilities.
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
     * Zero-allocation closest point on line segment AB to point P, written into `out`.
     * @returns {object} `out` with { x, y, t, distSq, dist, signedSide }
     */
    static closestPointOnSegmentInto(px, py, ax, ay, bx, by, out) {
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
        out.x = cx;
        out.y = cy;
        out.t = t;
        out.distSq = distSq;
        out.dist = Math.sqrt(distSq);
        out.signedSide = cross >= 0 ? 1 : -1;
        return out;
    }

    /**
     * Closest point on line segment AB to point P.
     * Returns { x: number, y: number, t: number, distSq: number, dist: number, signedSide: number }
     */
    static closestPointOnSegment(px, py, ax, ay, bx, by) {
        return Vector2D.closestPointOnSegmentInto(px, py, ax, ay, bx, by, {
            x: 0,
            y: 0,
            t: 0,
            distSq: 0,
            dist: 0,
            signedSide: 1
        });
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
     * Ray-circle intersection for sensor vs circular obstacle/boid detection.
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
        const t1 = (-b - discriminant) * 0.5;
        const t2 = (-b + discriminant) * 0.5;

        if (t1 >= 0 && t1 <= maxDist) return t1;
        if (t1 < 0 && t2 >= 0) return 0;
        return null;
    }

    /**
     * Oriented ray-ellipse intersection for accurate sensor vs boid detection.
     * Transforms ray into the target ellipse's local frame (cosH, sinH) with semi-axes (rx, ry).
     * Because (dx, dy) is a world-space unit vector, parameter t is exact world-space distance.
     */
    static rayEllipseIntersection(ox, oy, dx, dy, cx, cy, cosH, sinH, rx, ry, maxDist) {
        const px = ox - cx;
        const py = oy - cy;

        const invRx = 1 / rx;
        const invRy = 1 / ry;

        // Ray origin and direction in normalized ellipse space (where ellipse is the unit circle)
        const lx = (px * cosH + py * sinH) * invRx;
        const ly = (-px * sinH + py * cosH) * invRy;
        const ldx = (dx * cosH + dy * sinH) * invRx;
        const ldy = (-dx * sinH + dy * cosH) * invRy;

        const a = ldx * ldx + ldy * ldy;
        const halfB = lx * ldx + ly * ldy;
        const c = lx * lx + ly * ly - 1;

        const disc = halfB * halfB - a * c;
        if (disc < 0 || a < 1e-12) return null;

        const sqrtDisc = Math.sqrt(disc);
        const invA = 1 / a;
        const t1 = (-halfB - sqrtDisc) * invA;
        const t2 = (-halfB + sqrtDisc) * invA;

        if (t1 >= 0 && t1 <= maxDist) return t1;
        if (t1 < 0 && t2 >= 0) return 0;
        return null;
    }
}
