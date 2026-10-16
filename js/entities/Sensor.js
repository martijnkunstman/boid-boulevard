/**
 * Sensor - Front-mounted raycast sensor array for a Boid.
 * Detects:
 *   1. Road borders (curbs & intersection edges)
 *   2. Boids in front
 */
class Sensor {
    /**
     * @param {Boid} boid
     * @param {number} rayCount
     * @param {number} rayLength
     * @param {number} spreadAngle - Total spread angle in radians
     */
    constructor(boid, rayCount = 7, rayLength = 105, spreadAngle = (150 * Math.PI) / 180) {
        this.boid = boid;
        this.rayCount = rayCount;
        this.rayLength = rayLength;
        this.spreadAngle = spreadAngle;

        this.rayAngles = [];
        // Pre-allocate ray objects for zero-GC updates during high-speed training
        this.rays = [];
        for (let i = 0; i < this.rayCount; i++) {
            const frac = this.rayCount === 1 ? 0.5 : i / (this.rayCount - 1);
            this.rayAngles.push((frac - 0.5) * this.spreadAngle);
            this.rays.push({
                ox: 0,
                oy: 0,
                endX: 0,
                endY: 0,
                roadHitX: 0,
                roadHitY: 0,
                roadHit: false,
                roadReading: 0,
                boidHitX: 0,
                boidHitY: 0,
                boidHit: false,
                boidReading: 0
            });
        }

        // Readings in [0, 1] where 0 = clear/far, 1 = immediate proximity
        this.roadReadings = new Array(this.rayCount).fill(0);
        this.boidReadings = new Array(this.rayCount).fill(0);
        this.closingSpeedReading = 0;
        this._nearby = [];
    }

    /**
     * Updates all sensor rays against the city road boundaries and other boids.
     * @param {City} city
     * @param {Boid[]} boids
     * @param {SpatialGrid|null} grid - Grid built from `boids` this step; without it all boids are scanned
     */
    update(city, boids, grid = null) {
        const frontOffset = this.boid.length * 0.48;
        const ox = this.boid.x + Math.cos(this.boid.heading) * frontOffset;
        const oy = this.boid.y + Math.sin(this.boid.heading) * frontOffset;

        let maxClosingSpeed = 0;

        // Pre-filter nearby boids (reused array, no per-step allocation)
        const maxDistWithRadius = this.rayLength + 20;
        const nearbyBoids = this._nearby;

        if (grid) {
            grid.queryRadius(ox, oy, maxDistWithRadius, this.boid, nearbyBoids);
        } else {
            const maxDistSq = maxDistWithRadius * maxDistWithRadius;
            nearbyBoids.length = 0;
            for (let i = 0; i < boids.length; i++) {
                const other = boids[i];
                if (other === this.boid) continue;

                const dx = other.x - ox;
                const dy = other.y - oy;
                if (dx * dx + dy * dy <= maxDistSq) {
                    nearbyBoids.push(other);
                }
            }
        }

        for (let i = 0; i < this.rayCount; i++) {
            const relAngle = this.rayAngles[i];
            const rayAngle = this.boid.heading + relAngle;
            const dx = Math.cos(rayAngle);
            const dy = Math.sin(rayAngle);

            // 1. Sense road border
            const roadDist = city.raycastRoadSide(ox, oy, dx, dy, this.rayLength);
            const roadHit = roadDist < this.rayLength;
            const roadReading = roadHit
                ? Math.max(0, Math.min(1, 1 - roadDist / this.rayLength))
                : 0;
            this.roadReadings[i] = roadReading;

            // 2. Sense boids in front along this ray
            let closestBoidDist = this.rayLength;
            let hitBoid = null;

            for (let j = 0; j < nearbyBoids.length; j++) {
                const other = nearbyBoids[j];
                const hitRadius = Math.max(other.width, other.length * 0.55) + 2;
                const dist = Vector2D.rayCircleIntersection(
                    ox, oy, dx, dy,
                    other.x, other.y,
                    hitRadius,
                    closestBoidDist
                );
                // Equal distances go to the lowest id, so results do not depend on candidate order
                if (dist !== null && (dist < closestBoidDist ||
                    (dist === closestBoidDist && hitBoid && other.id < hitBoid.id))) {
                    closestBoidDist = dist;
                    hitBoid = other;
                }
            }

            let boidReading = 0;
            if (hitBoid) {
                boidReading = Math.max(0, Math.min(1, 1 - closestBoidDist / this.rayLength));
                // Only the center ray measures closing velocity: the +-25 deg rays would also catch
                // oncoming traffic passing in the other lane and trigger needless braking
                if (Math.abs(relAngle) < 0.2) {
                    const otherVelAlongRay =
                        Math.cos(hitBoid.heading) * hitBoid.speed * dx +
                        Math.sin(hitBoid.heading) * hitBoid.speed * dy;
                    const closing = (this.boid.speed - otherVelAlongRay) / Math.max(1, this.boid.maxSpeed);
                    if (closing > maxClosingSpeed) {
                        maxClosingSpeed = closing;
                    }
                }
            }
            this.boidReadings[i] = boidReading;

            const rayObj = this.rays[i];
            rayObj.ox = ox;
            rayObj.oy = oy;
            rayObj.endX = ox + dx * this.rayLength;
            rayObj.endY = oy + dy * this.rayLength;
            rayObj.roadHitX = ox + dx * roadDist;
            rayObj.roadHitY = oy + dy * roadDist;
            rayObj.roadHit = roadHit;
            rayObj.roadReading = roadReading;
            rayObj.boidHitX = ox + dx * closestBoidDist;
            rayObj.boidHitY = oy + dy * closestBoidDist;
            rayObj.boidHit = Boolean(hitBoid);
            rayObj.boidReading = boidReading;
        }

        this.closingSpeedReading = Math.max(-1, Math.min(1, maxClosingSpeed));
    }
}
