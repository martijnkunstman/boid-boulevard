/**
 * Sensor - Front-mounted raycast sensor array for a Boid.
 * Detects:
 *   1. Road borders (curbs & intersection edges)
 *   2. Boids in front, and the stop line when the boid must give right of way (on the same boid rays)
 */
class Sensor {
    // Virtual obstacle used for stop lines: stationary, sensed on the boid rays like a parked boid
    static STOP_LINE_RADIUS = 11;
    static STOP_LINE_OBSTACLE = { heading: 0, speed: 0 };

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
    }

    /**
     * Updates all sensor rays against the city road boundaries and other boids.
     * @param {City} city
     * @param {Boid[]} boids
     */
    update(city, boids) {
        const frontOffset = this.boid.length * 0.48;
        const ox = this.boid.x + Math.cos(this.boid.heading) * frontOffset;
        const oy = this.boid.y + Math.sin(this.boid.heading) * frontOffset;

        let maxClosingSpeed = 0;

        // Pre-filter nearby boids
        const maxDistWithRadius = this.rayLength + 20;
        const maxDistSq = maxDistWithRadius * maxDistWithRadius;
        const nearbyBoids = [];

        for (let i = 0; i < boids.length; i++) {
            const other = boids[i];
            if (other === this.boid) continue;

            const dx = other.x - ox;
            const dy = other.y - oy;
            if (dx * dx + dy * dy <= maxDistSq) {
                nearbyBoids.push(other);
            }
        }

        for (let i = 0; i < this.rayCount; i++) {
            const relAngle = this.rayAngles[i];
            const rayAngle = this.boid.heading + relAngle;
            const dx = Math.cos(rayAngle);
            const dy = Math.sin(rayAngle);

            // 1. Sense road border
            const roadHit = city.raycastRoadSide(ox, oy, dx, dy, this.rayLength);
            const roadReading = roadHit.hit
                ? Math.max(0, Math.min(1, 1 - roadHit.distance / this.rayLength))
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
                if (dist !== null && dist < closestBoidDist) {
                    closestBoidDist = dist;
                    hitBoid = other;
                }
            }

            // 3. A stop line the boid must yield at is sensed as a stationary obstacle
            if (this.boid.mustYield) {
                const yp = this.boid.yieldPoint;
                const dist = Vector2D.rayCircleIntersection(
                    ox, oy, dx, dy,
                    yp.x, yp.y,
                    Sensor.STOP_LINE_RADIUS,
                    closestBoidDist
                );
                if (dist !== null && dist < closestBoidDist) {
                    closestBoidDist = dist;
                    hitBoid = Sensor.STOP_LINE_OBSTACLE;
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
            rayObj.roadHitX = roadHit.x;
            rayObj.roadHitY = roadHit.y;
            rayObj.roadHit = roadHit.hit;
            rayObj.roadReading = roadReading;
            rayObj.boidHitX = ox + dx * closestBoidDist;
            rayObj.boidHitY = oy + dy * closestBoidDist;
            rayObj.boidHit = Boolean(hitBoid);
            rayObj.boidReading = boidReading;
        }

        this.closingSpeedReading = Math.max(-1, Math.min(1, maxClosingSpeed));
    }
}
