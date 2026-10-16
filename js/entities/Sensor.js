/**
 * Sensor - Front-mounted raycast sensor array for a Boid.
 * Detects:
 *   1. Side of the roads (curbs & intersection edges)
 *   2. Boids in front
 *   3. Stoplights (traffic lights at upcoming crossings)
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
        for (let i = 0; i < this.rayCount; i++) {
            const frac = this.rayCount === 1 ? 0.5 : i / (this.rayCount - 1);
            this.rayAngles.push((frac - 0.5) * this.spreadAngle);
        }

        // Readings in [0, 1] where 0 = clear/far, 1 = immediate proximity
        this.roadReadings = new Array(this.rayCount).fill(0);
        this.boidReadings = new Array(this.rayCount).fill(0);
        this.stoplightReading = 0;
        this.closingSpeedReading = 0;

        // Ray geometry for visualization
        this.rays = [];
    }

    /**
     * Updates all sensor rays against the city road boundaries, other boids, and stoplights.
     * @param {City} city
     * @param {Boid[]} boids
     * @param {object} roadState
     */
    update(city, boids, roadState) {
        const frontOffset = this.boid.length * 0.48;
        const ox = this.boid.x + Math.cos(this.boid.heading) * frontOffset;
        const oy = this.boid.y + Math.sin(this.boid.heading) * frontOffset;

        this.rays = [];
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

            // 1. Sense side of the road
            const roadHit = city.raycastRoadSide(ox, oy, dx, dy, this.rayLength);
            this.roadReadings[i] = roadHit.hit
                ? Math.max(0, Math.min(1, 1 - roadHit.distance / this.rayLength))
                : 0;

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

            if (hitBoid) {
                this.boidReadings[i] = Math.max(0, Math.min(1, 1 - closestBoidDist / this.rayLength));
                // If in central forward cone, compute relative closing velocity
                if (Math.abs(relAngle) < 0.45) {
                    const otherVelAlongRay =
                        Math.cos(hitBoid.heading) * hitBoid.speed * dx +
                        Math.sin(hitBoid.heading) * hitBoid.speed * dy;
                    const closing = (this.boid.speed - otherVelAlongRay) / Math.max(1, this.boid.maxSpeed);
                    if (closing > maxClosingSpeed) {
                        maxClosingSpeed = closing;
                    }
                }
            } else {
                this.boidReadings[i] = 0;
            }

            this.rays.push({
                ox,
                oy,
                endX: ox + dx * this.rayLength,
                endY: oy + dy * this.rayLength,
                roadHitX: roadHit.x,
                roadHitY: roadHit.y,
                roadHit: roadHit.hit,
                roadReading: this.roadReadings[i],
                boidHitX: ox + dx * closestBoidDist,
                boidHitY: oy + dy * closestBoidDist,
                boidHit: Boolean(hitBoid),
                boidReading: this.boidReadings[i]
            });
        }

        this.closingSpeedReading = Math.max(-1, Math.min(1, maxClosingSpeed));

        // 3. Sense stoplight ahead at approaching crossing
        this.stoplightReading = 0;
        if (roadState && roadState.trafficLight) {
            const signalVal = roadState.trafficLight.getSensorSignalValue();
            if (signalVal > 0 && roadState.distToStopLine <= this.rayLength && roadState.distToNode > roadState.approachingNode.radius - 2) {
                const proximity = 1 - roadState.distToStopLine / this.rayLength;
                this.stoplightReading = Math.max(0, Math.min(1, signalVal * (0.25 + 0.75 * proximity)));
            }
        }
    }
}
