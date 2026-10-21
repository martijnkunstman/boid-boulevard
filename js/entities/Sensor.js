/**
 * Sensor - Front-mounted raycast sensor array for a Boid.
 * Detects:
 *   1. Road borders (curbs & intersection edges via SDF sphere tracing)
 *   2. Boids in front (via oriented elliptical ray intersection matching physical vehicle dimensions)
 *   3. Interactive roadblock obstacles placed on the road
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

        this.rayAngles = new Float32Array(this.rayCount);
        this.rayCos = new Float32Array(this.rayCount);
        this.raySin = new Float32Array(this.rayCount);

        // Pre-allocate ray objects for zero-GC updates during high-speed training
        this.rays = [];
        for (let i = 0; i < this.rayCount; i++) {
            const frac = this.rayCount === 1 ? 0.5 : i / (this.rayCount - 1);
            const angle = (frac - 0.5) * this.spreadAngle;
            this.rayAngles[i] = angle;
            this.rayCos[i] = Math.cos(angle);
            this.raySin[i] = Math.sin(angle);
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
        this.roadReadings = new Float32Array(this.rayCount);
        this.boidReadings = new Float32Array(this.rayCount);
        this.closingSpeedReading = 0;
        this._nearby = [];
    }

    /**
     * Updates all sensor rays against the city road boundaries, other boids, and roadblock obstacles.
     * @param {City} city
     * @param {Boid[]} boids
     * @param {SpatialGrid|null} grid - Grid built from `boids`; without it all boids are scanned
     */
    update(city, boids, grid = null) {
        const boid = this.boid;
        const cosH = boid.cosH;
        const sinH = boid.sinH;
        const frontOffset = boid.length * 0.48;
        const ox = boid.x + cosH * frontOffset;
        const oy = boid.y + sinH * frontOffset;

        let maxClosingSpeed = 0;
        const rayLength = this.rayLength;
        const invRayLength = 1 / rayLength;

        // Pre-filter nearby boids (reused array, no per-step allocation)
        const maxDistWithRadius = rayLength + 20;
        const nearbyBoids = this._nearby;

        if (grid) {
            grid.queryRadius(ox, oy, maxDistWithRadius, boid, nearbyBoids);
        } else {
            const maxDistSq = maxDistWithRadius * maxDistWithRadius;
            nearbyBoids.length = 0;
            for (let i = 0; i < boids.length; i++) {
                const other = boids[i];
                if (other === boid) continue;

                const dx = other.x - ox;
                const dy = other.y - oy;
                if (dx * dx + dy * dy <= maxDistSq) {
                    nearbyBoids.push(other);
                }
            }
        }

        const obstacles = city.obstacles;
        const hasObstacles = obstacles && obstacles.length > 0;
        const nNearby = nearbyBoids.length;

        for (let i = 0; i < this.rayCount; i++) {
            const relAngle = this.rayAngles[i];
            const rCos = this.rayCos[i];
            const rSin = this.raySin[i];
            // Angle addition: cos(heading + relAngle), sin(heading + relAngle)
            const dx = cosH * rCos - sinH * rSin;
            const dy = sinH * rCos + cosH * rSin;

            // 1. Sense road border
            const roadDist = city.raycastRoadSide(ox, oy, dx, dy, rayLength);
            const roadHit = roadDist < rayLength;
            const roadReading = roadHit
                ? Math.max(0, Math.min(1, 1 - roadDist * invRayLength))
                : 0;
            this.roadReadings[i] = roadReading;

            // 2. Sense boids in front along this ray using oriented elliptical bounds
            let closestBoidDist = rayLength;
            let hitBoid = null;
            let hitObstacle = false;

            for (let j = 0; j < nNearby; j++) {
                const other = nearbyBoids[j];
                const rx = other.collisionLength + 1.2;
                const ry = other.collisionWidth + 0.9;
                const dist = Vector2D.rayEllipseIntersection(
                    ox, oy, dx, dy,
                    other.x, other.y,
                    other.cosH, other.sinH,
                    rx, ry,
                    closestBoidDist
                );
                // Equal distances go to the lowest id, so results do not depend on candidate order
                if (dist !== null && (dist < closestBoidDist ||
                    (dist === closestBoidDist && hitBoid && other.id < hitBoid.id))) {
                    closestBoidDist = dist;
                    hitBoid = other;
                }
            }

            // 2b. Sense interactive roadblock obstacles if present
            if (hasObstacles) {
                for (let k = 0; k < obstacles.length; k++) {
                    const obs = obstacles[k];
                    const dist = Vector2D.rayCircleIntersection(
                        ox, oy, dx, dy,
                        obs.x, obs.y,
                        obs.radius + 2,
                        closestBoidDist
                    );
                    if (dist !== null && dist < closestBoidDist) {
                        closestBoidDist = dist;
                        hitBoid = null;
                        hitObstacle = true;
                    }
                }
            }

            let boidReading = 0;
            if (hitBoid || hitObstacle) {
                boidReading = Math.max(0, Math.min(1, 1 - closestBoidDist * invRayLength));
                // Only the center ray measures closing velocity: filter out oncoming traffic in the
                // opposite lane (sameDir < -0.25) so oncoming cars on curves don't trigger phantom braking
                if (Math.abs(relAngle) < 0.2) {
                    if (hitBoid) {
                        const sameDir = cosH * hitBoid.cosH + sinH * hitBoid.sinH;
                        if (sameDir > -0.25) {
                            const otherVelAlongRay =
                                hitBoid.cosH * hitBoid.speed * dx +
                                hitBoid.sinH * hitBoid.speed * dy;
                            const closing = (boid.speed - otherVelAlongRay) / Math.max(1, boid.maxSpeed);
                            if (closing > maxClosingSpeed) {
                                maxClosingSpeed = closing;
                            }
                        }
                    } else if (hitObstacle) {
                        const closing = boid.speed / Math.max(1, boid.maxSpeed);
                        if (closing > maxClosingSpeed) {
                            maxClosingSpeed = closing;
                        }
                    }
                }
            }
            this.boidReadings[i] = boidReading;

            const rayObj = this.rays[i];
            rayObj.ox = ox;
            rayObj.oy = oy;
            rayObj.endX = ox + dx * rayLength;
            rayObj.endY = oy + dy * rayLength;
            rayObj.roadHitX = ox + dx * roadDist;
            rayObj.roadHitY = oy + dy * roadDist;
            rayObj.roadHit = roadHit;
            rayObj.roadReading = roadReading;
            rayObj.boidHitX = ox + dx * closestBoidDist;
            rayObj.boidHitY = oy + dy * closestBoidDist;
            rayObj.boidHit = Boolean(hitBoid || hitObstacle);
            rayObj.boidReading = boidReading;
        }

        this.closingSpeedReading = Math.max(-1, Math.min(1, maxClosingSpeed));
    }
}
