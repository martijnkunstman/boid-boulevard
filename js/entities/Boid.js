/**
 * Boid - Autonomous vehicle agent controlled by a NeuralNetwork and front Sensors.
 * Supports 1 to 4 configurable vehicle archetypes, bidirectional throttle (forward drive & backward braking),
 * and non-collapsing collision physics on a unified single-track road network.
 */
class Boid {
    static ARCHETYPES = [
        {
            name: 'Sedan',
            color: '#059669',
            length: 15,
            width: 7.8,
            maxSpeedRange: [94, 110],
            accelRange: [76, 88],
            brakeRange: [115, 130],
            turnRateRange: [2.8, 3.2],
            mass: 1.0,
            sensorRange: 108
        },
        {
            name: 'Compact',
            color: '#2563eb',
            length: 13,
            width: 7,
            maxSpeedRange: [80, 95],
            accelRange: [70, 84],
            brakeRange: [118, 135],
            turnRateRange: [3.2, 3.7],
            mass: 0.85,
            sensorRange: 98
        },
        {
            name: 'Sports',
            color: '#dc2626',
            length: 15.5,
            width: 8,
            maxSpeedRange: [116, 138],
            accelRange: [95, 115],
            brakeRange: [125, 150],
            turnRateRange: [2.6, 3.0],
            mass: 0.95,
            sensorRange: 120
        },
        {
            name: 'Heavy',
            color: '#d97706',
            length: 18.5,
            width: 9,
            maxSpeedRange: [68, 82],
            accelRange: [54, 66],
            brakeRange: [95, 115],
            turnRateRange: [2.2, 2.5],
            mass: 1.65,
            sensorRange: 115
        }
    ];

    /**
     * @param {number} id
     * @param {object} pose - { x, y, heading, road }
     * @param {NeuralNetwork|null} brain
     * @param {object|null} properties - { archetype, allowedTypesCount, ... }
     */
    constructor(id, pose, brain = null, properties = null) {
        this.id = id;
        this.x = pose.x;
        this.y = pose.y;
        this.heading = pose.heading || 0;
        this.currentRoad = pose.road || null;
        this.travelSign = pose.travelSign || 1;

        const maxTypes = Math.max(1, Math.min(4, properties?.allowedTypesCount || 4));
        const activePool = Boid.ARCHETYPES.slice(0, maxTypes);

        let arch = null;
        if (properties?.archetype) {
            arch = activePool.find(a => a.name === properties.archetype) || null;
        }
        if (!arch) {
            arch = activePool[Math.floor(Random.next() * activePool.length)];
        }

        this.archetype = arch.name;
        this.color = arch.color;
        this.length = arch.length;
        this.width = arch.width;
        this.mass = arch.mass;

        this.maxSpeed = properties?.maxSpeed || Boid._randInRange(arch.maxSpeedRange);
        this.maxReverseSpeed = this.maxSpeed * 0.16;
        this.acceleration = properties?.acceleration || Boid._randInRange(arch.accelRange);
        this.brakingForce = properties?.brakingForce || Boid._randInRange(arch.brakeRange);
        this.turnRate = properties?.turnRate || Boid._randInRange(arch.turnRateRange);
        this.sensorRange = properties?.sensorRange || arch.sensorRange;

        this.speed = this.maxSpeed * 0.25;
        this.steerInput = 0;
        this.throttleInput = 0;

        // Navigation & intersection routing memory
        this.plannedNextRoad = null;
        this.lastIntersectionNode = null;

        // Front sensor array & streamlined [17, 10, 2] Neural Network
        this.sensor = new Sensor(this, 7, this.sensorRange);
        if (brain) {
            this.brain = brain.clone();
        } else {
            this.brain = new NeuralNetwork([17, 10, 2]);
            this.brain.applyInitialReflexBias(0.35 + Random.next() * 0.35);
        }

        // Training & telemetry metrics
        this.fitness = 0;
        this.distanceTraveled = 0;
        this.roadCollisions = 0;
        this.boidCollisions = 0;
        this.timeAlive = 0;
        this.stuckTimer = 0;
        this.collisionCooldown = 0;
        this.lastInputs = new Array(17).fill(0);
    }

    static _randInRange([min, max]) {
        return min + Random.next() * (max - min);
    }

    /**
     * Main step update for this Boid.
     * @param {number} dt
     * @param {City} city
     * @param {Boid[]} allBoids
     */
    update(dt, city, allBoids) {
        this.timeAlive += dt;
        if (this.collisionCooldown > 0) {
            this.collisionCooldown = Math.max(0, this.collisionCooldown - dt);
        }

        // 1. Query current road state and smoothly transition onto outgoing road at intersections
        let roadState = city.queryBoidRoadState(
            this.x,
            this.y,
            this.heading,
            this.currentRoad,
            this.travelSign
        );

        if (this._updateIntersectionRouting(roadState)) {
            roadState = city.queryBoidRoadState(
                this.x,
                this.y,
                this.heading,
                this.currentRoad,
                this.travelSign
            );
        }
        this.currentRoad = roadState.road;
        this.travelSign = roadState.travelSign;

        // 2. Update front raycast sensors (road borders and boids in front)
        this.sensor.update(city, allBoids);

        // 3. Populate 17-element neural network input vector in-place (zero allocation)
        const normSpeed = Math.max(-1, Math.min(1, this.speed / this.maxSpeed));
        const effectiveHeadingError = this._computeGuidedHeadingError(roadState);

        const inputs = this.lastInputs;
        for (let i = 0; i < 7; i++) {
            inputs[i] = this.sensor.roadReadings[i];
            inputs[i + 7] = this.sensor.boidReadings[i];
        }
        inputs[14] = this.sensor.closingSpeedReading;
        inputs[15] = normSpeed;
        inputs[16] = effectiveHeadingError;

        // 4. Run Neural Network forward pass -> [steering, throttle]
        const outputs = this.brain.forward(inputs);
        this.steerInput = Math.max(-1, Math.min(1, outputs[0]));
        this.throttleInput = Math.max(-1, Math.min(1, outputs[1]));

        // 5. Apply bidirectional throttle physics (forward = accelerate, backwards = brake / reverse)
        let longitudinalAccel = 0;
        if (this.throttleInput >= 0) {
            longitudinalAccel = this.throttleInput * this.acceleration;
        } else {
            if (this.speed > 0.5) {
                // Moving forward: negative throttle acts as braking
                longitudinalAccel = this.throttleInput * this.brakingForce;
            } else {
                // Near stop or reversing: gentle reverse throttle
                longitudinalAccel = this.throttleInput * (this.acceleration * 0.35);
            }
        }

        const rollingDrag = -this.speed * 0.28;
        this.speed += (longitudinalAccel + rollingDrag) * dt;

        // If braking for an obstacle directly in front, prevent unwanted reverse creep
        if (this.sensor.boidReadings[3] > 0.3) {
            if (this.speed < 0) this.speed = 0;
        } else {
            this.speed = Math.max(-this.maxReverseSpeed, Math.min(this.maxSpeed, this.speed));
        }
        this.speed = Math.min(this.maxSpeed, this.speed);

        // 6. Apply steering physics
        const speedFactor = Math.min(1, Math.abs(this.speed) / 16 + 0.3);
        const reverseSign = this.speed < -1 ? -1 : 1;
        this.heading = Vector2D.normalizeAngle(
            this.heading + this.steerInput * this.turnRate * speedFactor * reverseSign * dt
        );

        // 7. Integrate position
        const stepDist = this.speed * dt;
        const nextX = this.x + Math.cos(this.heading) * stepDist;
        const nextY = this.y + Math.sin(this.heading) * stepDist;

        // 8. Check road boundary & slide smoothly along road borders without getting stuck
        const stillOnRoad = city.isPointOnRoad(nextX, nextY);
        if (stillOnRoad) {
            this.x = nextX;
            this.y = nextY;
        } else {
            if (this.collisionCooldown <= 0) {
                this.roadCollisions++;
                this.fitness -= 5.0;
                this.collisionCooldown = 0.45;
            }
            this.speed *= 0.72;

            // Rotate heading smoothly toward the forward lookahead point along the active road
            const angleDiff = Vector2D.normalizeAngle(roadState.desiredAngle - this.heading);
            this.heading = Vector2D.normalizeAngle(this.heading + angleDiff * 0.55);

            // Slide smoothly toward the forward roadCenter lookahead so boids never get stuck on corners
            const slideDist = Math.max(2.0, Math.abs(stepDist) * 0.7);
            const pullToCenter = new Vector2D(
                roadState.roadCenter.x - this.x,
                roadState.roadCenter.y - this.y
            ).normalize();
            const candX = this.x + pullToCenter.x * slideDist;
            const candY = this.y + pullToCenter.y * slideDist;
            if (city.isPointOnRoad(candX, candY)) {
                this.x = candX;
                this.y = candY;
            } else {
                this.x = roadState.roadCenter.x;
                this.y = roadState.roadCenter.y;
            }
        }

        // 9. Fitness & anti-stall evaluation
        this._evaluateFitness(dt, stepDist, roadState, city, allBoids);
    }

    _pickOutgoingRoad(node, currentRoad) {
        const candidates = node.getConnectedRoadsExcept(currentRoad);
        if (candidates.length === 0) return currentRoad;

        const smoothCandidates = candidates.filter(r => {
            const other = r.getOtherNode(node);
            const outAngle = Math.atan2(other.y - node.y, other.x - node.x);
            const turnAngle = Math.abs(Vector2D.normalizeAngle(outAngle - this.heading));
            return turnAngle < 2.25;
        });
        const pool = smoothCandidates.length > 0 ? smoothCandidates : candidates;
        return pool[Math.floor(Random.next() * pool.length)];
    }

    _updateIntersectionRouting(roadState) {
        const node = roadState.approachingNode;
        if (!node) return false;

        // 1. Choose a planned outgoing road when approaching an intersection
        if (roadState.distToNode < node.radius + 42 && !this.plannedNextRoad && this.lastIntersectionNode !== node) {
            this.plannedNextRoad = this._pickOutgoingRoad(node, roadState.road);
        }

        // 2. Once entering the intersection turn zone, immediately switch currentRoad & travelSign
        // onto the outgoing road so the boid never drives into the dead end of the incoming road.
        if (roadState.distToNode <= node.radius + 10) {
            const nextRoad = this.plannedNextRoad || this._pickOutgoingRoad(node, roadState.road);
            if (nextRoad && nextRoad !== roadState.road) {
                this.currentRoad = nextRoad;
                this.travelSign = nextRoad.nodeA === node ? 1 : -1;
                this.lastIntersectionNode = node;
                this.plannedNextRoad = null;
                return true;
            }
        }
        return false;
    }

    _computeGuidedHeadingError(roadState) {
        if (this.plannedNextRoad && roadState.distToNode < roadState.approachingNode.radius + 34) {
            const node = roadState.approachingNode;
            const nextRoad = this.plannedNextRoad;
            const sign = nextRoad.nodeA === node ? 1 : -1;
            const targetPoint = nextRoad.getPointAt(sign === 1 ? 0.28 : 0.72);
            const desiredAngle = Math.atan2(targetPoint.y - this.y, targetPoint.x - this.x);
            const angleDiff = Vector2D.normalizeAngle(this.heading - desiredAngle);
            return Math.max(-1, Math.min(1, angleDiff / (Math.PI * 0.4)));
        }
        return roadState.headingError;
    }

    _evaluateFitness(dt, stepDist, roadState, city, allBoids) {
        const forwardCos = Math.cos(roadState.angleDiff);
        if (this.speed > 2 && forwardCos > 0.2) {
            const validDist = stepDist * forwardCos;
            this.distanceTraveled += validDist;
            this.fitness += validDist * 0.25;
        } else if (this.speed < -1) {
            this.fitness -= Math.abs(stepDist) * 0.1;
        }

        // Anti-collapse / anti-stall check: if stuck, respawn cleanly
        if (Math.abs(this.speed) < 6) {
            this.stuckTimer += dt;
            this.fitness -= 1.2 * dt;
            if (this.stuckTimer > 3.6) {
                const newPose = city.getRandomSpawnPose(allBoids);
                this.x = newPose.x;
                this.y = newPose.y;
                this.heading = newPose.heading;
                this.currentRoad = newPose.road;
                this.travelSign = newPose.travelSign || 1;
                this.speed = this.maxSpeed * 0.35;
                this.stuckTimer = 0;
            }
        } else {
            this.stuckTimer = Math.max(0, this.stuckTimer - dt * 0.5);
        }
    }

    /**
     * Resolves pairwise boid-to-boid physical separation so boids NEVER collapse/stack onto each other
     * and never get pushed outside the road boundaries.
     * @param {Boid[]} boids
     * @param {City|null} city
     */
    static resolveBoidCollisions(boids, city = null) {
        const n = boids.length;
        for (let i = 0; i < n; i++) {
            const a = boids[i];
            for (let j = i + 1; j < n; j++) {
                const b = boids[j];

                const dx = b.x - a.x;
                const dy = b.y - a.y;
                const minDist = (a.length + b.length) * 0.42;
                const distSq = dx * dx + dy * dy;

                if (distSq < minDist * minDist && distSq > 1e-6) {
                    const dist = Math.sqrt(distSq);
                    const overlap = (minDist - dist) + 0.5;
                    const nx = dx / dist;
                    const ny = dy / dist;

                    const totalMass = a.mass + b.mass;
                    const ratioA = b.mass / totalMass;
                    const ratioB = a.mass / totalMass;

                    // Push apart so they never collapse into one another, while staying on road
                    const nextAX = a.x - nx * overlap * ratioA;
                    const nextAY = a.y - ny * overlap * ratioA;
                    const nextBX = b.x + nx * overlap * ratioB;
                    const nextBY = b.y + ny * overlap * ratioB;

                    if (!city || city.isPointOnRoad(nextAX, nextAY)) {
                        a.x = nextAX;
                        a.y = nextAY;
                    }
                    if (!city || city.isPointOnRoad(nextBX, nextBY)) {
                        b.x = nextBX;
                        b.y = nextBY;
                    }

                    // Determine who rear-ended whom and damp velocity
                    const aForwardDot = Math.cos(a.heading) * nx + Math.sin(a.heading) * ny;
                    const bForwardDot = -(Math.cos(b.heading) * nx + Math.sin(b.heading) * ny);

                    if (aForwardDot > bForwardDot) {
                        a.speed = Math.min(a.speed * 0.5, b.speed * 0.85);
                        if (a.collisionCooldown <= 0) {
                            a.boidCollisions++;
                            a.fitness -= 4.5;
                            a.collisionCooldown = 0.45;
                        }
                    } else {
                        b.speed = Math.min(b.speed * 0.5, a.speed * 0.85);
                        if (b.collisionCooldown <= 0) {
                            b.boidCollisions++;
                            b.fitness -= 4.5;
                            b.collisionCooldown = 0.45;
                        }
                    }
                }
            }
        }
    }
}
