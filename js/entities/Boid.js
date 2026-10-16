/**
 * Boid - Autonomous vehicle agent controlled by a NeuralNetwork and front Sensors.
 * Features varied speeds/archetype properties, bidirectional throttle (forward drive & backward braking),
 * elevation/slope physics, stoplight compliance, and non-collapsing collision physics.
 */
class Boid {
    static ARCHETYPES = [
        {
            name: 'Compact',
            color: '#2563eb',
            length: 13,
            width: 7,
            maxSpeedRange: [78, 96],
            accelRange: [68, 84],
            brakeRange: [115, 135],
            turnRateRange: [3.2, 3.8],
            mass: 0.85,
            sensorRange: 98
        },
        {
            name: 'Sedan',
            color: '#059669',
            length: 15,
            width: 7.8,
            maxSpeedRange: [92, 114],
            accelRange: [72, 90],
            brakeRange: [110, 130],
            turnRateRange: [2.7, 3.2],
            mass: 1.0,
            sensorRange: 108
        },
        {
            name: 'Sports',
            color: '#dc2626',
            length: 15.5,
            width: 8,
            maxSpeedRange: [116, 142],
            accelRange: [95, 118],
            brakeRange: [125, 150],
            turnRateRange: [2.5, 3.0],
            mass: 0.95,
            sensorRange: 120
        },
        {
            name: 'Heavy',
            color: '#d97706',
            length: 18.5,
            width: 9,
            maxSpeedRange: [66, 82],
            accelRange: [52, 66],
            brakeRange: [95, 115],
            turnRateRange: [2.1, 2.5],
            mass: 1.65,
            sensorRange: 115
        }
    ];

    /**
     * @param {number} id
     * @param {object} pose - { x, y, heading, road, layer, elevation }
     * @param {NeuralNetwork|null} brain
     * @param {object|null} properties
     */
    constructor(id, pose, brain = null, properties = null) {
        this.id = id;
        this.x = pose.x;
        this.y = pose.y;
        this.heading = pose.heading || 0;
        this.currentRoad = pose.road || null;
        this.layer = pose.layer || 0;
        this.elevation = pose.elevation || 0;

        // Assign vehicle archetype and varied physical properties
        const arch = properties?.archetype
            ? Boid.ARCHETYPES.find(a => a.name === properties.archetype) || Boid.ARCHETYPES[1]
            : Boid.ARCHETYPES[Math.floor(Math.random() * Boid.ARCHETYPES.length)];

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
        this.currentSlope = 0;

        // Navigation & intersection routing memory
        this.plannedNextRoad = null;
        this.lastIntersectionNode = null;

        // Front sensor array & Neural Network
        this.sensor = new Sensor(this, 7, this.sensorRange);
        if (brain) {
            this.brain = brain.clone();
        } else {
            this.brain = new NeuralNetwork([19, 14, 8, 2]);
            this.brain.applyInitialReflexBias(0.35 + Math.random() * 0.35);
        }

        // Training & telemetry metrics
        this.fitness = 0;
        this.distanceTraveled = 0;
        this.roadCollisions = 0;
        this.boidCollisions = 0;
        this.redLightViolations = 0;
        this.redLightStops = 0;
        this.timeAlive = 0;
        this.stuckTimer = 0;
        this.collisionCooldown = 0;
        this.lastInputs = new Array(19).fill(0);
    }

    static _randInRange([min, max]) {
        return min + Math.random() * (max - min);
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

        // 1. Query current road, lane, elevation, slope, and stoplight state
        const roadState = city.queryBoidRoadState(
            this.x,
            this.y,
            this.heading,
            this.currentRoad,
            this.layer
        );

        // Manage intersection routing so boids smoothly transition onto outgoing roads at crossings
        this._updateIntersectionRouting(roadState);

        this.currentRoad = roadState.road;
        this.elevation = roadState.elevation;
        this.layer = roadState.layer;
        this.currentSlope = roadState.slope;

        // 2. Update front raycast sensors (road sides, boids in front, stoplight ahead)
        this.sensor.update(city, allBoids, roadState);

        // 3. Build 19-element neural network input vector
        const normSpeed = Math.max(-1, Math.min(1, this.speed / this.maxSpeed));
        const effectiveLaneError = this._computeGuidedLaneError(roadState);

        const inputs = [
            ...this.sensor.roadReadings,     // 0..6: 7 road-side sensor rays
            ...this.sensor.boidReadings,     // 7..13: 7 boids-in-front sensor rays
            this.sensor.stoplightReading,    // 14: stoplight signal & distance ahead
            this.sensor.closingSpeedReading, // 15: closing velocity with boid in front
            normSpeed,                       // 16: current normalized speed
            this.currentSlope,               // 17: road elevation grade (uphill/downhill)
            effectiveLaneError               // 18: lane / turn alignment error
        ];
        this.lastInputs = inputs;

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
                // Near stop or reversing: gentle reverse throttle or hold brake
                longitudinalAccel = this.throttleInput * (this.acceleration * 0.35);
            }
        }

        // Apply elevation slope gravity effect (uphill slows boids down, downhill speeds them up)
        const gravityGradeForce = -this.currentSlope * 26.0 * this.mass;
        const rollingDrag = -this.speed * 0.28;

        this.speed += (longitudinalAccel + gravityGradeForce + rollingDrag) * dt;

        // If braking before a red light or obstacle in front, prevent unwanted reverse creep
        if (this.sensor.stoplightReading > 0.3 || this.sensor.boidReadings[3] > 0.3) {
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

        // 8. Check road boundary & keep boids inside the road network ("do not collapse")
        const stillOnRoad = city.isPointOnRoad(nextX, nextY, this.currentRoad, this.layer);
        if (stillOnRoad) {
            this.x = nextX;
            this.y = nextY;
        } else {
            // Hit the side of the road (curb or bridge guardrail)
            if (this.collisionCooldown <= 0) {
                this.roadCollisions++;
                this.fitness -= 5.0;
                this.collisionCooldown = 0.45;
            }
            this.speed *= 0.65;

            // Deflect boid gently back toward the right-hand lane center and align heading
            const pullToLane = new Vector2D(
                roadState.laneCenter.x - this.x,
                roadState.laneCenter.y - this.y
            ).normalize();
            this.x += pullToLane.x * 2.8;
            this.y += pullToLane.y * 2.8;

            const angleDiff = Vector2D.normalizeAngle(roadState.desiredAngle - this.heading);
            this.heading = Vector2D.normalizeAngle(this.heading + angleDiff * 0.48);
        }

        // 9. Stoplight compliance & fitness evaluation
        this._evaluateFitnessAndTrafficRules(dt, stepDist, roadState, city, allBoids);
    }

    _updateIntersectionRouting(roadState) {
        const node = roadState.approachingNode;
        if (!node) return;

        // Choose a planned outgoing road when approaching an intersection
        if (roadState.distToNode < node.radius + 38 && this.lastIntersectionNode !== node) {
            const candidates = node.getConnectedRoadsExcept(roadState.road);
            if (candidates.length > 0) {
                // Prefer roads that don't require a >130 degree hairpin turn unless necessary
                const smoothCandidates = candidates.filter(r => {
                    const other = r.getOtherNode(node);
                    const outAngle = Math.atan2(other.y - node.y, other.x - node.x);
                    const turnAngle = Math.abs(Vector2D.normalizeAngle(outAngle - this.heading));
                    return turnAngle < 2.25;
                });
                const pool = smoothCandidates.length > 0 ? smoothCandidates : candidates;
                this.plannedNextRoad = pool[Math.floor(Math.random() * pool.length)];
                this.lastIntersectionNode = node;
            }
        }

        // Once entering the intersection node radius, transition active road to plannedNextRoad
        if (this.plannedNextRoad && roadState.distToNode <= node.radius + 4) {
            this.currentRoad = this.plannedNextRoad;
            roadState.road = this.plannedNextRoad;
            this.plannedNextRoad = null;
        }
    }

    _computeGuidedLaneError(roadState) {
        if (this.plannedNextRoad && roadState.distToNode < roadState.approachingNode.radius + 28) {
            const node = roadState.approachingNode;
            const nextRoad = this.plannedNextRoad;
            const sign = nextRoad.nodeA === node ? 1 : -1;
            const targetPoint = nextRoad.getLanePointAt(sign === 1 ? 0.3 : 0.7, sign);
            const desiredAngle = Math.atan2(targetPoint.y - this.y, targetPoint.x - this.x);
            const angleDiff = Vector2D.normalizeAngle(this.heading - desiredAngle);
            return Math.max(-1, Math.min(1, angleDiff / (Math.PI * 0.4)));
        }
        return roadState.laneError;
    }

    _evaluateFitnessAndTrafficRules(dt, stepDist, roadState, city, allBoids) {
        const forwardCos = Math.cos(roadState.angleDiff);
        if (this.speed > 2 && forwardCos > 0.2) {
            const validDist = stepDist * forwardCos;
            this.distanceTraveled += validDist;

            // Reward forward progress + staying in the right-hand lane
            const laneBonus = Math.max(0.25, 1 - Math.abs(roadState.lateralOffset) / (roadState.road.halfWidth * 0.65));
            this.fitness += validDist * 0.25 * laneBonus;
        } else if (this.speed < -1) {
            this.fitness -= Math.abs(stepDist) * 0.1;
        }

        // Check stoplight compliance
        const light = roadState.trafficLight;
        const waitingAtRed = light && light.state === 'RED' && roadState.distToStopLine < 26;

        if (waitingAtRed && Math.abs(this.speed) < 10) {
            // Reward stopping at a red light if the boid is an active driver
            this.redLightStops += dt;
            if (this.distanceTraveled > 25 && this.redLightStops < 6.0) {
                this.fitness += 1.5 * dt;
            }
            this.stuckTimer = 0;
        } else if (
            light &&
            light.state === 'RED' &&
            roadState.distToStopLine < 5 &&
            roadState.distToNode > roadState.approachingNode.radius - 4 &&
            this.speed > 25
        ) {
            if (this.collisionCooldown <= 0) {
                this.redLightViolations++;
                this.fitness -= 8.0;
                this.collisionCooldown = 0.5;
            }
        }

        // Anti-collapse / anti-stall check: if stuck and NOT waiting at a red light, respawn cleanly
        if (Math.abs(this.speed) < 6 && !waitingAtRed) {
            this.stuckTimer += dt;
            this.fitness -= 1.2 * dt;
            if (this.stuckTimer > 3.6) {
                const newPose = city.getRandomSpawnPose(allBoids);
                this.x = newPose.x;
                this.y = newPose.y;
                this.heading = newPose.heading;
                this.currentRoad = newPose.road;
                this.layer = newPose.layer;
                this.elevation = newPose.elevation;
                this.speed = this.maxSpeed * 0.35;
                this.stuckTimer = 0;
            }
        } else {
            this.stuckTimer = Math.max(0, this.stuckTimer - dt * 0.5);
        }
    }

    /**
     * Resolves pairwise boid-to-boid physical separation on the same height level
     * so boids NEVER collapse/stack onto each other.
     * @param {Boid[]} boids
     */
    static resolveBoidCollisions(boids) {
        const n = boids.length;
        for (let i = 0; i < n; i++) {
            const a = boids[i];
            for (let j = i + 1; j < n; j++) {
                const b = boids[j];

                // Boids on a bridge vs ground road underneath do not collide
                if (Math.abs(a.elevation - b.elevation) > 3.5) continue;

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

                    // Push apart so they never collapse into one another
                    a.x -= nx * overlap * ratioA;
                    a.y -= ny * overlap * ratioA;
                    b.x += nx * overlap * ratioB;
                    b.y += ny * overlap * ratioB;

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
