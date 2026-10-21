/**
 * Boid - Autonomous vehicle agent controlled by a NeuralNetwork and front Sensors.
 * Supports 1 to 4 configurable vehicle archetypes, bidirectional throttle (forward drive & backward braking),
 * and non-collapsing collision physics on a two-way, keep-right road network.
 */
class Boid {
    // Boids sense and decide every this many physics steps (2 = 30 Hz at the 60 Hz physics rate);
    // steering and throttle are held in between, which roughly halves sensor and network work
    static DECISION_INTERVAL = 2;

    // Ghost trail: a position is recorded every TRAIL_INTERVAL physics steps, keeping the last
    // TRAIL_LENGTH positions (20 x 4 steps = ~1.3 s of driving at 60 Hz)
    static TRAIL_LENGTH = 20;
    static TRAIL_INTERVAL = 4;

    // After a stuck boid is respawned, a ghost dot marks where it was for this many simulated seconds
    static RESPAWN_GHOST_DURATION = 2.0;

    // Reference Sedan speed used to normalize multi-archetype distance fitness rewards
    static REFERENCE_MAX_SPEED = 102;

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
     * @param {object} pose - { x, y, heading, road, travelSign }
     * @param {NeuralNetwork|null} brain
     * @param {object|null} properties - { archetype, allowedTypesCount, ... }
     */
    constructor(id, pose, brain = null, properties = null) {
        this.id = id;
        this.x = pose.x;
        this.y = pose.y;
        this.heading = 0;
        this.cosH = 1;
        this.sinH = 0;
        this._setHeading(pose.heading || 0);

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

        // Elliptical collision shape: semi-axis along the heading and across it
        this.collisionLength = this.length * 0.42;
        this.collisionWidth = this.width * 0.5 + 1;

        this.maxSpeed = properties?.maxSpeed || Boid._randInRange(arch.maxSpeedRange);
        this.maxReverseSpeed = this.maxSpeed * 0.16;
        this.acceleration = properties?.acceleration || Boid._randInRange(arch.accelRange);
        this.brakingForce = properties?.brakingForce || Boid._randInRange(arch.brakeRange);
        this.turnRate = properties?.turnRate || Boid._randInRange(arch.turnRateRange);
        this.sensorRange = properties?.sensorRange || arch.sensorRange;

        this.speed = this.maxSpeed * 0.25;
        this.steerInput = 0;
        this.throttleInput = 0;
        // Steps until the next sense/decide update (see DECISION_INTERVAL); staggered by id
        this._decisionCounter = id % Boid.DECISION_INTERVAL;

        // Navigation & intersection routing memory
        this.plannedNextRoad = null;
        this.lastIntersectionNode = null;

        // Pre-allocated road-state and SDF normal scratch buffers (zero per-step heap allocations)
        this._roadState = {
            onRoad: true,
            road: null,
            t: 0,
            distToCenterline: 0,
            lateralOffset: 0,
            laneError: 0,
            inCrossing: false,
            travelSign: 1,
            travelDir: new Vector2D(),
            desiredAngle: 0,
            laneTarget: new Vector2D(),
            angleDiff: 0,
            headingError: 0,
            approachingNode: null,
            distToNode: 0
        };
        this._normalScratch = { x: 0, y: 0, dist: 0 };
        this._lanePointScratch = { x: 0, y: 0 };

        // Front sensor array & Neural Network (layout: NeuralNetwork.DEFAULT_LAYOUT, 17 inputs -> 2 outputs)
        this.sensor = new Sensor(this, 7, this.sensorRange);
        if (brain) {
            this.brain = brain.clone();
        } else {
            this.brain = new NeuralNetwork(NeuralNetwork.DEFAULT_LAYOUT);
            this.brain.applyInitialReflexBias(0.35 + Random.next() * 0.35);
        }

        // Training & telemetry metrics
        this.fitness = 0;
        this.distanceTraveled = 0;
        this.roadCollisions = 0;
        this.boidCollisions = 0;
        this.headOnCollisions = 0;
        this.wrongLaneTime = 0;
        this.timeAlive = 0;
        this.stuckTimer = 0;
        this.collisionCooldown = 0;
        this.lastInputs = new Float32Array(17);

        // Ghost trail ring buffer of recent positions (x0, y0, x1, y1, ...), oldest overwritten first
        this.trail = new Float32Array(Boid.TRAIL_LENGTH * 2);

        // Last respawn (for the ghost dot): old and new position, and simulated seconds since it happened
        this.respawnFromX = 0;
        this.respawnFromY = 0;
        this.respawnToX = 0;
        this.respawnToY = 0;
        this.respawnAge = Infinity;
        this.trailCount = 0;
        this.trailHead = 0;
        this._trailTick = 0;
    }

    _setHeading(angle) {
        this.heading = angle;
        this.cosH = Math.cos(angle);
        this.sinH = Math.sin(angle);
    }

    /**
     * Records the current position in the ghost trail every TRAIL_INTERVAL steps.
     */
    _recordTrail() {
        if (++this._trailTick < Boid.TRAIL_INTERVAL) return;
        this._trailTick = 0;
        this.trail[this.trailHead * 2] = this.x;
        this.trail[this.trailHead * 2 + 1] = this.y;
        this.trailHead = (this.trailHead + 1) % Boid.TRAIL_LENGTH;
        if (this.trailCount < Boid.TRAIL_LENGTH) this.trailCount++;
    }

    /**
     * Returns the i-th trail point, oldest first (0 .. trailCount - 1), as [x, y] into `out`.
     */
    getTrailPoint(i, out) {
        const index = (this.trailHead - this.trailCount + i + Boid.TRAIL_LENGTH) % Boid.TRAIL_LENGTH;
        out[0] = this.trail[index * 2];
        out[1] = this.trail[index * 2 + 1];
        return out;
    }

    clearTrail() {
        this.trailCount = 0;
        this.trailHead = 0;
        this._trailTick = 0;
    }

    static _randInRange([min, max]) {
        return min + Random.next() * (max - min);
    }

    /**
     * Main step update for this Boid.
     * @param {number} dt
     * @param {City} city
     * @param {Boid[]} allBoids
     * @param {SpatialGrid|null} grid
     */
    update(dt, city, allBoids, grid = null) {
        this.timeAlive += dt;
        this.respawnAge += dt;
        if (this.collisionCooldown > 0) {
            this.collisionCooldown = Math.max(0, this.collisionCooldown - dt);
        }

        // 1. Query current road state in-place (zero allocation) and transition at intersections
        let roadState = city.queryBoidRoadState(
            this.x,
            this.y,
            this.heading,
            this.currentRoad,
            this.travelSign,
            this._roadState,
            this.cosH,
            this.sinH
        );

        if (this._updateIntersectionRouting(roadState)) {
            roadState = city.queryBoidRoadState(
                this.x,
                this.y,
                this.heading,
                this.currentRoad,
                this.travelSign,
                this._roadState,
                this.cosH,
                this.sinH
            );
        }
        this.currentRoad = roadState.road;
        this.travelSign = roadState.travelSign;

        // 2-4. Sense and decide only every DECISION_INTERVAL steps (staggered by id, so each step only
        // half of the boids think). In between, the last steering/throttle outputs are held while
        // physics, routing and fitness keep running every step.
        const think = this._decisionCounter === 0;
        this._decisionCounter = (this._decisionCounter + 1) % Boid.DECISION_INTERVAL;

        if (think) {
            // 2. Update front raycast sensors (road borders, boids in front, obstacles)
            this.sensor.update(city, allBoids, grid);

            // 3. Populate 17-element neural network input vector in-place (zero allocation)
            const normSpeed = Math.max(-1, Math.min(1, this.speed / this.maxSpeed));
            const effectiveHeadingError = this._computeGuidedHeadingError(roadState);

            const inputs = this.lastInputs;
            const rReadings = this.sensor.roadReadings;
            const bReadings = this.sensor.boidReadings;
            for (let i = 0; i < 7; i++) {
                inputs[i] = rReadings[i];
                inputs[i + 7] = bReadings[i];
            }
            inputs[14] = this.sensor.closingSpeedReading;
            inputs[15] = normSpeed;
            inputs[16] = effectiveHeadingError;

            // 4. Run Neural Network forward pass -> [steering, throttle]
            const outputs = this.brain.forward(inputs);
            this.steerInput = Math.max(-1, Math.min(1, outputs[0]));
            this.throttleInput = Math.max(-1, Math.min(1, outputs[1]));
        }

        // 5. Apply bidirectional throttle physics (forward = accelerate, backwards = brake / reverse)
        let longitudinalAccel = 0;
        if (this.throttleInput >= 0) {
            longitudinalAccel = this.throttleInput * this.acceleration;
        } else if (this.speed > 0.5) {
            // Moving forward: negative throttle acts as braking
            longitudinalAccel = this.throttleInput * this.brakingForce;
        } else {
            // Near stop or reversing: gentle reverse throttle
            longitudinalAccel = this.throttleInput * (this.acceleration * 0.35);
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
        this._setHeading(
            Vector2D.normalizeAngle(
                this.heading + this.steerInput * this.turnRate * speedFactor * reverseSign * dt
            )
        );

        // 7. Integrate position
        const stepDist = this.speed * dt;
        const nextX = this.x + this.cosH * stepDist;
        const nextY = this.y + this.sinH * stepDist;

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
                if (city.recordVisuals) {
                    city.crashHeat.add(this.x, this.y);
                }
            }
            this.speed *= 0.72;

            // Rotate heading smoothly toward the forward lookahead point along the active road
            const angleDiff = Vector2D.normalizeAngle(roadState.desiredAngle - this.heading);
            this._setHeading(Vector2D.normalizeAngle(this.heading + angleDiff * 0.55));

            // Slide smoothly along the inward SDF gradient / lane target without teleporting forward
            const slideDist = Math.max(2.0, Math.abs(stepDist) * 0.7);
            const dxTarget = roadState.laneTarget.x - this.x;
            const dyTarget = roadState.laneTarget.y - this.y;
            const targetLen = Math.hypot(dxTarget, dyTarget);
            const pullX = targetLen > 1e-6 ? dxTarget / targetLen : this.cosH;
            const pullY = targetLen > 1e-6 ? dyTarget / targetLen : this.sinH;

            const candX = this.x + pullX * slideDist;
            const candY = this.y + pullY * slideDist;
            if (city.isPointOnRoad(candX, candY)) {
                this.x = candX;
                this.y = candY;
            } else {
                // Project back inside the road along the SDF inward normal instead of warping ahead
                const norm = city.sampleRoadNormal(this.x, this.y, this._normalScratch);
                const push = Math.max(2.5, norm.dist + 1.5);
                const projX = this.x + norm.x * push;
                const projY = this.y + norm.y * push;
                if (city.isPointOnRoad(projX, projY)) {
                    this.x = projX;
                    this.y = projY;
                } else {
                    this.x = roadState.laneTarget.x;
                    this.y = roadState.laneTarget.y;
                }
            }
        }

        // 8b. Resolve collision with any user-placed roadblock obstacles
        if (city.obstacles && city.obstacles.length > 0) {
            this._resolveObstacleCollisions(city);
        }

        // 9. Fitness & anti-stall evaluation
        this._evaluateFitness(dt, stepDist, roadState, city, allBoids);

        // 10-11. Visual telemetry (traffic heatmap & ghost trail): skipped in headless benchmarks
        if (city.recordVisuals) {
            city.trafficHeat.add(this.x, this.y, dt);
            this._recordTrail();
        }
    }

    _resolveObstacleCollisions(city) {
        const obstacles = city.obstacles;
        for (let i = 0; i < obstacles.length; i++) {
            const obs = obstacles[i];
            const dx = this.x - obs.x;
            const dy = this.y - obs.y;
            const minDist = obs.radius + this.collisionWidth;
            const distSq = dx * dx + dy * dy;
            if (distSq < minDist * minDist && distSq > 1e-6) {
                const dist = Math.sqrt(distSq);
                const nx = dx / dist;
                const ny = dy / dist;
                const overlap = minDist - dist + 0.5;
                const candX = this.x + nx * overlap;
                const candY = this.y + ny * overlap;
                if (city.isPointOnRoad(candX, candY)) {
                    this.x = candX;
                    this.y = candY;
                }
                this.speed *= 0.45;
                if (this.collisionCooldown <= 0) {
                    this.roadCollisions++;
                    this.fitness -= 4.5;
                    this.collisionCooldown = 0.45;
                    if (city.recordVisuals) {
                        city.crashHeat.add(this.x, this.y);
                    }
                }
            }
        }
    }

    _pickOutgoingRoad(node, currentRoad) {
        const roads = node.roads;
        if (roads.length <= 2) {
            // Fast path for degree-2 closed-loop corner nodes (zero array allocation)
            for (let i = 0; i < roads.length; i++) {
                if (roads[i] !== currentRoad) return roads[i];
            }
            return currentRoad;
        }

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
            // Aim for the keep-right lane of the outgoing road (zero allocation)
            const targetPoint = nextRoad.getLanePointInto(sign === 1 ? 0.28 : 0.72, sign, this._lanePointScratch);
            const desiredAngle = Math.atan2(targetPoint.y - this.y, targetPoint.x - this.x);
            const angleDiff = Vector2D.normalizeAngle(this.heading - desiredAngle);
            return Math.max(-1, Math.min(1, angleDiff / (Math.PI * 0.4)));
        }
        return roadState.headingError;
    }

    _evaluateFitness(dt, stepDist, roadState, city, allBoids) {
        // Keep right: left of the centerline is the opposite lane (lane rules are off inside crossings)
        const inOppositeLane = !roadState.inCrossing && roadState.lateralOffset < 0;

        const forwardCos = Math.cos(roadState.angleDiff);
        if (this.speed > 2 && forwardCos > 0.2) {
            const validDist = stepDist * forwardCos;
            this.distanceTraveled += validDist;
            this.fitness += validDist * 0.25 * (inOppositeLane ? 0.5 : 1);
        } else if (this.speed < -1) {
            this.fitness -= Math.abs(stepDist) * 0.1;
        }

        if (inOppositeLane && this.speed > 2) {
            this.wrongLaneTime += dt;
            this.fitness -= 1.5 * dt;
        }

        // Anti-collapse / anti-stall check: if stuck, respawn cleanly
        if (Math.abs(this.speed) < 6) {
            this.stuckTimer += dt;
            this.fitness -= 1.2 * dt;
            if (this.stuckTimer > 3.6) {
                const newPose = city.getRandomSpawnPose(allBoids);
                this.clearTrail(); // teleported: do not draw a trail across the map
                this.respawnFromX = this.x;
                this.respawnFromY = this.y;
                this.respawnToX = newPose.x;
                this.respawnToY = newPose.y;
                this.respawnAge = 0;
                this.x = newPose.x;
                this.y = newPose.y;
                this._setHeading(newPose.heading);
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
     * Radius of this boid's elliptical collision shape in world direction (nx, ny) (unit vector).
     */
    _collisionRadiusToward(nx, ny) {
        const along = this.collisionLength;
        const across = this.collisionWidth;
        const cos = this.cosH * nx + this.sinH * ny;
        const sinSq = Math.max(0, 1 - cos * cos);
        return (along * across) / Math.sqrt(across * across * cos * cos + along * along * sinSq);
    }

    /**
     * Advances all boids by one simulation step: uses the city's spatial grid for sensor queries,
     * rebuilds the grid once after all boids move, then resolves boid-to-boid collisions.
     * @param {Boid[]} boids
     * @param {City} city
     * @param {number} dt
     */
    static stepAll(boids, city, dt) {
        const grid = city.boidGrid;
        if (grid.boids !== boids) {
            grid.rebuild(boids);
        }
        for (let i = 0; i < boids.length; i++) {
            boids[i].update(dt, city, boids, grid);
        }
        grid.rebuild(boids);
        Boid.resolveBoidCollisions(boids, city, grid, false);
    }

    /**
     * Resolves pairwise boid-to-boid physical separation so boids NEVER collapse/stack onto each other
     * and never get pushed outside the road boundaries.
     * @param {Boid[]} boids
     * @param {City|null} city
     * @param {SpatialGrid|null} grid
     * @param {boolean} rebuildGrid - Whether to rebuild `grid` before pairing (false when already rebuilt by stepAll)
     */
    static resolveBoidCollisions(boids, city = null, grid = null, rebuildGrid = true) {
        const n = boids.length;

        if (!grid) {
            for (let i = 0; i < n; i++) {
                for (let j = i + 1; j < n; j++) {
                    Boid._resolvePair(boids[i], boids[j], city);
                }
            }
            return;
        }

        if (rebuildGrid) {
            grid.rebuild(boids);
        }
        const { cols, rows, cellStart, items, boidCell } = grid;
        for (let i = 0; i < n; i++) {
            const cell = boidCell[i];
            const col = cell % cols;
            const row = (cell - col) / cols;
            const r0 = row > 0 ? row - 1 : 0;
            const r1 = row < rows - 1 ? row + 1 : row;
            const c0 = col > 0 ? col - 1 : 0;
            const c1 = col < cols - 1 ? col + 1 : col;

            for (let r = r0; r <= r1; r++) {
                for (let c = c0; c <= c1; c++) {
                    const neighborCell = r * cols + c;
                    const end = cellStart[neighborCell + 1];
                    for (let k = cellStart[neighborCell]; k < end; k++) {
                        const j = items[k];
                        if (j > i) {
                            Boid._resolvePair(boids[i], boids[j], city);
                        }
                    }
                }
            }
        }
    }

    static _resolvePair(a, b, city) {
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        const distSq = dx * dx + dy * dy;

        // Quick reject using each boid's largest (lengthwise) collision radius
        const maxDist = a.collisionLength + b.collisionLength;
        if (distSq >= maxDist * maxDist || distSq <= 1e-6) return;

        const dist = Math.sqrt(distSq);
        const nx = dx / dist;
        const ny = dy / dist;

        // Orientation-aware contact distance: lengthwise radius front-to-back, widthwise radius
        // side-by-side, so boids passing in opposite lanes do not count as colliding
        const minDist = a._collisionRadiusToward(nx, ny) + b._collisionRadiusToward(-nx, -ny);
        if (dist >= minDist) return;

        const overlap = (minDist - dist) + 0.5;

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

        // Determine who rear-ended whom (or a head-on crash) and damp velocity
        const aForwardDot = a.cosH * nx + a.sinH * ny;
        const bForwardDot = -(b.cosH * nx + b.sinH * ny);
        const headingDot = a.cosH * b.cosH + a.sinH * b.sinH;

        if (headingDot < -0.5 && aForwardDot > 0.2 && bForwardDot > 0.2) {
            // Head-on: both drove into each other, both are at fault
            for (const boid of [a, b]) {
                boid.speed *= 0.3;
                if (boid.collisionCooldown <= 0) {
                    boid.boidCollisions++;
                    boid.headOnCollisions++;
                    boid.fitness -= 9.0;
                    boid.collisionCooldown = 0.45;
                    if (city && city.recordVisuals) city.crashHeat.add(boid.x, boid.y);
                }
            }
        } else if (aForwardDot > bForwardDot) {
            a.speed = Math.min(a.speed * 0.5, b.speed * 0.85);
            if (a.collisionCooldown <= 0) {
                a.boidCollisions++;
                a.fitness -= 4.5;
                a.collisionCooldown = 0.45;
                if (city && city.recordVisuals) city.crashHeat.add(a.x, a.y);
            }
        } else {
            b.speed = Math.min(b.speed * 0.5, a.speed * 0.85);
            if (b.collisionCooldown <= 0) {
                b.boidCollisions++;
                b.fitness -= 4.5;
                b.collisionCooldown = 0.45;
                if (city && city.recordVisuals) city.crashHeat.add(b.x, b.y);
            }
        }
    }
}
