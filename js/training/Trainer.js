/**
 * Trainer - Genetic Algorithm / Neuroevolution manager that trains the Boid population
 * over generations, tracks historical metrics for charts, and preserves elite brains.
 */
class Trainer {
    // Number of most recent generations kept in the chart history (in memory and in localStorage)
    static HISTORY_MAX_POINTS = 500;

    // A network only becomes champion after this many evaluations (averaged), so one lucky run never counts
    static CHAMPION_MIN_EVALS = 3;

    // With city rotation on, training switches to a new random city after every this many generations
    static CITY_ROTATION_INTERVAL = 10;

    // Benchmark ("driving test"): the top networks drive in full traffic on a fixed city with fixed
    // spawns and vehicle properties, so its scores show real progress instead of luck
    static BENCHMARK_SEED = 'benchmark-1';
    static BENCHMARK_SIZE = 10; // number of top networks tested
    static BENCHMARK_CARS = 55; // cars on the benchmark track (copies of the top networks)
    static BENCHMARK_ARCHETYPE = 'Sedan'; // same vehicle for every car, independent of the training types
    static BENCHMARK_STEPS = 1800; // 30 s at 60 Hz
    static BENCHMARK_INTERVAL = 10; // run after generation 1 and every 10th generation

    /**
     * @param {object} config
     */
    constructor(config = {}) {
        this.populationSize = config.populationSize || 55;
        // One vehicle type by default: with several types the fastest one takes over selection anyway
        this.boidTypesCount = config.boidTypesCount || 1;
        this.generationDuration = config.generationDuration || 30.0;
        this.mutationRate = config.mutationRate || 0.14;
        this.mutationStrength = config.mutationStrength || 0.24;
        this.rotateCities = config.rotateCities ?? true;

        this.generation = 1;
        this.generationTimer = 0;
        this.allTimeBestFitness = -Infinity;
        this.allTimeBestBrain = null;
        this.allTimeBestArchetype = 'Sedan';
        this.eliteBrains = [];
        this.history = [];
    }

    /**
     * Creates an initial population of Boids on the given city.
     * If saved elite brains exist (from localStorage), seeds the population from them.
     * @param {City} city
     * @returns {Boid[]}
     */
    createInitialPopulation(city) {
        const boids = [];
        const hasElites = this.eliteBrains && this.eliteBrains.length > 0;
        const activeTypes = Boid.ARCHETYPES.slice(0, Math.max(1, Math.min(4, this.boidTypesCount)));
        const poses = this._shuffledSpawnPoses(city);

        for (let i = 0; i < this.populationSize; i++) {
            const pose = poses[i];
            let brain = null;

            if (hasElites) {
                const template = this.eliteBrains[i % this.eliteBrains.length];
                brain = template.clone();
                if (i >= this.eliteBrains.length) {
                    brain.mutate(this.mutationRate, this.mutationStrength);
                }
            }

            const chosenArch = activeTypes[i % activeTypes.length].name;
            const boid = new Boid(i + 1, pose, brain, {
                allowedTypesCount: this.boidTypesCount,
                archetype: chosenArch
            });
            boids.push(boid);
        }

        // Prime initial sensor rays so they are immediately ready for rendering
        for (let i = 0; i < boids.length; i++) {
            boids[i].sensor.update(city, boids);
        }

        return boids;
    }

    /**
     * Evaluates the current population, records generation statistics, and breeds the next generation.
     * Ranking uses each network's average score over its last evaluations (see NeuralNetwork.recordScore),
     * so unchanged elites that keep performing stay on top while one-off lucky runs fall back.
     * @param {Boid[]} currentBoids
     * @param {City} city - City the next generation drives on (a new one when the city rotates)
     * @returns {Boid[]} New generation of boids
     */
    evolveNextGeneration(currentBoids, city) {
        for (const b of currentBoids) {
            b.brain.recordScore(b.fitness);
            b.rankScore = b.brain.meanScore();
        }
        const sorted = [...currentBoids].sort((a, b) => b.rankScore - a.rankScore);
        const count = Math.max(1, sorted.length);

        // Statistics of this generation's own run (raw, unaveraged)
        let bestBoid = null;
        let totalFitness = 0;
        let totalDistance = 0;
        let totalCollisions = 0;
        for (const b of currentBoids) {
            if (!bestBoid || b.fitness > bestBoid.fitness) bestBoid = b;
            totalFitness += b.fitness;
            totalDistance += b.distanceTraveled;
            totalCollisions += (b.roadCollisions + b.boidCollisions);
        }
        const bestFitness = bestBoid ? bestBoid.fitness : 0;
        const bestDistance = bestBoid ? bestBoid.distanceTraveled : 0;
        const avgFitness = totalFitness / count;
        const avgDistance = totalDistance / count;
        const avgCollisions = totalCollisions / count;
        const avgSpeed = avgDistance / Math.max(1, this.generationDuration);
        // Time actually driven (shorter when "Next Gen Now" ended the generation early)
        const drivenSeconds = this.generationTimer > 0 ? this.generationTimer : this.generationDuration;
        const avgEfficiency = Trainer.efficiencyPercent(currentBoids, drivenSeconds);

        // Champion: best average among networks evaluated at least CHAMPION_MIN_EVALS times. The champion
        // is re-evaluated every generation (slot 0), so a weaker average lets a better network take over.
        const champion = sorted.find(b => b.brain.evalScores.length >= Trainer.CHAMPION_MIN_EVALS) || null;
        if (champion) {
            this.allTimeBestBrain = champion.brain.clone();
            this.allTimeBestArchetype = champion.archetype;
            this.allTimeBestFitness = Math.max(this.allTimeBestFitness, champion.rankScore);
        } else if (!this.allTimeBestBrain && sorted[0]) {
            // Early generations: provisional champion until one has enough evaluations
            this.allTimeBestBrain = sorted[0].brain.clone();
            this.allTimeBestArchetype = sorted[0].archetype;
        }

        // Benchmark only every BENCHMARK_INTERVAL generations (the charts connect the measured points)
        const runBenchmark = this.generation === 1 || this.generation % Trainer.BENCHMARK_INTERVAL === 0;
        const benchmark = runBenchmark
            ? this._runBenchmark(sorted.slice(0, Trainer.BENCHMARK_SIZE), city.width, city.height)
            : null;

        // Record history entry for Progress Charts
        const entry = {
            generation: this.generation,
            bestFitness: Number(bestFitness.toFixed(2)),
            avgFitness: Number(avgFitness.toFixed(2)),
            bestDistance: Number(bestDistance.toFixed(1)),
            avgDistance: Number(avgDistance.toFixed(1)),
            avgCollisions: Number(avgCollisions.toFixed(2)),
            avgSpeed: Number(avgSpeed.toFixed(1)),
            avgEfficiency: Number(avgEfficiency.toFixed(1))
        };
        if (benchmark) {
            entry.benchmarkFitness = Number(benchmark.fitness.toFixed(2));
            entry.benchmarkDistance = Number(benchmark.distance.toFixed(1));
            entry.benchmarkCollisions = Number(benchmark.collisions.toFixed(2));
            entry.benchmarkEfficiency = Number(benchmark.efficiency.toFixed(1));
        }
        this.history.push(entry);
        this._trimHistory();

        // Keep top elites (by average score); slot 0 is the champion
        const eliteCount = Math.max(4, Math.floor(this.populationSize * 0.12));
        this.eliteBrains = sorted.slice(0, eliteCount).map(b => b.brain.clone());
        if (this.allTimeBestBrain) {
            this.eliteBrains[0] = this.allTimeBestBrain.clone();
        }

        const activeTypes = Boid.ARCHETYPES.slice(0, Math.max(1, Math.min(4, this.boidTypesCount)));
        const poses = this._shuffledSpawnPoses(city);

        // Breed next generation
        const nextBoids = [];
        for (let i = 0; i < this.populationSize; i++) {
            const pose = poses[i];
            let childBrain = null;
            let childArchetype = activeTypes[i % activeTypes.length].name;

            if (i === 0 && this.allTimeBestBrain) {
                // Slot 0: champion, unchanged so it is evaluated again
                childBrain = this.allTimeBestBrain.clone();
                childArchetype = this.allTimeBestArchetype;
            } else if (i < eliteCount && this.eliteBrains[i]) {
                // Elites preserved unchanged (and re-evaluated) with their matching archetype
                childBrain = this.eliteBrains[i].clone();
                childArchetype = sorted[i] ? sorted[i].archetype : childArchetype;
            } else if (i < this.populationSize - 6) {
                // Tournament selection + crossover + mutation
                const parentA = this._tournamentSelect(sorted, 4);
                const parentB = this._tournamentSelect(sorted, 4);
                childBrain = parentA.brain.crossover(parentB.brain);
                childBrain.mutate(this.mutationRate, this.mutationStrength);
                if (Random.next() < 0.85) {
                    childArchetype = parentA.archetype;
                }
            } else {
                // Fine-tuned small mutation of the best brains for local refinement
                const templateParent = sorted[i % Math.min(3, sorted.length)];
                childBrain = (templateParent ? templateParent.brain : this.allTimeBestBrain).clone();
                childBrain.mutate(this.mutationRate * 0.55, this.mutationStrength * 0.45);
                if (templateParent) childArchetype = templateParent.archetype;
            }

            const boid = new Boid(i + 1, pose, childBrain, {
                allowedTypesCount: this.boidTypesCount,
                archetype: childArchetype
            });
            nextBoids.push(boid);
        }

        // Prime initial sensor rays
        for (let i = 0; i < nextBoids.length; i++) {
            nextBoids[i].sensor.update(city, nextBoids);
        }

        this.generation++;
        this.generationTimer = 0;
        return nextBoids;
    }

    /**
     * True when the generation that just finished should be followed by a new city.
     */
    shouldRotateCity() {
        return this.rotateCities && this.generation % Trainer.CITY_ROTATION_INTERVAL === 0;
    }

    /**
     * Evenly spread spawn poses for the whole population, randomly assigned to population slots.
     */
    _shuffledSpawnPoses(city) {
        const poses = city.getSpawnPoses(this.populationSize);
        for (let i = poses.length - 1; i > 0; i--) {
            const j = Math.floor(Random.next() * (i + 1));
            [poses[i], poses[j]] = [poses[j], poses[i]];
        }
        return poses;
    }

    /**
     * Average share of the maximum possible distance (top speed x driving time) the boids covered, in %.
     * @param {Boid[]} boids
     * @param {number} seconds - Driving time
     */
    static efficiencyPercent(boids, seconds) {
        if (boids.length === 0 || seconds <= 0) return 0;
        let sum = 0;
        for (const b of boids) {
            sum += b.distanceTraveled / (b.maxSpeed * seconds);
        }
        return (100 * sum) / boids.length;
    }

    /**
     * Driving test in full traffic: BENCHMARK_CARS cars, each driving a copy of one of the given top
     * networks (round-robin), all as BENCHMARK_ARCHETYPE, for BENCHMARK_STEPS steps on the fixed
     * benchmark city. Uses its own random seed (restored afterwards), so every run has identical
     * conditions and the training run is not affected.
     * Returns the per-car averages of fitness, distance and collisions, and the efficiency in %.
     */
    _runBenchmark(topBoids, width, height) {
        if (topBoids.length === 0) return { fitness: 0, distance: 0, collisions: 0, efficiency: 0 };
        const savedRandom = Random.saveState();

        if (!this._benchmarkCity || this._benchmarkCity.width !== width || this._benchmarkCity.height !== height) {
            this._benchmarkCity = CityGenerator.generate(width, height, Trainer.BENCHMARK_SEED);
        }
        const city = this._benchmarkCity;
        Random.setSeed(`${Trainer.BENCHMARK_SEED}-run`);

        const poses = city.getSpawnPoses(Trainer.BENCHMARK_CARS);
        const boids = poses.map((pose, i) => new Boid(i + 1, pose, topBoids[i % topBoids.length].brain, {
            allowedTypesCount: 4,
            archetype: Trainer.BENCHMARK_ARCHETYPE
        }));
        for (let i = 0; i < boids.length; i++) {
            boids[i].sensor.update(city, boids);
        }
        for (let s = 0; s < Trainer.BENCHMARK_STEPS; s++) {
            Boid.stepAll(boids, city, 1 / 60);
        }

        Random.restoreState(savedRandom);
        let fitness = 0;
        let distance = 0;
        let collisions = 0;
        for (const b of boids) {
            fitness += b.fitness;
            distance += b.distanceTraveled;
            collisions += b.roadCollisions + b.boidCollisions;
        }
        const n = boids.length;
        return {
            fitness: fitness / n,
            distance: distance / n,
            collisions: collisions / n,
            efficiency: Trainer.efficiencyPercent(boids, Trainer.BENCHMARK_STEPS / 60)
        };
    }

    /**
     * Keeps only the most recent HISTORY_MAX_POINTS generations; older entries are dropped.
     */
    _trimHistory() {
        const excess = this.history.length - Trainer.HISTORY_MAX_POINTS;
        if (excess > 0) {
            this.history.splice(0, excess);
        }
    }

    _tournamentSelect(sortedBoids, k = 4) {
        const poolSize = Math.max(2, Math.floor(sortedBoids.length * 0.55));
        let best = null;
        for (let i = 0; i < k; i++) {
            const idx = Math.floor(Random.next() * poolSize);
            const candidate = sortedBoids[idx];
            if (!best || candidate.rankScore > best.rankScore) {
                best = candidate;
            }
        }
        return best || sortedBoids[0];
    }

    /**
     * Returns the currently highest-scoring live Boid.
     * @param {Boid[]} boids
     * @returns {Boid|null}
     */
    getBestLiveBoid(boids) {
        if (!boids || boids.length === 0) return null;
        let best = boids[0];
        for (let i = 1; i < boids.length; i++) {
            if (boids[i].fitness > best.fitness) {
                best = boids[i];
            }
        }
        return best;
    }

    serialize() {
        return {
            generation: this.generation,
            populationSize: this.populationSize,
            boidTypesCount: this.boidTypesCount,
            generationDuration: this.generationDuration,
            mutationRate: this.mutationRate,
            mutationStrength: this.mutationStrength,
            rotateCities: this.rotateCities,
            allTimeBestFitness: this.allTimeBestFitness,
            allTimeBestArchetype: this.allTimeBestArchetype,
            allTimeBestBrain: this.allTimeBestBrain ? this.allTimeBestBrain.serialize() : null,
            eliteBrains: this.eliteBrains.map(b => b.serialize()),
            history: this.history.slice(-Trainer.HISTORY_MAX_POINTS)
        };
    }

    restoreFromData(data) {
        if (!data) return;
        this.generation = data.generation || 1;
        this.populationSize = data.populationSize || this.populationSize;
        this.boidTypesCount = data.boidTypesCount || this.boidTypesCount;
        this.generationDuration = data.generationDuration || this.generationDuration;
        this.mutationRate = data.mutationRate ?? this.mutationRate;
        this.mutationStrength = data.mutationStrength ?? this.mutationStrength;
        this.rotateCities = data.rotateCities ?? this.rotateCities;
        this.allTimeBestFitness = data.allTimeBestFitness ?? -Infinity;
        this.allTimeBestArchetype = data.allTimeBestArchetype || 'Sedan';
        this.history = Array.isArray(data.history) ? data.history : [];
        this._trimHistory();

        // Saved brains are only reused if they match the current network layout; brains from an
        // older layout (or a corrupted save) are discarded so they can never be mixed with new ones
        const layout = NeuralNetwork.DEFAULT_LAYOUT;
        const loadBrain = saved => {
            if (!saved) return null;
            try {
                const nn = NeuralNetwork.deserialize(saved);
                return nn.hasLayout(layout) ? nn : null;
            } catch (err) {
                return null;
            }
        };

        this.allTimeBestBrain = loadBrain(data.allTimeBestBrain);
        if (!this.allTimeBestBrain) {
            // The record belonged to a discarded brain, so start the all-time best over
            this.allTimeBestFitness = -Infinity;
            this.allTimeBestArchetype = 'Sedan';
        }
        const savedElites = Array.isArray(data.eliteBrains) ? data.eliteBrains : [];
        this.eliteBrains = savedElites.map(loadBrain).filter(Boolean);

        const discarded = savedElites.length - this.eliteBrains.length +
            (data.allTimeBestBrain && !this.allTimeBestBrain ? 1 : 0);
        if (discarded > 0) {
            console.info(`Trainer: discarded ${discarded} saved brain(s) that do not match network layout [${layout.join(', ')}]`);
        }
    }
}
