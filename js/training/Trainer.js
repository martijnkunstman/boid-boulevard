/**
 * Trainer - Genetic Algorithm / Neuroevolution manager that trains the Boid population
 * over generations, tracks historical metrics for charts, and preserves elite brains.
 */
class Trainer {
    // Maximum number of generation history points kept (in memory and in localStorage)
    static HISTORY_MAX_POINTS = 500;

    /**
     * @param {object} config
     */
    constructor(config = {}) {
        this.populationSize = config.populationSize || 55;
        this.boidTypesCount = config.boidTypesCount || 2;
        this.generationDuration = config.generationDuration || 30.0;
        this.mutationRate = config.mutationRate || 0.14;
        this.mutationStrength = config.mutationStrength || 0.24;

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

        for (let i = 0; i < this.populationSize; i++) {
            const pose = city.getRandomSpawnPose(boids);
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
     * @param {Boid[]} currentBoids
     * @param {City} city
     * @returns {Boid[]} New generation of boids
     */
    evolveNextGeneration(currentBoids, city) {
        const sorted = [...currentBoids].sort((a, b) => b.fitness - a.fitness);
        const count = Math.max(1, sorted.length);

        const bestBoid = sorted[0];
        const bestFitness = bestBoid ? bestBoid.fitness : 0;
        const bestDistance = bestBoid ? bestBoid.distanceTraveled : 0;

        let totalFitness = 0;
        let totalDistance = 0;
        let totalCollisions = 0;

        for (const b of sorted) {
            totalFitness += b.fitness;
            totalDistance += b.distanceTraveled;
            totalCollisions += (b.roadCollisions + b.boidCollisions);
        }

        const avgFitness = totalFitness / count;
        const avgDistance = totalDistance / count;
        const avgCollisions = totalCollisions / count;
        const avgSpeed = avgDistance / Math.max(1, this.generationDuration);

        if (bestBoid && bestFitness > this.allTimeBestFitness) {
            this.allTimeBestFitness = bestFitness;
            this.allTimeBestBrain = bestBoid.brain.clone();
            this.allTimeBestArchetype = bestBoid.archetype;
        } else if (!this.allTimeBestBrain && bestBoid) {
            this.allTimeBestFitness = bestFitness;
            this.allTimeBestBrain = bestBoid.brain.clone();
            this.allTimeBestArchetype = bestBoid.archetype;
        }

        // Record history entry for Progress Charts
        this.history.push({
            generation: this.generation,
            bestFitness: Number(bestFitness.toFixed(2)),
            avgFitness: Number(avgFitness.toFixed(2)),
            bestDistance: Number(bestDistance.toFixed(1)),
            avgDistance: Number(avgDistance.toFixed(1)),
            avgCollisions: Number(avgCollisions.toFixed(2)),
            avgSpeed: Number(avgSpeed.toFixed(1))
        });
        this._compactHistory();

        // Keep top elites
        const eliteCount = Math.max(4, Math.floor(this.populationSize * 0.12));
        this.eliteBrains = sorted.slice(0, eliteCount).map(b => b.brain.clone());
        if (this.allTimeBestBrain) {
            this.eliteBrains[0] = this.allTimeBestBrain.clone();
        }

        const activeTypes = Boid.ARCHETYPES.slice(0, Math.max(1, Math.min(4, this.boidTypesCount)));

        // Breed next generation
        const nextBoids = [];
        for (let i = 0; i < this.populationSize; i++) {
            const pose = city.getRandomSpawnPose(nextBoids);
            let childBrain = null;
            let childArchetype = activeTypes[i % activeTypes.length].name;

            if (i === 0 && this.allTimeBestBrain) {
                // Slot 0: All-time champion brain untouched
                childBrain = this.allTimeBestBrain.clone();
                childArchetype = this.allTimeBestArchetype;
            } else if (i < eliteCount && this.eliteBrains[i]) {
                // Elites preserved untouched with their matching archetype
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
     * Keeps the history within HISTORY_MAX_POINTS without dropping the start of the run:
     * once the limit is exceeded, every odd-indexed point is removed, halving the history
     * (500 -> ~250) while keeping the first generation and the most recent one.
     */
    _compactHistory() {
        while (this.history.length > Trainer.HISTORY_MAX_POINTS) {
            this.history = this.history.filter((_, i) => i % 2 === 0);
        }
    }

    _tournamentSelect(sortedBoids, k = 4) {
        const poolSize = Math.max(2, Math.floor(sortedBoids.length * 0.55));
        let best = null;
        for (let i = 0; i < k; i++) {
            const idx = Math.floor(Random.next() * poolSize);
            const candidate = sortedBoids[idx];
            if (!best || candidate.fitness > best.fitness) {
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
            allTimeBestFitness: this.allTimeBestFitness,
            allTimeBestArchetype: this.allTimeBestArchetype,
            allTimeBestBrain: this.allTimeBestBrain ? this.allTimeBestBrain.serialize() : null,
            eliteBrains: this.eliteBrains.map(b => b.serialize()),
            history: this.history
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
        this.allTimeBestFitness = data.allTimeBestFitness ?? -Infinity;
        this.allTimeBestArchetype = data.allTimeBestArchetype || 'Sedan';
        this.allTimeBestBrain = data.allTimeBestBrain
            ? NeuralNetwork.deserialize(data.allTimeBestBrain)
            : null;
        this.eliteBrains = Array.isArray(data.eliteBrains)
            ? data.eliteBrains.map(b => NeuralNetwork.deserialize(b))
            : [];
        this.history = Array.isArray(data.history) ? data.history : [];
        this._compactHistory();
    }
}
