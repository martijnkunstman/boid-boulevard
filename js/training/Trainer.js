/**
 * Trainer - Genetic Algorithm / Neuroevolution manager that trains the Boid population
 * over generations, tracks historical metrics for charts, and preserves elite brains.
 */
class Trainer {
    /**
     * @param {object} config
     */
    constructor(config = {}) {
        this.populationSize = config.populationSize || 55;
        this.generationDuration = config.generationDuration || 18.0;
        this.mutationRate = config.mutationRate || 0.14;
        this.mutationStrength = config.mutationStrength || 0.24;

        this.generation = 1;
        this.generationTimer = 0;
        this.allTimeBestFitness = -Infinity;
        this.allTimeBestBrain = null;
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

            const boid = new Boid(i + 1, pose, brain);
            boids.push(boid);
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
        let totalViolations = 0;
        let totalStops = 0;

        for (const b of sorted) {
            totalFitness += b.fitness;
            totalDistance += b.distanceTraveled;
            totalCollisions += (b.roadCollisions + b.boidCollisions);
            totalViolations += b.redLightViolations;
            totalStops += b.redLightStops;
        }

        const avgFitness = totalFitness / count;
        const avgDistance = totalDistance / count;
        const avgCollisions = totalCollisions / count;
        const totalLightEvents = totalStops + totalViolations * 0.5;
        const redLightCompliance = totalLightEvents > 0.01
            ? Math.max(0, Math.min(100, (totalStops / (totalStops + totalViolations * 0.35)) * 100))
            : 100;

        if (bestBoid && bestFitness > this.allTimeBestFitness) {
            this.allTimeBestFitness = bestFitness;
            this.allTimeBestBrain = bestBoid.brain.clone();
        } else if (!this.allTimeBestBrain && bestBoid) {
            this.allTimeBestFitness = bestFitness;
            this.allTimeBestBrain = bestBoid.brain.clone();
        }

        // Record history entry for Progress Charts
        this.history.push({
            generation: this.generation,
            bestFitness: Number(bestFitness.toFixed(2)),
            avgFitness: Number(avgFitness.toFixed(2)),
            bestDistance: Number(bestDistance.toFixed(1)),
            avgDistance: Number(avgDistance.toFixed(1)),
            avgCollisions: Number(avgCollisions.toFixed(2)),
            redLightCompliance: Number(redLightCompliance.toFixed(1))
        });

        // Keep top elites
        const eliteCount = Math.max(4, Math.floor(this.populationSize * 0.12));
        this.eliteBrains = sorted.slice(0, eliteCount).map(b => b.brain.clone());
        if (this.allTimeBestBrain) {
            this.eliteBrains[0] = this.allTimeBestBrain.clone();
        }

        // Breed next generation
        const nextBoids = [];
        for (let i = 0; i < this.populationSize; i++) {
            const pose = city.getRandomSpawnPose(nextBoids);
            let childBrain = null;

            if (i === 0 && this.allTimeBestBrain) {
                // Slot 0: All-time champion brain untouched
                childBrain = this.allTimeBestBrain.clone();
            } else if (i < eliteCount && this.eliteBrains[i]) {
                // Elites preserved untouched
                childBrain = this.eliteBrains[i].clone();
            } else if (i < this.populationSize - 4) {
                // Tournament selection + crossover + mutation
                const parentA = this._tournamentSelect(sorted, 4);
                const parentB = this._tournamentSelect(sorted, 4);
                childBrain = parentA.brain.crossover(parentB.brain);
                childBrain.mutate(this.mutationRate, this.mutationStrength);
            } else {
                // Fine-tuned mutation of the best brain
                childBrain = (this.allTimeBestBrain || sorted[0].brain).clone();
                childBrain.mutate(this.mutationRate * 0.6, this.mutationStrength * 0.5);
            }

            const boid = new Boid(i + 1, pose, childBrain);
            nextBoids.push(boid);
        }

        this.generation++;
        this.generationTimer = 0;
        return nextBoids;
    }

    _tournamentSelect(sortedBoids, k = 4) {
        const poolSize = Math.max(2, Math.floor(sortedBoids.length * 0.55));
        let best = null;
        for (let i = 0; i < k; i++) {
            const idx = Math.floor(Math.random() * poolSize);
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
            generationDuration: this.generationDuration,
            mutationRate: this.mutationRate,
            mutationStrength: this.mutationStrength,
            allTimeBestFitness: this.allTimeBestFitness,
            allTimeBestBrain: this.allTimeBestBrain ? this.allTimeBestBrain.serialize() : null,
            eliteBrains: this.eliteBrains.map(b => b.serialize()),
            history: this.history.slice(-150) // Keep up to 150 generations in storage
        };
    }

    restoreFromData(data) {
        if (!data) return;
        this.generation = data.generation || 1;
        this.populationSize = data.populationSize || this.populationSize;
        this.generationDuration = data.generationDuration || this.generationDuration;
        this.mutationRate = data.mutationRate ?? this.mutationRate;
        this.mutationStrength = data.mutationStrength ?? this.mutationStrength;
        this.allTimeBestFitness = data.allTimeBestFitness ?? -Infinity;
        this.allTimeBestBrain = data.allTimeBestBrain
            ? NeuralNetwork.deserialize(data.allTimeBestBrain)
            : null;
        this.eliteBrains = Array.isArray(data.eliteBrains)
            ? data.eliteBrains.map(b => NeuralNetwork.deserialize(b))
            : [];
        this.history = Array.isArray(data.history) ? data.history : [];
    }
}
