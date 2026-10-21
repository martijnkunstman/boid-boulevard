/**
 * Headless benchmark & verification runner for Boid Boulevard.
 * Loads the vanilla JS classes in Node.js and runs reproducible multi-seed training benchmarks.
 *
 * Usage:
 *   node scripts/headless-bench.js [generations=60] [seeds=104729,271828,314159]
 */
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..');

const SCRIPT_FILES = [
    'js/math/Random.js',
    'js/math/Vector2D.js',
    'js/nn/NeuralNetwork.js',
    'js/world/RoadNode.js',
    'js/world/Road.js',
    'js/world/HeatGrid.js',
    'js/world/SpatialGrid.js',
    'js/world/City.js',
    'js/world/CityGenerator.js',
    'js/entities/Sensor.js',
    'js/entities/Boid.js',
    'js/training/Trainer.js'
];

function loadSimulationClasses() {
    const context = vm.createContext({
        Math,
        Number,
        String,
        Boolean,
        Array,
        Object,
        Set,
        Map,
        Float32Array,
        Int32Array,
        Uint8Array,
        console,
        performance
    });

    for (const relPath of SCRIPT_FILES) {
        const fullPath = path.join(ROOT, relPath);
        const code = fs.readFileSync(fullPath, 'utf8');
        vm.runInContext(code, context, { filename: relPath });
    }

    vm.runInContext(
        `globalThis.exports = { Random, Vector2D, NeuralNetwork, RoadNode, Road, HeatGrid, SpatialGrid, City, CityGenerator, Sensor, Boid, Trainer };`,
        context
    );
    return context.exports;
}

function runSeed(classes, seed, generations, popSize = 55) {
    const { Random, CityGenerator, Boid, Trainer } = classes;
    Random.setSeed(seed);

    const trainer = new Trainer({
        populationSize: popSize,
        boidTypesCount: 1,
        generationDuration: 30.0,
        mutationRate: 0.14,
        mutationStrength: 0.24,
        rotateCities: true
    });

    const width = 1020;
    const height = 620;
    let city = CityGenerator.generate(width, height, seed);
    if ('recordVisuals' in city) city.recordVisuals = false;

    let boids = trainer.createInitialPopulation(city);
    const dt = 1 / 60;
    const stepsPerGen = Math.round(trainer.generationDuration / dt);

    let totalPhysicsSteps = 0;
    const t0 = performance.now();

    for (let g = 1; g <= generations; g++) {
        for (let s = 0; s < stepsPerGen; s++) {
            city.update(dt);
            Boid.stepAll(boids, city, dt);
            trainer.generationTimer += dt;
            totalPhysicsSteps++;
        }

        let nextCity = city;
        if (trainer.shouldRotateCity()) {
            const nextSeed = trainer.nextCitySeed
                ? trainer.nextCitySeed()
                : String(Math.floor(100000 + Random.next() * 900000));
            nextCity = CityGenerator.generate(width, height, nextSeed);
            if ('recordVisuals' in nextCity) nextCity.recordVisuals = false;
        }

        boids = trainer.evolveNextGeneration(boids, nextCity);
        city = nextCity;
    }

    const elapsedMs = performance.now() - t0;
    const lastBench = [...trainer.history].reverse().find(h => h.benchmarkEfficiency !== undefined) || {};

    return {
        seed,
        generations,
        elapsedMs,
        stepsPerSec: Math.round((totalPhysicsSteps / elapsedMs) * 1000),
        msPerGen: Number((elapsedMs / generations).toFixed(2)),
        allTimeBestFitness: Number(trainer.allTimeBestFitness.toFixed(1)),
        benchFitness: lastBench.benchmarkFitness ?? 0,
        benchDistance: lastBench.benchmarkDistance ?? 0,
        benchCollisions: lastBench.benchmarkCollisions ?? 0,
        benchEfficiency: lastBench.benchmarkEfficiency ?? 0
    };
}

function main() {
    const generations = parseInt(process.argv[2] || '40', 10);
    const seeds = (process.argv[3] || '104729,271828,314159').split(',');
    const classes = loadSimulationClasses();

    console.log(`Running headless benchmark: ${generations} generations x ${seeds.length} seeds...`);
    const results = [];
    for (const seed of seeds) {
        const res = runSeed(classes, seed.trim(), generations);
        results.push(res);
        console.log(
            `  Seed ${res.seed}: ${res.msPerGen} ms/gen (${res.stepsPerSec} steps/s) | ` +
            `Bench Eff: ${res.benchEfficiency}% | Bench Col: ${res.benchCollisions} | ` +
            `Bench Fit: ${res.benchFitness} | Champ Fit: ${res.allTimeBestFitness}`
        );
    }

    const avg = key => (results.reduce((s, r) => s + r[key], 0) / results.length);
    console.log('\n--- Mean Across Seeds ---');
    console.log(`  Speed:              ${avg('msPerGen').toFixed(2)} ms/gen (${Math.round(avg('stepsPerSec'))} steps/s)`);
    console.log(`  Benchmark Eff:      ${avg('benchEfficiency').toFixed(2)}%`);
    console.log(`  Benchmark Collis:   ${avg('benchCollisions').toFixed(2)}`);
    console.log(`  Benchmark Fitness:  ${avg('benchFitness').toFixed(2)}`);
    console.log(`  Champion Fitness:   ${avg('allTimeBestFitness').toFixed(2)}`);
}

if (require.main === module) {
    main();
}

module.exports = { loadSimulationClasses, runSeed };
