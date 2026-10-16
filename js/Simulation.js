/**
 * Simulation - Main application orchestrator connecting City, Boids, Trainer,
 * StorageManager, SimulationRenderer, DebugScreen, and ChartScreen.
 */
class Simulation {
    // Milliseconds of physics per animation frame in Turbo mode (leaves headroom for the browser at 60 fps)
    static TURBO_FRAME_BUDGET_MS = 14;

    constructor() {
        this.simCanvas = document.getElementById('sim-canvas');
        this.nnCanvas = document.getElementById('nn-canvas');
        this.telemetryContainer = document.getElementById('debug-telemetry');

        this.renderer = new SimulationRenderer(this.simCanvas);
        this.debugScreen = new DebugScreen(this.nnCanvas, this.telemetryContainer);
        this.chartScreen = new ChartScreen({
            fitnessCanvas: document.getElementById('chart-fitness'),
            distanceCanvas: document.getElementById('chart-distance'),
            collisionCanvas: document.getElementById('chart-collisions'),
            speedCanvas: document.getElementById('chart-speed')
        });

        this.trainer = new Trainer({
            populationSize: 55,
            boidTypesCount: 2,
            generationDuration: 30.0,
            mutationRate: 0.14,
            mutationStrength: 0.24
        });

        this.city = null;
        this.boids = [];
        this.selectedBoid = null;

        this.isPaused = false;
        this.simSpeed = 1;
        this.turboMode = false;
        this.lastFrameTime = performance.now();
        this.uiRefreshAccumulator = 0;

        this._initFromStorageOrRandom();
        this._bindEvents();
        this._updateHeaderStats();
        this.renderer.render(this.city, this.boids, this.getInspectedBoid());
        this.debugScreen.render(this.getInspectedBoid(), this.city, this.trainer, this.boids);
        this.chartScreen.render(this.trainer, this.boids);

        requestAnimationFrame(ts => this._loop(ts));
    }

    _initFromStorageOrRandom() {
        const saved = StorageManager.loadProgress();
        if (saved && saved.trainer) {
            this.trainer.restoreFromData(saved.trainer);
            if (saved.city) {
                this.city = City.deserialize(saved.city);
                if (this.city && this.city.seed) {
                    Random.setSeed(this.city.seed);
                }
            }
        }

        if (!this.city) {
            const initialSeed = Random.generateSeedString();
            this.city = CityGenerator.generate(this.simCanvas.width, this.simCanvas.height, initialSeed);
        }

        this._syncControlsFromState();
        this.boids = this.trainer.createInitialPopulation(this.city);
        StorageManager.saveProgress(this.trainer, this.city);
    }

    _syncControlsFromState() {
        const seedInput = document.getElementById('input-seed');
        if (seedInput && this.city) {
            seedInput.value = String(this.city.seed ?? Random.currentSeed);
        }
        const popInput = document.getElementById('input-pop-size');
        if (popInput) {
            popInput.value = this.trainer.populationSize;
        }
        const selectTypes = document.getElementById('select-boid-types');
        if (selectTypes) {
            selectTypes.value = String(this.trainer.boidTypesCount);
        }
        const selectDuration = document.getElementById('select-gen-duration');
        if (selectDuration) {
            selectDuration.value = String(Math.round(this.trainer.generationDuration));
        }
    }

    /**
     * Applies a user-specified seed from the UI input to deterministically generate that city.
     */
    applyCustomSeed(seedValue) {
        const cleanSeed = String(seedValue ?? '').trim() || Random.generateSeedString();
        this.city = CityGenerator.generate(this.simCanvas.width, this.simCanvas.height, cleanSeed);
        this._syncControlsFromState();
        this.boids = this.trainer.createInitialPopulation(this.city);
        this.selectedBoid = null;
        StorageManager.saveProgress(this.trainer, this.city);
        this._updateHeaderStats();
        this.renderer.render(this.city, this.boids, this.getInspectedBoid());
        this.debugScreen.render(this.getInspectedBoid(), this.city, this.trainer, this.boids);
    }

    /**
     * Clears localStorage and restarts the entire simulation with a new random seed, roads, and boids.
     */
    restartAndClearStorage() {
        StorageManager.clearProgress();
        const popInput = document.getElementById('input-pop-size');
        const popSize = popInput ? parseInt(popInput.value, 10) || 55 : 55;
        const selectTypes = document.getElementById('select-boid-types');
        const typesCount = selectTypes ? parseInt(selectTypes.value, 10) || 2 : 2;
        const selectDuration = document.getElementById('select-gen-duration');
        const genDuration = selectDuration ? parseInt(selectDuration.value, 10) || 30 : 30;

        this.trainer = new Trainer({
            populationSize: popSize,
            boidTypesCount: typesCount,
            generationDuration: genDuration,
            mutationRate: 0.14,
            mutationStrength: 0.24
        });

        const newSeed = Random.generateSeedString();
        this.city = CityGenerator.generate(this.simCanvas.width, this.simCanvas.height, newSeed);
        this._syncControlsFromState();
        this.boids = this.trainer.createInitialPopulation(this.city);
        this.selectedBoid = null;

        this._updateHeaderStats();
        this.renderer.render(this.city, this.boids, this.getInspectedBoid());
        this.debugScreen.render(this.getInspectedBoid(), this.city, this.trainer, this.boids);
        this.chartScreen.render(this.trainer, this.boids);
    }

    /**
     * Generates a new random city with a fresh seed while keeping trained neural network progress.
     */
    generateNewRandomCityKeepBrains() {
        const newSeed = Random.generateSeedString();
        this.city = CityGenerator.generate(this.simCanvas.width, this.simCanvas.height, newSeed);
        this._syncControlsFromState();
        this.boids = this.trainer.createInitialPopulation(this.city);
        this.selectedBoid = null;
        StorageManager.saveProgress(this.trainer, this.city);
        this._updateHeaderStats();
        this.renderer.render(this.city, this.boids, this.getInspectedBoid());
    }

    /**
     * Immediately finishes the current generation and evolves the next one.
     */
    forceNextGeneration() {
        this.boids = this.trainer.evolveNextGeneration(this.boids, this.city);
        this.selectedBoid = null;
        StorageManager.saveProgress(this.trainer, this.city);
        this._updateHeaderStats();
        this.renderer.render(this.city, this.boids, this.getInspectedBoid());
        this.chartScreen.render(this.trainer, this.boids);
    }

    getInspectedBoid() {
        if (this.selectedBoid && this.boids.includes(this.selectedBoid)) {
            return this.selectedBoid;
        }
        return this.trainer.getBestLiveBoid(this.boids);
    }

    _bindEvents() {
        // Apply Seed button & input Enter key
        const inputSeed = document.getElementById('input-seed');
        const btnApplySeed = document.getElementById('btn-apply-seed');
        if (btnApplySeed && inputSeed) {
            btnApplySeed.addEventListener('click', () => {
                this.applyCustomSeed(inputSeed.value);
            });
            inputSeed.addEventListener('keydown', e => {
                if (e.key === 'Enter') {
                    this.applyCustomSeed(inputSeed.value);
                }
            });
        }

        // Restart & Clear Local Storage button
        const btnRestartClear = document.getElementById('btn-restart-clear');
        if (btnRestartClear) {
            btnRestartClear.addEventListener('click', () => {
                this.restartAndClearStorage();
            });
        }

        // New Random City (Keep Brains) button
        const btnNewCity = document.getElementById('btn-new-city');
        if (btnNewCity) {
            btnNewCity.addEventListener('click', () => {
                this.generateNewRandomCityKeepBrains();
            });
        }

        // Force Next Generation button
        const btnNextGen = document.getElementById('btn-next-gen');
        if (btnNextGen) {
            btnNextGen.addEventListener('click', () => {
                this.forceNextGeneration();
            });
        }

        // Pause / Resume button
        const btnPause = document.getElementById('btn-pause');
        if (btnPause) {
            btnPause.addEventListener('click', () => {
                this.isPaused = !this.isPaused;
                btnPause.textContent = this.isPaused ? 'Resume' : 'Pause';
            });
        }

        // Simulation Speed selector (1x .. 64x)
        const selectSpeed = document.getElementById('select-speed');
        if (selectSpeed) {
            selectSpeed.addEventListener('change', e => {
                this.simSpeed = parseInt(e.target.value, 10) || 1;
            });
        }

        // Turbo (Decoupled Render) checkbox
        const chkTurbo = document.getElementById('chk-turbo');
        if (chkTurbo) {
            this.turboMode = chkTurbo.checked;
            chkTurbo.addEventListener('change', () => {
                this.turboMode = chkTurbo.checked;
                if (!this.turboMode) {
                    this.renderer.render(this.city, this.boids, this.getInspectedBoid());
                }
            });
        }

        // Generation Duration selector (15s, 30s, 45s, 60s)
        const selectDuration = document.getElementById('select-gen-duration');
        if (selectDuration) {
            selectDuration.addEventListener('change', e => {
                this.trainer.generationDuration = parseInt(e.target.value, 10) || 30;
                StorageManager.saveProgress(this.trainer, this.city);
                this._updateHeaderStats();
            });
        }

        // Boid Types Count selector (1 to 4 types)
        const selectBoidTypes = document.getElementById('select-boid-types');
        if (selectBoidTypes) {
            selectBoidTypes.addEventListener('change', e => {
                const count = Math.max(1, Math.min(4, parseInt(e.target.value, 10) || 2));
                this.trainer.boidTypesCount = count;
                this.boids = this.trainer.createInitialPopulation(this.city);
                this.selectedBoid = null;
                StorageManager.saveProgress(this.trainer, this.city);
                this.renderer.render(this.city, this.boids, this.getInspectedBoid());
                this.debugScreen.render(this.getInspectedBoid(), this.city, this.trainer, this.boids);
            });
        }

        // Population size selector
        const inputPopSize = document.getElementById('input-pop-size');
        if (inputPopSize) {
            inputPopSize.value = this.trainer.populationSize;
            inputPopSize.addEventListener('change', e => {
                const val = Math.max(15, Math.min(120, parseInt(e.target.value, 10) || 55));
                e.target.value = val;
                this.trainer.populationSize = val;
            });
        }

        // Sensor visibility toggle
        const chkAllSensors = document.getElementById('chk-all-sensors');
        if (chkAllSensors) {
            this.renderer.showAllSensors = chkAllSensors.checked;
            const updateSensorToggle = () => {
                this.renderer.showAllSensors = chkAllSensors.checked;
                this.renderer.render(this.city, this.boids, this.getInspectedBoid());
            };
            chkAllSensors.addEventListener('change', updateSensorToggle);
            chkAllSensors.addEventListener('input', updateSensorToggle);
        }

        // View screen switcher tabs
        const tabButtons = document.querySelectorAll('.tab-btn');
        tabButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                tabButtons.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                const viewMode = btn.getAttribute('data-view');
                this._setScreenViewMode(viewMode);
            });
        });

        // Click on simulation canvas to select a specific boid to inspect
        this.simCanvas.addEventListener('click', e => {
            const rect = this.simCanvas.getBoundingClientRect();
            const scaleX = this.simCanvas.width / rect.width;
            const scaleY = this.simCanvas.height / rect.height;
            const mx = (e.clientX - rect.left) * scaleX;
            const my = (e.clientY - rect.top) * scaleY;

            let closest = null;
            let closestDist = 22;
            for (const b of this.boids) {
                const d = Math.hypot(b.x - mx, b.y - my);
                if (d < closestDist) {
                    closestDist = d;
                    closest = b;
                }
            }
            this.selectedBoid = closest;
            this.renderer.render(this.city, this.boids, this.getInspectedBoid());
            this.debugScreen.render(this.getInspectedBoid(), this.city, this.trainer, this.boids);
        });
    }

    _setScreenViewMode(mode) {
        const secSim = document.getElementById('section-simulation');
        const secDebug = document.getElementById('section-debug');
        const secCharts = document.getElementById('section-charts');

        if (mode === 'sim') {
            secSim.classList.remove('hidden');
            secDebug.classList.add('hidden');
            secCharts.classList.add('hidden');
        } else if (mode === 'debug') {
            secSim.classList.add('hidden');
            secDebug.classList.remove('hidden');
            secCharts.classList.add('hidden');
        } else if (mode === 'charts') {
            secSim.classList.add('hidden');
            secDebug.classList.add('hidden');
            secCharts.classList.remove('hidden');
        } else {
            // 'all' - Show all screens
            secSim.classList.remove('hidden');
            secDebug.classList.remove('hidden');
            secCharts.classList.remove('hidden');
        }

        this.debugScreen.render(this.getInspectedBoid(), this.city, this.trainer, this.boids);
        this.chartScreen.render(this.trainer, this.boids);
    }

    _loop(timestamp) {
        const rawDt = Math.min(0.05, (timestamp - this.lastFrameTime) / 1000);
        this.lastFrameTime = timestamp;

        let genCompletedThisFrame = false;

        if (!this.isPaused) {
            const fixedStep = 1 / 60;

            if (this.turboMode) {
                // Turbo: run as many physics steps as fit in the frame's time budget,
                // so any speedup in the simulation directly becomes training throughput.
                const deadline = performance.now() + Simulation.TURBO_FRAME_BUDGET_MS;
                do {
                    for (let s = 0; s < 4; s++) {
                        if (this._stepSimulation(fixedStep)) {
                            genCompletedThisFrame = true;
                        }
                    }
                } while (performance.now() < deadline);
            } else {
                for (let s = 0; s < this.simSpeed; s++) {
                    if (this._stepSimulation(fixedStep)) {
                        genCompletedThisFrame = true;
                    }
                }
            }
        }

        const inspected = this.getInspectedBoid();

        // Decoupled rendering: in Turbo mode, skip per-frame canvas rendering unless a generation just completed
        if (!this.turboMode || genCompletedThisFrame || this.isPaused) {
            this.renderer.render(this.city, this.boids, inspected);
        }

        this.uiRefreshAccumulator += rawDt;
        const refreshInterval = this.turboMode ? 0.45 : 0.2;
        if (this.uiRefreshAccumulator >= refreshInterval || genCompletedThisFrame) {
            this.uiRefreshAccumulator = 0;
            this._updateHeaderStats();
            this.debugScreen.render(inspected, this.city, this.trainer, this.boids);
            this.chartScreen.render(this.trainer, this.boids);
        }

        requestAnimationFrame(ts => this._loop(ts));
    }

    /**
     * Advances physics by dt and returns true if a generation completed on this step.
     */
    _stepSimulation(dt) {
        this.city.update(dt);

        for (let i = 0; i < this.boids.length; i++) {
            this.boids[i].update(dt, this.city, this.boids);
        }

        // Prevent boids from collapsing into each other or getting pushed off-road
        Boid.resolveBoidCollisions(this.boids, this.city);

        this.trainer.generationTimer += dt;
        if (this.trainer.generationTimer >= this.trainer.generationDuration) {
            this.boids = this.trainer.evolveNextGeneration(this.boids, this.city);
            this.selectedBoid = null;
            StorageManager.saveProgress(this.trainer, this.city);
            return true;
        }
        return false;
    }

    _updateHeaderStats() {
        const best = this.trainer.getBestLiveBoid(this.boids);
        const genEl = document.getElementById('stat-gen');
        const timeEl = document.getElementById('stat-timer');
        const boidsEl = document.getElementById('stat-boids');
        const fitEl = document.getElementById('stat-best-fit');
        const allTimeEl = document.getElementById('stat-alltime-fit');

        if (genEl) genEl.textContent = this.trainer.generation;
        if (timeEl) {
            timeEl.textContent = `${this.trainer.generationTimer.toFixed(1)}s / ${this.trainer.generationDuration.toFixed(0)}s`;
        }
        if (boidsEl) boidsEl.textContent = this.boids.length;
        if (fitEl) fitEl.textContent = best ? best.fitness.toFixed(1) : '0.0';
        if (allTimeEl) {
            const bestOverall = Math.max(
                this.trainer.allTimeBestFitness > -1e5 ? this.trainer.allTimeBestFitness : 0,
                best ? best.fitness : 0
            );
            allTimeEl.textContent = bestOverall.toFixed(1);
        }
    }
}
