/**
 * Simulation - Main application orchestrator connecting City, Boids, Trainer,
 * StorageManager, SimulationRenderer, DebugScreen, and ChartScreen.
 */
class Simulation {
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
            complianceCanvas: document.getElementById('chart-compliance')
        });

        this.trainer = new Trainer({
            populationSize: 55,
            generationDuration: 18.0,
            mutationRate: 0.14,
            mutationStrength: 0.24
        });

        this.city = null;
        this.boids = [];
        this.selectedBoid = null;

        this.isPaused = false;
        this.simSpeed = 1;
        this.lastFrameTime = performance.now();
        this.uiRefreshAccumulator = 0;

        this._initFromStorageOrRandom();
        this._bindEvents();
        this._updateHeaderStats();
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
            }
        }

        if (!this.city) {
            this.city = CityGenerator.generate(this.simCanvas.width, this.simCanvas.height);
        }

        this.boids = this.trainer.createInitialPopulation(this.city);
        StorageManager.saveProgress(this.trainer, this.city);
    }

    /**
     * Clears localStorage and restarts the entire simulation with random roads and random boids.
     */
    restartAndClearStorage() {
        StorageManager.clearProgress();
        const popInput = document.getElementById('input-pop-size');
        const popSize = popInput ? parseInt(popInput.value, 10) || 55 : 55;

        this.trainer = new Trainer({
            populationSize: popSize,
            generationDuration: 18.0,
            mutationRate: 0.14,
            mutationStrength: 0.24
        });

        this.city = CityGenerator.generate(this.simCanvas.width, this.simCanvas.height);
        this.boids = this.trainer.createInitialPopulation(this.city);
        this.selectedBoid = null;

        this._updateHeaderStats();
        this.renderer.render(this.city, this.boids, this.getInspectedBoid());
        this.debugScreen.render(this.getInspectedBoid(), this.city, this.trainer, this.boids);
        this.chartScreen.render(this.trainer, this.boids);
    }

    /**
     * Generates a new random city while keeping trained neural network progress.
     */
    generateNewRandomCityKeepBrains() {
        this.city = CityGenerator.generate(this.simCanvas.width, this.simCanvas.height);
        this.boids = this.trainer.createInitialPopulation(this.city);
        this.selectedBoid = null;
        StorageManager.saveProgress(this.trainer, this.city);
        this._updateHeaderStats();
    }

    /**
     * Immediately finishes the current generation and evolves the next one.
     */
    forceNextGeneration() {
        this.boids = this.trainer.evolveNextGeneration(this.boids, this.city);
        this.selectedBoid = null;
        StorageManager.saveProgress(this.trainer, this.city);
        this._updateHeaderStats();
        this.chartScreen.render(this.trainer, this.boids);
    }

    getInspectedBoid() {
        if (this.selectedBoid && this.boids.includes(this.selectedBoid)) {
            return this.selectedBoid;
        }
        return this.trainer.getBestLiveBoid(this.boids);
    }

    _bindEvents() {
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

        // Simulation Speed selector
        const selectSpeed = document.getElementById('select-speed');
        if (selectSpeed) {
            selectSpeed.addEventListener('change', e => {
                this.simSpeed = parseInt(e.target.value, 10) || 1;
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

        // Refresh debug & charts immediately when switching screens
        this.debugScreen.render(this.getInspectedBoid(), this.city, this.trainer, this.boids);
        this.chartScreen.render(this.trainer, this.boids);
    }

    _loop(timestamp) {
        const rawDt = Math.min(0.05, (timestamp - this.lastFrameTime) / 1000);
        this.lastFrameTime = timestamp;

        if (!this.isPaused) {
            const fixedStep = 1 / 60;
            const steps = this.simSpeed;

            for (let s = 0; s < steps; s++) {
                this._stepSimulation(fixedStep);
            }
        }

        const inspected = this.getInspectedBoid();
        this.renderer.render(this.city, this.boids, inspected);

        this.uiRefreshAccumulator += rawDt;
        if (this.uiRefreshAccumulator >= 0.2) {
            this.uiRefreshAccumulator = 0;
            this._updateHeaderStats();
            this.debugScreen.render(inspected, this.city, this.trainer, this.boids);
            this.chartScreen.render(this.trainer, this.boids);
        }

        requestAnimationFrame(ts => this._loop(ts));
    }

    _stepSimulation(dt) {
        this.city.update(dt);

        for (let i = 0; i < this.boids.length; i++) {
            this.boids[i].update(dt, this.city, this.boids);
        }

        // Prevent boids from collapsing into each other
        Boid.resolveBoidCollisions(this.boids);

        this.trainer.generationTimer += dt;
        if (this.trainer.generationTimer >= this.trainer.generationDuration) {
            this.boids = this.trainer.evolveNextGeneration(this.boids, this.city);
            this.selectedBoid = null;
            StorageManager.saveProgress(this.trainer, this.city);
            this.chartScreen.render(this.trainer, this.boids);
        }
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
