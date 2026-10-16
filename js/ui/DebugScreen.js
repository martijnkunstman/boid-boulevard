/**
 * DebugScreen - Renders the live Debug Screen including the Neural Network architecture/activation
 * visualizer, front sensor readings, steering & bidirectional throttle gauges, and city/storage diagnostics.
 */
class DebugScreen {
    /**
     * @param {HTMLCanvasElement} nnCanvas
     * @param {HTMLElement} telemetryContainer
     */
    constructor(nnCanvas, telemetryContainer) {
        this.nnCanvas = nnCanvas;
        this.nnCtx = nnCanvas ? nnCanvas.getContext('2d') : null;
        this.telemetryContainer = telemetryContainer;
    }

    /**
     * Updates the Debug Screen with live data from the inspected boid, city, and trainer.
     * @param {Boid|null} boid
     * @param {City} city
     * @param {Trainer} trainer
     * @param {Boid[]} allBoids
     */
    render(boid, city, trainer, allBoids) {
        if (this.nnCtx && boid && boid.brain) {
            this._renderNeuralNetwork(this.nnCtx, boid.brain);
        }
        if (this.telemetryContainer) {
            this._renderTelemetryHTML(boid, city, trainer, allBoids);
        }
    }

    _renderNeuralNetwork(ctx, brain) {
        const w = this.nnCanvas.width;
        const h = this.nnCanvas.height;
        ctx.fillStyle = '#111827';
        ctx.fillRect(0, 0, w, h);

        const layers = brain.layerSizes;
        const numLayers = layers.length;
        const padX = 76;
        const padY = 20;
        const usableW = w - padX * 2;
        const usableH = h - padY * 2;

        // Compute neuron coordinates
        const coords = [];
        for (let l = 0; l < numLayers; l++) {
            const count = layers[l];
            const x = padX + (l / (numLayers - 1)) * usableW;
            const layerCoords = [];
            for (let i = 0; i < count; i++) {
                const y = count === 1
                    ? h * 0.5
                    : padY + (i / (count - 1)) * usableH;
                layerCoords.push({ x, y });
            }
            coords.push(layerCoords);
        }

        // Draw synapses (weights)
        for (let l = 0; l < brain.weights.length; l++) {
            const wMat = brain.weights[l];
            for (let j = 0; j < wMat.length; j++) {
                for (let i = 0; i < wMat[j].length; i++) {
                    const weight = wMat[j][i];
                    const act = brain.lastActivations[l] ? (brain.lastActivations[l][i] || 0) : 0;
                    const signal = weight * act;
                    const mag = Math.min(1, Math.abs(weight) / 1.8);
                    if (mag < 0.08) continue;

                    ctx.strokeStyle = signal >= 0
                        ? `rgba(59, 130, 246, ${(0.12 + mag * 0.55).toFixed(2)})`
                        : `rgba(239, 68, 68, ${(0.12 + mag * 0.55).toFixed(2)})`;
                    ctx.lineWidth = 0.5 + mag * 1.6;
                    ctx.beginPath();
                    ctx.moveTo(coords[l][i].x, coords[l][i].y);
                    ctx.lineTo(coords[l + 1][j].x, coords[l + 1][j].y);
                    ctx.stroke();
                }
            }
        }

        const inputLabels = [
            'Road -75°', 'Road -45°', 'Road -22°', 'Road 0°', 'Road +22°', 'Road +45°', 'Road +75°',
            'Boid -75°', 'Boid -45°', 'Boid -22°', 'Boid 0°', 'Boid +22°', 'Boid +45°', 'Boid +75°',
            'ClosingV', 'Speed', 'HeadingErr'
        ];
        const outputLabels = ['Steer (L/R)', 'Throttle/Brake'];

        // Draw neurons & activations
        ctx.font = '10px monospace';
        for (let l = 0; l < numLayers; l++) {
            const acts = brain.lastActivations[l] || [];
            for (let i = 0; i < coords[l].length; i++) {
                const { x, y } = coords[l][i];
                const val = acts[i] || 0;
                const radius = l === 0 ? 5.2 : (l === numLayers - 1 ? 8.5 : 6.2);

                ctx.beginPath();
                ctx.arc(x, y, radius, 0, Math.PI * 2);
                if (val >= 0) {
                    const intensity = Math.min(255, Math.round(50 + val * 205));
                    ctx.fillStyle = `rgb(30, ${intensity}, 90)`;
                } else {
                    const intensity = Math.min(255, Math.round(50 + Math.abs(val) * 205));
                    ctx.fillStyle = `rgb(${intensity}, 45, 45)`;
                }
                ctx.fill();
                ctx.strokeStyle = '#e5e7eb';
                ctx.lineWidth = 1;
                ctx.stroke();

                if (l === 0 && inputLabels[i]) {
                    ctx.fillStyle = '#d1d5db';
                    ctx.textAlign = 'right';
                    ctx.fillText(inputLabels[i], x - 8, y + 3);
                } else if (l === numLayers - 1 && outputLabels[i]) {
                    ctx.fillStyle = '#fde047';
                    ctx.textAlign = 'left';
                    ctx.fillText(`${outputLabels[i]}: ${val.toFixed(2)}`, x + 11, y + 3);
                }
            }
        }
    }

    _renderTelemetryHTML(boid, city, trainer, allBoids) {
        if (!boid) {
            this.telemetryContainer.innerHTML = '<p>No active boid selected.</p>';
            return;
        }

        const storageInfo = StorageManager.getStorageInfo();
        const diagCount = city.roads.filter(r => r.isDiagonal).length;
        const crossingCount = city.nodes.filter(n => n.roads.length >= 3).length;
        const maxRoadsConnected = Math.max(0, ...city.nodes.map(n => n.roads.length));

        const steerPct = Math.round(((boid.steerInput + 1) * 0.5) * 100);
        const throttleVal = boid.throttleInput;
        const throttleMode = throttleVal >= 0
            ? `Forward Drive (+${(throttleVal * 100).toFixed(0)}%)`
            : `Braking / Reverse (${(throttleVal * 100).toFixed(0)}%)`;

        const roadBars = boid.sensor.roadReadings
            .map((v, idx) => `<span class="sensor-chip" title="Road Ray ${idx + 1}">${(v * 100).toFixed(0)}%</span>`)
            .join(' ');
        const boidBars = boid.sensor.boidReadings
            .map((v, idx) => `<span class="sensor-chip boid-chip" title="Boid Ray ${idx + 1}">${(v * 100).toFixed(0)}%</span>`)
            .join(' ');

        this.telemetryContainer.innerHTML = `
            <div class="debug-grid">
                <div class="debug-box">
                    <h4>Inspected Boid #${boid.id} (${boid.archetype})</h4>
                    <table>
                        <tr><td>Fitness Score:</td><td><strong>${boid.fitness.toFixed(1)}</strong></td></tr>
                        <tr><td>Distance Traveled:</td><td>${boid.distanceTraveled.toFixed(0)} px</td></tr>
                        <tr><td>Speed / Max Speed:</td><td>${boid.speed.toFixed(1)} / ${boid.maxSpeed.toFixed(1)} px/s</td></tr>
                        <tr><td>Accel / Brake Force:</td><td>${boid.acceleration.toFixed(0)} / ${boid.brakingForce.toFixed(0)}</td></tr>
                        <tr><td>Turn Rate / Mass:</td><td>${boid.turnRate.toFixed(2)} rad/s | ${boid.mass.toFixed(2)}x</td></tr>
                        <tr><td>Collisions (Road / Boid):</td><td>${boid.roadCollisions} / ${boid.boidCollisions}</td></tr>
                    </table>
                </div>

                <div class="debug-box">
                    <h4>Neural Network Outputs &amp; Sensors</h4>
                    <table>
                        <tr>
                            <td>Steering Output:</td>
                            <td>
                                <strong>${boid.steerInput.toFixed(2)}</strong>
                                <div class="bar-track">
                                    <div class="bar-fill steer-fill" style="width:${steerPct}%"></div>
                                </div>
                            </td>
                        </tr>
                        <tr>
                            <td>Throttle / Brake:</td>
                            <td>
                                <strong>${throttleMode}</strong>
                                <div class="bar-track">
                                    <div class="bar-fill ${throttleVal >= 0 ? 'throttle-pos' : 'throttle-neg'}"
                                         style="width:${Math.round(Math.abs(throttleVal) * 100)}%"></div>
                                </div>
                            </td>
                        </tr>
                        <tr><td>Road-Border Rays (7):</td><td>${roadBars}</td></tr>
                        <tr><td>Boid-Ahead Rays (7):</td><td>${boidBars}</td></tr>
                        <tr><td>Closing Speed Ahead:</td><td>${(boid.sensor.closingSpeedReading * 100).toFixed(0)}%</td></tr>
                    </table>
                </div>

                <div class="debug-box">
                    <h4>City Map &amp; Persistence</h4>
                    <table>
                        <tr><td>Generation:</td><td><strong>${trainer.generation}</strong> (${trainer.generationTimer.toFixed(1)}s / ${trainer.generationDuration}s)</td></tr>
                        <tr><td>All-Time Best Fitness:</td><td>${trainer.allTimeBestFitness > -1e5 ? trainer.allTimeBestFitness.toFixed(1) : '0.0'}</td></tr>
                        <tr><td>Active Boids:</td><td>${allBoids.length}</td></tr>
                        <tr><td>Nodes / Crossings:</td><td>${city.nodes.length} nodes / ${crossingCount} crossings</td></tr>
                        <tr><td>Max Roads at Any Crossing:</td><td><strong>${maxRoadsConnected}</strong> (Limit &le; 4)</td></tr>
                        <tr><td>Roads (Total / Diagonal):</td><td>${city.roads.length} total / ${diagCount} diagonal</td></tr>
                        <tr><td>LocalStorage Saved:</td><td>${storageInfo.hasData ? `Yes (${(storageInfo.sizeBytes / 1024).toFixed(1)} KB at ${storageInfo.savedAt})` : 'Empty'}</td></tr>
                    </table>
                </div>
            </div>
        `;
    }
}
