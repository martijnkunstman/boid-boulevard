/**
 * ChartScreen - Renders training progress charts across generations onto HTML5 canvases:
 *   1. Best & Average Fitness per Generation
 *   2. Best & Average Distance Traveled per Generation
 *   3. Average Collisions (Road + Boid) per Generation, plus the benchmark
 *   4. Efficiency: share of the maximum possible distance (top speed x driving time), plus the benchmark
 */
class ChartScreen {
    /**
     * @param {object} canvases - { fitnessCanvas, distanceCanvas, collisionCanvas, efficiencyCanvas }
     */
    constructor(canvases) {
        this.fitnessCanvas = canvases.fitnessCanvas;
        this.distanceCanvas = canvases.distanceCanvas;
        this.collisionCanvas = canvases.collisionCanvas;
        this.efficiencyCanvas = canvases.efficiencyCanvas;
    }

    /**
     * Renders all 4 charts using trainer history + live current generation snapshot.
     * @param {Trainer} trainer
     * @param {Boid[]} liveBoids
     */
    render(trainer, liveBoids = []) {
        const data = this._buildCombinedHistory(trainer, liveBoids);

        if (this.fitnessCanvas) {
            this._drawLineChart(this.fitnessCanvas, data, {
                title: 'Fitness per Generation',
                series: [
                    { key: 'bestFitness', label: 'Best', color: '#16a34a' },
                    { key: 'avgFitness', label: 'Avg', color: '#2563eb' },
                    { key: 'benchmarkFitness', label: 'Benchmark', color: '#f97316' }
                ]
            });
        }

        if (this.distanceCanvas) {
            this._drawLineChart(this.distanceCanvas, data, {
                title: 'Distance per Generation (px)',
                series: [
                    { key: 'bestDistance', label: 'Best', color: '#9333ea' },
                    { key: 'avgDistance', label: 'Avg', color: '#0284c7' },
                    { key: 'benchmarkDistance', label: 'Benchmark', color: '#f97316' }
                ]
            });
        }

        if (this.collisionCanvas) {
            this._drawLineChart(this.collisionCanvas, data, {
                title: 'Collisions per Boid (Lower is Better)',
                series: [
                    { key: 'avgCollisions', label: 'Avg', color: '#dc2626' },
                    { key: 'benchmarkCollisions', label: 'Benchmark', color: '#f97316' }
                ],
                minY: 0
            });
        }

        if (this.efficiencyCanvas) {
            this._drawLineChart(this.efficiencyCanvas, data, {
                title: 'Efficiency (% of max possible distance)',
                series: [
                    { key: 'avgEfficiency', label: 'Avg', color: '#d97706' },
                    { key: 'benchmarkEfficiency', label: 'Benchmark', color: '#f97316' }
                ],
                minY: 0,
                maxY: 100
            });
        }
    }

    _buildCombinedHistory(trainer, liveBoids) {
        const list = [...trainer.history];
        if (liveBoids && liveBoids.length > 0) {
            let bestFit = -Infinity;
            let bestDist = 0;
            let sumFit = 0;
            let sumDist = 0;
            let sumCol = 0;
            let sumSpeed = 0;

            for (const b of liveBoids) {
                if (b.fitness > bestFit) bestFit = b.fitness;
                if (b.distanceTraveled > bestDist) bestDist = b.distanceTraveled;
                sumFit += b.fitness;
                sumDist += b.distanceTraveled;
                sumCol += (b.roadCollisions + b.boidCollisions);
                sumSpeed += Math.max(0, b.speed);
            }

            const n = liveBoids.length;
            // Live efficiency over the time driven so far (skipped right after a generation starts)
            const elapsed = trainer.generationTimer;
            const liveEfficiency = elapsed >= 1 ? Trainer.efficiencyPercent(liveBoids, elapsed) : undefined;

            list.push({
                generation: trainer.generation,
                bestFitness: Number(bestFit.toFixed(2)),
                avgFitness: Number((sumFit / n).toFixed(2)),
                bestDistance: Number(bestDist.toFixed(1)),
                avgDistance: Number((sumDist / n).toFixed(1)),
                avgCollisions: Number((sumCol / n).toFixed(2)),
                avgSpeed: Number((sumSpeed / n).toFixed(1)),
                avgEfficiency: liveEfficiency === undefined ? undefined : Number(liveEfficiency.toFixed(1)),
                isLive: true
            });
        }
        return list;
    }

    _drawLineChart(canvas, data, config) {
        const ctx = canvas.getContext('2d');
        const w = canvas.width;
        const h = canvas.height;

        ctx.fillStyle = '#ffffff';
        ctx.fillRect(0, 0, w, h);

        const padLeft = 48;
        const padRight = 18;
        const padTop = 30;
        const padBottom = 28;
        const plotW = w - padLeft - padRight;
        const plotH = h - padTop - padBottom;

        // Title
        ctx.fillStyle = '#111827';
        ctx.font = 'bold 12px sans-serif';
        ctx.textAlign = 'left';
        ctx.fillText(config.title, padLeft, 18);

        // Legend
        let legendX = w - padRight;
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'right';
        for (let s = config.series.length - 1; s >= 0; s--) {
            const ser = config.series[s];
            ctx.fillStyle = ser.color;
            ctx.fillText(`● ${ser.label}`, legendX, 18);
            legendX -= ctx.measureText(`● ${ser.label}`).width + 14;
        }

        // Determine Y axis range
        let minVal = config.minY !== undefined ? config.minY : Infinity;
        let maxVal = config.maxY !== undefined ? config.maxY : -Infinity;

        // Points without a value for a series (e.g. benchmark on older history or the live point) are skipped
        const hasValue = (item, key) => Number.isFinite(item[key]);

        for (const item of data) {
            for (const ser of config.series) {
                if (!hasValue(item, ser.key)) continue;
                const v = item[ser.key];
                if (config.minY === undefined && v < minVal) minVal = v;
                if (config.maxY === undefined && v > maxVal) maxVal = v;
            }
        }

        if (!isFinite(minVal)) minVal = 0;
        if (!isFinite(maxVal)) maxVal = 10;
        if (Math.abs(maxVal - minVal) < 1e-3) {
            maxVal = minVal + 10;
            if (minVal > 0) minVal = 0;
        }

        // Draw grid lines & Y labels
        const gridSteps = 4;
        ctx.font = '10px monospace';
        ctx.textAlign = 'right';
        for (let g = 0; g <= gridSteps; g++) {
            const frac = g / gridSteps;
            const y = padTop + plotH - frac * plotH;
            const val = minVal + frac * (maxVal - minVal);

            ctx.strokeStyle = '#e5e7eb';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(padLeft, y);
            ctx.lineTo(w - padRight, y);
            ctx.stroke();

            ctx.fillStyle = '#4b5563';
            ctx.fillText(val.toFixed(0), padLeft - 6, y + 3);
        }

        // Axes border
        ctx.strokeStyle = '#9ca3af';
        ctx.lineWidth = 1;
        ctx.strokeRect(padLeft, padTop, plotW, plotH);

        if (data.length === 0) return;

        // X position by generation number (not array index), so a compacted history
        // with uneven point spacing still shows the whole run to scale
        const firstGen = data[0].generation;
        const lastGen = data[data.length - 1].generation;
        const genSpan = lastGen - firstGen;
        const xForPoint = i => (genSpan > 0
            ? padLeft + ((data[i].generation - firstGen) / genSpan) * plotW
            : padLeft + plotW * 0.5);

        // Draw each series
        for (const ser of config.series) {
            ctx.strokeStyle = ser.color;
            ctx.lineWidth = 2;
            ctx.beginPath();

            let started = false;
            for (let i = 0; i < data.length; i++) {
                if (!hasValue(data[i], ser.key)) continue;
                const x = xForPoint(i);
                const val = data[i][ser.key];
                const normY = (val - minVal) / (maxVal - minVal);
                const y = padTop + plotH - Math.max(0, Math.min(1, normY)) * plotH;

                if (!started) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
                started = true;
            }
            ctx.stroke();

            // Draw data points
            for (let i = 0; i < data.length; i++) {
                if (!hasValue(data[i], ser.key)) continue;
                const x = xForPoint(i);
                const val = data[i][ser.key];
                const normY = (val - minVal) / (maxVal - minVal);
                const y = padTop + plotH - Math.max(0, Math.min(1, normY)) * plotH;

                ctx.fillStyle = data[i].isLive ? '#f59e0b' : ser.color;
                ctx.beginPath();
                ctx.arc(x, y, data[i].isLive ? 3.5 : 2.3, 0, Math.PI * 2);
                ctx.fill();
            }
        }

        // X-axis generation labels
        ctx.fillStyle = '#374151';
        ctx.font = '10px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`Gen ${firstGen}`, padLeft + 16, h - 8);
        if (data.length > 1) {
            ctx.fillText(`Gen ${lastGen} (live)`, w - padRight - 34, h - 8);
        }
    }
}
