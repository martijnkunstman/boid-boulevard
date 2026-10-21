/**
 * SimulationRenderer - Renders the flat 2D two-way city road network with lane markings,
 * crossings, boids, and front sensor rays onto an HTML5 Canvas.
 */
class SimulationRenderer {
    /**
     * @param {HTMLCanvasElement} canvas
     */
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.showSensorsForBest = true;
        this.showAllSensors = false;
        this.showTrails = true;
        this._trailPoint = [0, 0];

        // Map overlay: 'none' | 'traffic' | 'crashes' | 'sdf'
        this.overlayMode = 'none';
        this._heatCanvas = document.createElement('canvas');
        this._sdfCanvas = document.createElement('canvas');
        this._sdfCanvasCity = null;
    }

    static OVERLAYS = {
        traffic: {
            title: 'Traffic density (recent generations)',
            stops: [[37, 99, 235], [34, 197, 94], [250, 204, 21], [220, 38, 38]],
            low: 'quiet',
            high: 'busy',
            gamma: 0.5,
            minVisible: 0.002
        },
        crashes: {
            title: 'Crash locations (recent generations)',
            stops: [[250, 204, 21], [249, 115, 22], [220, 38, 38], [127, 29, 29]],
            low: 'few',
            high: 'many',
            gamma: 0.8,
            minVisible: 0.12
        },
        sdf: {
            title: 'Road distance field (SDF)',
            stops: [[30, 64, 175], [147, 197, 253], [255, 255, 255], [249, 115, 22]],
            low: 'road center',
            high: 'off-road'
        }
    };

    /**
     * Main render entry point.
     * @param {City} city
     * @param {Boid[]} boids
     * @param {Boid|null} inspectedBoid
     */
    render(city, boids, inspectedBoid = null) {
        const ctx = this.ctx;
        const w = this.canvas.width;
        const h = this.canvas.height;

        // 1. Transparent background: only the roads are drawn, the page shows through elsewhere
        ctx.clearRect(0, 0, w, h);

        // 2. Render road curbs/borders and unified asphalt road surfaces
        this._renderRoadBorders(ctx, city);
        this._renderRoadSurfaces(ctx, city);
        this._renderLaneMarkings(ctx, city);

        // 2b. Optional map overlay (heatmaps or distance field) between roads and boids
        if (this.overlayMode === 'traffic') {
            this._renderHeatmap(ctx, city, city.trafficHeat, SimulationRenderer.OVERLAYS.traffic);
        } else if (this.overlayMode === 'crashes') {
            this._renderHeatmap(ctx, city, city.crashHeat, SimulationRenderer.OVERLAYS.crashes);
        } else if (this.overlayMode === 'sdf') {
            this._renderDistanceField(ctx, city);
        }

        // 2d. Fading ghost trails behind the boids
        if (this.showTrails) {
            this._renderTrails(ctx, boids);
        }

        // 3. Render all boids
        for (const boid of boids) {
            this._renderBoid(ctx, boid, boid === inspectedBoid);
        }

        // 4. Render front sensor rays for inspected boid (or all boids)
        if (this.showAllSensors) {
            for (const boid of boids) {
                this._renderBoidSensors(ctx, boid, boid === inspectedBoid);
            }
        } else if (this.showSensorsForBest && inspectedBoid) {
            this._renderBoidSensors(ctx, inspectedBoid, true);
        }

        // 5. Overlay legend on top of everything
        const overlay = SimulationRenderer.OVERLAYS[this.overlayMode];
        if (overlay) {
            this._renderOverlayLegend(ctx, overlay, h);
        }
    }

    /**
     * Returns the [r, g, b] color at t in [0, 1] along evenly spaced color stops.
     */
    static _rampColor(stops, t) {
        const scaled = Math.max(0, Math.min(1, t)) * (stops.length - 1);
        const i = Math.min(stops.length - 2, Math.floor(scaled));
        const f = scaled - i;
        const a = stops[i];
        const b = stops[i + 1];
        return [
            a[0] + (b[0] - a[0]) * f,
            a[1] + (b[1] - a[1]) * f,
            a[2] + (b[2] - a[2]) * f
        ];
    }

    /**
     * Draws a HeatGrid as a smoothly upscaled, semi-transparent color overlay.
     * Values are normalized to the 98th percentile of active cells (so one outlier cell does not
     * wash out the rest), shaped by the overlay's gamma, and hidden below its minVisible level.
     */
    _renderHeatmap(ctx, city, grid, overlay) {
        const max = grid.percentile(0.98);
        if (max <= 0) return;

        const off = this._heatCanvas;
        if (off.width !== grid.cols || off.height !== grid.rows) {
            off.width = grid.cols;
            off.height = grid.rows;
        }
        const offCtx = off.getContext('2d');
        const image = offCtx.createImageData(grid.cols, grid.rows);
        const px = image.data;

        for (let i = 0; i < grid.data.length; i++) {
            const v = Math.min(1, grid.data[i] / max);
            if (v <= overlay.minVisible) continue;
            const t = Math.pow(v, overlay.gamma);
            const [r, g, b] = SimulationRenderer._rampColor(overlay.stops, t);
            const p = i * 4;
            px[p] = r;
            px[p + 1] = g;
            px[p + 2] = b;
            px[p + 3] = 255 * (0.15 + 0.7 * t);
        }
        offCtx.putImageData(image, 0, 0);

        // Upscale into a full-size buffer, then mask it to the road shapes so the smoothed
        // heat does not bleed onto the grass between roads
        const buffer = this._heatBuffer || (this._heatBuffer = document.createElement('canvas'));
        if (buffer.width !== this.canvas.width || buffer.height !== this.canvas.height) {
            buffer.width = this.canvas.width;
            buffer.height = this.canvas.height;
        }
        const bufCtx = buffer.getContext('2d');
        bufCtx.globalCompositeOperation = 'source-over';
        bufCtx.clearRect(0, 0, buffer.width, buffer.height);
        bufCtx.imageSmoothingEnabled = true;
        bufCtx.drawImage(off, 0, 0, grid.cols * grid.cellSize, grid.rows * grid.cellSize);

        bufCtx.globalCompositeOperation = 'destination-in';
        bufCtx.drawImage(this._getRoadMask(city), 0, 0);
        bufCtx.globalCompositeOperation = 'source-over';

        ctx.drawImage(buffer, 0, 0);
    }

    /**
     * Returns an opaque-on-road / transparent-elsewhere mask canvas for the city, cached per city.
     */
    _getRoadMask(city) {
        if (this._roadMaskCity === city) return this._roadMask;

        const mask = this._roadMask || (this._roadMask = document.createElement('canvas'));
        mask.width = this.canvas.width;
        mask.height = this.canvas.height;
        const mCtx = mask.getContext('2d');
        mCtx.lineCap = 'round';
        mCtx.strokeStyle = '#000';
        for (const road of city.roads) {
            mCtx.lineWidth = road.width;
            mCtx.beginPath();
            mCtx.moveTo(road.nodeA.x, road.nodeA.y);
            mCtx.lineTo(road.nodeB.x, road.nodeB.y);
            mCtx.stroke();
        }
        this._roadMaskCity = city;
        return mask;
    }

    /**
     * Draws the city's signed distance field: blue inside roads (darker = further from the border),
     * a white line at the border itself, orange just outside, and contour lines every 4 px.
     * The image is static per city, so it is built once and cached.
     */
    _renderDistanceField(ctx, city) {
        const off = this._sdfCanvas;
        const cols = city.sdfCols;
        const rows = city.sdfRows;

        if (this._sdfCanvasCity !== city) {
            off.width = cols;
            off.height = rows;
            const offCtx = off.getContext('2d');
            const image = offCtx.createImageData(cols, rows);
            const px = image.data;
            const far = City.SDF_FAR_DISTANCE;
            const stops = SimulationRenderer.OVERLAYS.sdf.stops;

            for (let i = 0; i < city.sdf.length; i++) {
                const d = city.sdf[i];
                if (d >= far) continue;
                const p = i * 4;
                let r, g, b, a;
                if (Math.abs(d) < 0.8) {
                    // Road border (zero crossing)
                    r = 255; g = 255; b = 255; a = 255;
                } else if (d < 0) {
                    // On road: 0 = road center, 0.5 = border
                    const depth = Math.min(1, -d / (Road.DEFAULT_WIDTH * 0.5));
                    [r, g, b] = SimulationRenderer._rampColor(stops, 0.5 - depth * 0.5);
                    a = 210;
                } else {
                    // Just outside the road, fading out toward SDF_FAR_DISTANCE
                    [r, g, b] = stops[stops.length - 1];
                    a = 200 * (1 - d / far);
                }
                // Contour lines every 4 px of distance
                if (Math.abs(d) % 4 < 0.5 && Math.abs(d) >= 0.8) {
                    r *= 0.6; g *= 0.6; b *= 0.6;
                }
                px[p] = r;
                px[p + 1] = g;
                px[p + 2] = b;
                px[p + 3] = a;
            }
            offCtx.putImageData(image, 0, 0);
            this._sdfCanvasCity = city;
        }

        ctx.save();
        ctx.imageSmoothingEnabled = true;
        ctx.drawImage(off, 0, 0, cols * City.SDF_CELL_SIZE, rows * City.SDF_CELL_SIZE);
        ctx.restore();
    }

    _renderOverlayLegend(ctx, overlay, canvasHeight) {
        const x = 10;
        const y = canvasHeight - 50;

        ctx.save();
        ctx.font = 'bold 11px sans-serif';
        const barW = Math.max(180, Math.ceil(ctx.measureText(overlay.title).width));
        ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
        ctx.strokeStyle = '#9ca3af';
        ctx.lineWidth = 1;
        ctx.fillRect(x, y, barW + 20, 42);
        ctx.strokeRect(x, y, barW + 20, 42);

        ctx.fillStyle = '#111827';
        ctx.font = 'bold 11px sans-serif';
        ctx.textAlign = 'left';
        ctx.fillText(overlay.title, x + 10, y + 14);

        const gradient = ctx.createLinearGradient(x + 10, 0, x + 10 + barW, 0);
        overlay.stops.forEach((c, i) => {
            gradient.addColorStop(i / (overlay.stops.length - 1), `rgb(${c[0]}, ${c[1]}, ${c[2]})`);
        });
        ctx.fillStyle = gradient;
        ctx.fillRect(x + 10, y + 19, barW, 8);

        ctx.fillStyle = '#4b5563';
        ctx.font = '10px sans-serif';
        ctx.fillText(overlay.low, x + 10, y + 38);
        ctx.textAlign = 'right';
        ctx.fillText(overlay.high, x + 10 + barW, y + 38);
        ctx.restore();
    }

    _renderRoadBorders(ctx, city) {
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.strokeStyle = '#1f2937';

        // Smooth round-capped curb outlines for all roads (no sticking-out node discs)
        for (const road of city.roads) {
            ctx.lineWidth = road.width + 4;
            ctx.beginPath();
            ctx.moveTo(road.nodeA.x, road.nodeA.y);
            ctx.lineTo(road.nodeB.x, road.nodeB.y);
            ctx.stroke();
        }
    }

    _renderRoadSurfaces(ctx, city) {
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.strokeStyle = '#4b5563';

        // Smooth unified asphalt surface for all roads and intersections
        for (const road of city.roads) {
            ctx.lineWidth = road.width;
            ctx.beginPath();
            ctx.moveTo(road.nodeA.x, road.nodeA.y);
            ctx.lineTo(road.nodeB.x, road.nodeB.y);
            ctx.stroke();
        }
    }

    /**
     * Dashed centerline separating the two lanes, stopping at the edge of each crossing.
     */
    _renderLaneMarkings(ctx, city) {
        ctx.save();
        ctx.strokeStyle = 'rgba(243, 244, 246, 0.75)';
        ctx.lineWidth = 1.5;
        ctx.lineCap = 'butt';
        ctx.setLineDash([9, 9]);

        for (const road of city.roads) {
            const startGap = road.nodeA.radius + 4;
            const endGap = road.nodeB.radius + 4;
            if (road.length <= startGap + endGap) continue;
            ctx.beginPath();
            ctx.moveTo(road.nodeA.x + road.dir.x * startGap, road.nodeA.y + road.dir.y * startGap);
            ctx.lineTo(road.nodeB.x - road.dir.x * endGap, road.nodeB.y - road.dir.y * endGap);
            ctx.stroke();
        }
        ctx.restore();
    }

    /**
     * Draws each boid's recent positions as a line in its own color that fades out toward the oldest
     * point and ends at the boid's current position.
     */
    _renderTrails(ctx, boids) {
        const point = this._trailPoint;
        ctx.save();
        ctx.lineCap = 'round';
        for (const boid of boids) {
            const count = boid.trailCount;
            if (count < 2) continue;
            ctx.strokeStyle = boid.color;
            ctx.lineWidth = boid.width * 0.6;

            boid.getTrailPoint(0, point);
            let prevX = point[0];
            let prevY = point[1];
            for (let i = 1; i <= count; i++) {
                let x;
                let y;
                if (i < count) {
                    boid.getTrailPoint(i, point);
                    x = point[0];
                    y = point[1];
                } else {
                    x = boid.x;
                    y = boid.y;
                }
                ctx.globalAlpha = 0.45 * (i / count);
                ctx.beginPath();
                ctx.moveTo(prevX, prevY);
                ctx.lineTo(x, y);
                ctx.stroke();
                prevX = x;
                prevY = y;
            }
        }
        ctx.restore();
    }

    _renderBoid(ctx, boid, isInspected = false) {
        ctx.save();
        ctx.translate(boid.x, boid.y);
        ctx.rotate(boid.heading);

        const halfL = boid.length * 0.5;
        const halfW = boid.width * 0.5;

        // Highlight halo if this is the inspected / best boid
        if (isInspected) {
            ctx.strokeStyle = '#facc15';
            ctx.lineWidth = 2.2;
            ctx.beginPath();
            ctx.arc(0, 0, halfL + 5, 0, Math.PI * 2);
            ctx.stroke();
        }

        // Vehicle body
        ctx.fillStyle = boid.color;
        ctx.strokeStyle = '#111827';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.roundRect(-halfL, -halfW, boid.length, boid.width, 2.5);
        ctx.fill();
        ctx.stroke();

        // Front windshield (indicates front direction)
        ctx.fillStyle = '#bae6fd';
        ctx.fillRect(halfL * 0.15, -halfW + 1.3, halfL * 0.55, boid.width - 2.6);

        // Rear brake lights (glow bright red when throttle < -0.08 i.e. braking/reverse)
        const isBraking = boid.throttleInput < -0.08;
        ctx.fillStyle = isBraking ? '#ff1e1e' : '#7f1d1d';
        ctx.fillRect(-halfL - 0.5, -halfW + 0.8, 2.0, 2.0);
        ctx.fillRect(-halfL - 0.5, halfW - 2.8, 2.0, 2.0);

        ctx.restore();
    }

    _renderBoidSensors(ctx, boid, isPrimary = true) {
        if (!boid.sensor || !boid.sensor.rays) return;
        ctx.save();
        const alpha = isPrimary ? 0.92 : 0.68;

        for (const ray of boid.sensor.rays) {
            let hitX = ray.endX;
            let hitY = ray.endY;
            let color = `rgba(34, 197, 94, ${alpha})`; // Clear = green

            if (ray.boidHit && (!ray.roadHit || ray.boidReading >= ray.roadReading)) {
                hitX = ray.boidHitX;
                hitY = ray.boidHitY;
                color = `rgba(239, 68, 68, ${alpha})`; // Boid in front = red
            } else if (ray.roadHit) {
                hitX = ray.roadHitX;
                hitY = ray.roadHitY;
                color = `rgba(251, 191, 36, ${alpha})`; // Road border = bright amber
            }

            ctx.strokeStyle = color;
            ctx.lineWidth = isPrimary ? 1.8 : 1.25;
            ctx.beginPath();
            ctx.moveTo(ray.ox, ray.oy);
            ctx.lineTo(hitX, hitY);
            ctx.stroke();

            if (ray.roadHit || ray.boidHit) {
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(hitX, hitY, isPrimary ? 3.0 : 2.2, 0, Math.PI * 2);
                ctx.fill();
            }
        }

        ctx.restore();
    }
}
