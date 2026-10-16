/**
 * SimulationRenderer - Renders the flat 2D city road network, stoplight crossings,
 * boids, and front sensor rays onto an HTML5 Canvas.
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
    }

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

        // 1. Background & rectangular city boundary enclosure
        ctx.fillStyle = '#e5e7eb';
        ctx.fillRect(0, 0, w, h);

        this._renderCityBounds(ctx, city);

        // 2. Render road curbs/outlines and asphalt surfaces
        this._renderRoadBorders(ctx, city);
        this._renderRoadSurfaces(ctx, city);

        // 3. Render stoplights (stop lines & signal boxes) at crossings
        this._renderTrafficLights(ctx, city);

        // 4. Render all boids
        for (const boid of boids) {
            this._renderBoid(ctx, boid, boid === inspectedBoid);
        }

        // 5. Render front sensor rays for inspected boid (or all boids)
        if (this.showAllSensors) {
            for (const boid of boids) {
                this._renderBoidSensors(ctx, boid, boid === inspectedBoid);
            }
        } else if (this.showSensorsForBest && inspectedBoid) {
            this._renderBoidSensors(ctx, inspectedBoid, true);
        }
    }

    _renderCityBounds(ctx, city) {
        const b = city.bounds;

        // City block interior turf inside the rectangular boundary
        ctx.fillStyle = '#dbe4d0';
        ctx.fillRect(b.left - 24, b.top - 24, b.width + 48, b.height + 48);

        // Rectangular town boundary limit frame (showing no roads exit this rectangle)
        ctx.save();
        ctx.strokeStyle = '#6b7280';
        ctx.lineWidth = 2;
        ctx.setLineDash([8, 6]);
        ctx.strokeRect(b.left - 22, b.top - 22, b.width + 44, b.height + 44);
        ctx.restore();
    }

    _renderRoadBorders(ctx, city) {
        ctx.lineCap = 'round';
        ctx.strokeStyle = '#1f2937';

        // Curb outlines for all roads
        for (const road of city.roads) {
            ctx.lineWidth = road.width + 4;
            ctx.beginPath();
            ctx.moveTo(road.nodeA.x, road.nodeA.y);
            ctx.lineTo(road.nodeB.x, road.nodeB.y);
            ctx.stroke();
        }

        // Curb outlines for intersection nodes
        ctx.fillStyle = '#1f2937';
        for (const node of city.nodes) {
            ctx.beginPath();
            ctx.arc(node.x, node.y, node.radius + 2, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    _renderRoadSurfaces(ctx, city) {
        ctx.lineCap = 'round';
        ctx.strokeStyle = '#3f4652';

        // Asphalt surface for roads
        for (const road of city.roads) {
            ctx.lineWidth = road.width;
            ctx.beginPath();
            ctx.moveTo(road.nodeA.x, road.nodeA.y);
            ctx.lineTo(road.nodeB.x, road.nodeB.y);
            ctx.stroke();
        }

        // Intersection node asphalt surfaces
        ctx.fillStyle = '#3f4652';
        for (const node of city.nodes) {
            ctx.beginPath();
            ctx.arc(node.x, node.y, node.radius, 0, Math.PI * 2);
            ctx.fill();
        }

        // Center dashed lane dividers on roads
        ctx.save();
        ctx.strokeStyle = '#d1d5db';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([8, 8]);
        for (const road of city.roads) {
            const start = road.getPointAt(0.12);
            const end = road.getPointAt(0.88);
            ctx.beginPath();
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
        }
        ctx.restore();
    }

    _renderTrafficLights(ctx, city) {
        for (const node of city.nodes) {
            if (!node.trafficController) continue;

            for (const light of node.trafficController.lights) {
                const color =
                    light.state === 'GREEN' ? '#22c55e' :
                    light.state === 'YELLOW' ? '#eab308' : '#ef4444';

                // Draw colored stop line across the incoming lane
                ctx.save();
                ctx.strokeStyle = color;
                ctx.lineWidth = 3.2;
                ctx.beginPath();
                ctx.moveTo(light.stopLineStart.x, light.stopLineStart.y);
                ctx.lineTo(light.stopLineEnd.x, light.stopLineEnd.y);
                ctx.stroke();

                // Draw traffic signal head on the right-hand curb
                ctx.fillStyle = '#111827';
                ctx.beginPath();
                ctx.arc(light.signalPos.x, light.signalPos.y, 4.5, 0, Math.PI * 2);
                ctx.fill();

                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(light.signalPos.x, light.signalPos.y, 3.0, 0, Math.PI * 2);
                ctx.fill();
                ctx.restore();
            }
        }
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
        const alpha = isPrimary ? 0.78 : 0.25;

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
                color = `rgba(245, 158, 11, ${alpha})`; // Side of road = amber
            }

            ctx.strokeStyle = color;
            ctx.lineWidth = isPrimary ? 1.4 : 0.8;
            ctx.beginPath();
            ctx.moveTo(ray.ox, ray.oy);
            ctx.lineTo(hitX, hitY);
            ctx.stroke();

            if (isPrimary && (ray.roadHit || ray.boidHit)) {
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(hitX, hitY, 2.6, 0, Math.PI * 2);
                ctx.fill();
            }
        }

        ctx.restore();
    }
}
