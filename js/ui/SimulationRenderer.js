/**
 * SimulationRenderer - Renders the 2D city road network, elevation contours,
 * stoplight crossings, grade-separated bridges, boids, and sensor rays onto an HTML5 Canvas.
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
        this.showElevationLabels = true;
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

        this._renderCityBoundsAndTerrain(ctx, city);

        // Separate ground roads vs bridge roads
        const groundRoads = [];
        const bridgeRoads = [];
        for (const road of city.roads) {
            if (road.isBridge) {
                bridgeRoads.push(road);
            } else {
                groundRoads.push(road);
            }
        }

        // 2. Render ground-level road curbs/outlines and intersections
        this._renderGroundRoadBorders(ctx, city, groundRoads);
        this._renderGroundRoadSurfaces(ctx, city, groundRoads);

        // 3. Render ground-level boids (layer === 0)
        for (const boid of boids) {
            if (boid.layer === 0) {
                this._renderBoid(ctx, boid, boid === inspectedBoid);
            }
        }

        // 4. Render Bridges (layer === 1): drop shadow, piers, bridge deck, guardrails, lane markings
        for (const bridge of bridgeRoads) {
            this._renderBridge(ctx, bridge);
        }

        // 5. Render stoplights (stop lines & signal boxes) at crossings
        this._renderTrafficLights(ctx, city);

        // 6. Render elevated bridge-level boids (layer === 1)
        for (const boid of boids) {
            if (boid.layer === 1) {
                this._renderBoid(ctx, boid, boid === inspectedBoid);
            }
        }

        // 7. Render front sensor rays for inspected boid (or all boids)
        if (this.showAllSensors) {
            for (const boid of boids) {
                this._renderBoidSensors(ctx, boid, boid === inspectedBoid);
            }
        } else if (this.showSensorsForBest && inspectedBoid) {
            this._renderBoidSensors(ctx, inspectedBoid, true);
        }

        // 8. Optional elevation badges on nodes & bridges
        if (this.showElevationLabels) {
            this._renderElevationLabels(ctx, city, bridgeRoads);
        }
    }

    _renderCityBoundsAndTerrain(ctx, city) {
        const b = city.bounds;

        // City block interior turf inside the rectangular boundary
        ctx.fillStyle = '#dbe4d0';
        ctx.fillRect(b.left - 24, b.top - 24, b.width + 48, b.height + 48);

        // Render elevation hill contour rings (height differences)
        for (const hill of city.hills) {
            const rings = 4;
            for (let r = rings; r >= 1; r--) {
                const frac = r / rings;
                const rad = hill.radius * frac;
                ctx.beginPath();
                ctx.arc(hill.x, hill.y, rad, 0, Math.PI * 2);
                const alpha = 0.08 + (1 - frac) * 0.09;
                ctx.fillStyle = `rgba(130, 155, 95, ${alpha.toFixed(3)})`;
                ctx.fill();
                ctx.strokeStyle = 'rgba(105, 128, 78, 0.25)';
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        }

        // Rectangular town boundary limit frame (showing no roads exit this rectangle)
        ctx.save();
        ctx.strokeStyle = '#6b7280';
        ctx.lineWidth = 2;
        ctx.setLineDash([8, 6]);
        ctx.strokeRect(b.left - 22, b.top - 22, b.width + 44, b.height + 44);
        ctx.restore();
    }

    _renderGroundRoadBorders(ctx, city, groundRoads) {
        ctx.lineCap = 'round';

        // Curb outlines for ground roads
        for (const road of groundRoads) {
            ctx.strokeStyle = '#1f2937';
            ctx.lineWidth = road.width + 4;
            ctx.beginPath();
            ctx.moveTo(road.nodeA.x, road.nodeA.y);
            ctx.lineTo(road.nodeB.x, road.nodeB.y);
            ctx.stroke();
        }

        // Curb outlines for bridge entry/exit ramps near nodes
        for (const road of city.roads) {
            if (!road.isBridge) continue;
            ctx.strokeStyle = '#1f2937';
            ctx.lineWidth = road.width + 4;
            ctx.beginPath();
            const pA = road.getPointAt(0.2);
            ctx.moveTo(road.nodeA.x, road.nodeA.y);
            ctx.lineTo(pA.x, pA.y);
            const pB = road.getPointAt(0.8);
            ctx.moveTo(pB.x, pB.y);
            ctx.lineTo(road.nodeB.x, road.nodeB.y);
            ctx.stroke();
        }

        // Curb outlines for intersection nodes
        for (const node of city.nodes) {
            ctx.fillStyle = '#1f2937';
            ctx.beginPath();
            ctx.arc(node.x, node.y, node.radius + 2, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    _renderGroundRoadSurfaces(ctx, city, groundRoads) {
        // Asphalt surface shaded subtly by elevation (higher terrain is slightly lighter/warmer)
        for (const road of groundRoads) {
            const avgElev = (road.nodeA.elevation + road.nodeB.elevation) * 0.5;
            const shade = Math.round(58 + Math.min(28, avgElev * 2.0));
            ctx.strokeStyle = `rgb(${shade}, ${shade + 3}, ${shade + 8})`;
            ctx.lineWidth = road.width;
            ctx.lineCap = 'round';
            ctx.beginPath();
            ctx.moveTo(road.nodeA.x, road.nodeA.y);
            ctx.lineTo(road.nodeB.x, road.nodeB.y);
            ctx.stroke();
        }

        // Intersection node asphalt surfaces
        for (const node of city.nodes) {
            const shade = Math.round(58 + Math.min(28, node.elevation * 2.0));
            ctx.fillStyle = `rgb(${shade}, ${shade + 3}, ${shade + 8})`;
            ctx.beginPath();
            ctx.arc(node.x, node.y, node.radius, 0, Math.PI * 2);
            ctx.fill();
        }

        // Center dashed lane dividers on ground roads
        ctx.save();
        ctx.strokeStyle = '#d1d5db';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([7, 7]);
        for (const road of groundRoads) {
            const start = road.getPointAt(0.12);
            const end = road.getPointAt(0.88);
            ctx.beginPath();
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
            ctx.stroke();
        }
        ctx.restore();
    }

    _renderBridge(ctx, bridge) {
        ctx.save();

        // 1. Cast drop shadow onto the ground/road below
        ctx.strokeStyle = 'rgba(0, 0, 0, 0.32)';
        ctx.lineWidth = bridge.width + 8;
        ctx.lineCap = 'butt';
        const pStart = bridge.getPointAt(0.14);
        const pEnd = bridge.getPointAt(0.86);
        ctx.beginPath();
        ctx.moveTo(pStart.x + 5, pStart.y + 6);
        ctx.lineTo(pEnd.x + 5, pEnd.y + 6);
        ctx.stroke();

        // 2. Bridge support piers at t=0.28 and t=0.72
        for (const tPier of [0.26, 0.74]) {
            const c = bridge.getPointAt(tPier);
            const n = bridge.rightNormal;
            const hw = bridge.halfWidth + 4;
            ctx.strokeStyle = '#374151';
            ctx.lineWidth = 5;
            ctx.beginPath();
            ctx.moveTo(c.x - n.x * hw, c.y - n.y * hw);
            ctx.lineTo(c.x + n.x * hw, c.y + n.y * hw);
            ctx.stroke();
        }

        // 3. Bridge outer concrete guardrail border
        ctx.strokeStyle = '#9ca3af';
        ctx.lineWidth = bridge.width + 6;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(bridge.nodeA.x, bridge.nodeA.y);
        ctx.lineTo(bridge.nodeB.x, bridge.nodeB.y);
        ctx.stroke();

        // 4. Elevated bridge asphalt deck (slightly brighter gray-blue to show elevated height)
        ctx.strokeStyle = '#525b6c';
        ctx.lineWidth = bridge.width;
        ctx.beginPath();
        ctx.moveTo(bridge.nodeA.x, bridge.nodeA.y);
        ctx.lineTo(bridge.nodeB.x, bridge.nodeB.y);
        ctx.stroke();

        // 5. Bridge side steel guardrails along the elevated span
        const rStart = bridge.getPointAt(0.14);
        const rEnd = bridge.getPointAt(0.86);
        const offset = bridge.halfWidth + 1.5;
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2.2;
        for (const side of [-1, 1]) {
            const ox = bridge.rightNormal.x * offset * side;
            const oy = bridge.rightNormal.y * offset * side;
            ctx.beginPath();
            ctx.moveTo(rStart.x + ox, rStart.y + oy);
            ctx.lineTo(rEnd.x + ox, rEnd.y + oy);
            ctx.stroke();
        }

        // 6. Bridge center lane divider
        ctx.strokeStyle = '#fde68a';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([7, 6]);
        ctx.beginPath();
        ctx.moveTo(rStart.x, rStart.y);
        ctx.lineTo(rEnd.x, rEnd.y);
        ctx.stroke();

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

        // Slight visual scale boost when elevated on a bridge
        const scale = 1 + Math.min(0.14, boid.elevation * 0.008);
        ctx.scale(scale, scale);
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
            // Draw base ray up to closest hit
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

    _renderElevationLabels(ctx, city, bridgeRoads) {
        ctx.save();
        ctx.font = '10px monospace';
        ctx.textAlign = 'center';

        // Node terrain elevation labels
        for (const node of city.nodes) {
            if (node.elevation >= 1.5) {
                ctx.fillStyle = 'rgba(17, 24, 39, 0.75)';
                ctx.fillText(`${node.elevation.toFixed(1)}m`, node.x, node.y - node.radius - 5);
            }
        }

        // Bridge overpass peak labels
        for (const bridge of bridgeRoads) {
            const mid = bridge.getPointAt(0.5);
            const z = bridge.getElevationAt(0.5);
            ctx.fillStyle = '#111827';
            ctx.fillRect(mid.x - 24, mid.y - 18, 48, 13);
            ctx.fillStyle = '#fde047';
            ctx.fillText(`BR ${z.toFixed(1)}m`, mid.x, mid.y - 8);
        }

        ctx.restore();
    }
}
