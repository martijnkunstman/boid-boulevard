/**
 * TrafficLight & IntersectionController - Manages stoplights at road crossings.
 */
class TrafficLight {
    /**
     * @param {RoadNode} node - Intersection node this light guards
     * @param {Road} road - Road approaching this node
     * @param {number} phaseGroup - Phase index in the intersection controller
     */
    constructor(node, road, phaseGroup = 0) {
        this.node = node;
        this.road = road;
        this.phaseGroup = phaseGroup;
        this.state = 'RED'; // 'GREEN' | 'YELLOW' | 'RED'

        // Compute geometry for stop line and signal head on the approaching lane
        this._computeGeometry();
    }

    _computeGeometry() {
        const other = this.road.getOtherNode(this.node);
        // Unit vector pointing FROM intersection node TOWARD the other node
        const awayDir = new Vector2D(other.x - this.node.x, other.y - this.node.y).normalize();
        // Unit vector pointing INTO the intersection node (approaching direction)
        this.approachDir = awayDir.scale(-1);
        // Right-hand side vector for approaching traffic
        const rightNormal = new Vector2D(-this.approachDir.y, this.approachDir.x);

        const stopDist = this.node.radius + 7;
        const roadCenterStop = new Vector2D(
            this.node.x + awayDir.x * stopDist,
            this.node.y + awayDir.y * stopDist
        );

        const halfWidth = this.road.width * 0.5;
        // Stop line spans across the right-hand (incoming) lane
        this.stopLineStart = roadCenterStop.clone();
        this.stopLineEnd = roadCenterStop.add(rightNormal.scale(halfWidth - 1));
        this.stopLineCenter = roadCenterStop.add(rightNormal.scale(halfWidth * 0.5));

        // Signal box positioned on the right-hand curb beside the stop line
        this.signalPos = roadCenterStop.add(rightNormal.scale(halfWidth + 5));
    }

    /**
     * Numerical urgency value for sensors:
     * 0.0 = GREEN (go)
     * 0.55 = YELLOW (prepare to stop)
     * 1.0 = RED (stop)
     */
    getSensorSignalValue() {
        if (this.state === 'RED') return 1.0;
        if (this.state === 'YELLOW') return 0.55;
        return 0.0;
    }
}

class IntersectionController {
    /**
     * @param {RoadNode} node
     */
    constructor(node) {
        this.node = node;
        this.lights = [];
        this.phases = [];
        this.currentPhase = 0;
        this.phaseSubState = 'GREEN'; // 'GREEN' | 'YELLOW' | 'ALL_RED'
        this.greenDuration = 5.5 + Math.random() * 2.5;
        this.yellowDuration = 1.4;
        this.allRedDuration = 0.5;
        this.timer = Math.random() * this.greenDuration;

        this._buildPhases();
    }

    _buildPhases() {
        const roads = this.node.roads;
        if (roads.length === 0) return;

        // Sort roads by angle around the node
        const entries = roads.map(road => {
            const other = road.getOtherNode(this.node);
            const angle = Math.atan2(other.y - this.node.y, other.x - this.node.x);
            return { road, angle, assigned: false };
        });

        this.phases = [];
        this.lights = [];

        for (let i = 0; i < entries.length; i++) {
            if (entries[i].assigned) continue;
            entries[i].assigned = true;
            const phaseIdx = this.phases.length;
            const phaseLights = [];

            const lightA = new TrafficLight(this.node, entries[i].road, phaseIdx);
            phaseLights.push(lightA);
            this.lights.push(lightA);

            // Pair with the most directly opposite road (within ~38 degrees of 180 deg)
            let bestPartner = -1;
            let bestDiff = 0.68; // ~39 degrees in radians
            for (let j = i + 1; j < entries.length; j++) {
                if (entries[j].assigned) continue;
                const oppositeAngle = Vector2D.normalizeAngle(entries[i].angle + Math.PI);
                const diff = Math.abs(Vector2D.normalizeAngle(entries[j].angle - oppositeAngle));
                if (diff < bestDiff) {
                    bestDiff = diff;
                    bestPartner = j;
                }
            }

            if (bestPartner !== -1) {
                entries[bestPartner].assigned = true;
                const lightB = new TrafficLight(this.node, entries[bestPartner].road, phaseIdx);
                phaseLights.push(lightB);
                this.lights.push(lightB);
            }

            this.phases.push(phaseLights);
        }

        this.currentPhase = Math.floor(Math.random() * this.phases.length);
        this._syncLightStates();
    }

    update(dt) {
        if (this.phases.length === 0) return;
        this.timer += dt;

        if (this.phaseSubState === 'GREEN' && this.timer >= this.greenDuration) {
            this.timer = 0;
            this.phaseSubState = 'YELLOW';
            this._syncLightStates();
        } else if (this.phaseSubState === 'YELLOW' && this.timer >= this.yellowDuration) {
            this.timer = 0;
            this.phaseSubState = 'ALL_RED';
            this._syncLightStates();
        } else if (this.phaseSubState === 'ALL_RED' && this.timer >= this.allRedDuration) {
            this.timer = 0;
            this.currentPhase = (this.currentPhase + 1) % this.phases.length;
            this.phaseSubState = 'GREEN';
            this._syncLightStates();
        }
    }

    _syncLightStates() {
        for (let p = 0; p < this.phases.length; p++) {
            const state = (p === this.currentPhase)
                ? (this.phaseSubState === 'ALL_RED' ? 'RED' : this.phaseSubState)
                : 'RED';
            for (const light of this.phases[p]) {
                light.state = state;
            }
        }
    }

    /**
     * Get the traffic light controlling traffic arriving on `road` into `this.node`.
     * @param {Road} road
     * @returns {TrafficLight|null}
     */
    getLightForRoad(road) {
        for (const light of this.lights) {
            if (light.road === road) return light;
        }
        return null;
    }
}
