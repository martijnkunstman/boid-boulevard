/**
 * RoadNode - Represents an intersection or road junction in the city.
 * Includes elevation (height differences) and optional TrafficLight controller.
 */
class RoadNode {
    /**
     * @param {number} id
     * @param {number} x
     * @param {number} y
     * @param {number} elevation - Terrain height at this node (in meters, e.g. 0..14)
     * @param {boolean} isPerimeter - Whether this node lies on the outer city boundary
     */
    constructor(id, x, y, elevation = 0, isPerimeter = false) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.elevation = elevation;
        this.isPerimeter = isPerimeter;
        this.roads = [];
        this.radius = 20;
        this.trafficController = null;
    }

    addRoad(road) {
        if (!this.roads.includes(road)) {
            this.roads.push(road);
            this.radius = Math.max(20, this.roads.length >= 4 ? 23 : 20);
        }
    }

    get position() {
        return new Vector2D(this.x, this.y);
    }

    /**
     * Returns connected roads except the given road, sorted by angle.
     */
    getConnectedRoadsExcept(excludedRoad) {
        return this.roads.filter(r => r !== excludedRoad);
    }
}
