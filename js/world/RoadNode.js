/**
 * RoadNode - Represents an intersection or road junction in the city.
 * Every crossing has at most 4 connected roads.
 */
class RoadNode {
    /**
     * @param {number} id
     * @param {number} x
     * @param {number} y
     * @param {boolean} isPerimeter - Whether this node lies on the outer city boundary
     */
    constructor(id, x, y, isPerimeter = false) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.isPerimeter = isPerimeter;
        this.roads = [];
        this.radius = 18;
    }

    /**
     * Adds a road to this node if it has fewer than 4 roads connected.
     * @param {Road} road
     * @returns {boolean}
     */
    addRoad(road) {
        if (this.roads.length >= 4) return false;
        if (!this.roads.includes(road)) {
            this.roads.push(road);
            this.radius = road.halfWidth || 18;
            return true;
        }
        return false;
    }

    get position() {
        return new Vector2D(this.x, this.y);
    }

    /**
     * Returns connected roads except the given road.
     */
    getConnectedRoadsExcept(excludedRoad) {
        return this.roads.filter(r => r !== excludedRoad);
    }
}
