/**
 * SpatialGrid - Uniform grid that buckets boids by position each simulation step, so sensor
 * and collision checks only look at boids in nearby cells instead of every boid.
 * Built with a counting sort into flat typed arrays (no per-step allocations once sized).
 */
class SpatialGrid {
    /**
     * @param {number} width - World width in px
     * @param {number} height - World height in px
     * @param {number} cellSize - Cell size in px
     */
    constructor(width, height, cellSize = 64) {
        this.cellSize = cellSize;
        this.invCell = 1 / cellSize;
        this.cols = Math.max(1, Math.ceil(width / cellSize));
        this.rows = Math.max(1, Math.ceil(height / cellSize));
        const cellCount = this.cols * this.rows;
        // Items of cell c are items[cellStart[c] .. cellStart[c + 1] - 1] (indices into boids)
        this.cellStart = new Int32Array(cellCount + 1);
        this.cellFill = new Int32Array(cellCount);
        this.items = new Int32Array(0);
        this.boidCell = new Int32Array(0);
        this.boids = [];
    }

    _col(x) {
        const c = Math.floor(x * this.invCell);
        return c < 0 ? 0 : c >= this.cols ? this.cols - 1 : c;
    }

    _row(y) {
        const r = Math.floor(y * this.invCell);
        return r < 0 ? 0 : r >= this.rows ? this.rows - 1 : r;
    }

    /**
     * Re-buckets all boids by their current position.
     * @param {Boid[]} boids
     */
    rebuild(boids) {
        const n = boids.length;
        if (this.items.length < n) {
            this.items = new Int32Array(n);
            this.boidCell = new Int32Array(n);
        }
        this.boids = boids;

        const fill = this.cellFill;
        fill.fill(0);
        for (let i = 0; i < n; i++) {
            const cell = this._row(boids[i].y) * this.cols + this._col(boids[i].x);
            this.boidCell[i] = cell;
            fill[cell]++;
        }

        // Prefix sums -> cell start offsets, then scatter indices into place
        const start = this.cellStart;
        start[0] = 0;
        for (let c = 0; c < fill.length; c++) {
            start[c + 1] = start[c] + fill[c];
            fill[c] = start[c];
        }
        for (let i = 0; i < n; i++) {
            this.items[fill[this.boidCell[i]]++] = i;
        }
    }

    /**
     * Fills `out` with the boids (other than `exclude`) within `radius` of (x, y).
     * `slack` widens the scanned cells to cover boids that moved since the last rebuild.
     */
    queryRadius(x, y, radius, exclude, out, slack = 4) {
        out.length = 0;
        const reach = radius + slack;
        const c0 = this._col(x - reach);
        const c1 = this._col(x + reach);
        const r0 = this._row(y - reach);
        const r1 = this._row(y + reach);
        const radiusSq = radius * radius;
        const boids = this.boids;

        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) {
                const cell = r * this.cols + c;
                const end = this.cellStart[cell + 1];
                for (let k = this.cellStart[cell]; k < end; k++) {
                    const other = boids[this.items[k]];
                    if (other === exclude) continue;
                    const dx = other.x - x;
                    const dy = other.y - y;
                    if (dx * dx + dy * dy <= radiusSq) {
                        out.push(other);
                    }
                }
            }
        }
        return out;
    }
}
