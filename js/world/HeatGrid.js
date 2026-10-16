/**
 * HeatGrid - Coarse 2D accumulation grid over the city, used for traffic density
 * and crash location heatmaps.
 */
class HeatGrid {
    /**
     * @param {number} width - World width in px
     * @param {number} height - World height in px
     * @param {number} cellSize - Cell size in px
     */
    constructor(width, height, cellSize = 10) {
        this.cellSize = cellSize;
        this.invCell = 1 / cellSize;
        this.cols = Math.ceil(width / cellSize);
        this.rows = Math.ceil(height / cellSize);
        this.data = new Float32Array(this.cols * this.rows);
    }

    /**
     * Adds an amount to the cell containing world point (x, y). Points outside the grid are ignored.
     */
    add(x, y, amount = 1) {
        const c = Math.floor(x * this.invCell);
        const r = Math.floor(y * this.invCell);
        if (c < 0 || r < 0 || c >= this.cols || r >= this.rows) return;
        this.data[r * this.cols + c] += amount;
    }

    /**
     * Multiplies all cells by factor, so older activity fades out over generations.
     */
    decay(factor) {
        const d = this.data;
        for (let i = 0; i < d.length; i++) {
            d[i] *= factor;
        }
    }

    clear() {
        this.data.fill(0);
    }

    /**
     * Returns the value at fraction q (0..1) of the sorted non-zero cells, or 0 if all cells are empty.
     */
    percentile(q) {
        const active = [];
        const d = this.data;
        for (let i = 0; i < d.length; i++) {
            if (d[i] > 0) active.push(d[i]);
        }
        if (active.length === 0) return 0;
        active.sort((a, b) => a - b);
        return active[Math.min(active.length - 1, Math.floor(q * active.length))];
    }
}
