/**
 * Random - Deterministic seeded Pseudo-Random Number Generator (Mulberry32).
 * Supports both static global usage and isolated instance streams (`new Random(seed)`),
 * allowing reproducible city generation, boid properties, and neural network evolution.
 */
class Random {
    static currentSeed = '104729';
    static _state = 104729;
    static _hasSpare = false;
    static _spare = 0;

    /**
     * Creates an isolated deterministic PRNG stream with its own seed and state.
     * @param {string|number} seed
     */
    constructor(seed = '104729') {
        this.currentSeed = '104729';
        this._state = 104729;
        this._hasSpare = false;
        this._spare = 0;
        this.setSeed(seed);
    }

    /**
     * Sets the seed on this PRNG instance.
     * @param {string|number} seed
     */
    setSeed(seed) {
        const str = String(seed ?? '').trim() || '104729';
        this.currentSeed = str;
        this._state = Random._hashSeed(str);
        this._hasSpare = false;
        this._spare = 0;
    }

    /**
     * Returns a deterministic float in [0, 1) from this instance (Mulberry32).
     * @returns {number}
     */
    next() {
        this._state = (this._state + 0x6d2b79f5) | 0;
        let t = this._state;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    }

    range(min, max) {
        return min + this.next() * (max - min);
    }

    int(min, max) {
        return Math.floor(this.range(min, max));
    }

    /**
     * Sets the active static seed (number or string) and resets the generator state.
     * @param {string|number} seed
     */
    static setSeed(seed) {
        const str = String(seed ?? '').trim() || '104729';
        Random.currentSeed = str;
        Random._state = Random._hashSeed(str);
        Random._hasSpare = false;
        Random._spare = 0;
    }

    /**
     * Captures the current generator state, so a side computation (e.g. the training benchmark)
     * can use its own seed without disturbing the main random sequence.
     * @returns {{seed: string, state: number, hasSpare: boolean, spare: number}}
     */
    static saveState() {
        return {
            seed: Random.currentSeed,
            state: Random._state,
            hasSpare: Random._hasSpare,
            spare: Random._spare
        };
    }

    /**
     * Restores a state captured with saveState().
     * @param {{seed: string, state: number, hasSpare?: boolean, spare?: number}} saved
     */
    static restoreState(saved) {
        Random.currentSeed = saved.seed;
        Random._state = saved.state;
        Random._hasSpare = Boolean(saved.hasSpare);
        Random._spare = saved.spare || 0;
    }

    /**
     * Generates a new random 6-digit seed string using native Math.random().
     * @returns {string}
     */
    static generateSeedString() {
        return String(Math.floor(100000 + Math.random() * 900000));
    }

    /**
     * Hashes any string into a non-zero 32-bit unsigned integer (FNV-1a).
     * @param {string} str
     * @returns {number}
     */
    static _hashSeed(str) {
        let h = 2166136261 >>> 0;
        for (let i = 0; i < str.length; i++) {
            h ^= str.charCodeAt(i);
            h = Math.imul(h, 16777619);
        }
        return (h >>> 0) || 1;
    }

    /**
     * Returns a deterministic float in [0, 1) (Mulberry32).
     * @returns {number}
     */
    static next() {
        Random._state = (Random._state + 0x6d2b79f5) | 0;
        let t = Random._state;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    }

    /**
     * Returns a deterministic standard normal random variable N(0, 1) using polar Marsaglia
     * with spare value caching (halves log/sqrt calls during mutation).
     * @returns {number}
     */
    static gaussian() {
        if (Random._hasSpare) {
            Random._hasSpare = false;
            return Random._spare;
        }
        let u = 0;
        let v = 0;
        let s = 0;
        do {
            u = Random.next() * 2 - 1;
            v = Random.next() * 2 - 1;
            s = u * u + v * v;
        } while (s >= 1 || s === 0);
        const mul = Math.sqrt((-2.0 * Math.log(s)) / s);
        Random._spare = v * mul;
        Random._hasSpare = true;
        return u * mul;
    }

    /**
     * Returns a deterministic float in [min, max).
     */
    static range(min, max) {
        return min + Random.next() * (max - min);
    }

    /**
     * Returns a deterministic integer in [min, max).
     */
    static int(min, max) {
        return Math.floor(Random.range(min, max));
    }
}
