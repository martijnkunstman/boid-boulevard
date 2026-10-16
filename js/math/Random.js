/**
 * Random - Deterministic seeded Pseudo-Random Number Generator (Mulberry32).
 * Allows reproducible city generation, boid properties, and neural network initialization.
 */
class Random {
    static currentSeed = '104729';
    static _state = 104729;

    /**
     * Sets the active seed (number or string) and resets the generator state.
     * @param {string|number} seed
     */
    static setSeed(seed) {
        const str = String(seed ?? '').trim() || '104729';
        Random.currentSeed = str;
        Random._state = Random._hashSeed(str);
    }

    /**
     * Captures the current generator state, so a side computation (e.g. the training benchmark)
     * can use its own seed without disturbing the main random sequence.
     * @returns {{seed: string, state: number}}
     */
    static saveState() {
        return { seed: Random.currentSeed, state: Random._state };
    }

    /**
     * Restores a state captured with saveState().
     * @param {{seed: string, state: number}} saved
     */
    static restoreState(saved) {
        Random.currentSeed = saved.seed;
        Random._state = saved.state;
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
