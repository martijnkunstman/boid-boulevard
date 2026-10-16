/**
 * StorageManager - Handles saving, restoring, and clearing simulation & training progress
 * in browser localStorage.
 */
class StorageManager {
    static STORAGE_KEY = 'boid_boulevard_progress_v5';

    /**
     * Saves both the Trainer progress (generations, brains, chart history) and City layout.
     * @param {Trainer} trainer
     * @param {City} city
     */
    static saveProgress(trainer, city) {
        try {
            const payload = {
                savedAt: new Date().toISOString(),
                trainer: trainer.serialize(),
                city: city ? city.serialize() : null
            };
            localStorage.setItem(StorageManager.STORAGE_KEY, JSON.stringify(payload));
            return true;
        } catch (err) {
            console.warn('StorageManager: Could not save to localStorage:', err);
            return false;
        }
    }

    /**
     * Loads saved progress from localStorage if available.
     * @returns {object|null}
     */
    static loadProgress() {
        try {
            const raw = localStorage.getItem(StorageManager.STORAGE_KEY);
            if (!raw) return null;
            return JSON.parse(raw);
        } catch (err) {
            console.warn('StorageManager: Could not parse saved progress:', err);
            return null;
        }
    }

    /**
     * Clears saved simulation progress from localStorage.
     */
    static clearProgress() {
        try {
            localStorage.removeItem(StorageManager.STORAGE_KEY);
            return true;
        } catch (err) {
            console.warn('StorageManager: Could not clear localStorage:', err);
            return false;
        }
    }

    /**
     * Returns metadata about current localStorage usage for the Debug Screen.
     */
    static getStorageInfo() {
        try {
            const raw = localStorage.getItem(StorageManager.STORAGE_KEY);
            if (!raw) {
                return { hasData: false, sizeBytes: 0, savedAt: 'None' };
            }
            const parsed = JSON.parse(raw);
            return {
                hasData: true,
                sizeBytes: raw.length,
                savedAt: parsed.savedAt ? new Date(parsed.savedAt).toLocaleTimeString() : 'Unknown'
            };
        } catch (e) {
            return { hasData: false, sizeBytes: 0, savedAt: 'Error' };
        }
    }
}
