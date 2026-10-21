/**
 * StorageManager - Handles saving, restoring, clearing, and JSON file export/import
 * of simulation & training progress in browser localStorage.
 * Caches storage metadata in memory so DebugScreen telemetry never re-parses JSON on render frames.
 */
class StorageManager {
    static STORAGE_KEY = 'boid_boulevard_progress_v6';
    static _cachedInfo = null;

    /**
     * Builds the serializable snapshot payload for both localStorage and JSON file export.
     */
    static buildPayload(trainer, city) {
        return {
            version: '1.13.0',
            savedAt: new Date().toISOString(),
            trainer: trainer.serialize(),
            city: city ? city.serialize() : null
        };
    }

    /**
     * Saves both the Trainer progress (generations, brains, chart history) and City layout.
     * @param {Trainer} trainer
     * @param {City} city
     */
    static saveProgress(trainer, city) {
        try {
            const payload = StorageManager.buildPayload(trainer, city);
            const raw = JSON.stringify(payload);
            localStorage.setItem(StorageManager.STORAGE_KEY, raw);
            StorageManager._cachedInfo = {
                hasData: true,
                sizeBytes: raw.length,
                savedAt: new Date(payload.savedAt).toLocaleTimeString()
            };
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
            if (!raw) {
                StorageManager._cachedInfo = { hasData: false, sizeBytes: 0, savedAt: 'None' };
                return null;
            }
            const parsed = JSON.parse(raw);
            StorageManager._cachedInfo = {
                hasData: true,
                sizeBytes: raw.length,
                savedAt: parsed.savedAt ? new Date(parsed.savedAt).toLocaleTimeString() : 'Unknown'
            };
            return parsed;
        } catch (err) {
            console.warn('StorageManager: Could not parse saved progress:', err);
            StorageManager._cachedInfo = { hasData: false, sizeBytes: 0, savedAt: 'Error' };
            return null;
        }
    }

    /**
     * Clears saved simulation progress from localStorage.
     */
    static clearProgress() {
        try {
            localStorage.removeItem(StorageManager.STORAGE_KEY);
            StorageManager._cachedInfo = { hasData: false, sizeBytes: 0, savedAt: 'None' };
            return true;
        } catch (err) {
            console.warn('StorageManager: Could not clear localStorage:', err);
            return false;
        }
    }

    /**
     * Returns cached metadata about current localStorage usage for the Debug Screen (O(1), zero JSON.parse).
     */
    static getStorageInfo() {
        if (StorageManager._cachedInfo) {
            return StorageManager._cachedInfo;
        }
        try {
            const raw = localStorage.getItem(StorageManager.STORAGE_KEY);
            if (!raw) {
                StorageManager._cachedInfo = { hasData: false, sizeBytes: 0, savedAt: 'None' };
            } else {
                const parsed = JSON.parse(raw);
                StorageManager._cachedInfo = {
                    hasData: true,
                    sizeBytes: raw.length,
                    savedAt: parsed.savedAt ? new Date(parsed.savedAt).toLocaleTimeString() : 'Unknown'
                };
            }
        } catch (e) {
            StorageManager._cachedInfo = { hasData: false, sizeBytes: 0, savedAt: 'Error' };
        }
        return StorageManager._cachedInfo;
    }

    /**
     * Exports the current trainer & city state to a downloadable .json file in the browser.
     */
    static exportToFile(trainer, city) {
        const payload = StorageManager.buildPayload(trainer, city);
        const json = JSON.stringify(payload, null, 2);
        const blob = new Blob([json], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        const seed = city ? city.seed : 'seed';
        a.href = url;
        a.download = `boid-boulevard-gen${trainer.generation}-${seed}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }
}
