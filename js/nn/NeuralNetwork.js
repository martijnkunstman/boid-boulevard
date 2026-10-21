/**
 * NeuralNetwork - Feedforward Multi-Layer Perceptron for Boid control.
 * Default architecture: [17, 12, 8, 2] (two hidden layers; in headless tests it outperformed [17, 10, 2]
 * after ~300 generations, while starting slower).
 * Uses contiguous Float32Array buffers for weights, biases, and activations for cache-friendly, zero-GC inference.
 * Outputs:
 *   output[0]: Steering in [-1, 1] (left to right)
 *   output[1]: Throttle in [-1, 1] (positive = forward acceleration, negative = backward braking/reverse)
 */
class NeuralNetwork {
    static DEFAULT_LAYOUT = [17, 12, 8, 2];

    // Number of most recent generation scores remembered per network (see recordScore)
    static EVAL_WINDOW = 5;

    // Heavy-tailed mutation: this share of mutation steps is MUTATION_JUMP_FACTOR times larger.
    // Occasional big jumps explore further while most steps stay small; in headless tests this beat
    // both uniformly weaker and uniformly stronger mutation on the benchmark
    static MUTATION_JUMP_CHANCE = 0.1;
    static MUTATION_JUMP_FACTOR = 5;

    /**
     * @param {number[]} layerSizes - Array of neuron counts per layer, default NeuralNetwork.DEFAULT_LAYOUT
     * @param {boolean} skipInit - If true, skips random weight initialization (used by clone/deserialize)
     */
    constructor(layerSizes = NeuralNetwork.DEFAULT_LAYOUT, skipInit = false) {
        this.layerSizes = [...layerSizes];
        this.weights = [];
        this.biases = [];
        this.lastActivations = this.layerSizes.map(size => new Float32Array(size));

        // Fitness of this exact network in its most recent generations (newest last). Unchanged copies
        // (elites, champion) keep it; crossover and mutation produce a new network and clear it.
        this.evalScores = [];

        if (!skipInit) {
            this._initializeRandom();
        }
    }

    _initializeRandom() {
        this.weights = [];
        this.biases = [];

        for (let l = 0; l < this.layerSizes.length - 1; l++) {
            const inCount = this.layerSizes[l];
            const outCount = this.layerSizes[l + 1];
            const scale = Math.sqrt(2 / (inCount + outCount));

            const layerWeights = new Array(outCount);
            const layerBiases = new Float32Array(outCount);

            for (let j = 0; j < outCount; j++) {
                const row = new Float32Array(inCount);
                for (let i = 0; i < inCount; i++) {
                    row[i] = (Random.next() * 2 - 1) * scale;
                }
                layerWeights[j] = row;
                layerBiases[j] = (Random.next() * 2 - 1) * 0.25;
            }

            this.weights.push(layerWeights);
            this.biases.push(layerBiases);
        }
    }

    /**
     * Seeds a weak initial driving reflex with random variation so initial boids
     * have diverse behaviors without spinning in place on Generation 1.
     *
     * Input layout (17 inputs):
     *   0..6   : 7 road-border rays (left -75 deg to right +75 deg), 1 = close to road border
     *   7..13  : 7 boid-ahead rays (left -75 deg to right +75 deg), 1 = close to boid ahead
     *   14     : closing speed with boid ahead (0 = clear, 1 = fast closing)
     *   15     : current normalized speed
     *   16     : road/turn/lane heading error (-1 = angled left of target lane, +1 = angled right)
     */
    applyInitialReflexBias(strength = 0.45) {
        if (this.layerSizes.length < 2) return;
        const h0Count = this.layerSizes[1];
        const w0 = this.weights[0];
        const b0 = this.biases[0];

        // Neuron 0 in hidden layer 1: Left road border / boid detector -> steer right (+)
        if (h0Count > 0) {
            w0[0][0] += 1.2 * strength;
            w0[0][1] += 1.05 * strength;
            w0[0][2] += 0.75 * strength;
            w0[0][7] += 0.85 * strength;
            w0[0][8] += 0.75 * strength;
            w0[0][16] -= 1.25 * strength; // Angled left of road -> steer right
        }
        // Neuron 1 in hidden layer 1: Right road border / boid detector -> steer left (-)
        if (h0Count > 1) {
            w0[1][4] += 0.75 * strength;
            w0[1][5] += 1.05 * strength;
            w0[1][6] += 1.2 * strength;
            w0[1][12] += 0.75 * strength;
            w0[1][13] += 0.85 * strength;
            w0[1][16] += 1.25 * strength; // Angled right of road -> steer left
        }
        // Neuron 2 in hidden layer 1: Cruise drive tendency -> forward throttle (+)
        if (h0Count > 2) {
            b0[2] += 0.95 * strength;
            w0[2][15] -= 0.35 * strength; // Regulate near max speed
        }
        // Neuron 3 in hidden layer 1: Front hazard brake detector -> backward throttle (-)
        if (h0Count > 3) {
            w0[3][3] += 0.75 * strength;  // Center road border ahead
            w0[3][9] += 0.9 * strength;   // Front-left boid
            w0[3][10] += 1.5 * strength;  // Center boid directly ahead
            w0[3][11] += 0.9 * strength;  // Front-right boid
            w0[3][14] += 1.1 * strength;  // Closing speed ahead
        }

        // Propagate channels through intermediate hidden layers if > 2 layers
        for (let l = 1; l < this.weights.length - 1; l++) {
            const wl = this.weights[l];
            for (let c = 0; c < Math.min(4, wl.length, wl[0].length); c++) {
                wl[c][c] += 1.05 * strength;
            }
        }

        // Connect to final output layer [steering, throttle]
        const lastW = this.weights[this.weights.length - 1];
        const lastB = this.biases[this.biases.length - 1];
        if (lastW.length >= 2 && lastW[0].length >= 4) {
            // Output 0: Steering
            lastW[0][0] += 1.25 * strength;
            lastW[0][1] -= 1.25 * strength;

            // Output 1: Throttle (forward > 0, braking < 0)
            lastW[1][2] += 1.1 * strength;
            lastW[1][3] -= 1.75 * strength;
            lastB[1] += 0.35 * strength;
        }
    }

    /**
     * Compute network forward pass using pre-allocated Float32Array activation buffers (zero GC).
     * @param {Float32Array|number[]} inputs
     * @returns {Float32Array} [steering, throttle]
     */
    forward(inputs) {
        const inLayer = this.lastActivations[0];
        const inLen = inLayer.length;
        for (let i = 0; i < inLen; i++) {
            inLayer[i] = inputs[i] || 0;
        }

        let current = inLayer;
        const numLayers = this.weights.length;
        for (let l = 0; l < numLayers; l++) {
            const layerWeights = this.weights[l];
            const layerBiases = this.biases[l];
            const next = this.lastActivations[l + 1];
            const outLen = layerWeights.length;
            const curLen = current.length;

            for (let j = 0; j < outLen; j++) {
                let sum = layerBiases[j];
                const row = layerWeights[j];
                for (let i = 0; i < curLen; i++) {
                    sum += row[i] * current[i];
                }
                next[j] = Math.tanh(sum);
            }
            current = next;
        }

        return current;
    }

    /**
     * True if this network has exactly the given layer sizes, including matching weight and bias shapes
     * (so a saved network from an older layout, or a corrupted save, is never mixed in).
     * @param {number[]} layout
     */
    hasLayout(layout) {
        if (this.layerSizes.length !== layout.length) return false;
        for (let l = 0; l < layout.length; l++) {
            if (this.layerSizes[l] !== layout[l]) return false;
        }
        if (this.weights.length !== layout.length - 1 || this.biases.length !== layout.length - 1) return false;
        for (let l = 0; l < this.weights.length; l++) {
            const rows = this.weights[l];
            if (rows.length !== layout[l + 1] || this.biases[l].length !== layout[l + 1]) return false;
            for (const row of rows) {
                if (row.length !== layout[l]) return false;
            }
        }
        return true;
    }

    /**
     * Fast deep clone using skipInit=true and native Float32Array copies (does not touch Random state).
     */
    clone() {
        const copy = new NeuralNetwork(this.layerSizes, true);
        copy.weights = this.weights.map(layer => layer.map(row => new Float32Array(row)));
        copy.biases = this.biases.map(layer => new Float32Array(layer));
        copy.evalScores = [...this.evalScores];
        return copy;
    }

    /**
     * Uniform + neuron-wise crossover between two networks.
     * @param {NeuralNetwork} partner
     * @returns {NeuralNetwork}
     */
    crossover(partner) {
        // Networks with different layouts cannot be mixed weight-by-weight
        if (!partner.hasLayout(this.layerSizes)) {
            return this.clone();
        }
        const child = this.clone();
        child.evalScores = [];
        for (let l = 0; l < child.weights.length; l++) {
            const cLayerW = child.weights[l];
            const pLayerW = partner.weights[l];
            const cLayerB = child.biases[l];
            const pLayerB = partner.biases[l];

            for (let j = 0; j < cLayerW.length; j++) {
                const takePartnerNeuron = Random.next() < 0.5;
                if (takePartnerNeuron) {
                    cLayerB[j] = pLayerB[j];
                } else if (Random.next() < 0.3) {
                    cLayerB[j] = (cLayerB[j] + pLayerB[j]) * 0.5;
                }

                const cRow = cLayerW[j];
                const pRow = pLayerW[j];
                for (let i = 0; i < cRow.length; i++) {
                    const r = Random.next();
                    if (r < 0.45) {
                        cRow[i] = pRow[i];
                    } else if (r < 0.65) {
                        cRow[i] = (cRow[i] + pRow[i]) * 0.5;
                    }
                }
            }
        }
        return child;
    }

    /**
     * Mutates weights and biases in-place.
     * @param {number} rate - Probability of mutating each parameter (0..1)
     * @param {number} amount - Mutation step magnitude
     */
    mutate(rate = 0.12, amount = 0.25) {
        this.evalScores = [];
        for (let l = 0; l < this.weights.length; l++) {
            const layerW = this.weights[l];
            const layerB = this.biases[l];
            for (let j = 0; j < layerW.length; j++) {
                if (Random.next() < rate) {
                    const bVal = layerB[j] + NeuralNetwork._mutationStep(amount);
                    layerB[j] = bVal < -3.5 ? -3.5 : bVal > 3.5 ? 3.5 : bVal;
                }
                const row = layerW[j];
                for (let i = 0; i < row.length; i++) {
                    if (Random.next() < rate) {
                        const wVal = row[i] + NeuralNetwork._mutationStep(amount);
                        row[i] = wVal < -3.5 ? -3.5 : wVal > 3.5 ? 3.5 : wVal;
                    }
                }
            }
        }
    }

    /**
     * One mutation step: Gaussian noise scaled by amount, occasionally (MUTATION_JUMP_CHANCE) multiplied
     * by MUTATION_JUMP_FACTOR.
     */
    static _mutationStep(amount) {
        const step = NeuralNetwork.gaussianRandom() * amount;
        return Random.next() < NeuralNetwork.MUTATION_JUMP_CHANCE ? step * NeuralNetwork.MUTATION_JUMP_FACTOR : step;
    }

    static gaussianRandom() {
        return Random.gaussian();
    }

    /**
     * Records one generation's fitness for this network, keeping the last EVAL_WINDOW scores.
     */
    recordScore(score) {
        this.evalScores.push(score);
        if (this.evalScores.length > NeuralNetwork.EVAL_WINDOW) {
            this.evalScores.shift();
        }
    }

    /**
     * Average of the remembered scores: a fairer ranking than a single, possibly lucky, generation.
     */
    meanScore() {
        if (this.evalScores.length === 0) return 0;
        let sum = 0;
        for (const v of this.evalScores) sum += v;
        return sum / this.evalScores.length;
    }

    serialize() {
        return {
            layerSizes: this.layerSizes,
            evalScores: this.evalScores,
            weights: this.weights.map(layer => layer.map(row => Array.from(row))),
            biases: this.biases.map(layer => Array.from(layer))
        };
    }

    static deserialize(data) {
        if (!data || !Array.isArray(data.layerSizes)) {
            return new NeuralNetwork();
        }
        const nn = new NeuralNetwork(data.layerSizes, true);
        nn.weights = data.weights.map(layer =>
            layer.map(row => new Float32Array(row.map(w => (Number.isFinite(w) ? w : 0))))
        );
        nn.biases = data.biases.map(layer =>
            new Float32Array(layer.map(b => (Number.isFinite(b) ? b : 0)))
        );
        nn.evalScores = Array.isArray(data.evalScores)
            ? data.evalScores.filter(Number.isFinite).slice(-NeuralNetwork.EVAL_WINDOW)
            : [];
        return nn;
    }
}
