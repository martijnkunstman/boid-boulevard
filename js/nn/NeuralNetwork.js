/**
 * NeuralNetwork - Feedforward Multi-Layer Perceptron for Boid control.
 * Outputs:
 *   output[0]: Steering in [-1, 1] (left to right)
 *   output[1]: Throttle in [-1, 1] (positive = forward acceleration, negative = backward braking/reverse)
 */
class NeuralNetwork {
    /**
     * @param {number[]} layerSizes - Array of neuron counts per layer, e.g. [18, 14, 8, 2]
     */
    constructor(layerSizes = [18, 14, 8, 2]) {
        this.layerSizes = [...layerSizes];
        this.weights = [];
        this.biases = [];
        this.lastActivations = this.layerSizes.map(size => new Array(size).fill(0));

        this._initializeRandom();
    }

    _initializeRandom() {
        this.weights = [];
        this.biases = [];

        for (let l = 0; l < this.layerSizes.length - 1; l++) {
            const inCount = this.layerSizes[l];
            const outCount = this.layerSizes[l + 1];
            const scale = Math.sqrt(2 / (inCount + outCount));

            const layerWeights = [];
            const layerBiases = [];

            for (let j = 0; j < outCount; j++) {
                const row = [];
                for (let i = 0; i < inCount; i++) {
                    row.push((Math.random() * 2 - 1) * scale);
                }
                layerWeights.push(row);
                layerBiases.push((Math.random() * 2 - 1) * 0.25);
            }

            this.weights.push(layerWeights);
            this.biases.push(layerBiases);
        }
    }

    /**
     * Seeds a weak initial driving reflex with heavy random variation so initial boids
     * have diverse behaviors without spinning in place on Generation 1.
     *
     * Input layout (18 inputs):
     *   0..6   : 7 road-edge rays (left -75 deg to right +75 deg), 1 = close to road side
     *   7..13  : 7 boid-ahead rays (left -75 deg to right +75 deg), 1 = close to boid ahead
     *   14     : stoplight sensor ahead (0 = green/clear, 1 = red light close ahead)
     *   15     : closing speed with boid ahead (0 = clear, 1 = fast closing)
     *   16     : current normalized speed
     *   17     : lane alignment error (-1 = left of lane/angled left, +1 = right of lane/angled right)
     */
    applyInitialReflexBias(strength = 0.45) {
        if (this.layerSizes.length < 2) return;
        const h0Count = this.layerSizes[1];
        const w0 = this.weights[0];
        const b0 = this.biases[0];

        // Neuron 0 in hidden layer 1: Left-obstacle / left-of-lane detector -> steer right (+)
        if (h0Count > 0) {
            w0[0][0] += 1.1 * strength;
            w0[0][1] += 1.0 * strength;
            w0[0][2] += 0.7 * strength;
            w0[0][7] += 0.8 * strength;
            w0[0][8] += 0.7 * strength;
            w0[0][17] -= 1.35 * strength; // Negative laneError = angled/offset left -> steer right
        }
        // Neuron 1 in hidden layer 1: Right-obstacle / right-of-lane detector -> steer left (-)
        if (h0Count > 1) {
            w0[1][4] += 0.7 * strength;
            w0[1][5] += 1.0 * strength;
            w0[1][6] += 1.1 * strength;
            w0[1][12] += 0.7 * strength;
            w0[1][13] += 0.8 * strength;
            w0[1][17] += 1.35 * strength; // Positive laneError = angled/offset right -> steer left
        }
        // Neuron 2 in hidden layer 1: Cruise drive tendency -> forward throttle (+)
        if (h0Count > 2) {
            b0[2] += 0.95 * strength;
            w0[2][16] -= 0.35 * strength; // Regulate near max speed
        }
        // Neuron 3 in hidden layer 1: Front hazard & stoplight brake detector -> backward throttle (-)
        if (h0Count > 3) {
            w0[3][3] += 0.65 * strength;  // Center road boundary ahead
            w0[3][9] += 0.85 * strength;  // Front-left boid
            w0[3][10] += 1.45 * strength; // Center boid directly ahead
            w0[3][11] += 0.85 * strength; // Front-right boid
            w0[3][14] += 1.65 * strength; // Red stoplight ahead
            w0[3][15] += 1.0 * strength;  // Closing speed ahead
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
            lastW[0][0] += 1.25 * strength; // Turn right when left sensors / left lane error fire
            lastW[0][1] -= 1.25 * strength; // Turn left when right sensors / right lane error fire

            // Output 1: Throttle (forward > 0, braking < 0)
            lastW[1][2] += 1.1 * strength;  // Base forward drive
            lastW[1][3] -= 1.75 * strength; // Brake when boid/red light/wall in front
            lastB[1] += 0.35 * strength;
        }
    }

    /**
     * Compute network forward pass.
     * @param {number[]} inputs
     * @returns {number[]} [steering, throttle]
     */
    forward(inputs) {
        let current = inputs.slice(0, this.layerSizes[0]);
        while (current.length < this.layerSizes[0]) {
            current.push(0);
        }
        this.lastActivations[0] = current;

        for (let l = 0; l < this.weights.length; l++) {
            const layerWeights = this.weights[l];
            const layerBiases = this.biases[l];
            const next = new Array(layerWeights.length);

            for (let j = 0; j < layerWeights.length; j++) {
                let sum = layerBiases[j];
                const row = layerWeights[j];
                for (let i = 0; i < current.length; i++) {
                    sum += row[i] * current[i];
                }
                next[j] = Math.tanh(sum);
            }

            current = next;
            this.lastActivations[l + 1] = current;
        }

        return current;
    }

    clone() {
        const copy = new NeuralNetwork(this.layerSizes);
        copy.weights = this.weights.map(layer => layer.map(row => [...row]));
        copy.biases = this.biases.map(layer => [...layer]);
        return copy;
    }

    /**
     * Uniform + neuron-wise crossover between two networks.
     * @param {NeuralNetwork} partner
     * @returns {NeuralNetwork}
     */
    crossover(partner) {
        const child = this.clone();
        for (let l = 0; l < child.weights.length; l++) {
            for (let j = 0; j < child.weights[l].length; j++) {
                const takePartnerNeuron = Math.random() < 0.5;
                if (takePartnerNeuron) {
                    child.biases[l][j] = partner.biases[l][j];
                } else if (Math.random() < 0.3) {
                    child.biases[l][j] = (this.biases[l][j] + partner.biases[l][j]) * 0.5;
                }

                for (let i = 0; i < child.weights[l][j].length; i++) {
                    const r = Math.random();
                    if (r < 0.45) {
                        child.weights[l][j][i] = partner.weights[l][j][i];
                    } else if (r < 0.65) {
                        child.weights[l][j][i] = (this.weights[l][j][i] + partner.weights[l][j][i]) * 0.5;
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
        for (let l = 0; l < this.weights.length; l++) {
            for (let j = 0; j < this.weights[l].length; j++) {
                if (Math.random() < rate) {
                    this.biases[l][j] += NeuralNetwork.gaussianRandom() * amount;
                    this.biases[l][j] = Math.max(-3.5, Math.min(3.5, this.biases[l][j]));
                }
                for (let i = 0; i < this.weights[l][j].length; i++) {
                    if (Math.random() < rate) {
                        this.weights[l][j][i] += NeuralNetwork.gaussianRandom() * amount;
                        this.weights[l][j][i] = Math.max(-3.5, Math.min(3.5, this.weights[l][j][i]));
                    }
                }
            }
        }
    }

    static gaussianRandom() {
        let u = 0;
        let v = 0;
        while (u === 0) u = Math.random();
        while (v === 0) v = Math.random();
        return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
    }

    serialize() {
        return {
            layerSizes: this.layerSizes,
            weights: this.weights,
            biases: this.biases
        };
    }

    static deserialize(data) {
        if (!data || !Array.isArray(data.layerSizes)) {
            return new NeuralNetwork();
        }
        const nn = new NeuralNetwork(data.layerSizes);
        nn.weights = data.weights.map(layer => layer.map(row => [...row]));
        nn.biases = data.biases.map(layer => [...layer]);
        return nn;
    }
}
