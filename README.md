# Boid Boulevard

**Boid Boulevard** is a browser-based 2D city traffic and neuroevolution simulation built with vanilla HTML5 Canvas, CSS, and Object-Oriented JavaScript (zero external dependencies). Autonomous vehicle agents (**boids**) equipped with front-facing raycast sensors and feedforward neural networks learn from scratch how to steer, accelerate, brake, navigate crossings and diagonal avenues, and avoid collisions inside a closed rectangular city.

---

## Key Features

- **Closed Rectangular City Generation**
  - Procedural city road network strictly enclosed inside a rectangular perimeter so no road ever leads outside the city.
  - Features a perimeter boulevard, interior avenues, crossings (strictly capped at **$\le 4$ roads per crossing**), and **diagonal shortcut roads**.
  - Unified single-track roads where boids can drive in any direction while staying within road boundaries.
- **Deterministic Random Seed**
  - Uses a seeded Mulberry32 PRNG (`Random`) so entering the same seed reproduces the exact same city layout, spawn poses, and genetic mutations.
- **Feedforward Neural Network (`[17 -> 10 -> 2]`)**
  - **17 Inputs**:
    - `7` Front Road-Border Ray Sensors (`-75°`, `-45°`, `-22°`, `0°`, `+22°`, `+45°`, `+75°`)
    - `7` Front Boid-Detection Ray Sensors (same 7 angles)
    - `1` Closing Speed sensor toward the nearest boid ahead
    - `1` Normalized Current Speed (`speed / maxSpeed`)
    - `1` Signed Road Alignment Error (`sin(headingError)`)
  - **10 Hidden Neurons** (`tanh` activation)
  - **2 Continuous Outputs** (`tanh` activation in `[-1, +1]`):
    1. **Steering Direction**: `-1` (full left) to `+1` (full right)
    2. **Bidirectional Throttle / Brake**: `> 0` accelerates forward; `< 0` brakes when moving forward or reverses from a stop
- **1 to 4 Selectable Boid Archetypes**
  - Choose between **1 to 4 vehicle types** (`Sedan`, `Compact`, `Sports`, `Heavy`), each with distinct top speed, acceleration, braking force, turn rate, radius, and mass.
  - Offspring inherit their parent's vehicle archetype during evolution so neural networks specialize in their vehicle's physical dynamics.
- **Non-Collapsing Physics**
  - Hard road-border containment and iterative pairwise circle-circle collision resolution keep boids from overlapping or escaping the road network.
- **High-Speed & Decoupled "Turbo" Training**
  - Fixed-timestep (`1/60s`) physics loop supports `1x`, `2x`, `4x`, `8x`, `16x`, `32x`, and `64x` speed multipliers without physics tunneling.
  - **Turbo (Skip Render)** mode decouples visualization from physics, skipping intermediate canvas redraws and rendering once per completed generation for maximum training throughput.
  - Configurable generation duration (`15s`, `30s`, `45s`, `60s`).
- **Automatic Persistence (`localStorage`)**
  - Automatically saves the city map, generation counter, elite neural networks, and historical chart metrics after every generation.
  - Includes a **Restart & Clear Local Storage** button to wipe saved data and start fresh.
- **Three Built-In Screens**
  1. **City Simulation Screen**: Interactive canvas view of the city, boids, and raycast sensors (click any boid to inspect it).
  2. **Debug Screen**: Live neural network topology & synapse activation visualizer, steering/throttle gauges, sensor readouts, and city/storage diagnostics.
  3. **Training Progress Charts Screen**: Multi-generation line charts for Best/Average Fitness, Average Distance Traveled, Average Collisions, and Road Alignment & Speed Efficiency.

---

## How Training (Neuroevolution) Works

Each generation runs for a fixed duration (default `30s`, or `1,800` physics steps at `60 Hz`). At every step, each boid's fitness is updated based on its driving behavior:

- **Rewards**:
  - Forward distance traveled along the road (`+1.65` per pixel)
  - Alignment with the road axis and maintaining steady cruising speed
- **Penalties**:
  - Scraping or hitting road borders (`-5.5` per impact plus continuous rubbing penalty)
  - Colliding with other boids (`-12.0` per collision)
  - Idling or stalling (`-4.0 / sec`)
  - Driving in reverse (`-3.5 / sec`)

At the end of a generation, `Trainer` ranks all boids by fitness:
1. **Elitism**: The top 6 brains (and the all-time best brain) are copied unchanged into the next generation.
2. **Tournament Selection & Crossover**: Top-performing boids are selected via tournament selection and combined via uniform weight/bias crossover.
3. **Gaussian Mutation**: Offspring weights and biases receive random perturbations (`mutationRate = 0.14`, `mutationStrength = 0.24`) to discover better driving behaviors.

---

## Project Structure (OOP Architecture)

```text
boid-boulevard/
├── index.html                  # Main application layout & controls
├── css/
│   └── style.css               # Utilitarian UI styling
└── js/
    ├── main.js                 # Application bootstrap
    ├── Simulation.js           # Main simulation controller, loop & UI bindings
    ├── math/
    │   ├── Random.js           # Deterministic seeded PRNG (Mulberry32)
    │   └── Vector2D.js         # 2D vector math operations
    ├── nn/
    │   └── NeuralNetwork.js    # Feedforward neural network, crossover & mutation
    ├── world/
    │   ├── RoadNode.js         # Intersection / waypoint node
    │   ├── Road.js             # Unified single-track road segment & AABB geometry
    │   ├── City.js             # City graph, boundary containment & road queries
    │   └── CityGenerator.js    # Procedural closed rectangular city generator
    ├── entities/
    │   ├── Sensor.js           # 7-ray road border & boid ahead raycaster
    │   └── Boid.js             # Autonomous vehicle physics, archetypes & fitness
    ├── training/
    │   └── Trainer.js          # Genetic algorithm & generation history tracker
    ├── storage/
    │   └── StorageManager.js   # Browser localStorage save/load/clear manager
    └── ui/
        ├── SimulationRenderer.js # Canvas renderer for city, roads, sensors & boids
        ├── DebugScreen.js        # Live neural network & telemetry inspector
        └── ChartScreen.js        # Multi-generation training progress charts
```

---

## Running the Simulation

No build step or package installation is required:

1. Clone the repository:
   ```bash
   git clone https://github.com/martijnkunstman/boid-boulevard.git
   cd boid-boulevard
   ```
2. Open `index.html` directly in any modern web browser, or serve the folder locally:
   ```bash
   npx serve .
   ```
