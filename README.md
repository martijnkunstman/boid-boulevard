# Boid Boulevard

**Boid Boulevard** is a browser-based 2D city traffic and neuroevolution simulation built with vanilla HTML5 Canvas, CSS, and Object-Oriented JavaScript (zero external dependencies). Autonomous vehicle agents (**boids**) equipped with front-facing raycast sensors and feedforward neural networks learn from scratch how to steer, accelerate, brake, navigate crossings and diagonal avenues, and avoid collisions inside a closed rectangular city.

---

## Key Features

- **Closed Rectangular City Generation**
  - Procedural city road network strictly enclosed inside a rectangular perimeter so no road ever leads outside the city.
  - Features a perimeter boulevard, interior avenues, crossings (strictly capped at **$\le 4$ roads per crossing**), and **diagonal shortcut roads**.
  - **Two-way roads** (`53 px` wide) with a dashed centerline: traffic keeps right, each direction in its own lane.
  - **Right of way at crossings** (3+ roads): a boid approaching a crossing gives way to traffic from its right, to anyone already on the crossing, and to oncoming traffic when it turns left itself. While it must give way, a red stop line appears in its lane and is sensed as a stationary obstacle on its boid rays (no extra network inputs). A boid that can no longer stop in time is committed and may cross; if four boids block each other, the one that waited longest goes.
- **Deterministic Random Seed**
  - Uses a seeded Mulberry32 PRNG (`Random`) so entering the same seed reproduces the exact same city layout, spawn poses, and genetic mutations.
- **Feedforward Neural Network (`[17 -> 10 -> 2]`)**
  - **17 Inputs**:
    - `7` Front Road-Border Ray Sensors (`-75°`, `-50°`, `-25°`, `0°`, `+25°`, `+50°`, `+75°`)
    - `7` Front Boid-Detection Ray Sensors (same 7 angles; also detect the stop line when the boid must give way)
    - `1` Closing Speed sensor toward the boid straight ahead (center ray only, so oncoming traffic in the other lane does not trigger it)
    - `1` Normalized Current Speed (`speed / maxSpeed`)
    - `1` Signed heading error toward a lookahead point in the boid's own (right-hand) lane, or into the correct lane of the next road when turning at a crossing
  - **10 Hidden Neurons** (`tanh` activation)
  - **2 Continuous Outputs** (`tanh` activation in `[-1, +1]`):
    1. **Steering Direction**: `-1` (full left) to `+1` (full right)
    2. **Bidirectional Throttle / Brake**: `> 0` accelerates forward; `< 0` brakes when moving forward or reverses from a stop
- **1 to 4 Selectable Boid Archetypes**
  - Choose between **1 to 4 vehicle types** (`Sedan`, `Compact`, `Sports`, `Heavy`), each with distinct top speed, acceleration, braking force, turn rate, radius, and mass.
  - Offspring inherit their parent's vehicle archetype during evolution so neural networks specialize in their vehicle's physical dynamics.
- **Non-Collapsing Physics**
  - Hard road-border containment and pairwise collision resolution keep boids from overlapping or escaping the road network.
  - Each boid has an elliptical collision shape (its length front-to-back, its width side-by-side), so boids passing each other in opposite lanes do not collide.
- **High-Speed & Decoupled "Turbo" Training**
  - Fixed-timestep (`1/60s`) physics loop supports `1x`, `2x`, `4x`, `8x`, `16x`, `32x`, and `64x` speed multipliers without physics tunneling.
  - Configurable generation duration (`15s`, `30s`, `45s`, `60s`).
  - **Turbo (Skip Render)** mode decouples visualization from physics: each animation frame runs as many physics steps as fit in a `14 ms` time budget (instead of a fixed count) and renders the canvas only once per completed generation, so any physics speedup directly becomes training throughput.
- **Signed Distance Field Road Queries**
  - On creation, each city precomputes a `2 px` grid of signed distances to the nearest road border (~160 KB, built in under 1 ms).
  - On-road checks are a single bilinear lookup, and road-border sensor rays use sphere tracing over the field instead of fixed-step sampling, making road raycasts 2-3x faster and more accurate.
- **Automatic Persistence (`localStorage`)**
  - Automatically saves the city map, generation counter, elite neural networks, and historical chart metrics after every generation.
  - Chart history keeps at most `500` points: when that is exceeded, every other point is removed (back to ~`250`), so the charts always show the complete run from generation 1 at a coarser resolution for older generations.
  - Includes a **Restart & Clear Local Storage** button to wipe saved data and start fresh.
- **Three Built-In Screens**
  1. **City Simulation Screen**: Interactive canvas view of the city, boids, and raycast sensors (click any boid to inspect it). An **Overlay** selector draws one of these over the roads:
     - **Traffic heatmap**: where boids spend their time
     - **Crash heatmap**: where road-border and boid-to-boid collisions happen
     - **Road distance field (SDF)**: the precomputed signed distance field used for road queries

     Heatmaps fade by half at the end of every generation, so they reflect recent driving behavior.
  2. **Debug Screen**: Live neural network topology & synapse activation visualizer, steering/throttle gauges, sensor readouts, and city/storage diagnostics.
  3. **Training Progress Charts Screen**: Multi-generation line charts for Best/Average Fitness, Average Distance Traveled, Average Collisions, and Average Population Speed.

---

## How Training (Neuroevolution) Works

Each generation runs for a fixed duration (default `30s`, or `1,800` physics steps at `60 Hz`). At every step, each boid's fitness is updated based on its driving behavior:

- **Rewards**:
  - Forward distance traveled along the road (`+0.25` per pixel), weighted by alignment with the road direction (`cos(angle)`); only counted while moving forward and roughly aligned with the road, and halved while driving in the opposite lane
- **Penalties**:
  - Hitting road borders (`-5.0` per impact, with a `0.45s` cooldown)
  - Rear-ending another boid (`-4.5` per collision for the boid at fault, with a `0.45s` cooldown)
  - Head-on collisions (`-9.0` for both boids)
  - Driving in the opposite lane (`-1.5 / sec`; lane rules do not apply inside crossings)
  - Not giving right of way: entering a crossing past the stop line while required to yield (`-80`; it has to outweigh the distance lost by waiting)
  - Waiting at a stop line does not count as stalling
  - Idling or stalling below `6 px/s` (`-1.2 / sec`); a boid stuck for more than `3.6s` is respawned at a new location
  - Driving in reverse (`-0.1` per pixel reversed)

At the end of a generation, `Trainer` ranks all boids by fitness:
1. **Elitism**: The top 12% of brains (at least 4; 6 with the default population of 55), including the all-time best brain, are copied unchanged into the next generation.
2. **Tournament Selection & Crossover**: Parents are picked by tournament selection (`k = 4`) from the top 55% of the population and combined via neuron-wise and per-weight crossover.
3. **Gaussian Mutation**: Offspring weights and biases receive random perturbations (`mutationRate = 0.14`, `mutationStrength = 0.24`), clamped to `[-3.5, 3.5]`. The last 6 slots are fine-tuned copies of the top 3 brains with smaller mutations.

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
    │   ├── Road.js             # Two-way road segment, lane geometry & AABB
    │   ├── HeatGrid.js         # Coarse grid for traffic & crash heatmaps
    │   ├── City.js             # City graph, road distance field (SDF) & road queries
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
