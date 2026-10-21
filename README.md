# Boid Boulevard

**Boid Boulevard** is a browser-based 2D city traffic and neuroevolution simulation built with vanilla HTML5 Canvas, CSS, and Object-Oriented JavaScript (zero external dependencies). Autonomous vehicle agents (**boids**) equipped with front-facing raycast sensors and feedforward neural networks learn from scratch how to steer, accelerate, brake, take left and right turns, and avoid collisions on a closed-loop or multi-intersection track with two-way traffic.

---

## Key Features

- **Selectable Track Topologies (`Closed Loop`, `Cross-Link Circuit`, `City Grid`)**
  - **Closed Loop (default)**: A single **closed circuit without any splits**: `10`-`14` corner points are placed around the map center at increasing angles; `3`-`4` non-adjacent corners are pulled inward so the track has at least **`3` turns to each side** (left and right). Every candidate track is validated (no turn sharper than ~`110°`, segments at least `110 px`, at least `24 px` of clearance between road surfaces).
  - **Cross-Link Circuit**: A validated closed loop plus `1`-`2` interior chord roads connecting opposite sides of the circuit, creating real 3-way intersections and route choices.
  - **City Grid**: A `4x3` multi-block street grid with organic intersection jitter, 3-way T-junctions, and 4-way crossings.
  - **Two-way roads** (`64 px` wide) with a dashed centerline: traffic keeps right, each direction in its own lane.
- **Deterministic Random Seed & Isolated PRNG Streams**
  - Uses a seeded Mulberry32 PRNG (`Random`) with Polar Marsaglia Gaussian sampling and isolated `Random` instance streams (`trainer.cityRng`) so entering the same seed reproduces the exact same city rotation sequence even when interacting with the UI.
- **Feedforward Neural Network (`[17 -> 12 -> 8 -> 2]`)**
  - Contiguous `Float32Array` weights, biases, and activation buffers with zero-allocation forward passes and fast typed-array cloning.
  - **17 Inputs**:
    - `7` Front Road-Border Ray Sensors (`-75°`, `-50°`, `-25°`, `0°`, `+25°`, `+50°`, `+75°`)
    - `7` Front Boid & Obstacle Ray Sensors (same 7 angles, using oriented elliptical ray-vehicle intersection)
    - `1` Closing Speed sensor toward same-direction traffic or obstacles straight ahead
    - `1` Normalized Current Speed (`speed / maxSpeed`)
    - `1` Signed heading & lane-offset error toward a lookahead point in the boid's own (right-hand) lane
  - **Two Hidden Layers** of `12` and `8` neurons (`tanh` activation).
  - **2 Continuous Outputs** (`tanh` activation in `[-1, +1]`):
    1. **Steering Direction**: `-1` (full left) to `+1` (full right)
    2. **Bidirectional Throttle / Brake**: `> 0` accelerates forward; `< 0` brakes when moving forward or reverses from a stop
- **1 to 4 Selectable Boid Archetypes with Niche Speciation**
  - Choose between **1 to 4 vehicle types** (`Sedan`, `Compact`, `Sports`, `Heavy`), each with distinct top speed, acceleration, braking force, turn rate, dimensions, and mass.
  - When multiple vehicle types are active, fitness evaluation normalizes forward distance by vehicle speed ratio and preserves elite representatives per archetype (**niche elitism**), so slower `Heavy` and `Compact` vehicles specialize alongside `Sedan` and `Sports`.
- **Non-Collapsing Elliptical Physics & SDF Wall Sliding**
  - Hard road-border containment (using SDF gradient inward normals) and pairwise collision resolution keep boids from overlapping or escaping the road network.
  - Both collision physics and sensor raycasts use each boid's **oriented elliptical shape** (lengthwise front-to-back, widthwise side-by-side), so boids passing each other in opposite lanes neither collide nor trigger false oncoming-traffic braking.
- **Interactive Sandbox Controls**
  - **Interactive Roadblocks (`Shift+Click`)**: `Shift+Click` anywhere on a road to place or remove a circular hazard obstacle; boids detect roadblocks with their front ray sensors and steer around them in real time.
  - **JSON Export & Import**: Export trained champion/elite brains, training history, and city layouts to a `.json` file or import a saved `.json` snapshot at any time.
- **High-Speed & Decoupled "Turbo" Training**
  - Fixed-timestep (`1/60s`) physics loop supports `1x` to `64x` speed multipliers without physics tunneling.
  - Boids **sense and decide at 30 Hz** (every other physics step, staggered by boid ID) with cached heading trigonometry (`cosH`, `sinH`), precomputed sensor ray rotation tables, zero-allocation road state queries, and a single `SpatialGrid` rebuild per step.
  - **Turbo (Skip Render)** mode runs as many physics steps as fit in a `14 ms` frame budget and renders only once per completed generation.
- **Three Built-In Screens**
  1. **City Simulation Screen**: Interactive canvas view with cached static road layer, **Ghost Trails**, respawn indicators, and selectable **Overlays** (Traffic heatmap, Crash heatmap, Road distance field SDF).
  2. **Debug Screen**: Live neural network topology & synapse activation visualizer, steering/throttle gauges, sensor readouts, and city/storage diagnostics.
  3. **Training Progress Charts Screen**: Multi-generation line charts for Best/Average/Benchmark Fitness, Distance Traveled, Collisions per boid, and **Efficiency** (% of maximum possible distance).

---

## Project Structure (OOP Architecture)

```text
boid-boulevard/
├── index.html                  # Main application layout & controls
├── css/
│   └── style.css               # Utilitarian UI styling
├── scripts/
│   └── headless-bench.js       # Node.js CLI multi-seed headless benchmark runner
└── js/
    ├── main.js                 # Application bootstrap
    ├── Simulation.js           # Main simulation controller, loop & UI bindings
    ├── math/
    │   ├── Random.js           # Deterministic seeded PRNG (Mulberry32 + Polar Marsaglia)
    │   └── Vector2D.js         # Zero-allocation 2D vector & ray-ellipse math
    ├── nn/
    │   └── NeuralNetwork.js    # Float32Array feedforward network, crossover & mutation
    ├── world/
    │   ├── RoadNode.js         # Intersection / waypoint node
    │   ├── Road.js             # Two-way road segment, precomputed normals & lane geometry
    │   ├── HeatGrid.js         # Coarse grid for traffic & crash heatmaps
    │   ├── SpatialGrid.js      # Per-step counting-sort boid bucketing for neighbor queries
    │   ├── City.js             # City graph, road SDF, obstacles & zero-GC road queries
    │   └── CityGenerator.js    # Procedural loop, cross-link & city grid track generator
    ├── entities/
    │   ├── Sensor.js           # 7-ray SDF road border & elliptical boid/obstacle raycaster
    │   └── Boid.js             # Autonomous vehicle physics, archetypes & fitness
    ├── training/
    │   └── Trainer.js          # Genetic algorithm, niche speciation & benchmark runner
    ├── storage/
    │   └── StorageManager.js   # Cached localStorage persistence & JSON file export/import
    └── ui/
        ├── SimulationRenderer.js # Cached canvas renderer for city, overlays, obstacles & boids
        ├── DebugScreen.js        # Live neural network & telemetry inspector
        └── ChartScreen.js        # Multi-generation training progress charts
```

---

## Running the Simulation & Headless Benchmarks

No build step or package installation is required:

1. Open `index.html` directly in any modern web browser, or serve the folder locally:
   ```bash
   npx serve .
   ```
2. Run the automated multi-seed headless benchmark in Node.js:
   ```bash
   node scripts/headless-bench.js 50
   ```
