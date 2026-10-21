# Boid Boulevard

**Boid Boulevard** is a browser-based 2D city traffic and neuroevolution simulation built with vanilla HTML5 Canvas, CSS, and Object-Oriented JavaScript (zero external dependencies). Autonomous vehicle agents (**boids**) equipped with front-facing raycast sensors and feedforward neural networks learn from scratch how to steer, accelerate, brake, take left and right turns, and avoid collisions on a closed-loop track with two-way traffic.

---

## Key Features

- **Closed-Loop Track Generation**
  - Each city is a single **closed circuit without any splits**: every corner joins exactly two roads, so there are no crossings or forks.
  - `10`-`14` corner points are placed around the map center at increasing angles (so the loop never crosses itself); `3`-`4` non-adjacent corners are pulled inward, so the track has at least **`3` turns to each side** (left and right, depending on the driving direction).
  - Every candidate track is validated (no turn sharper than ~`110°`, segments at least `110 px`, at least `24 px` of space between the road surfaces of non-adjacent segments); invalid candidates are discarded and a new one is drawn from the same seed.
  - **Two-way roads** (`64 px` wide) with a dashed centerline: traffic keeps right, each direction in its own lane. In headless tests with 120 boids, `64 px` roads halved the share of stuck boids compared with `53 px` (fewer traffic jams, ~27% more distance, ~30% fewer collisions); wider roads make driving easier but do not make the networks smarter (networks trained on narrow or wide roads drive equally well on the same test).
- **Deterministic Random Seed**
  - Uses a seeded Mulberry32 PRNG (`Random`) so entering the same seed reproduces the exact same city layout, spawn poses, and genetic mutations.
- **Feedforward Neural Network (`[17 -> 12 -> 8 -> 2]`)**
  - **17 Inputs**:
    - `7` Front Road-Border Ray Sensors (`-75°`, `-50°`, `-25°`, `0°`, `+25°`, `+50°`, `+75°`)
    - `7` Front Boid-Detection Ray Sensors (same 7 angles)
    - `1` Closing Speed sensor toward the boid straight ahead (center ray only, so oncoming traffic in the other lane does not trigger it)
    - `1` Normalized Current Speed (`speed / maxSpeed`)
    - `1` Signed heading error toward a lookahead point in the boid's own (right-hand) lane, or into the correct lane of the next road when taking a corner
  - **Two Hidden Layers** of `12` and `8` neurons (`tanh` activation). In headless tests (3 seeds, trained from scratch) this layout overtook the earlier single `10`-neuron layer after ~300 generations (about 3% more distance, 6% higher fitness, 20% fewer collisions), but it learns more slowly during the first ~100 generations. A later comparison on the closed-loop tracks with the full-traffic benchmark (200 generations, 3 seeds) confirmed it: two hidden layers (`[17, 12, 8, 2]` and `[17, 16, 12, 2]`) reached ~91% benchmark efficiency, single hidden layers of `6` or `16` neurons ~88% with more collisions, and the larger `[17, 16, 12, 2]` was not clearly better than the current layout.
  - **2 Continuous Outputs** (`tanh` activation in `[-1, +1]`):
    1. **Steering Direction**: `-1` (full left) to `+1` (full right)
    2. **Bidirectional Throttle / Brake**: `> 0` accelerates forward; `< 0` brakes when moving forward or reverses from a stop
- **1 to 4 Selectable Boid Archetypes**
  - Choose between **1 to 4 vehicle types** (`Sedan`, `Compact`, `Sports`, `Heavy`), each with distinct top speed, acceleration, braking force, turn rate, radius, and mass.
  - **Default: 1 type (`Sedan`).** In headless tests (55 boids, 60 generations, 3 seeds) a single type gave fewer collisions (`9.3` vs `12.2` with 2 types and `19.0` with 4), fewer stuck boids and less noisy scores. With several types the fastest one takes over the population anyway (Sedan ~91% with 2 types, Sports ~61% with 4), because fitness rewards distance.
  - Offspring inherit their parent's vehicle archetype during evolution so neural networks specialize in their vehicle's physical dynamics.
- **Non-Collapsing Physics**
  - Hard road-border containment and pairwise collision resolution keep boids from overlapping or escaping the road network.
  - Each boid has an elliptical collision shape (its length front-to-back, its width side-by-side), so boids passing each other in opposite lanes do not collide.
- **High-Speed & Decoupled "Turbo" Training**
  - Fixed-timestep (`1/60s`) physics loop supports `1x`, `2x`, `4x`, `8x`, `16x`, `32x`, and `64x` speed multipliers without physics tunneling.
  - Boids **sense and decide at 30 Hz** (every other physics step, staggered so half of the boids think each step) and hold their steering and throttle in between, while movement, collisions and fitness keep running at 60 Hz. This makes a step ~1.4-1.7x faster; in headless tests (60 generations, 3 seeds) training results stayed within noise of deciding every step.
  - Configurable generation duration (`15s`, `30s`, `45s`, `60s`).
  - **Turbo (Skip Render)** mode decouples visualization from physics: each animation frame runs as many physics steps as fit in a `14 ms` time budget (instead of a fixed count) and renders the canvas only once per completed generation, so any physics speedup directly becomes training throughput.
- **Signed Distance Field Road Queries**
  - On creation, each city precomputes a `2 px` grid of signed distances to the nearest road border (~160 KB, built in under 1 ms).
  - On-road checks are a single bilinear lookup, and road-border sensor rays use sphere tracing over the field instead of fixed-step sampling, making road raycasts 2-3x faster and more accurate. Road rays return a plain distance, so sensor updates allocate nothing.
- **Spatial Grid for Boid Neighbors**
  - Every step, boids are bucketed into a `128 px` grid (counting sort into flat typed arrays). Sensors only test boids in nearby cells, and collisions only pair boids in the same or adjacent cells, making collision resolution 2-3.5x faster.
  - The grid gives exactly the same sensor readings and collision pairs as checking every boid: a ray starting inside another boid always counts as distance `0`, and equal distances go to the lowest boid id, so results never depend on the order boids are checked in.
- **Automatic Persistence (`localStorage`)**
  - Automatically saves the city map, generation counter, elite neural networks, and historical chart metrics after every generation.
  - Saved networks are only reused if they match the current network layout; networks from an older layout (or a corrupted save) are discarded on load, and training continues with fresh networks.
  - Chart history keeps the last `500` generations (in memory and in storage); older generations are dropped, so the charts show the most recent 500 generations at full detail.
  - Includes a **Restart & Clear Local Storage** button to wipe saved data and start fresh.
- **Three Built-In Screens**
  1. **City Simulation Screen**: Interactive canvas view of the city, boids, and raycast sensors (click any boid to inspect it). **Ghost Trails** (checkbox, on by default) draw a fading line in each boid's color along its last ~`1.3 s` of driving (a position every `4` physics steps, the last `20` kept); a boid that is respawned after getting stuck starts a fresh trail. With the same checkbox, every respawn is shown for `2 s` (simulated time): a fading **ghost dot** where the boid got stuck, a faint dashed line to where it was placed, and an expanding ring at the new spot. An **Overlay** selector draws one of these over the roads:
     - **Traffic heatmap**: where boids spend their time
     - **Crash heatmap**: where road-border and boid-to-boid collisions happen
     - **Road distance field (SDF)**: the precomputed signed distance field used for road queries

     Heatmaps fade by half at the end of every generation, so they reflect recent driving behavior.
  2. **Debug Screen**: Live neural network topology & synapse activation visualizer, steering/throttle gauges, sensor readouts, and city/storage diagnostics.
  3. **Training Progress Charts Screen**: Multi-generation line charts for Best/Average/Benchmark Fitness, Best/Average/Benchmark Distance Traveled, Average/Benchmark Collisions per boid, and Average/Benchmark **Efficiency** (distance covered as a share of the maximum possible distance, i.e. top speed x driving time).

---

## How Training (Neuroevolution) Works

Each generation runs for a fixed duration (default `30s`, or `1,800` physics steps at `60 Hz`). At every step, each boid's fitness is updated based on its driving behavior:

- **Rewards**:
  - Forward distance traveled along the road (`+0.25` per pixel), weighted by alignment with the road direction (`cos(angle)`); only counted while moving forward and roughly aligned with the road, and halved while driving in the opposite lane
- **Penalties**:
  - Hitting road borders (`-5.0` per impact, with a `0.45s` cooldown)
  - Rear-ending another boid (`-4.5` per collision for the boid at fault, with a `0.45s` cooldown)
  - Head-on collisions (`-9.0` for both boids)
  - Driving in the opposite lane (`-1.5 / sec`; lane rules do not apply inside corners, where the lanes of two roads overlap)
  - Idling or stalling below `6 px/s` (`-1.2 / sec`); a boid stuck for more than `3.6s` is respawned at a new location
  - Driving in reverse (`-0.1` per pixel reversed)

Generations start from **evenly spread spawn positions** (spaced along all roads, alternating directions, randomly assigned to boids) instead of independent random spots, so fewer boids get an unusually easy or hard start.

At the end of a generation, `Trainer` ranks all boids by the **average score of their network over its last 5 evaluations** (a network keeps its score history only while it is copied unchanged; crossover or mutation starts a new one). A one-off lucky run therefore cannot dominate: unchanged elites are re-evaluated every generation and fall back if they were only lucky.
1. **Champion**: The network with the best average over at least `3` evaluations becomes the champion and is re-inserted unchanged (slot 0). It is re-evaluated every generation, so a better-performing network can take over. The header's "All-Time Best" shows the best champion average reached so far.
2. **Elitism**: The top 12% of networks by average score (at least 4; 6 with the default population of 55), including the champion, are copied unchanged into the next generation.
3. **Tournament Selection & Crossover**: Parents are picked by tournament selection (`k = 4`, on average score) from the top 55% of the population and combined via neuron-wise and per-weight crossover.
4. **Heavy-Tailed Gaussian Mutation**: Offspring weights and biases receive random perturbations (`mutationRate = 0.14`, `mutationStrength = 0.24`), clamped to `[-3.5, 3.5]`; `10%` of mutation steps are `5x` larger (`MUTATION_JUMP_CHANCE`, `MUTATION_JUMP_FACTOR`), so most changes stay small while occasional big jumps explore further. In headless tests (200 generations, 3 seeds) this gave the best benchmark result (~92% efficiency, ~2.7 collisions per car, on every seed), while uniformly stronger mutation (`0.30` / `0.50`) was clearly worse (~86%, twice the collisions in the population) and uniformly weaker mutation learned more slowly. The last 6 slots are fine-tuned copies of the top 3 networks with smaller mutations.

**City rotation** (checkbox "Rotate City", on by default): every `10` generations training continues on a new random city, so networks learn to drive rather than memorizing one map. The new city's seed comes from the seeded generator, so a run stays reproducible.

**Benchmark ("driving test")**: after generation 1 and every `10`th generation the top `10` networks drive in **full traffic**: `55` cars (copies of the top networks, all `Sedan`) for `30 s` on a fixed benchmark track with fixed spawns and vehicle properties (its own seed, without disturbing the training run). It records distance, collisions, efficiency and fitness per car. Because the conditions are identical every time, the orange **Benchmark** lines show real progress, while the best/average lines include luck and city changes. (An earlier, easier benchmark of 10 cars for 15 s already reached ~96% of the maximum distance within 30 generations, so it could no longer show progress.) A benchmark run costs about as much as a short generation, roughly 8% extra training time.

**Why collision penalties are not higher**: in headless tests (150 generations, 3 seeds) multiplying all collision penalties by 2, 4 or 8 hardly reduced collisions (6.3 to 5.6 per boid) but made boids slow and hesitant (efficiency 82% down to 55%, stuck boids 2% up to 19%), so the penalties stay as listed above.

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
    │   ├── SpatialGrid.js      # Per-step boid bucketing for fast neighbor queries
    │   ├── City.js             # City graph, road distance field (SDF) & road queries
    │   └── CityGenerator.js    # Procedural closed-loop track generator
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
