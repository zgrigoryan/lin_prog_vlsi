# LP-Based Sequence-Pair Floorplanner

Educational C++17 implementation of the main algorithmic structure from:

Jae-Gon Kim and Yeong-Dae Kim, "A Linear Programming-Based Algorithm for Floorplanning in VLSI Design," IEEE TCAD, 2003.

The code implements sequence-pair topology search, hard and soft rectangular blocks, compact longest-path placement, and a fixed-sequence-pair LP model with an isolated solver backend. HiGHS is the default intended LP backend; MOSEK and CPLEX can be added by implementing `LPSolver`.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

Create experiment output directories, or clean solver run directories, with:

```bash
cmake --build build --target output-dirs
cmake --build build --target clean-outputs
```

If HiGHS is installed with a CMake package, the LP backend is enabled automatically. Without HiGHS, construction mode and tests still build, while LP runs report that the solver is unavailable.

### Installing HiGHS

One straightforward local install:

```bash
git clone https://github.com/ERGO-Code/HiGHS.git
cd HiGHS
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build --prefix ~/opt/highs
```

Then configure this floorplanner against that install:

```bash
cmake -S . -B build \
  -DFP_WITH_HIGHS=ON \
  -DCMAKE_PREFIX_PATH=$HOME/opt/highs

cmake --build build -j
ctest --test-dir build
```

To build explicitly without HiGHS:

```bash
cmake -S . -B build -DFP_WITH_HIGHS=OFF
cmake --build build
```

## CLI

```bash
./build/floorplanner --mcnc apte --mcnc-dir mcnc_hard --mode SA-CT-LP --solver highs --iterations 10000 --output out/apte
```

Recommended commands:

```bash
./build/floorplanner --mcnc apte --mcnc-dir mcnc_hard --mode CT --solver none --output out/apte_ct
./build/floorplanner --mcnc apte --mcnc-dir mcnc_hard --mode LP --solver highs --output out/apte_lp
./build/floorplanner --mcnc apte --mcnc-dir mcnc_hard --mode SA-CT --solver none --iterations 1000 --output out/apte_sa_ct
./build/floorplanner --mcnc apte --mcnc-dir mcnc_hard --mode SA-LP --solver highs --iterations 1000 --output out/apte_sa_lp
./build/floorplanner --mcnc apte --mcnc-dir mcnc_hard --mode SA-CT-LP --solver highs --iterations 1000 --output out/apte_sa_ct_lp
```

MCNC benchmark commands:

```bash
./build/floorplanner --mcnc apte --mcnc-dir mcnc_hard --mode SA-LP --solver highs --iterations 1000 --output out/apte
./build/floorplanner --blocks mcnc_hard/apte.block --nets mcnc_hard/apte.nets --mode SA-LP --solver highs --iterations 1000 --output out/apte
```

The MCNC reader supports the bundled hard-block benchmarks in `mcnc_hard/` and reads terminal coordinates as fixed I/O pads for HPWL and LP net bounding boxes. The `Outline:` line is treated as a true fixed outline.

Soft-block versions can be generated from the hard MCNC files:

```bash
sh run-mcnc-soft.sh
```

This writes MCNC-style `.block/.nets` files under `mcnc_soft/`. Each hard block is converted to a soft block with the same area. The generated soft block line format is:

```text
name soft area minAspectRatio maxAspectRatio
```

The default aspect-ratio bounds use the original ratio and its inverse:

```text
r = original_height / original_width
minAspectRatio = min(r, 1/r)
maxAspectRatio = max(r, 1/r)
```

Use `RATIO_PADDING` to expand these bounds:

```bash
RATIO_PADDING=1.2 sh run-mcnc-soft.sh
```

Run a soft benchmark with the regular JSON input path:

```bash
./build/floorplanner --mcnc apte --mcnc-dir mcnc_soft --mode SA-CT-LP --solver highs --iterations 1000 --output out/apte_soft
```

Modes:

- `CT`: deterministic construction for the identity sequence-pair.
- `LP`: LP optimization for the identity sequence-pair.
- `SA-CT`: simulated annealing evaluated by construction.
- `SA-LP`: simulated annealing evaluated by LP.
- `SA-CT-LP`: simulated annealing evaluated by construction, followed by LP refinement of the best sequence-pair.

Solvers:

- `highs`: implemented when compiled with `FP_WITH_HIGHS=ON` and HiGHS is found.
- `none`: allowed for `CT` and `SA-CT`; LP-backed modes fail with a clear error.
- `mosek`: planned backend, not implemented yet.
- `cplex`: planned backend, not implemented yet.

LP-backed modes require an available solver. `CT` and `SA-CT` do not.

### Exporting LP Models

Use `--export-lp` or `--export-mps` to write the LP model for the selected sequence-pair:

```bash
./build/floorplanner \
  --mcnc apte \
  --mcnc-dir mcnc_hard \
  --mode LP \
  --solver highs \
  --export-mps out/model.mps \
  --output out/lp
```

For simulated annealing modes, export writes only the final best sequence-pair model, not every intermediate candidate. The exported model is intended for comparison with HiGHS command-line tools, MOSEK, and CPLEX.

Convenience scripts:

```bash
sh run-highs.sh
sh run-mosek.sh
```

By default these run the `apte` MCNC benchmark with `SA-CT-LP`, export `model.mps`/`model.lp`, generate `floorplan.png`, and verify the exported MPS with the external solver. Override defaults with environment variables:

```bash
BENCHMARK=ami33 ITERATIONS=2000 sh run-highs.sh
BENCHMARK=ami33 ITERATIONS=2000 sh run-mosek.sh
INPUT=custom.json MODE=LP OUTPUT=out/custom_highs sh run-highs.sh
```

Annealing workflow options adapted from the companion 3D floorplanner are available when you want more robust experimental runs:

```bash
AUTO_TEMPERATURE=1 VERBOSE_SA=1 BENCHMARK=ami33 ITERATIONS=5000 sh run-highs.sh
AUTO_EPOCH_LENGTH=1 BENCHMARK=ami49 ITERATIONS=10000 sh run-highs.sh
```

`AUTO_TEMPERATURE=1` samples random sequence-pair moves and calibrates the starting temperature for an approximate 80% initial acceptance rate. `AUTO_EPOCH_LENGTH=1` uses `max(200, num_blocks^2)` moves per cooling step. `VERBOSE_SA=1` prints temperature, acceptance rate, and feasible-best progress.
The summary JSON records both requested and effective annealing settings, including `initialTemperatureUsed`, `epochLengthUsed`, `totalMoves`, and `acceptedMoves`.

To run the bundled MCNC set with both external solver checks:

```bash
ITERATIONS=1000 sh run-all-mcnc.sh
```

This runs `apte`, `xerox`, `hp`, `ami33`, and `ami49` with both HiGHS and MOSEK checks. Outputs are written under:

```text
out/mcnc_SA-CT-LP/<benchmark>_highs/
out/mcnc_SA-CT-LP/<benchmark>_mosek/
```

Each successful result directory contains `summary.json`, `placements.csv`, `model.mps`, `model.lp`, solver solution files, and `floorplan.png`.

Analyze previous runs:

```bash
sh run-analyze.sh
```

This scans `out/` and writes aggregate tables and plots under:

```text
results/comparisons/
```

Outputs:

- `placements.csv`: `block_name,x,y,width,height,type,layer`
- `summary.json`: final metrics, chosen sequence-pair, run metadata, problem metadata
- console summary with objective, chip size, wirelength, runtime, and block placements

## Notes

- Aspect ratio is represented as `height / width`, matching `width = sqrt(area / r)` and `height = sqrt(area * r)`.
- The LP objective uses `areaWeight * (W + H) + wireWeight * sum(netWidth + netHeight)` as a linearized chip-size objective.
- Soft-block area is enforced with a tangent-style linear surrogate `w + alpha*h >= 2*sqrt(area*alpha)` and an iterative alpha correction loop.
- Nets use block centers for HPWL and LP bounding boxes. Pin-aware wirelength is intentionally simplified.
- Hard-block orientations are fixed before LP by running the construction method for the same sequence-pair.
- The LP applies transitive reduction to sequence-pair precedence DAGs before adding non-overlap constraints, following Kim and Kim's goal of reducing redundant LP constraints.
- Soft-block construction candidates are log-spaced aspect ratios.
- Exact pin orientation/flipping and 3D extensions are not implemented in this 2D baseline.

## Paper Fidelity

The implementation follows the paper's main architecture: sequence-pair topology, simulated annealing topology search, construction-based hard orientation selection, and LP-based coordinate/dimension optimization for a fixed sequence-pair.

Intentional simplifications and engineering choices:

- Block-center HPWL is used for movable block pins; MCNC terminals are read as fixed pads.
- Soft-block area is handled by iterative linear cuts instead of an exact nonlinear model.
- Soft construction candidates are log-spaced aspect ratios.
- `SA-LP` uses a construction penalty only when an LP candidate is infeasible, so fixed-outline MCNC searches can keep moving.
- MOSEK is used through exported MPS/LP files, not a native C++ backend yet.
