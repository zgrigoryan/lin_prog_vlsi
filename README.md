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
./build/floorplanner --input examples/small.json --mode SA-LP --solver highs --iterations 10000 --output out/
```

Recommended commands:

```bash
./build/floorplanner --input examples/small.json --mode CT --solver none --output out/ct
./build/floorplanner --input examples/small.json --mode LP --solver highs --output out/lp
./build/floorplanner --input examples/small.json --mode SA-CT --solver none --iterations 1000 --output out/sa_ct
./build/floorplanner --input examples/small.json --mode SA-LP --solver highs --iterations 1000 --output out/sa_lp
./build/floorplanner --input examples/small.json --mode SA-CT-LP --solver highs --iterations 1000 --output out/sa_ct_lp
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
  --input examples/small.json \
  --mode LP \
  --solver highs \
  --export-mps out/model.mps \
  --output out/lp
```

For simulated annealing modes, export writes only the final best sequence-pair model, not every intermediate candidate. The exported model is intended for comparison with HiGHS command-line tools, MOSEK, and CPLEX.

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
- The current LP keeps all pairwise sequence-pair precedence constraints. Redundant-constraint removal from Kim and Kim is left as future work.
- Soft-block construction candidates are log-spaced aspect ratios.
- Exact pin orientation/flipping and 3D extensions are not implemented in this 2D baseline.
