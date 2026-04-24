# TDoA Solver Benchmarks

This folder contains temporary-but-useful native C++ benchmarks for estimating
the cost of the current Eigen-based TDoA Newton-Raphson solver.

The benchmark runs on the host machine, not on the ESP32S3. Use the results for
relative comparison and risk assessment, then validate on target hardware with
`esp_timer_get_time()` instrumentation around the solver path.

## Build And Run

From the repository root:

```bash
g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_DEBUG \
  -Ilib/Eigen \
  -Ilib/tdoa_newton_raphson/src \
  .agents/tdoa-precision/benchmarks/tdoa_solver_benchmark.cpp \
  lib/tdoa_newton_raphson/src/tdoa_newton_raphson.cpp \
  -o /tmp/tdoa_solver_benchmark

/tmp/tdoa_solver_benchmark
```

For a size-optimized build, closer to embedded release style:

```bash
g++ -std=c++17 -Os -DNDEBUG -DEIGEN_NO_DEBUG \
  -Ilib/Eigen \
  -Ilib/tdoa_newton_raphson/src \
  .agents/tdoa-precision/benchmarks/tdoa_solver_benchmark.cpp \
  lib/tdoa_newton_raphson/src/tdoa_newton_raphson.cpp \
  -o /tmp/tdoa_solver_benchmark_os

/tmp/tdoa_solver_benchmark_os
```

## Scenarios

The benchmark covers:

- 4-anchor 2D with 6 TDoA pair measurements.
- 4-anchor 3D with 6 measurements.
- 6-anchor 2D/3D with 15 measurements.
- 8-anchor 3D with all 28 measurements.
- 8-anchor 3D with sparse 12-measurement subsets.
- Optional forced maximum-iteration cases for 10 and 20 iterations.

It reports average, p50, p95, p99, max time, and average solver iterations.

## Allocation Tracking

To check whether the solver performs heap allocations inside each solve, build
with linker malloc wrappers:

```bash
g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_DEBUG \
  -DTRACK_MALLOC \
  -DEIGEN_USE_MALLOC -DEIGEN_STACK_ALLOCATION_LIMIT=0 \
  -Ilib/Eigen \
  -Ilib/tdoa_newton_raphson/src \
  .agents/tdoa-precision/benchmarks/tdoa_solver_benchmark.cpp \
  lib/tdoa_newton_raphson/src/tdoa_newton_raphson.cpp \
  -Wl,--wrap=malloc -Wl,--wrap=free -Wl,--wrap=calloc -Wl,--wrap=realloc \
  -o /tmp/tdoa_solver_benchmark_allocs

/tmp/tdoa_solver_benchmark_allocs 5000
```

This is a host-side approximation, but it is useful for identifying whether the
current Eigen usage is creating avoidable heap traffic.

## Covariance/Jacobian Overhead

To estimate the cost of the covariance calculation separately from the iterative
position solve:

```bash
g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_DEBUG \
  -DEIGEN_USE_MALLOC -DEIGEN_STACK_ALLOCATION_LIMIT=0 \
  -Ilib/Eigen \
  -Ilib/tdoa_newton_raphson/src \
  .agents/tdoa-precision/benchmarks/tdoa_covariance_overhead_benchmark.cpp \
  -o /tmp/tdoa_covariance_overhead

/tmp/tdoa_covariance_overhead 20000
```

This benchmark approximates the solver step and covariance math used by the
current implementation. It does not replace target-side timing on ESP32S3.
