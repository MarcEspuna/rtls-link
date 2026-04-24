# TDoA Solver Benchmark Results

Date: 2026-04-16

Machine: host Linux, `g++ 13.3.0`

Important limitation: these are native host timings, not ESP32S3 timings. They
are useful for comparing scenario shape and identifying obvious algorithmic
risks, but target timing still needs `esp_timer_get_time()` or cycle-counter
instrumentation on the board.

## Code Path

Benchmarked the current solver implementation:

- `lib/tdoa_newton_raphson/src/tdoa_newton_raphson.hpp`
- `lib/tdoa_newton_raphson/src/tdoa_newton_raphson.cpp`

Runtime firmware currently calls the solver from:

- `src/uwb/uwb_tdoa_tag.cpp`

The current firmware constant is `NUM_ITERATIONS = 10`. The benchmark also
includes forced 20-iteration cases for stress testing.

## Host Build Flags Matching ESP Eigen Defines

```bash
g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_DEBUG \
  -DEIGEN_USE_MALLOC -DEIGEN_STACK_ALLOCATION_LIMIT=0 \
  -Ilib/Eigen \
  -Ilib/tdoa_newton_raphson/src \
  .agents/tdoa-precision/benchmarks/tdoa_solver_benchmark.cpp \
  lib/tdoa_newton_raphson/src/tdoa_newton_raphson.cpp \
  -o /tmp/tdoa_solver_benchmark_o2_esp_eigen
```

Results, 20,000 samples:

| Scenario | Measurements | Dim | Mean | P95 | P99 | Avg Iterations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 anchors 2D all-pairs | 6 | 2 | 0.78 us | 0.87 us | 0.88 us | 3.57 |
| 4 anchors 3D all-pairs | 6 | 3 | 1.96 us | 5.43 us | 6.04 us | 6.76 |
| 6 anchors 2D all-pairs | 15 | 2 | 1.21 us | 1.31 us | 1.33 us | 3.64 |
| 6 anchors 3D all-pairs | 15 | 3 | 2.24 us | 2.72 us | 3.11 us | 4.72 |
| 8 anchors 3D sparse | 12 | 3 | 1.54 us | 1.67 us | 1.70 us | 3.75 |
| 8 anchors 3D all-pairs | 28 | 3 | 2.63 us | 2.86 us | 2.90 us | 3.73 |
| 8 anchors 3D forced-10 | 28 | 3 | 5.66 us | 5.83 us | 5.89 us | 10.00 |
| 8 anchors 3D forced-20 | 28 | 3 | 11.16 us | 11.51 us | 17.89 us | 20.00 |

## Size-Optimized Host Build

```bash
g++ -std=c++17 -Os -DNDEBUG -DEIGEN_NO_DEBUG \
  -DEIGEN_USE_MALLOC -DEIGEN_STACK_ALLOCATION_LIMIT=0 \
  ...
```

Results, 20,000 samples:

| Scenario | Measurements | Dim | Mean | P95 | P99 | Avg Iterations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 anchors 2D all-pairs | 6 | 2 | 2.28 us | 2.52 us | 2.60 us | 3.57 |
| 4 anchors 3D all-pairs | 6 | 3 | 6.33 us | 16.91 us | 18.57 us | 6.76 |
| 6 anchors 2D all-pairs | 15 | 2 | 4.17 us | 4.50 us | 4.58 us | 3.64 |
| 6 anchors 3D all-pairs | 15 | 3 | 8.28 us | 9.98 us | 17.95 us | 4.72 |
| 8 anchors 3D sparse | 12 | 3 | 5.85 us | 6.24 us | 6.55 us | 3.75 |
| 8 anchors 3D all-pairs | 28 | 3 | 10.73 us | 11.42 us | 17.64 us | 3.73 |
| 8 anchors 3D forced-10 | 28 | 3 | 23.36 us | 23.66 us | 35.33 us | 10.00 |
| 8 anchors 3D forced-20 | 28 | 3 | 46.46 us | 47.14 us | 58.80 us | 20.00 |

## Allocation Tracking

Allocation-tracking command:

```bash
g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_DEBUG \
  -DTRACK_MALLOC \
  -DEIGEN_USE_MALLOC -DEIGEN_STACK_ALLOCATION_LIMIT=0 \
  ...
  -Wl,--wrap=malloc -Wl,--wrap=free -Wl,--wrap=calloc -Wl,--wrap=realloc \
  -o /tmp/tdoa_solver_benchmark_allocs
```

Results, 5,000 samples:

| Scenario | Mean malloc calls / solve | Mean bytes / solve |
| --- | ---: | ---: |
| 4 anchors 2D all-pairs | 0.00 | 0.00 |
| 4 anchors 3D all-pairs | 1.97 | 47.30 |
| 6 anchors 2D all-pairs | 0.00 | 0.00 |
| 6 anchors 3D all-pairs | 0.02 | 0.48 |
| 8 anchors 3D sparse | 0.00 | 0.00 |
| 8 anchors 3D all-pairs | 0.00 | 0.00 |
| 8 anchors 3D forced-10 | 0.00 | 0.00 |
| 8 anchors 3D forced-20 | 0.00 | 0.00 |

The realistic 8-anchor 3D cases did not show heap allocation inside each solve
in this native build. The allocation seen in weak 4-anchor 3D geometry appears
linked to the less stable convergence/covariance path.

## Covariance/Jacobian Overhead

The covariance sent through MAVLink is not the Jacobian itself. The solver
recomputes a final Jacobian and estimates:

```text
P = inverse(J^T J) * measurement_variance
```

The following benchmark estimates the iterative 3D solve cost separately from
the final covariance/Jacobian cost:

```bash
g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_DEBUG \
  -DEIGEN_USE_MALLOC -DEIGEN_STACK_ALLOCATION_LIMIT=0 \
  -Ilib/Eigen \
  -Ilib/tdoa_newton_raphson/src \
  .agents/tdoa-precision/benchmarks/tdoa_covariance_overhead_benchmark.cpp \
  -o /tmp/tdoa_covariance_overhead_o2

/tmp/tdoa_covariance_overhead_o2 20000
```

Host `-O2` results:

| Scenario | Step | Mean | P95 | P99 |
| --- | --- | ---: | ---: | ---: |
| 8 anchors all pairs | solve without covariance | 2.120 us | 2.323 us | 2.363 us |
| 8 anchors all pairs | covariance only | 0.471 us | 0.483 us | 0.523 us |
| 8 anchors sparse 12 | solve without covariance | 1.305 us | 1.449 us | 1.489 us |
| 8 anchors sparse 12 | covariance only | 0.226 us | 0.242 us | 0.271 us |

Host `-Os` results:

| Scenario | Step | Mean | P95 | P99 |
| --- | --- | ---: | ---: | ---: |
| 8 anchors all pairs | solve without covariance | 8.366 us | 8.973 us | 10.884 us |
| 8 anchors all pairs | covariance only | 1.719 us | 1.761 us | 3.430 us |
| 8 anchors sparse 12 | solve without covariance | 4.471 us | 4.828 us | 4.919 us |
| 8 anchors sparse 12 | covariance only | 0.962 us | 1.006 us | 1.127 us |

Interpretation: covariance computation is smaller than the iterative solve, but
not negligible. In these host runs it was about 15-20 percent of the 3D solve
cost. On ESP32S3 it should be guarded by a runtime/compile-time option because
the current solver uses software-emulated double arithmetic.

## Interpretation

The host benchmark does not indicate a matrix-size explosion from moving to 3D.
With 8 anchors and all 28 pair measurements, the current solver converged in
about 3.7 iterations in the synthetic geometry.

The more important ESP32S3 concern is numeric type:

- the current solver uses `double`;
- ESP32S3 has hardware acceleration for single-precision floating point;
- double-precision operations are software-emulated on Espressif cores with
  single-precision FPUs.

This means host x86 timings understate target cost. A float-based solver is
likely a better long-term embedded path if target measurements show the current
double solver is expensive.

## ESP32S3 Cross-Compile Check

Compiled the current solver object with the PlatformIO ESP32S3 toolchain:

```bash
~/.platformio/packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++ \
  -std=c++2a -Os -DNDEBUG -DESP32S3_UWB_BOARD \
  -DEIGEN_USE_MALLOC -DEIGEN_STACK_ALLOCATION_LIMIT=0 \
  -Ilib/Eigen \
  -Ilib/tdoa_newton_raphson/src \
  -c lib/tdoa_newton_raphson/src/tdoa_newton_raphson.cpp \
  -o /tmp/tdoa_newton_raphson_esp32s3.o
```

Object size:

```text
text=39986 bytes, data=0, bss=20
```

Undefined software-double/math symbols included:

```text
__adddf3
__subdf3
__muldf3
__divdf3
__floatsidf
__eqdf2
__gedf2
__gtdf2
__ledf2
__ltdf2
__nedf2
sqrt
```

This confirms that the current target build is using software-emulated double
arithmetic for the solver. That is the main reason to benchmark on hardware
before enabling 3D permanently.

## Recommended Target Instrumentation

Add timing around the solver call in `src/uwb/uwb_tdoa_tag.cpp`:

```cpp
const int64_t t0 = esp_timer_get_time();
auto result = tdoa_estimator::newtonRaphson(...);
const int64_t t1 = esp_timer_get_time();
LOG_DEBUG("TDoA solver: %lld us, iter=%d, meas=%d, cov=%d",
          static_cast<long long>(t1 - t0),
          result.iterations,
          static_cast<int>(tdoas.size()),
          result.covarianceValid ? 1 : 0);
```

Measure:

- 2D current mode,
- 3D with 8 anchors sparse pairs,
- 3D with 8 anchors all pairs,
- forced or bad initial guess,
- covariance enabled/disabled after adding a solver option to skip covariance.

## Optimization Notes

If target timing is too high:

1. Convert solver math to `float` or add a `float` 3D solver variant.
2. Add an option to skip covariance calculation inside the solver when the
   caller does not need MAVLink covariance.
3. Reuse residual/Jacobian work between step calculation and covariance.
4. Replace `completeOrthogonalDecomposition()` with a small fixed-size normal
   equation solve using `J.transpose() * J` and LDLT/LLT if numerical behavior
   is acceptable.
5. Keep `NUM_ITERATIONS = 10` or lower after validating convergence from the
   previous position estimate.
