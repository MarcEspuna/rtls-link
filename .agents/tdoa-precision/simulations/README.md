# TDoA Precision Simulations

Run from the repository root:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario all
```

Run one scenario:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario baseline-2d
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario consistency-3d
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario height-separation-3d
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario height-separation-fine-3d
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario self-survey
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario quantization
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario smoothing
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario dynamic-adelay
```

Use a fixed seed for repeatability:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py --scenario all --seed 42
```

The script depends only on Python 3 and NumPy.

The model is intentionally compact. It is not a physics-accurate DW1000
simulator; it is a strategy-comparison harness. Use it to compare correction
ideas and expected sensitivity, then validate with the static-tag experiment.

## Follow-Up Harnesses

Online per-anchor correction experiments:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_novel_ideas_sim.py \
  --layout 4_2d --seed 1 --repeats 3

python3 .agents/tdoa-precision/simulations/tdoa_novel_ideas_sim.py \
  --layout 8_3d --seed 1 --repeats 3
```

Latest strategy-stack comparison:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_strategy_stack_sim.py \
  --layout 4_2d --model all --seeds 16 --point-repeats 16

python3 .agents/tdoa-precision/simulations/tdoa_strategy_stack_sim.py \
  --layout 8_3d --model all --seeds 10 --point-repeats 10
```

Symmetric-only fallback sensitivity:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_strategy_stack_sim.py \
  --layout 4_2d --model symmetric_good --seeds 16 --point-repeats 16
```

Use `strategy-stack-results.md` for the current implementation-order
recommendation.

Practical Z-geometry search:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_geometry_search_sim.py --dop-only
python3 .agents/tdoa-precision/simulations/tdoa_geometry_search_sim.py --mc-finalists 6
```

Use `geometry-search-results.md` for the current anchor-layout recommendation.
