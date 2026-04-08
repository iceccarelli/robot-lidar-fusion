# Benchmark Scenarios

This directory contains the deterministic scenario definitions used to validate the repository's simulation-facing contracts, runtime telemetry, health events, and regression artifacts.

## Included scenarios

- `gazebo_static_obstacles.yaml` validates replay-driven fused-perception navigation against static obstacles.
- `isaac_dynamic_replan.yaml` validates a replanning-oriented path where the simulation contract is Isaac Sim but the benchmark remains deterministic and CI-friendly.

## Execution

Run the benchmark suite from the repository root:

```bash
python3 benchmarks/run_benchmarks.py
```

Artifacts are written to `benchmarks/artifacts/` and can be archived by CI as regression evidence.
