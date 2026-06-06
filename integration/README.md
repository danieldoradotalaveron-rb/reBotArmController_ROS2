# Fork integration tests

Pytest suites for **overlay submodules** that depend on the driver fork
(`rebotarm_msgs`, `rebotarm_bringup`, vendored SDK, colcon-built packages).

Each overlay keeps **unit tests** in its own repo (CI without external deps).
Everything here runs from the driver fork root via GitHub Actions
[`.github/workflows/integration.yml`](../.github/workflows/integration.yml).

## Layout

```text
integration/
├── README.md                          # this file
├── _template/                         # copy when adding a new overlay
│   └── README.md
├── rebotarm_monitor/
│   └── test/                          # tracker + orchestrator tests
└── rebotarm_cartesian_teleop/
    └── test/                          # FK/IK, bringup, mapper/core tests
```

## Local commands

```bash
just build-all
just test-monitor-integration
just test-teleop-integration
```

## Adding a new overlay

1. Copy `_template/` to `integration/<overlay_name>/`.
2. Move external-dependent tests from the overlay submodule into `test/`.
3. Keep self-contained tests in `<overlay>/src/<pkg>/test/unit/`.
4. Extend [`.github/actions/setup-overlay-workspace`](../.github/actions/setup-overlay-workspace/action.yml) to colcon-build the new overlay.
5. Add a `pytest integration/<overlay_name>/test` step to `integration.yml`.
