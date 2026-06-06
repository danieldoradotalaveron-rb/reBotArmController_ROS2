# <overlay_name> — fork integration tests

Pytest suite that requires the **driver fork workspace** and a colcon-built
`<package_name>` overlay.

Unit tests that run without the driver fork live in the overlay submodule under
`src/<package_name>/test/unit/`.

## Run locally

From the driver fork root (after `just build-all` or equivalent):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source <overlay_workspace>/install/setup.bash
python3 -m pytest integration/<overlay_name>/test -q
```

## CI

Registered in [`.github/workflows/integration.yml`](../../.github/workflows/integration.yml)
and built by [`.github/actions/setup-overlay-workspace`](../../.github/actions/setup-overlay-workspace/action.yml).
