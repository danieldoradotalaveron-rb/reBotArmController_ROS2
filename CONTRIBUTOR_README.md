# Contributor README

How to build and run this fork. For what the fork adds and why, see
[`FORK_CHANGES.md`](FORK_CHANGES.md).

## Setup

```bash
git clone --recurse-submodules \
  https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2.git
cd reBotArmController_ROS2
sudo apt install just
just build-all
```

If your ROS distro is not Jazzy, edit `ros_setup` at the top of `justfile`.

## Recipes

| Prefix | Use |
|--------|-----|
| `build-*` | compile (no robot side effects) |
| `run-*` | launch a node/GUI (one per terminal) |
| `svc-*` | call a `/rebotarm/…` service |

```bash
just                                       # list recipes
just run-driver /dev/ttyRebotB601          # terminal 1
just run-monitor /dev/ttyRebotB601         # terminal 2
just run-rqt                               # terminal 3
just svc-park                              # slow return to rest
```

Default device: `/dev/ttyACM0` (Seeed `arm.yaml`). Ctrl+C in the driver
terminal triggers the safe-park shutdown.

Full list: `build-driver`, `build-monitor`, `build-all`, `clean`,
`run-driver [dev]`, `run-monitor [dev]`, `run-rqt`, `run-gravity`,
`svc-park`, `svc-enable`, `svc-disable`, `test-monitor`.
