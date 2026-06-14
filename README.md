# reBotArm ROS2 SDK

> **`main` tracks latest main @ [Seeed upstream](https://github.com/Seeed-Projects/reBotArmController_ROS2), plus a small `safe_park` shutdown patch.**  
> Broader fork work (monitor, teleop, `just`, CI) lives on
> [`devel/based-on-upstream-d3a415e`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/tree/devel/based-on-upstream-d3a415e),
> branched from upstream [`d3a415e`](https://github.com/Seeed-Projects/reBotArmController_ROS2/commit/d3a415ef24a560dbd76b07bcb3837d7d0918a97d)
> before those upstream advances.

---

| Branch | Use |
|--------|-----|
| **`main`** (this) | Latest Seeed `main` + configurable `safe_park` (`config/safe_park.yaml`) |
| **`devel/based-on-upstream-d3a415e`** | Fork features — [CONTRIBUTOR_README](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/blob/devel/based-on-upstream-d3a415e/CONTRIBUTOR_README.md) |

---

## Safe Park

`safe_home` still means Seeed zero-home (`q=0`). This branch adds `safe_park`, a configurable shutdown target intended to avoid the small drop/impact observed when disabling at electrical zero.

Configuration lives in [`src/rebotarm_bringup/config/safe_park.yaml`](src/rebotarm_bringup/config/safe_park.yaml). The driver seeds its initial pos_vel target from that pose, exposes `/rebotarm/park`, and the gravity-compensation example exits via `safe_park` before disabling.

---

**Clone fork features:**

```bash
git clone --recurse-submodules -b devel/based-on-upstream-d3a415e \
  https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2.git
```
