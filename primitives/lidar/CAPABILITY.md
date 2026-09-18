---
description: Expose simulated MID-360 planar scans, point clouds, and snapshots.
---

# MID-360 lidar

The active MuJoCo runtime computes rays against environment collision geometry. The
2D scan is used by Nav2; the 3D cloud is available to Mapping. `snapshot`
returns the newest complete 2D scan and fails until a scan has arrived.

SPZ is visual only: valid returns require a registered collision XML whose
environment geoms are visible to the sensor ray group.
