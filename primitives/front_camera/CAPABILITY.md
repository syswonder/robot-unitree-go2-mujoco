---
description: Expose the Go2 front RGB-D camera for live Scene semantic navigation.
---

# Front RGB-D camera

RGB, aligned depth, camera intrinsics, and mount extrinsics share the
`front_camera_optical_frame` geometry. RGB and depth snapshots return the newest
frame as JPEG. Depth snapshots are normalized for inspection and are not a
metric-depth transport; use the continuous float-depth ROS topic for geometry.

Activation requires RGB, metric depth, valid intrinsics and the latched
base_link-to-front_camera_optical_frame transform. Use Scene list_objects and
goal_near to obtain an observed object's navigation goal, then call navigation.

In SPZ environments RGB includes the Gaussian visual layer while depth comes
from registered MuJoCo collision geometry. In mesh environments both are
rendered from the same mesh coordinate frame.
