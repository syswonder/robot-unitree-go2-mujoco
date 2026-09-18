---
description: Stream the simulated Go2 body inertial measurement.
---

# Go2 IMU

The stream contains MuJoCo-derived angular velocity and linear acceleration in
`imu_link`, mounted at [-0.02557, 0, 0.04232] metres from base_link. The
runtime ID remains `mid360_imu`. The primitive becomes ACTIVE only after a
sample proves that the selected simulator runtime is running.
