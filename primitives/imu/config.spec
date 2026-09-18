config:
  topic:
    type: string
    default: /mid360/imu
    description: Absolute ROS 2 Imu topic produced by the simulation bridge.
    example: /mid360/imu
    failure: CMD_INIT fails when invalid or when no IMU sample arrives before the timeout.
  sentinel_timeout_s:
    type: float
    unit: seconds
    default: 30.0
    range: greater than 0
    description: Maximum wait for the first valid IMU sample during CMD_INIT.
    example: 90
    failure: CMD_INIT fails on a non-numeric, non-positive, or expired value.
