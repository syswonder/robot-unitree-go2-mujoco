config:
  scan_topic:
    type: string
    default: /scan
    description: Absolute ROS 2 LaserScan topic used by navigation and snapshots.
    example: /scan
    failure: CMD_INIT fails when invalid or when no scan arrives before the timeout.
  cloud_topic:
    type: string
    default: /mid360/points
    description: Absolute ROS 2 PointCloud2 topic for the simulated MID-360 cloud.
    example: /mid360/points
    failure: CMD_INIT fails when the value is not an absolute non-root topic.
  sentinel_timeout_s:
    type: float
    unit: seconds
    default: 30.0
    range: greater than 0
    description: Maximum wait for the first valid LaserScan during CMD_INIT.
    example: 90
    failure: CMD_INIT fails on a non-numeric, non-positive, or expired value.
