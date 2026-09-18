config:
  rgb_topic:
    type: string
    default: /front/rgb
    description: Absolute ROS 2 RGB Image topic.
    example: /front/rgb
    failure: CMD_INIT rejects invalid topics; CMD_ACTIVATE fails without a valid RGB frame.
  depth_topic:
    type: string
    default: /front/depth
    description: Required aligned optical-axis float-depth Image topic in metres.
    example: /front/depth
    failure: CMD_INIT fails for an invalid topic; CMD_ACTIVATE fails without valid metric depth.
  info_topic:
    type: string
    default: /front/camera_info
    description: Absolute CameraInfo topic matching the RGB and depth geometry.
    example: /front/camera_info
    failure: CMD_INIT rejects invalid topics; CMD_ACTIVATE requires valid front optical intrinsics.
  extrinsics_topic:
    type: string
    default: /front/extrinsics
    description: Required latched base_link-to-front_camera_optical_frame transform topic.
    example: /front/extrinsics
    failure: CMD_INIT fails for an invalid topic; CMD_ACTIVATE fails without a valid mount transform.
  sentinel_timeout_s:
    type: float
    unit: seconds
    default: 30.0
    range: greater than 0
    description: Total activation deadline for RGB, depth, intrinsics and the latched mount.
    example: 90
    failure: CMD_INIT rejects non-finite or non-positive values; CMD_ACTIVATE fails on timeout.
