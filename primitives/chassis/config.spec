config:
  odom_topic:
    type: string
    default: /odom
    description: Absolute ROS 2 Odometry feedback topic.
    example: /odom
    failure: CMD_INIT fails when the value is not an absolute non-root topic.
  command_topic:
    type: string
    default: /cmd_vel
    description: Absolute ROS 2 Twist command topic used by move and twist_in.
    example: /cmd_vel
    failure: CMD_INIT fails when the value is not an absolute non-root topic.
