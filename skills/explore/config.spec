# Package configuration passed to Explore's lifecycle CMD_INIT.
# Request timeout_s and max_speed_m_s remain in the existing Explore request.
config:
  robot_radius_m:
    type: float
    default: 0.4031128874149275
    description: Conservative disk enclosing the Go2 0.70 x 0.40 m footprint.
  approach_distance_m:
    type: float
    default: 0.8
    description: Maximum known-free raster distance from an approach to frontier cells.
  failed_goal_radius_m:
    type: float
    default: 0.6
    description: World-coordinate radius excluded around a failed navigation goal.
  failed_goal_ttl_s:
    type: float
    default: 60
    description: Seconds before a failed goal neighbourhood becomes eligible again.
# All values must be finite and positive; approach_distance_m must exceed
# robot_radius_m. Unknown fields fail initialization. Do not lower the radius
# below the robot's enclosing footprint radius.
