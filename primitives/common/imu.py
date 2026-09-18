"""Go2 body IMU primitive; mid360_imu remains its deployment identity."""
from robonix_api import Primitive, Ok, Err

from .runtime import package_root, provider_id, timeout_value, topic_value

provider = Primitive(
    id=provider_id("mid360_imu"), namespace="robonix/primitive/imu",
    pkg_root=package_root())


@provider.on_init
def initialize(config):
    """Declare the IMU stream after verifying that simulation is live."""
    try:
        topic = topic_value(config, "topic", "/mid360/imu")
        sentinel_timeout_s = timeout_value(config)
    except ValueError as error:
        return Err(str(error))
    if not provider.wait_for_topic(topic, "Imu", sentinel_timeout_s):
        return Err(f"no Imu received on {topic}; start the simulator runtime first")
    provider.declare_ros2_topic("robonix/primitive/imu/imu", topic, qos="best_effort")
    return Ok()


@provider.on_shutdown
def shutdown():
    """Complete lifecycle shutdown."""
    return Ok()


if __name__ == "__main__":
    provider.run()
