# py_uroc/map_tf_publisher.py
"""
Bridges MAVROS /mavros/local_position/pose into /tf.

Creates a transform from `map` to `base_link` frame (global ENU).
Publishes at the rate of the MAVROS pose topic and does not create
any new coordinate conversions.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .qos_profile import BEST_EFFORT_QOS


class MapTFPublisher(Node):
    def __init__(self) -> None:
        super().__init__("map_tf_publisher")

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            BEST_EFFORT_QOS,
        )

        self.br = TransformBroadcaster(self)

    # ---------- callbacks --------------------------------------------------

    def pose_callback(self, msg: PoseStamped) -> None:
        """
        Convert the incoming PoseStamped into a TransformStamped.

        Forward it on /tf with frame semantics:
            parent = "map"
            child  = "base_link"
        """
        tf_msg = TransformStamped()
        tf_msg.header = msg.header  # stamp + "map"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation = msg.pose.orientation

        self.br.sendTransform(tf_msg)


# ---------- main -----------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapTFPublisher()
    node.get_logger().info("Started map→base_link TF bridge")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
