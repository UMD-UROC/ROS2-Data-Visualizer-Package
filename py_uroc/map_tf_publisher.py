# py_uroc/map_tf_publisher.py
"""
Bridges MAVROS /mavros/local_position/pose into /tf so that `base_link`
has a parent `map` frame (global ENU).  Publishes at the rate of the MAVROS
pose topic and does not create any new coordinate conversions.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSProfile


class MapTFPublisher(Node):
    def __init__(self) -> None:
        super().__init__("map_tf_publisher")

        # Best‑effort QoS (matches MAVROS publishers)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos,
        )

        self.br = TransformBroadcaster(self)

    # ---------- callbacks --------------------------------------------------

    def pose_callback(self, msg: PoseStamped) -> None:
        """
        Convert the incoming PoseStamped into a TransformStamped and
        forward it on /tf.  Frame semantics:

            parent = "map"
            child  = "base_link"
        """
        tf_msg = TransformStamped()
        tf_msg.header = msg.header           # stamp + "map"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation     = msg.pose.orientation

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
