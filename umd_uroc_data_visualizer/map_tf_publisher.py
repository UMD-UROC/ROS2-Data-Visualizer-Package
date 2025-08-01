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
from .node_utils import NodeShutdownHandler, setup_node_logging, log_periodic_status


class MapTFPublisher(Node):
    def __init__(self, debug: bool = False) -> None:
        super().__init__("map_tf_publisher")

        # Setup logging
        self.logger = setup_node_logging(self, debug)
        self.debug = debug
        
        # Setup graceful shutdown handling
        self.shutdown_handler = NodeShutdownHandler(self)
        
        # Initialize counters for periodic status reporting
        self.pose_callback_count = 0

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            BEST_EFFORT_QOS,
        )

        self.br = TransformBroadcaster(self)
        
        self.logger.info(f"Map TF publisher initialized (debug={'enabled' if debug else 'disabled'})")
        if debug:
            self.logger.info("Publishing map->base_link transforms from MAVROS pose data")

    # ---------- callbacks --------------------------------------------------

    def pose_callback(self, msg: PoseStamped) -> None:
        """
        Convert the incoming PoseStamped into a TransformStamped.

        Forward it on /tf with frame semantics:
            parent = "map"
            child  = "base_link"
        """
        self.pose_callback_count += 1
        
        tf_msg = TransformStamped()
        tf_msg.header = msg.header  # stamp + "map"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation = msg.pose.orientation

        self.br.sendTransform(tf_msg)
        
        # Debug-only status reporting (control center doesn't need transform position details)
        if self.debug:
            log_periodic_status(
                self,
                f"Published map->base_link transform at position [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}]",
                self.pose_callback_count,
                100  # Log every 100 transforms
            )
            self.logger.debug(f"Published transform: pos=[{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}]")


# ---------- main -----------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    
    # Check for debug flag in arguments
    debug = '--debug' in (args or [])
    
    try:
        node = MapTFPublisher(debug=debug)
        node.get_logger().info("Started map→base_link TF bridge")
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Graceful shutdown is handled by NodeShutdownHandler
        pass
    except Exception as e:
        print(f"Unexpected error in map TF publisher: {e}")
    finally:
        # Cleanup is handled by NodeShutdownHandler
        pass


if __name__ == "__main__":
    main()
