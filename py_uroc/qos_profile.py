"""
ROS2 Quality of Service (QoS) profile configurations for UROC nodes.

This module defines standard QoS profiles used across the UROC package
for different types of message communication patterns.
"""

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# QoS profile for high-frequency sensor data where message loss is acceptable
# but low latency is important (e.g., pose updates, sensor streams)
BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# QoS profile for critical command and control messages where message delivery
# must be guaranteed (e.g., gimbal commands, configuration updates)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
