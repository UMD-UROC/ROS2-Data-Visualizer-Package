"""
Common utilities for UROC ROS2 nodes.

Provides shared functionality for graceful shutdown handling, 
logging configuration, and debug mode support across all nodes.
"""

import signal
import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node


class NodeShutdownHandler:
    """
    Handles graceful shutdown for ROS2 nodes.
    
    Provides signal handling for CTRL-C and proper cleanup procedures
    to prevent crashes during node termination.
    """
    
    def __init__(self, node: Node, background_threads: Optional[list] = None):
        """
        Initialize shutdown handler.
        
        Parameters
        ----------
        node : Node
            The ROS2 node to manage
        background_threads : list, optional
            List of background threads to terminate during shutdown
        """
        self.node = node
        self.background_threads = background_threads or []
        self.shutdown_requested = False
        
        # Register signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.node.get_logger().info(f"Received signal {signum}, initiating graceful shutdown...")
            self._shutdown()
    
    def _shutdown(self):
        """Perform graceful shutdown."""
        try:
            # Signal any shutdown events first (for mavlink_bridge)
            if hasattr(self.node, 'shutdown_event'):
                self.node.shutdown_event.set()
            
            # Special cleanup for mavlink_bridge
            if hasattr(self.node, 'cleanup_connection'):
                self.node.cleanup_connection()
            
            # Stop background threads
            for thread in self.background_threads:
                if thread.is_alive():
                    self.node.get_logger().info(f"Stopping background thread: {thread.name}")
                    # Most threads should check rclpy.ok() and exit naturally
                    thread.join(timeout=2.0)
                    if thread.is_alive():
                        self.node.get_logger().warn(f"Background thread {thread.name} did not stop gracefully")
            
            # Destroy node
            self.node.get_logger().info("Shutting down node...")
            self.node.destroy_node()
            
            # Shutdown ROS2
            if rclpy.ok():
                rclpy.shutdown()
                
        except Exception as e:
            self.node.get_logger().error(f"Error during shutdown: {e}")
        finally:
            sys.exit(0)


def setup_node_logging(node: Node, debug: bool = False):
    """
    Configure logging for a node.
    
    Parameters
    ----------
    node : Node
        The ROS2 node to configure
    debug : bool
        Enable debug-level logging
    """
    logger = node.get_logger()
    
    if debug:
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        logger.info("Debug logging enabled")
    else:
        logger.set_level(rclpy.logging.LoggingSeverity.INFO)
    
    return logger


def log_node_status(node: Node, status_msg: str, debug: bool = False):
    """
    Log node status with appropriate level.
    
    Parameters
    ----------
    node : Node
        The ROS2 node
    status_msg : str
        Status message to log
    debug : bool
        Whether this is debug-level information
    """
    logger = node.get_logger()
    if debug:
        logger.debug(status_msg)
    else:
        logger.info(status_msg)


def log_periodic_status(node: Node, message: str, counter: int, period: int = 100):
    """
    Log status periodically to avoid spam.
    
    Parameters
    ----------
    node : Node
        The ROS2 node
    message : str
        Message to log
    counter : int
        Current iteration counter
    period : int
        Log every N iterations
    """
    if counter % period == 0:
        node.get_logger().info(f"{message} (iteration {counter})")