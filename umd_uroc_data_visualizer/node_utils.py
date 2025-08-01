"""
Common utilities for UROC ROS2 nodes.

Provides shared functionality for graceful shutdown handling, 
logging configuration, debug mode support, and status heartbeat
for control center dashboard display.
"""

import signal
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node


class NodeShutdownHandler:
    """
    Handles graceful shutdown for ROS2 nodes.
    
    Provides signal handling for CTRL-C and proper cleanup procedures
    to prevent crashes during node termination. Also provides status
    heartbeat functionality for control center dashboard.
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
        self._spinning = False
        
        # Register signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Status heartbeat for control center dashboard
        self._last_heartbeat = time.time()
        self._status_timer = node.create_timer(5.0, self._status_heartbeat)
        
        # Status tracking
        self._node_name = node.get_name()
        self._errors_count = 0
        self._data_received = False
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.node.get_logger().info(f"Received signal {signum}, initiating graceful shutdown...")
            
            # Just set the flag - don't call rclpy.shutdown() from signal handler
            # The spin loop will detect this and exit properly
    
    def _status_heartbeat(self):
        """Provide periodic status updates for control center dashboard."""
        if self.shutdown_requested:
            return
            
        current_time = time.time()
        uptime = int(current_time - self._start_time)  # Actual uptime calculation
        
        # Color codes for status dashboard
        GREEN = "\033[92m"  # Healthy
        YELLOW = "\033[93m" # Warning  
        RED = "\033[91m"    # Error
        RESET = "\033[0m"   # Reset color
        
        if self._errors_count > 10:
            color = RED
            status = "ERROR"
        elif self._errors_count > 0:
            color = YELLOW
            status = "WARNING"
        else:
            color = GREEN
            status = "HEALTHY"
            
        # Status dashboard format suitable for control center
        self.node.get_logger().info(
            f"{color}[STATUS]{RESET} {self._node_name}: {color}{status}{RESET} "
            f"(errors: {self._errors_count}, data: {'YES' if self._data_received else 'NO'})"
        )
    
    def report_data_received(self):
        """Call this when node receives data to update status."""
        self._data_received = True
        
    def report_error(self):
        """Call this when node encounters an error."""
        self._errors_count += 1
    
    def spin_with_shutdown(self):
        """
        Spin the node with proper shutdown handling.
        
        This replaces the standard rclpy.spin() call and provides
        better control over shutdown sequence.
        """
        self._spinning = True
        
        try:
            while rclpy.ok() and not self.shutdown_requested:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.shutdown_requested = True
        finally:
            self._spinning = False
            # Only call rclpy.shutdown() here if it hasn't been called already
            if rclpy.ok():
                rclpy.shutdown()
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
            
            # Destroy status timer before destroying node
            if hasattr(self, '_status_timer'):
                self._status_timer.destroy()
            
            # Destroy node
            self.node.get_logger().info("Shutting down node...")
            self.node.destroy_node()
            
            # Don't call rclpy.shutdown() here since it's already called in spin_with_shutdown
                
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
    Log status periodically to avoid spam - DEBUG MODE ONLY.
    
    This function is now only used in debug mode to avoid
    verbose output during normal control center operations.
    
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
    # Only log if we're in debug mode - check if debug attribute exists
    if hasattr(node, 'debug') and node.debug and counter % period == 0:
        node.get_logger().debug(f"{message} (iteration {counter})")