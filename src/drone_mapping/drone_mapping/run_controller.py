"""
Run Controller Node
-------------------
Monitors loop closures from RTAB-Map and shuts down the entire experiment
stack after LC 6 (one beyond PHASE3_LC_WINDOW=5), giving cluster_tracker
time to finish saving PLY files before killing processes.

Shutdown sequence:
  1. Wait 5 s  — cluster_tracker finishes writing PLY + CSV
  2. SIGINT    — cluster_tracker, rtabmap
  3. Wait 2 s  — clean ROS shutdown
  4. SIGKILL   — PX4 (does not respond to SIGINT)
  5. SIGINT    — gz sim, parameter_bridge, synced_broadcaster
"""

import subprocess
import time

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# One loop closure beyond PHASE3_LC_WINDOW to ensure Phase 3 has completed
SHUTDOWN_AT_LC  = 100
RAM_CHECK_PERIOD = 5.0   # seconds between RAM checks
RAM_LIMIT_PCT    = 98.0  # trigger emergency shutdown above this %


class RunControllerNode(Node):

    def __init__(self):
        super().__init__('run_controller')
        self._lc_count  = 0
        self._last_lc_id = 0
        self._shutdown_triggered = False

        _best_effort_qos = QoSProfile(
            depth       = 0,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_ALL,
        )
        try:
            from rtabmap_msgs.msg import Info
            self.create_subscription(Info, '/rtabmap/info', self._info_callback, _best_effort_qos)
            self.get_logger().info(
                f'Run Controller ready — will shut down at LC {SHUTDOWN_AT_LC} '
                f'or if RAM exceeds {RAM_LIMIT_PCT}%'
            )
        except ImportError:
            self.get_logger().warn('rtabmap_msgs not available — run controller disabled')

        self.create_timer(RAM_CHECK_PERIOD, self._ram_check)

    def _info_callback(self, msg):
        if self._shutdown_triggered:
            return
        if msg.loop_closure_id > 0 and msg.loop_closure_id != self._last_lc_id:
            self._last_lc_id  = msg.loop_closure_id
            self._lc_count   += 1
            self.get_logger().info(f'Run Controller: LC {self._lc_count}/{SHUTDOWN_AT_LC}')

            if self._lc_count >= SHUTDOWN_AT_LC:
                self._shutdown_triggered = True
                self._shutdown()

    def _ram_check(self):
        if self._shutdown_triggered:
            return
        ram_pct = psutil.virtual_memory().percent
        if ram_pct >= RAM_LIMIT_PCT:
            self.get_logger().error(
                f'RAM at {ram_pct:.1f}% — emergency shutdown triggered!'
            )
            self._shutdown_triggered = True
            self._shutdown()

    def _shutdown(self):
        self.get_logger().info('LC target reached — initiating experiment shutdown...')

        # Step 1: give cluster_tracker time to finish saving
        self.get_logger().info('Waiting 5s for cluster_tracker to finish saving...')
        time.sleep(5)

        # Step 2: graceful shutdown of ROS nodes in gnome-terminals
        self.get_logger().info('Sending SIGINT to cluster_tracker and rtabmap...')
        subprocess.run(['pkill', '-SIGINT', '-f', 'cluster_tracker'], check=False)
        subprocess.run(['pkill', '-SIGINT', '-f', 'rtabmap'],         check=False)

        time.sleep(2)

        # Step 3: kill PX4 (ignores SIGINT)
        self.get_logger().info('Killing PX4...')
        subprocess.run(['pkill', '-9', '-f', 'px4'], check=False)

        # Step 4: shut down Gazebo and remaining ROS infrastructure
        self.get_logger().info('Shutting down Gazebo and bridge...')
        subprocess.run(['pkill', '-SIGINT', '-f', 'gz sim'],             check=False)
        subprocess.run(['pkill', '-SIGINT', '-f', 'parameter_bridge'],   check=False)
        subprocess.run(['pkill', '-SIGINT', '-f', 'synced_broadcaster'], check=False)

        self.get_logger().info('Shutdown complete.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RunControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
