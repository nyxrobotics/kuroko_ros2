#!/usr/bin/python3

import time
from typing import List

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import GetWorldProperties
from std_srvs.srv import Empty


class GazeboUnpauser(Node):
    def __init__(self, expected_models: List[str], wait_after_spawn: float):
        super().__init__('unpause_physics')
        self.expected_models = expected_models
        self.wait_after_spawn = float(wait_after_spawn)

        # Gazebo Classic with gazebo_ros typically exposes these services without a /gazebo prefix in ROS 2.
        # Keep both variants to be resilient across setups.
        self.get_world_clients = [
            self.create_client(GetWorldProperties, '/get_world_properties'),
            self.create_client(GetWorldProperties, '/gazebo/get_world_properties'),
        ]
        self.unpause_clients = [
            self.create_client(Empty, '/unpause_physics'),
            self.create_client(Empty, '/gazebo/unpause_physics'),
        ]

    def _wait_for_any_service(self, clients, timeout_sec: float):
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            for c in clients:
                if c.service_is_ready():
                    return c
                c.wait_for_service(timeout_sec=0.2)
        return None

    def _call_get_world(self, client) -> GetWorldProperties.Response:
        req = GetWorldProperties.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('GetWorldProperties call timed out')
        return future.result()

    def wait_for_models(self, timeout_sec: float = 60.0) -> bool:
        if not self.expected_models:
            self.get_logger().warn('No expected_models specified; proceeding immediately.')
            return True

        self.get_logger().info(f"Waiting for models to spawn: {self.expected_models}")
        gw = self._wait_for_any_service(self.get_world_clients, timeout_sec=10.0)
        if gw is None:
            self.get_logger().error('GetWorldProperties service not available')
            return False

        start = time.time()
        while rclpy.ok():
            try:
                res = self._call_get_world(gw)
                names = set(res.model_names)
                missing = [m for m in self.expected_models if m not in names]
                if not missing:
                    self.get_logger().info('All expected models have spawned.')
                    return True
                self.get_logger().info(f"Waiting for models: {missing}")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"GetWorldProperties failed: {e}")

            if (time.time() - start) > timeout_sec:
                self.get_logger().warn(f"Still waiting after {timeout_sec:.1f} seconds...")
                start = time.time()

            time.sleep(0.5)

        self.get_logger().error('ROS shutdown before all models spawned.')
        return False

    def unpause_physics(self) -> bool:
        self.get_logger().info('Waiting for /unpause_physics service...')
        unpause = self._wait_for_any_service(self.unpause_clients, timeout_sec=10.0)
        if unpause is None:
            self.get_logger().error('Unpause service not available')
            return False

        req = Empty.Request()
        future = unpause.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('Unpause service call timed out')
            return False
        self.get_logger().info('Physics unpaused.')
        return True


def main() -> None:
    rclpy.init()

    # Parameters are passed from launch files as ROS parameters.
    node = Node('unpause_physics_param_reader')
    node.declare_parameter('expected_models', '')
    node.declare_parameter('wait_after_spawn', 1.0)
    expected_str = node.get_parameter('expected_models').get_parameter_value().string_value
    wait_after_spawn = node.get_parameter('wait_after_spawn').get_parameter_value().double_value
    node.destroy_node()

    expected_models = [s for s in expected_str.split() if s]
    unpauser = GazeboUnpauser(expected_models, wait_after_spawn)

    if unpauser.wait_for_models(timeout_sec=60.0):
        unpauser.get_logger().info(f"Waiting {unpauser.wait_after_spawn:.1f} seconds after model spawn...")
        time.sleep(unpauser.wait_after_spawn)
        unpauser.unpause_physics()

    unpauser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
