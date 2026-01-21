#!/usr/bin/python3

import argparse
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, SetModelConfiguration


@dataclass
class JointInit:
    name: str
    position: float


def parse_joint_args(args: List[str]) -> Tuple[List[JointInit], List[str]]:
    joints: List[JointInit] = []
    i = 0
    while i < len(args):
        if args[i] == '-J':
            if i + 2 >= len(args):
                raise ValueError("-J requires <joint_name> <position>")
            joints.append(JointInit(args[i + 1], float(args[i + 2])))
            i += 3
            continue
        i += 1
    return joints, args


def pose_from_xyzrpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> Pose:
    # Minimal quaternion conversion without extra dependencies
    import math

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = cr * cp * cy + sr * sp * sy
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy
    return pose


class EntitySpawner(Node):
    def __init__(self):
        super().__init__('spawn_entity_from_param')

        # Service names vary depending on how Gazebo is launched.
        self.spawn_clients = [
            self.create_client(SpawnEntity, '/spawn_entity'),
            self.create_client(SpawnEntity, '/gazebo/spawn_entity'),
        ]
        self.set_cfg_clients = [
            self.create_client(SetModelConfiguration, '/set_model_configuration'),
            self.create_client(SetModelConfiguration, '/gazebo/set_model_configuration'),
        ]

    def _wait_for_any_service(self, clients, timeout_sec: float):
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            for c in clients:
                if c.service_is_ready():
                    return c
                c.wait_for_service(timeout_sec=0.2)
        return None

    def spawn_from_param(
        self,
        entity_name: str,
        xml_param: str,
        robot_namespace: str,
        initial_pose: Pose,
        initial_joints: List[JointInit],
    ) -> None:
        xml = self.get_parameter(xml_param).get_parameter_value().string_value
        if not xml:
            raise RuntimeError(f"Parameter '{xml_param}' is empty")

        spawner = self._wait_for_any_service(self.spawn_clients, timeout_sec=10.0)
        if spawner is None:
            raise RuntimeError('SpawnEntity service not available')

        req = SpawnEntity.Request()
        req.name = entity_name
        req.xml = xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose

        future = spawner.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('SpawnEntity call timed out')

        res = future.result()
        if not res.success:
            raise RuntimeError(f"SpawnEntity failed: {res.status_message}")

        self.get_logger().info(f"Spawned entity '{entity_name}'")

        if not initial_joints:
            return

        setter = self._wait_for_any_service(self.set_cfg_clients, timeout_sec=10.0)
        if setter is None:
            self.get_logger().warn('SetModelConfiguration service not available; skipping joint initialization')
            return

        req2 = SetModelConfiguration.Request()
        req2.model_name = entity_name
        req2.urdf_param_name = xml_param
        req2.joint_names = [j.name for j in initial_joints]
        req2.joint_positions = [float(j.position) for j in initial_joints]

        future2 = setter.call_async(req2)
        rclpy.spin_until_future_complete(self, future2, timeout_sec=10.0)
        if not future2.done() or future2.result() is None:
            self.get_logger().warn('SetModelConfiguration call timed out')
            return
        res2 = future2.result()
        if not res2.success:
            self.get_logger().warn(f"SetModelConfiguration failed: {res2.status_message}")
            return
        self.get_logger().info('Initialized joint positions')


def main() -> None:
    parser = argparse.ArgumentParser(description='Spawn a Gazebo entity from a ROS parameter containing URDF/SDF XML')
    parser.add_argument('--name', required=True, help='Entity name')
    parser.add_argument('--param', required=True, help='Node parameter name that contains the URDF/SDF XML')
    parser.add_argument('--namespace', default='', help='Robot namespace for Gazebo plugins')

    parser.add_argument('-x', type=float, default=0.0)
    parser.add_argument('-y', type=float, default=0.0)
    parser.add_argument('-z', type=float, default=0.0)
    parser.add_argument('-R', type=float, default=0.0, help='Roll (rad)')
    parser.add_argument('-P', type=float, default=0.0, help='Pitch (rad)')
    parser.add_argument('-Y', type=float, default=0.0, help='Yaw (rad)')

    # Keep ROS 1 compatible joint init tokens: -J <joint> <pos>
    known, unknown = parser.parse_known_args()
    initial_joints, _ = parse_joint_args(unknown)

    rclpy.init()
    spawner = EntitySpawner()

    # The XML must be available on this node as a parameter. Launch files should set it before starting this node.
    spawner.declare_parameter(known.param, '')

    initial_pose = pose_from_xyzrpy(known.x, known.y, known.z, known.R, known.P, known.Y)
    try:
        spawner.spawn_from_param(
            entity_name=known.name,
            xml_param=known.param,
            robot_namespace=known.namespace,
            initial_pose=initial_pose,
            initial_joints=initial_joints,
        )
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
