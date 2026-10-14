"""Verify typed static ballistic solutions with synthetic approved calibration only."""

import json
import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from shooter_calculator.msg import AimTarget, ShotSolution


def test_shot_solution_calibration(tmp_path):
    """Check physical-unit separation, yaw-transformed origin and invalid conditions."""
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('shot_solution_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    targets = node.create_publisher(AimTarget, '/auto_mode/aim_target', 1)
    transforms = TransformBroadcaster(node)
    outputs = []
    node.create_subscription(ShotSolution, '/shooter/solution', outputs.append, 10)
    config = {
        'schema_version': 1, 'approved': True, 'calibration_id': 'synthetic-only',
        'map_frame': 'map', 'robot_frame': 'base_link', 'shooter_offset_m': [0.5, 0, 0.3],
        'launch_angle_deg': 45.0, 'cloth_mass_kg': 0.12, 'cloth_area_m2': 0.0,
        'drag_coefficient': 0.0, 'max_height_error_m': 0.05,
        'calibration_pairs': [[1, 0.5], [20, 10]],
    }
    executable = os.environ['SHOOTER_EXECUTABLE']

    def run_case(profile, expected_calibrated=True, expected_range=True):
        """Run one synthetic profile and inspect outputs without actuator topics."""
        file = tmp_path / 'calibration.json'
        file.write_text(json.dumps(profile))
        outputs.clear()
        process = subprocess.Popen([executable, '--ros-args', '-p', f'calibration_file:={file}',
                                    '-p', 'target_timeout_s:=0.1', '-p', 'pose_timeout_s:=0.1'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        target_id, target_x, target_y, target_z = 1, 3.5, 0.0, 0.3
        yaw = 0.0
        generation = 1
        feed_target, feed_pose = True, True
        explicit_invalid = False
        frozen = None

        def step():
            """Publish current synthetic pose and target or deliberately freeze either."""
            stamp = node.get_clock().now().to_msg()
            if feed_pose:
                pose = TransformStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = 'map'
                pose.child_frame_id = 'base_link'
                pose.transform.rotation.z = math.sin(yaw / 2)
                pose.transform.rotation.w = math.cos(yaw / 2)
                transforms.sendTransform(pose)
            if feed_target:
                target = AimTarget()
                target.header.stamp = stamp
                target.header.frame_id = 'map'
                target.producer_session = 123
                target.generation = generation
                target.target_id = target_id
                target.valid = not explicit_invalid
                target.position.x, target.position.y, target.position.z = target_x, target_y, target_z
                target.confidence = 1.0
                targets.publish(frozen or target)
                executor.spin_once(timeout_sec=0.005)
                return frozen or target
            executor.spin_once(timeout_sec=0.005)

        def wait_for(predicate):
            """Wait for a typed solution with a bounded observation interval."""
            end = time.monotonic() + 5
            while time.monotonic() < end:
                assert process.poll() is None
                step()
                if outputs and predicate(outputs[-1]):
                    return outputs[-1]
            raise AssertionError('shot solution outcome timed out')

        try:
            if not expected_calibrated:
                result = wait_for(lambda value: not value.valid and 'Calibration' in value.reason)
                assert result.belt_speed_m_s == 0
                return
            if not expected_range:
                result = wait_for(lambda value: not value.valid and 'outside measured' in value.reason)
                assert result.belt_speed_m_s == 0 and not result.in_calibration_range
                return
            result = wait_for(lambda value: value.valid)
            # Independent zero-drag projectile formula: v=sqrt(g*d/sin(2*angle)).
            expected_speed = math.sqrt(9.81 * 3.0)
            assert abs(result.initial_speed_m_s - expected_speed) < 0.04
            assert abs(result.belt_speed_m_s - result.initial_speed_m_s / 2) < 1e-6
            assert result.belt_speed_m_s > 1.0  # No old [0,1] ratio clipping.
            assert result.in_calibration_range and result.calibration_id == 'synthetic-only'
            # Rotating the robot rotates its forward offset, preserving the same range.
            yaw = math.pi / 2
            generation += 1
            target_x, target_y = 0.0, 3.5
            result = wait_for(lambda value: value.valid and value.generation == generation
                              and abs(value.initial_speed_m_s - expected_speed) < 0.04)
            assert abs(result.belt_speed_m_s - result.initial_speed_m_s / 2) < 1e-6
            target_x, target_y = 10000.0, 0.0
            result = wait_for(lambda value: not value.valid and 'unreachable' in value.reason)
            assert result.belt_speed_m_s == 0
            target_x, target_y = 0.0, 3.5
            result = wait_for(lambda value: value.valid)
            feed_target = False
            result = wait_for(lambda value: not value.valid and 'expired' in value.reason)
            assert result.belt_speed_m_s == 0
            feed_target = True
            result = wait_for(lambda value: value.valid)
            frozen = step()
            result = wait_for(lambda value: not value.valid and 'expired' in value.reason)
            frozen = None
            result = wait_for(lambda value: value.valid)
            feed_pose = False
            result = wait_for(lambda value: not value.valid and 'pose' in value.reason)
            feed_pose = True
            result = wait_for(lambda value: value.valid)
            explicit_invalid = True
            result = wait_for(lambda value: not value.valid and 'AimTarget invalid' in value.reason)
            explicit_invalid = False
            target_id = 7
            result = wait_for(lambda value: not value.valid and 'Moving target' in value.reason)
            assert node.count_publishers('/belt/speed_ratio') == 0
            assert node.count_publishers('/belt/init') == 0
        finally:
            process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=5)

    try:
        run_case(config)
        narrow = dict(config)
        narrow['calibration_pairs'] = [[1, 0.5], [4, 2]]
        run_case(narrow, expected_range=False)
        unapproved = dict(config)
        unapproved['approved'] = False
        run_case(unapproved, expected_calibrated=False)
        malformed = dict(config)
        malformed['calibration_pairs'] = [[2, 1], [1, 2]]
        file = tmp_path / 'invalid.json'
        file.write_text(json.dumps(malformed))
        result = subprocess.run([executable, '--ros-args', '-p', f'calibration_file:={file}'],
                                capture_output=True, timeout=5)
        assert result.returncode == 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
