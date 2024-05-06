# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from threading import Event

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


class SimulationTestNode(Node):
    __test__ = False

    # The inaccuracies in measurement uncertainties and wheel slippage
    # cause the rosbot_xl_base_controller to determine inaccurate odometry.
    ACCURACY = 0.10  # 10% accuracy
    VELOCITY_STABILIZATION_DELAY = 2

    def __init__(self, name="test_node", namespace=None):
        super().__init__(name, namespace=namespace)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Robot callback
        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self.joint_states_callback, 10
        )
        self.controller_sub = self.create_subscription(
            Odometry, "rosbot_xl_base_controller/odom", self.controller_callback, 10
        )
        self.ekf_sub = self.create_subscription(
            Odometry, "odometry/filtered", self.ekf_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "imu_broadcaster/imu", self.imu_callback, 10)

        # Timer - send cmd_vel and check if the time needed for speed stabilization has elapsed
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Destination velocity for cmd_vel
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_yaw = 0.0

        # Debug values
        self.controller_twist = None
        self.ekf_twist = None

        # Robot test flags and events
        self.is_controller_msg = False
        self.is_controller_odom_correct = False
        self.is_ekf_msg = False
        self.is_ekf_odom_correct = False
        self.is_imu_msg = False
        self.is_joint_msg = False
        self.robot_initialized_event = Event()
        self.vel_stabilization_time_event = Event()

        # Using /clock topic as time source (checking the simulation time)
        use_sim_time = rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time])

        # Time values
        self.current_time = 1e-9 * self.get_clock().now().nanoseconds
        self.goal_received_time = None

    def reset_flags(self):
        self.is_controller_odom_correct = False
        self.is_ekf_odom_correct = False
        self.vel_stabilization_time_event.clear()

    def set_destination_speed(self, v_x, v_y, v_yaw):
        self.get_logger().info(f"Setting cmd_vel: x: {v_x}, y: {v_y}, yaw: {v_yaw}")
        self.v_x = v_x
        self.v_y = v_y
        self.v_yaw = v_yaw
        self.goal_received_time = 1e-9 * self.get_clock().now().nanoseconds
        self.reset_flags()

    def is_twist_ok(self, twist: Twist):
        def are_close_to_each_other(current_value, dest_value, tolerance=self.ACCURACY, eps=0.01):
            acceptable_range = dest_value * tolerance
            return abs(current_value - dest_value) <= acceptable_range + eps

        x_ok = are_close_to_each_other(twist.linear.x, self.v_x)
        y_ok = are_close_to_each_other(twist.linear.y, self.v_y)
        yaw_ok = are_close_to_each_other(twist.angular.z, self.v_yaw)

        return x_ok and y_ok and yaw_ok

    def is_initialized(self):
        if self.is_controller_msg and self.is_ekf_msg and self.is_imu_msg and self.is_joint_msg:
            self.get_logger().info("Robot initialized!", once=True)
            self.robot_initialized_event.set()
        return self.robot_initialized_event.is_set()

    def controller_callback(self, data: Odometry):
        if not self.is_controller_msg:
            self.get_logger().info("Controller message arrived")
        self.is_controller_msg = True
        self.controller_twist = data.twist.twist
        self.is_controller_odom_correct = self.is_twist_ok(data.twist.twist)

    def ekf_callback(self, data: Odometry):
        if not self.is_ekf_msg:
            self.get_logger().info("EKF message arrived")
        self.is_ekf_msg = True
        self.ekf_twist = data.twist.twist
        self.is_ekf_odom_correct = self.is_twist_ok(data.twist.twist)

    def imu_callback(self, data):
        if not self.is_imu_msg:
            self.get_logger().info("IMU message arrived")
        self.is_imu_msg = True

    def joint_states_callback(self, data):
        if not self.is_joint_msg:
            self.get_logger().info("Joint State message arrived")
        self.is_joint_msg = True

    def timer_callback(self):
        self.current_time = 1e-9 * self.get_clock().now().nanoseconds

        if self.is_initialized() and self.goal_received_time:
            self.publish_cmd_vel_msg()

            if self.current_time > self.goal_received_time + self.VELOCITY_STABILIZATION_DELAY:
                self.get_logger().info(
                    "Speed stabilization time has expired", throttle_duration_sec=1
                )
                self.vel_stabilization_time_event.set()

    def publish_cmd_vel_msg(self):
        twist_msg = Twist()

        twist_msg.linear.x = self.v_x
        twist_msg.linear.y = self.v_y
        twist_msg.angular.z = self.v_yaw

        self.cmd_vel_pub.publish(twist_msg)

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception("Exception occurred, value: ", exep_value)
        self.shutdown()


def wait_for_initialization(node: SimulationTestNode, robot_name="ROSbot"):
    assert node.robot_initialized_event.wait(60), (
        f"{robot_name} does not initialized correctly!\n\tIs controller_msg:"
        f" {node.is_controller_msg}\n\tIs ekf_msg: {node.is_ekf_msg}\n\tIs imu_msg:"
        f" {node.is_imu_msg}\n\tIs joint_msg: {node.is_joint_msg}"
    )


def x_speed_test(node: SimulationTestNode, v_x=0.0, v_y=0.0, v_yaw=0.0, robot_name="ROSbot"):
    node.set_destination_speed(v_x, v_y, v_yaw)
    # node.rate.sleep()
    assert node.vel_stabilization_time_event.wait(20.0), (
        f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
        " since setting the target speed is:"
        f" {(node.current_time - node.goal_received_time):.1f}."
    )
    assert node.is_controller_odom_correct, (
        f"{robot_name}: does not move properly in x direction. Check"
        f" rosbot_xl_base_controller! Twist: {node.controller_twist}"
        f"\nCommand: x: {v_x}, y:{v_y}, yaw:{v_yaw}"
    )
    assert node.is_ekf_odom_correct, (
        f"{robot_name}: does not move properly in x direction. Check ekf_filter_node!"
        f" Twist: {node.ekf_twist}"
    )


def y_speed_test(node: SimulationTestNode, v_x=0.0, v_y=0.0, v_yaw=0.0, robot_name="ROSbot"):
    node.set_destination_speed(v_x, v_y, v_yaw)

    assert node.vel_stabilization_time_event.wait(20.0), (
        f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
        " since setting the target speed is:"
        f" {(node.current_time - node.goal_received_time):.1f}."
    )
    assert node.is_controller_odom_correct, (
        f"{robot_name} does not move properly in y direction. Check"
        f" rosbot_xl_base_controller! Twist: {node.controller_twist}"
        f"\nCommand: x: {v_x}, y:{v_y}, yaw:{v_yaw}"
    )
    assert node.is_ekf_odom_correct, (
        f"{robot_name} does not move properly in y direction. Check ekf_filter_node!"
        f" Twist: {node.ekf_twist}"
    )


def yaw_speed_test(node: SimulationTestNode, v_x=0.0, v_y=0.0, v_yaw=0.0, robot_name="ROSbot"):
    node.set_destination_speed(v_x, v_y, v_yaw)

    assert node.vel_stabilization_time_event.wait(20.0), (
        f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
        " since setting the target speed is:"
        f" {(node.current_time - node.goal_received_time):.1f}."
    )
    assert node.is_controller_odom_correct, (
        f"{robot_name} does not rotate properly. Check rosbot_xl_base_controller! Twist:"
        f" {node.controller_twist}"
        f"\nCommand: x: {v_x}, y:{v_y}, yaw:{v_yaw}"
    )
    assert (
        node.is_ekf_odom_correct
    ), f"{robot_name} does not rotate properly. Check ekf_filter_node! Twist: {node.ekf_twist}"


def diff_test(node: SimulationTestNode, robot_name="ROSbot"):
    wait_for_initialization(node, robot_name)
    # 0.8 m/s and 3.14 rad/s are controller's limits defined in
    # rosbot_xl_controller/config/mecanum_drive_controller.yaml
    x_speed_test(node, v_x=0.8, robot_name=robot_name)
    yaw_speed_test(node, v_yaw=3.14, robot_name=robot_name)


def mecanum_test(node: SimulationTestNode, robot_name="ROSbot"):
    wait_for_initialization(node, robot_name)
    # 0.8 m/s and 3.14 rad/s are controller's limits defined in
    # rosbot_xl_controller/config/mecanum_drive_controller.yaml
    x_speed_test(node, v_x=0.8, robot_name=robot_name)
    y_speed_test(node, v_y=0.8, robot_name=robot_name)
    yaw_speed_test(node, v_yaw=3.14, robot_name=robot_name)
