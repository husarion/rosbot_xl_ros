#!/usr/bin/env python
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class ROSbotXLKinematics(Node):
    def __init__(self, name):
        super().__init__(name)
        self.lin_x = 0
        self.lin_y = 0
        self.ang_z = 0
        self.robot_width = 0
        self.robot_length = 0
        self.wheel_radius = 0  # Distance of the wheel center, to the roller center
        self.encoder_resolution = 0
        self.FR_enc_speed = 0  # front right encoder speed
        self.FL_enc_speed = 0
        self.RR_enc_speed = 0
        self.RL_enc_speed = 0
        self.robot_x_pos = 0
        self.robot_y_pos = 0
        self.robot_th_pos = 0
        self.power_factor = 0
        self.scale_factor_x = 0.25
        self.scale_factor_y = 0.25
        self.scale_factor_th = 0.125

        self.last_time = self.get_sec()
        self.get_logger().info("Startup time time = {} [s]".format(self.last_time))

        
        self.create_subscription(Twist, "/cmd_vel", self.cmdVelCallback, 1)
        self.create_subscription(JointState, '/motors_response', self.motorsResponseCallback, 1)

        self.wheel_vel_pub = self.create_publisher(JointState, '/motors_cmd', 1)
        self.odom_pub = self.create_publisher(Odometry, "/odom/wheel", 1)

    def get_sec(self):
        return self.get_clock().now().nanoseconds/10e8

    def motorsResponseCallback(self, data):
        now = self.get_sec()
        dt_ = now - self.last_time
        self.last_time = now

        robot_x_pos, robot_y_pos, robot_th_pos = self.inverseKinematics(
                    data.velocity[0], data.velocity[0], data.velocity[0], data.velocity[0], dt_)

        qx, qy, qz, qw = self.eulerToQuaternion(robot_th_pos, 0, 0)

        self.get_logger().info("received motors response, time diff = {}".format(self.last_time))

        odom_msg = Odometry()

        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = robot_x_pos
        odom_msg.pose.pose.position.y = robot_y_pos
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom_msg)
        

    def cmdVelCallback(self, data):
        # forward kinematics
        self.lin_x = data.linear.x * self.scale_factor_x  # m/s
        self.lin_y = data.linear.y * self.scale_factor_y  # m/s
        self.ang_z = data.angular.z * self.scale_factor_th  # rad/s
        self.forwardKinematics()

    def forwardKinematics(self, x_data, y_data, th_data):
        print("not implemented")
        return 1

    def inverseKinematics(self, wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RL_ang_vel, wheel_RR_ang_vel):
        print("not implemented")
        return 1

    def eulerToQuaternion(self, yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
            math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
            math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]


