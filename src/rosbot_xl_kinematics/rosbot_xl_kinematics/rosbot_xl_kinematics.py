#!/usr/bin/env python
import math
from rclpy.node import Node


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

    def eulerToQuaternion(yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
            math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
            math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]


