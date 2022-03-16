#!/usr/bin/env python3

import math
import rclpy
from rosbot_xl_kinematics.rosbot_xl_kinematics import ROSbotXLKinematics
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class ROSbotXLClassic(ROSbotXLKinematics):
    def __init__(self):
        super().__init__("ROSbotXL_kinematics")
        self.robot_width =  0.200
        self.robot_length = 0.170
        self.wheel_radius = 0.05
        self.power_factor = 0.04166667

        self.get_logger().info("Classic Kinematics node Initializad!")


    def forwardKinematics(self):
        # Classic:
        wheel_front_right = (1/self.wheel_radius) * (self.lin_x +
                                                    (self.robot_width + self.robot_length) * self.ang_z)  # rad/s
        wheel_front_left = (1/self.wheel_radius) * (self.lin_x -
                                                    (self.robot_width + self.robot_length) * self.ang_z)
        wheel_rear_right = (1/self.wheel_radius) * (self.lin_x +
                                                    (self.robot_width + self.robot_length) * self.ang_z)
        wheel_rear_left = (1/self.wheel_radius) * (self.lin_x -
                                                   (self.robot_width + self.robot_length) * self.ang_z)

        msg = JointState()

        msg.name = ["FR","FL","RR","RL"]
        msg.velocity = [wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left]
        
        self.get_logger().info("Sending motors cmd")

        self.wheel_vel_pub.publish(msg)  


    def inverseKinematics(self, wheel_FL_ang_vel, wheel_FR_ang_vel, wheel_RL_ang_vel, wheel_RR_ang_vel, dt_):
        # Classic:
        linear_velocity_x_ = (wheel_FL_ang_vel + wheel_FR_ang_vel +
                              wheel_RL_ang_vel + wheel_RR_ang_vel) * (self.wheel_radius/4)
        linear_velocity_y_ = (-wheel_FL_ang_vel + wheel_FR_ang_vel +
                              wheel_RL_ang_vel - wheel_RR_ang_vel) * (self.wheel_radius/4)
        angular_velocity_z_ = (-wheel_FL_ang_vel + wheel_FR_ang_vel - wheel_RL_ang_vel + wheel_RR_ang_vel) * (
            self.wheel_radius/(4 * (self.robot_width / 2 + self.robot_length / 2)))

        delta_heading = angular_velocity_z_ / dt_  # [radians]
        self.robot_th_pos = self.robot_th_pos + delta_heading
        delta_x = (linear_velocity_x_ * math.cos(self.robot_th_pos) -
                   linear_velocity_y_ * math.sin(self.robot_th_pos)) / dt_  # [m]
        delta_y = (linear_velocity_x_ * math.sin(self.robot_th_pos) +
                   linear_velocity_y_ * math.cos(self.robot_th_pos)) / dt_  # [m]
        self.robot_x_pos = self.robot_x_pos + delta_x
        self.robot_y_pos = self.robot_y_pos + delta_y
        return self.robot_x_pos, self.robot_y_pos, self.robot_th_pos


def main(args=None):
    rclpy.init(args=args)
    node = ROSbotXLClassic()
    rclpy.spin(node)


if __name__ == "__main__":
    try:
        main()
    except rclpy.ROSInterruptException:
        rclpy.shutdown()
