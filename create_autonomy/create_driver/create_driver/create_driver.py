import math
import os, sys
import time

from . import create

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Twist,
    TransformStamped,
    Point,
    Pose,
    Quaternion,
    Twist,
    Vector3,
)
from create_msgs.msg import Bumper, ChargingState, DefineSong, PlaySong


class CreateDriver(Node):
    enc_left_old = None
    enc_right_old = None

    def __init__(self):
        super().__init__('create_driver')
        self.declare_parameter("roomba_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("max_forward", 40.0) # [cm/s]
        self.declare_parameter("max_rotation", 100.0) # [cm/s]

        roomba_port = self.get_parameter("roomba_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value

        self.max_forward = self.get_parameter("max_forward").get_parameter_value().double_value
        self.max_rotation = self.get_parameter("max_rotation").get_parameter_value().double_value

        self.robot = create.Create(roomba_port, BAUD_RATE=baud_rate)
        self.robot.toSafeMode()
        self.sensors = self.robot.sensors([create.WALL_SIGNAL, create.WALL_IR_SENSOR, create.LEFT_BUMP, create.RIGHT_BUMP, create.ENCODER_LEFT, create.ENCODER_RIGHT, create.CLIFF_LEFT_SIGNAL, create.CLIFF_FRONT_LEFT_SIGNAL, create.CLIFF_FRONT_RIGHT_SIGNAL, create.CLIFF_RIGHT_SIGNAL, create.DIRT_DETECTED])

        self.cmd_vel_sub_ = self.create_subscription(Twist,
                                                    "cmd_vel",
                                                    self.cmd_vel_callback,
                                                    qos_profile_sensor_data)
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 1)
        self.bumper_pub_ = self.create_publisher(Bumper, "bumper", 1)
        self.charging_state_pub_ = self.create_publisher(ChargingState, "charging_state", 1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def timer_callback(self):
        senses = self.robot.sensors([create.WALL_SIGNAL, create.WALL_IR_SENSOR, create.LEFT_BUMP, create.RIGHT_BUMP, create.ENCODER_LEFT, create.ENCODER_RIGHT, create.CLIFF_LEFT_SIGNAL, create.CLIFF_FRONT_LEFT_SIGNAL, create.CLIFF_FRONT_RIGHT_SIGNAL, create.CLIFF_RIGHT_SIGNAL, create.DIRT_DETECTED])
        current_time = self.get_clock().now().to_msg()

        # バンパーセンサー
        bumper = Bumper()
        bumper.header.frame_id = "bumper"
        bumper.header.stamp = current_time
        bumper.is_left_pressed = senses[create.LEFT_BUMP] == 1
        bumper.is_right_pressed = senses[create.RIGHT_BUMP] == 1
        bumper.light_signal_left = self.robot.senseFunc(create.LIGHTBUMP_LEFT)()
        bumper.light_signal_front_left = self.robot.senseFunc(create.LIGHTBUMP_FRONT_LEFT)()
        bumper.light_signal_center_left = self.robot.senseFunc(create.LIGHTBUMP_CENTER_LEFT)()
        bumper.light_signal_center_right = self.robot.senseFunc(create.LIGHTBUMP_CENTER_RIGHT)()
        bumper.light_signal_front_right = self.robot.senseFunc(create.LIGHTBUMP_FRONT_RIGHT)()
        bumper.light_signal_right = self.robot.senseFunc(create.LIGHTBUMP_RIGHT)()
        self.bumper_pub_.publish(bumper)

        # エンコーダー
        enc_left = senses[create.ENCODER_LEFT]
        enc_right = senses[create.ENCODER_RIGHT]
        # 回転を計算
        if self.enc_left_old is not None and self.enc_right_old is not None:
            left_diff  = self.robot._getEncoderDelta(self.enc_left_old, enc_left)
            right_diff = self.robot._getEncoderDelta(self.enc_right_old, enc_right)

            left_mm = math.pi * left_diff / create.TICK_PER_MM / 180
            right_mm = math.pi * right_diff / create.TICK_PER_MM / 180
            print(left_mm, right_mm)
        self.enc_left_old = enc_left
        self.enc_right_old = enc_right
        
        # オドメトリを取得 [m, m, rad]
        x, y, th = self.robot.getPose(dist='cm',angle='deg')
        x, y = x/100, y/100
        th = math.pi + th / 180

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = self.euler_to_quaternion(0, 0, th)

        # first, we'll publish the transform over tf2_ros
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = current_time
        transform_stamped.header.frame_id = "base_link"
        transform_stamped.child_frame_id = "odom"
        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = odom_quat[0]
        transform_stamped.transform.rotation.y = odom_quat[1]
        transform_stamped.transform.rotation.z = odom_quat[2]
        transform_stamped.transform.rotation.w = odom_quat[3]

        odom_broadcaster = tf2_ros.TransformBroadcaster(self, qos_profile_sensor_data)
        odom_broadcaster.sendTransform(transform_stamped)

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = 0.0
        quat = Quaternion()
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        pose = Pose()
        pose.position = pt
        pose.orientation = quat
        odom.pose.pose = pose

        # publish the message
        self.odom_pub_.publish(odom)

        # 傾きセンサー
        senses[create.CLIFF_LEFT_SIGNAL]
        senses[create.CLIFF_FRONT_LEFT_SIGNAL]
        senses[create.CLIFF_FRONT_RIGHT_SIGNAL]
        senses[create.CLIFF_RIGHT_SIGNAL]

    def cmd_vel_callback(self, cmd_vel):
        x_vel = cmd_vel.linear.x
        a_vel = cmd_vel.angular.z
        # left_wheel_vel = x_vel - self.axle_len / 2.0 * a_vel
        # right_wheel_vel = x_vel + self.axle_len / 2.0 * a_vel
        self.robot.go(x_vel*self.max_forward, a_vel*self.max_rotation)
        # self.cmd_vel_pub_.publish(cmd_vel)

    def on_destroy(self):
        self.robot.go(0,0)
        self.robot.close()


def main(args=None):
    rclpy.init(args=args)

    create_driver = CreateDriver()

    while rclpy.ok():
        rclpy.spin_once(create_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    create_driver.on_destroy()
    create_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()