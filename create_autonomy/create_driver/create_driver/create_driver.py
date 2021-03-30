import os, sys
from . import create
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from create_msgs.msg import Bumper, ChargingState, DefineSong, PlaySong


class CreateDriver(Node):

    def __init__(self):
        super().__init__('create_driver')
        self.declare_parameter("roomba_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("max_forward", 50.0) # [m/s]
        self.declare_parameter("max_rotation", 100.0) # [m/s]

        self.declare_parameter("axle_len", 0.235) # [m]

        roomba_port = self.get_parameter("roomba_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value

        self.max_forward = self.get_parameter("max_forward").get_parameter_value().double_value
        self.max_rotation = self.get_parameter("max_rotation").get_parameter_value().double_value

        self.axle_len = self.get_parameter("axle_len").get_parameter_value().double_value

        self.robot = create.Create(roomba_port, BAUD_RATE=baud_rate)
        self.robot.toSafeMode()
        self.sensors = self.robot.sensors([create.WALL_SIGNAL, create.WALL_IR_SENSOR, create.LEFT_BUMP, create.RIGHT_BUMP, create.ENCODER_LEFT, create.ENCODER_RIGHT, create.CLIFF_LEFT_SIGNAL, create.CLIFF_FRONT_LEFT_SIGNAL, create.CLIFF_FRONT_RIGHT_SIGNAL, create.CLIFF_RIGHT_SIGNAL, create.DIRT_DETECTED])

        self.cmd_vel_sub_ = self.create_subscription(Twist,
                                                    "cmd_vel",
                                                    self.cmd_vel_callback,
                                                    qos_profile_sensor_data)
        self.bumper_pub_ = self.create_publisher(Bumper, "bumper", 1)
        self.charging_state_pub_ = self.create_publisher(ChargingState, "charging_state", 1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        senses = self.robot.sensors([create.WALL_SIGNAL, create.WALL_IR_SENSOR, create.LEFT_BUMP, create.RIGHT_BUMP, create.ENCODER_LEFT, create.ENCODER_RIGHT, create.CLIFF_LEFT_SIGNAL, create.CLIFF_FRONT_LEFT_SIGNAL, create.CLIFF_FRONT_RIGHT_SIGNAL, create.CLIFF_RIGHT_SIGNAL, create.DIRT_DETECTED])
        # 赤外線センサー
        wall_ir = senses[create.WALL_IR_SENSOR]
        wall_signal = senses[create.WALL_SIGNAL]
        # バンパーセンサー
        left_bump = senses[create.LEFT_BUMP]
        right_bump = senses[create.RIGHT_BUMP]
        bumper = Bumper()
        bumper.header.frame_id = "bumper"
        bumper.header.stamp = self.get_clock().now().to_msg()
        bumper.is_left_pressed = left_bump == 1
        bumper.is_right_pressed = right_bump == 1
        # Values in range [0, 4095]
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
        # print("enc:", enc_left, enc_right)
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