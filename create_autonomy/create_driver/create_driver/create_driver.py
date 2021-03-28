import os, sys
# from . import create
from  pycreate2 import Create2
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
        self.declare_parameter("max_forward", 0.05) # [m/s]
        self.declare_parameter("max_rotation", 0.2) # [m/s]

        roomba_port = self.get_parameter("roomba_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        max_forward = self.get_parameter("max_forward").get_parameter_value().integer_value
        max_rotation = self.get_parameter("max_rotation").get_parameter_value().integer_value

        # self.robot = create.Create(roomba_port, BAUD_RATE=baud_rate)
        self.robot = Create2(roomba_port)
        self.robot.start()
        self.robot.safe()
        # self.robot.toSafeMode()
        # self.sensors = self.robot.sensors([create.WALL_SIGNAL, create.WALL_IR_SENSOR, create.LEFT_BUMP, create.RIGHT_BUMP, create.ENCODER_LEFT, create.ENCODER_RIGHT, create.CLIFF_LEFT_SIGNAL, create.CLIFF_FRONT_LEFT_SIGNAL, create.CLIFF_FRONT_RIGHT_SIGNAL, create.CLIFF_RIGHT_SIGNAL, create.DIRT_DETECTED])

        self.cmd_vel_sub_ = self.create_subscription(Twist,
                                                    "cmd_vel",
                                                    self.cmd_vel_callback,
                                                    qos_profile_sensor_data)
        self.bumper_pub_ = self.create_publisher(Bumper, "bumper", 1)
        self.charging_state_pub_ = self.create_publisher(ChargingState, "charging_state", 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        sensors = self.robot.get_sensors()
        # # 赤外線センサー
        # self.senses[create.WALL_IR_SENSOR]
        # self.senses[create.WALL_SIGNAL]
        # バンパーセンサー
        bumper = Bumper()
        bumper.header.frame_id = "bumper"
        bumper.header.stamp = self.get_clock().now().to_msg()
        bumper.is_left_pressed = False
        bumper.is_right_pressed = False
        # Values in range [0, 4095]
        bumper.light_signal_left = sensors.light_bumper_left
        bumper.light_signal_front_left = sensors.light_bumper_front_left
        bumper.light_signal_center_left = sensors.light_bumper_center_left
        bumper.light_signal_center_right = sensors.light_bumper_center_right
        bumper.light_signal_front_right = sensors.light_bumper_front_right
        bumper.light_signal_right = sensors.light_bumper_right
        self.bumper_pub_.publish(bumper)
        # # エンコーダー
        # self.senses[create.ENCODER_LEFT]
        # self.senses[create.ENCODER_RIGHT]
        # # 傾きセンサー?
        # self.senses[create.CLIFF_LEFT_SIGNAL]
        # self.senses[create.CLIFF_FRONT_LEFT_SIGNAL]
        # self.senses[create.CLIFF_FRONT_RIGHT_SIGNAL]
        # self.senses[create.CLIFF_RIGHT_SIGNAL]

    def cmd_vel_callback(self, cmd_vel):
        self.robot.go(robot_dir*FWD_SPEED,robot_rot*ROT_SPEED)
        # self.cmd_vel_pub_.publish(cmd_vel)

    def on_destroy(self):
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