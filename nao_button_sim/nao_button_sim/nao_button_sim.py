# Copyright 2021 Kenji Brameld
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

import os

from nao_sensor_msgs.msg import Buttons

from pynput import keyboard

import rclpy
from rclpy.node import Node


chest = False
l_foot_bumper_left = False
l_foot_bumper_right = False
r_foot_bumper_left = False
r_foot_bumper_right = False


def on_press(key):
    try:
        if key.char == 'g':
            global chest
            chest = True
        elif key.char == 'c':
            global l_foot_bumper_left
            l_foot_bumper_left = True
        elif key.char == 'v':
            global l_foot_bumper_right
            l_foot_bumper_right = True
        elif key.char == 'b':
            global r_foot_bumper_left
            r_foot_bumper_left = True
        elif key.char == 'n':
            global r_foot_bumper_right
            r_foot_bumper_right = True
    except AttributeError:
        pass


def on_release(key):
    try:
        if key.char == 'g':
            global chest
            chest = False
        elif key.char == 'c':
            global l_foot_bumper_left
            l_foot_bumper_left = False
        elif key.char == 'v':
            global l_foot_bumper_right
            l_foot_bumper_right = False
        elif key.char == 'b':
            global r_foot_bumper_left
            r_foot_bumper_left = False
        elif key.char == 'n':
            global r_foot_bumper_right
            r_foot_bumper_right = False
    except AttributeError:
        pass


class ButtonsPublisher(Node):

    def __init__(self):
        super().__init__('buttons_publisher')

        self.declare_parameter('frequency', 50)

        self.publisher_ = self.create_publisher(Buttons, 'sensors/buttons', 10)
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        if frequency == 0:
            print('Could not read parameter correctly. frequency must be an ' +
                  'integer greater than zero')
            exit()

        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)

    def timer_callback(self):
        msg = Buttons()
        msg.chest = chest
        msg.l_foot_bumper_left = l_foot_bumper_left
        msg.l_foot_bumper_right = l_foot_bumper_right
        msg.r_foot_bumper_left = r_foot_bumper_left
        msg.r_foot_bumper_right = r_foot_bumper_right
        self.publisher_.publish(msg)

        printStatus([chest, l_foot_bumper_left, l_foot_bumper_right,
                     r_foot_bumper_left, r_foot_bumper_right])


def printStatus(bool_vec):
    status_str = '|'
    for b in bool_vec:
        status_str += '{:^19}|'.format(press_status(b))
    print(status_str, end='\r')


def print_assigned_keys():
    print('                                 ---------  Assigned Keys  --------'
          '-                                 ')
    print('                                      G - Chest button')
    print('                                      C - Left Foot Bumper Left')
    print('                                      V - Left Foot Bumper Right')
    print('                                      B - Right Foot Bumper Left')
    print('                                      N - Right Foot Bumper Right')
    print('\n')


def print_button_press_status_header():
    print('---------------------------------------  BUTTON PRESS STATUS  ----'
          '-----------------------------------')

    print('\n|{:^19}|{:^19}|{:^19}|{:^19}|{:^19}|' .format(
        'Chest (G)',
        'LFoot BumperL (C)',
        'LFoot BumperR (V)',
        'RFoot BumperL (B)',
        'RFoot BumperR (N)'))
    print('==================================================================='
          '==================================')


def press_status(pressed):
    return 'Pressed' if pressed else ''


def main(args=None):
    os.system('stty -echo')

    print_assigned_keys()
    print_button_press_status_header()

    # listen in a non-blocking fashion:
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    rclpy.init(args=args)
    buttons_publisher = ButtonsPublisher()
    rclpy.spin(buttons_publisher)

    # Destroy the node explicitly
    buttons_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
