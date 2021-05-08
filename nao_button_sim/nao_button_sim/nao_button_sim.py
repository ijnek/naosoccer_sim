import rclpy
from rclpy.node import Node
from pynput import keyboard

from nao_interfaces.msg import Buttons


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

        self.publisher_ = self.create_publisher(Buttons, 'buttons', 10)
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        if frequency == 0:
            print('Could not read parameter correctly. frequency must be an integer greater than zero')
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


def main(args=None):
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
