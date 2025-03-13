import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import Button
from std_msgs.msg import String

class ROS2GUI(Node):
    def __init__(self):
        super().__init__('teleop_gui')

        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.counter = 0

        self.init_gui()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title('ROS 2 GUI')

        self.label = tk.Label(self.root, text='ROS 2 GUI Example')
        self.label.pack(pady=10)

        self.button1 = Button(self.root, text='Publish Message', command=self.publish_message)
        self.button1.pack(pady=5)

        self.button2 = Button(self.root, text='Quit', command=self.quit_gui)
        self.button2.pack(pady=5)

    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'
        self.publisher.publish(msg)
        self.counter += 1

    def quit_gui(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2GUI()
    try:
        node.run()
    finally:
        # Cleanup code, if any
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

