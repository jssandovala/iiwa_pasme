import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class ToolsCenterMarkerPublisher(Node):
    def __init__(self):
        super().__init__('tools_center_marker_publisher')
        self.tools_center_marker_publisher = self.create_publisher(PoseStamped, '/tools_center_marker_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_positions)
        self.current_time = 0.0
        self.total_points = 400
        self.published_points = 0

        # Square parameters
        self.square_side_length = 0.1
        self.initial_position = [0.6247, 0.0, 0.1]

        # Calculate the corner points of the square
        self.square_corners = [
            [self.initial_position[0] + 0.5 * self.square_side_length, self.initial_position[1] + 0.5 * self.square_side_length],
            [self.initial_position[0] - 0.5 * self.square_side_length, self.initial_position[1] + 0.5 * self.square_side_length],
            [self.initial_position[0] - 0.5 * self.square_side_length, self.initial_position[1] - 0.5 * self.square_side_length],
            [self.initial_position[0] + 0.5 * self.square_side_length, self.initial_position[1] - 0.5 * self.square_side_length],
        ]

        self.current_position = self.initial_position

    def publish_positions(self):
        if self.published_points >= self.total_points:
            self.get_logger().info("Published {} points. Stopping the node.".format(self.published_points))
            self.timer.cancel()
            return

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'iiwa_base'

        if self.current_time < 5.0:
            # Stay in the initial position for the first 5 seconds
            self.current_position = self.initial_position
        elif 5.0 <= self.current_time <= 10.0:
            # Move from the initial position to the first corner
            start_point = self.initial_position
            end_point = self.square_corners[0]
            self.move_to_point(start_point, end_point, 5.0)
        elif 10.0 < self.current_time <= 20.0:
            # Perform the square trajectory
            square_side_time = (self.current_time - 10.0) % 10.0
            if 0.0 <= square_side_time < 5.0:
                start_point = self.square_corners[0]
                end_point = self.square_corners[1]
            else:
                start_point = self.square_corners[1]
                end_point = self.square_corners[2]
            self.move_to_point(start_point, end_point, 5.0)
        elif 20.0 < self.current_time <= 30.0:
            # Perform the square trajectory
            square_side_time = (self.current_time - 20.0) % 20.0
            if 0.0 <= square_side_time < 5.0:
                start_point = self.square_corners[2]
                end_point = self.square_corners[3]
            else:
                start_point = self.square_corners[3]
                end_point = self.square_corners[0]
            self.move_to_point(start_point, end_point, 5.0)
        elif 30.0 < self.current_time <= 35.0:
            # Move from the initial position to the first corner
            start_point = self.square_corners[0]
            end_point = self.initial_position
            self.move_to_point(start_point, end_point, 5.0)
            print(self.published_points)

        pose_msg.pose.position.x = self.current_position[0]
        pose_msg.pose.position.y = self.current_position[1]
        pose_msg.pose.position.z = self.initial_position[2]  # Keep the same Z coordinate

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.tools_center_marker_publisher.publish(pose_msg)

        # Increase the published points counter
        self.published_points += 1
        self.current_time += 0.1

    def move_to_point(self, start_point, end_point, duration):
        t = min((self.current_time - 10.0) % duration, duration) / duration
        self.current_position = [
            start_point[0] + t * (end_point[0] - start_point[0]),
            start_point[1] + t * (end_point[1] - start_point[1]),
        ]

def main(args=None):
    rclpy.init(args=args)
    tools_center_marker_publisher = ToolsCenterMarkerPublisher()
    rclpy.spin(tools_center_marker_publisher)

if __name__ == '__main__':
    main()
