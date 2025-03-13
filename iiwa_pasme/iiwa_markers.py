import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_markers)
        self.marker_id = 0
        # Tools center marker (yellow)
        self.tools_center_marker = self.create_marker(0.6247, 0.0, 0.1, 1.0, 1.0, 0.0)
        # RCM marker (pink)
        self.rcm_marker = self.create_marker(0.6247, 0.0, 0.3276, 1.0, 0.5, 0.5)
        self.rcm_marker.color.a = 0.2

        # Subscribe to the topic that provides updated positions for tools center
        self.subscription = self.create_subscription(
            PoseStamped,
            'tools_center_marker_pose',
            self.update_tools_center_callback,
            10)

    def publish_markers(self):
        # Tools center marker (yellow)
        self.marker_publisher.publish(self.tools_center_marker)
        
        # RCM marker (pink)
        self.marker_publisher.publish(self.rcm_marker)

    def update_tools_center_callback(self, msg):
        # Update the position of the tools center marker based on the received message
        if msg.header.frame_id == 'iiwa_base':
            self.tools_center_marker.pose.position = msg.pose.position

    def create_marker(self, x, y, z, r, g, b, marker_type=Marker.SPHERE):
        marker = Marker()
        marker.header.frame_id = 'iiwa_base'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.marker_id
        self.marker_id += 1

        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        return marker

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
