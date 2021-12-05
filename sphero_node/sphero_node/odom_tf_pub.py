import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import nav_msgs.msg

class FramePublisher(Node):

    def __init__(self):
        super().__init__('sphero_tf_publisher')

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        #subscribe to the odometry message
        self.odom_subscriber = self.create_subscription(
            nav_msgs.msg.Odometry,
            '/odom',
            self.handle_odometry,
            1)

    def handle_odometry(self, msg: nav_msgs.msg.Odometry):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        #t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
