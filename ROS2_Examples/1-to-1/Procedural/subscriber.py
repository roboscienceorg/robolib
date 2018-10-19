import rclpy
from std_msgs.msg import String

rclpy.init(args=None)
node = rclpy.create_node('min_sub')

def chatter_callback(msg):
    global node
    node.get_logger().info('This is what I heard: "%s"' % msg.data)

subscription = node.create_subscription(String, 'topic', chatter_callback)

while rclpy.ok():
    rclpy.spin_once(node)
