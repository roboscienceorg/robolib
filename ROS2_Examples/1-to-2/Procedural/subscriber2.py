import rclpy
from std_msgs.msg import String

def chatter_callback(msg):
    global node
    node.get_logger().info('Subscriber 2 heard: "%s"' % msg.data)

rclpy.init(args=None)
node = rclpy.create_node('min_sub')

subscription = node.create_subscription(String, 'topic', chatter_callback)

while rclpy.ok():
    rclpy.spin_once(node)
