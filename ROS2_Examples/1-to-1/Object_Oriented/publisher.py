import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherExample(Node):
    def __init__(self, node_name="PublisherNode", topic_name="topic"):
        super().__init__(node_name)

        self.node_name = node_name
        self.topic_name = topic_name

        self.pub = self.create_publisher(String, topic_name)       
        self.timer = self.create_timer(1, self.publish_hello)

    def publish_hello(self):
        msg = String()
        msg.data = "Hello!"
        self.pub.publish(msg)
