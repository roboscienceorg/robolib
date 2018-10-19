import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberExample(Node):
    def __init__(self, node_name="SubscriberNode", topic_name="topic"):
        super().__init__(node_name)

        self.node_name = node_name 
        self.topic_name = topic_name

        self.sub = self.create_subscription(String, topic_name, self.topic_callback)

    def topic_callback(self, msg):
        print("In subscriber", self.node_name, ": read", msg.data, "from topic", self.topic_name)
