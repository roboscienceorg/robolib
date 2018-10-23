import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherExample(Node):
    '''
    This class is an example of a basic publisher in ROS2. It simply establishes
    a publisher on the provided topic and publishes "Hello!" at 1 Hz.
    '''

    def __init__(self, node_name="PublisherNode", topic_name="topic"):
        '''
        This function creates and initializes the node. 

        Parameters
        ----------
        node_name:  String
            This is the name by which ROS2 will identify the node.
        topic_name: String
            This is the name of the topic where the string "Hello!" will be
            published.

        Returns
        -------
        A publisher node
        '''
        super().__init__(node_name)

        self.node_name = node_name
        self.topic_name = topic_name

        self.pub = self.create_publisher(String, topic_name)       
        self.timer = self.create_timer(1, self.publish_hello)

    def publish_hello(self):
        '''
        This function simply publishes the string "Hello!" to the topic
        identified in the object's initialization function.
        '''
        msg = String()
        msg.data = "Hello!"
        self.pub.publish(msg)
