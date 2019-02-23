import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberExample(Node):
    '''
    This class is an example of a basic subscriber in ROS2. It simply establishes
    a subscription on the provided topic and publishes the message when received.
    '''

    def __init__(self, node_name="SubscriberNode", topic_name="topic"):
        '''
        This function creates and initializes the node. 

        Parameters
        ----------
        node_name:  String
            This is the name by which ROS2 will identify the node.
        topic_name: String
            This is the name of the topic where the subscriber will listen.

        Returns
        -------
        A subscriber node
        '''
 
        super().__init__(node_name)

        self.node_name = node_name 
        self.topic_name = topic_name

        # Establish the subscription and the callback on reception
        self.sub = self.create_subscription(String, topic_name, self.topic_callback)

    def topic_callback(self, msg):
        '''
        This function simply prints the message heard on the topic identified in the
        object's initialization function.
        '''
        self.get_logger().info('This is what I heard: "%s"' % msg.data) 
#        print("In subscriber ("+ self.node_name +"): read \""+ msg.data + "\" from topic"+ self.topic_name + "\n")
