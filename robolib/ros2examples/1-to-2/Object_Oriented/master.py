import rclpy
from subscriber1 import Subscriber1Example
from subscriber2 import Subscriber2Example
from publisher import PublisherExample
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

# Initialize rclpy
rclpy.init(args=None)

# This establishes an executor which will run all the nodes
# In this case they will all be run on one thread
exec = SingleThreadedExecutor()

# This builds the nodes
pub = PublisherExample(topic_name="/Topic1")
sub1 = Subscriber1Example(node_name="Sub1", topic_name="/Topic1")
sub2 = Subscriber2Example(node_name="Sub2", topic_name="/Topic1")

# Add them to the executor to be executed
exec.add_node(pub)
exec.add_node(sub1)
exec.add_node(sub2)

# Actually run the nodes
exec.spin()

# Teardown
pub.destroy_node()
sub1.destroy_node()
sub2.destry_node()

rclpy.shutdown()
