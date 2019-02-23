import rclpy
from subscriber import SubscriberExample
from publisher import PublisherExample
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

# Initialize rclpy
rclpy.init(args=None)

# This establishes an executor which will run all the nodes
# In this case they will all be run on one thread
exec = SingleThreadedExecutor()

# This builds the nodes
pub = PublisherExample(topic_name="/Topic1")
sub = SubscriberExample(topic_name="/Topic1")

# Add them to the executor to be executed
exec.add_node(pub)
exec.add_node(sub)

# Actually run the nodes
exec.spin()

# Teardown
pub.destroy_node()
sub.destroy_node()

rclpy.shutdown()
