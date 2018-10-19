import rclpy
from subscriber import SubscriberExample
from publisher import PublisherExample
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

rclpy.init(args=None)

exec = SingleThreadedExecutor()

pub = PublisherExample(topic_name="/Topic1")
sub = SubscriberExample(topic_name="/Topic1")

exec.add_node(pub)
exec.add_node(sub)

exec.spin()

pub.destroy_node()
sub.destroy_node()

rclpy.shutdown()
