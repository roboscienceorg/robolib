import rclpy
import time
from std_msgs.msg import String

rclpy.init(args=None)
pnode = rclpy.create_node('minimal_publisher')

publisher = pnode.create_publisher(String, 'topic')


while(True): 
    msg = String(data="Hello, World!")
    publisher.publish(msg)
    print('Publishing the message "Hello, World!"')
    time.sleep(1)
