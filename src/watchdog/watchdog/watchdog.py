import rclp
from rclpy.node import Node

from geometrymsgs.msg import Twist
from stdmsgs.msg import String

class WatchdogNode(Node):

    def init(self):
        super().init('watchdog')
        self.publisher = self.createpublisher(Twist, 'cmdvel', 10)
        self.create_subscription(Twist, 'input_cmd', self.cmd_callback, 10)
        self.create_subscription(String, 'controller_cmd', self.controller_callback, 10)
        self.get_logger().info('Watchdog node started')
        self.before_started = True
        self.after_stopped = False

    def cmd_callback(self, msg):
        if self.after_stopped:
            return
        elif self.before_started:
            msg.linear.x = float()
            self.publisher.publish(msg)
        else:
            # This makes the turtle go backwards
            # (just so you know it's working)
            msg.linear.x=msg.linear.x*-1.0
            self.get_logger().info(f'msg.linear.x: {msg.linear.x}')
            self.publisher.publish(msg)

    def controller_callback(self, msg):
        if msg.data == 'start':
            self.before_started = False
        elif msg.data == 'stop':
            self.after_stopped = True
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name == '__main':
    main()
