import rclpy
from rclpy.node import Node

from random import randint # random integer
from cmath import pi # = 3 ^^

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn

COLLISION_DIST = 0.3 # distance 
MAX_VELOCITY = 0.101 # maximum velocity to move with

class VelocityController(DrivingSwarmNode):

    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scanner = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()
        
    def get_angle(self):
        random = randint(1, 359) #random degree
        while self.scanner[random] < COLLISION_DIST: # check if angle will resolve collision
            random = randint(1, 359) # get new angle to test
        return (2 * pi) /(360 / random) # return angular.z value to turn 

    def timer_cb(self):
        msg = Twist()
        v = 0.0
        if not self.scanner:
            return # if no scanner data -> dont move
        if min(self.scanner[0], self.scanner[1]) < COLLISION_DIST: # if scanner forwards is smaller then safe distance -> turn
            msg.angular.z = self.get_angle() # move random angle
        else:
            v = MAX_VELOCITY # move forward
        
        msg.linear.x = v
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        self.scanner = msg.ranges # save the distance points from the scanner
        



def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
