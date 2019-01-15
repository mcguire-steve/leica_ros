#! /usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls vert angular speed
# axis 0 aka left stick horizonal controls horiz angular speed
class LeicaTeleop(object):
    def __init__(self):
        # publishing to "cmd_vel" to control the totalstation
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        #suck in some parameters that map joystick inputs to TS movement
        self.h_axis = rospy.get_param('~h_axis', 0)
        self.v_axis = rospy.get_param('~v_axis', 1)
        
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, self.callback)

    def callback(self, data):
        twist = Twist()
        #Max speed is 0.79 rad/s according to the manual
        twist.angular.z = -0.5*data.axes[self.h_axis]
        twist.angular.y = 0.5*data.axes[self.v_axis]
        self.pub.publish(twist)

# Intializes everything
def start():
    rospy.init_node('leica_joy')
    node = LeicaTeleop()
    rospy.spin()

if __name__ == '__main__':
    start()
