#!/usr/bin/python
import roslib; roslib.load_manifest('create_supervisor')
import rospy

from geometry_msgs.msg import Twist
from irobot_create_2_1.msg import SensorPacket
from irobot_create_2_1.srv import *

class CreateSupervisor:
    def __init__(self):
        self.currently_charging = True
        self.pub = rospy.Publisher('cmd_vel', Twist)

    def sense(self, req):
        if not self.currently_charging and req.batteryCharge < 2300:
            self.currently_charging = True
            rospy.wait_for_service('demo')
            try:
                call_demo = rospy.ServiceProxy('demo', Demo)
                call_demo(1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
#        print req.batteryCapacity - req.batteryCharge
        if self.currently_charging and req.batteryCapacity - req.batteryCharge < 100:
#            print "In here..."
            twist = Twist()
            twist.linear.x = -0.5
            self.pub.publish(twist)
            rospy.sleep(3.0)
            twist.linear.x = 0
            self.pub.publish(twist)
            self.currently_charging = False


if __name__ == '__main__':
    node = rospy.init_node('create_supervisor')
    supervisor = CreateSupervisor()

    rospy.Subscriber('sensorPacket', SensorPacket, supervisor.sense)

    rospy.spin()
