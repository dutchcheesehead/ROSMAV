#!/usr/bin/env python
import roslib; roslib.load_manifest('move')
import rospy
from geometry_msgs.msg import Twist
import move.msg
import actionlib


pub = None
sp = None

class MoveAction():
    _feedback = move.msg.MoveFeedback()
    _result   = move.msg.MoveResult()


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                move.msg.MoveAction,
                                                execute_cb=self.execute_cb)
        self._as.start()

    def execute_cb(self, goal):
        # publish and quit
        self._feedback.feedback = False
        self._as.publish_feedback(self._feedback)

        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

        g = (goal.command)[0]
        if g == "forward":
            twist.linear.x = 1
        elif g == "backward":
            twist.linear.x = -1
        elif g == "left":
            twist.angular.z = 1
        elif g == "right":
            twist.angular.z = -1
        elif g == "stop":
            pass
        else:
            # Failed to parse!
            rospy.logerr("Move failed to parse the command \"%s\""%g)
            self._result.success = False
            self._as.set_aborted(self._result)
            return

        # Succeeded parsing!
        pub.publish(twist)
        self._feedback.feedback = True
        self._as.publish_feedback(self._feedback)
        # if self._as.is_preempt_requested():
        #     rospy.loginfo('%s: Preempted' % self._action_name)
        #     self._as.set_preempted()
        # else:
        self._result.success = True
        self._as.set_succeeded(self._result)
        return




def msgToSp(msg):
    global sp
    sp = msg

def main():
    global pub
    rospy.init_node('move')
    pub = rospy.Publisher('cmd_vel',Twist)
    MoveAction(rospy.get_name())
    rospy.spin()
    #s = rospy.myargv()[1:]





if __name__ == '__main__':
    main()



