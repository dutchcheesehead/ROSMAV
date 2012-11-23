#!/usr/bin/env python
import roslib; roslib.load_manifest('step_nav')
import rospy
import sys, select, termios, tty
from step_nav.msg import Fudge

msg = """
Reading from the keyboard  and Publishing to Fudge!

q/z : increase/decrease fudge by 10%
anything else : stop

CTRL-C to quit
"""
keyBindings={
    'q':1.1,
    'a':-1.0,
    'z':0.9,
    }

global f 
f = Fudge()
f.fudge = 0.3

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
	
    pub = rospy.Publisher('fudge', Fudge)
    rospy.init_node('fudge')

    try:
        print msg
        status = 0

        while(1):
            key = getKey()
            if key in keyBindings.keys():
                f.fudge = f.fudge * keyBindings[key]

                print "new value: %5.3f" % f.fudge

                if (status == 14):
                    print msg
                status = (status + 1) % 15
            else:
                x = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub.publish(f)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


