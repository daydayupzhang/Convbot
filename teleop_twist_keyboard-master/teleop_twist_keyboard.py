#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from message.msg import Teleop
from geometry_msgs.msg import Twist 

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Teleop!
---------------------------
Chassis moving around:
   w
a  s   d

Manipulator operating
j : left (-x)
l : right (+x)
i : front (+y)
, : back (-y)
u : up (+z)
m : down (-z)

anything else : stop

q/z : increase/decrease linear speed by 10%
e/c : increase/decrease angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0,0,0,0),
        's':(-1,0,0,0,0,0),
        'a':(0,1,0,0,0,0),
        'd':(0,-1,0,0,0,0),
        'j':(0,0,-1,0,0,0),
        'l':(0,0,1,0,0,0),
        'i':(0,0,0,1,0,0),
        ',':(0,0,0,-1,0,0),
        'u':(0,0,0,0,1,0),
        'm':(0,0,0,0,-1,0),
        'k':(0,0,0,0,0,1),
        'K':(0,0,0,0,0,-1),
    }

speedBindings={
        'q':(1.1,1),
        'z':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_convbot', Teleop, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 1.0)
    turn = rospy.get_param("~turn", 1.0)
    armspeed = 5
    fwd = 0
    th = 0
    x = 0
    y = 0
    z = 0
    grp = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                fwd = moveBindings[key][0]
                th = moveBindings[key][1]
                x = moveBindings[key][2]
                y = moveBindings[key][3]
                z = moveBindings[key][4]
                grp = moveBindings[key][5]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                fwd = 0
                th = 0
                x = 0
                y = 0
                z = 0
                grp = 0
                if (key == '\x03'):
                    break

            teleop = Teleop()
            teleop.fwd = fwd*speed; teleop.th = th*turn; teleop.x = x*armspeed;
            teleop.y = y*armspeed; teleop.z = z*armspeed; teleop.grp = grp 
            pub.publish(teleop)

    except Exception as e:
        print(e)

    finally:
        teleop = Teleop()
        teleop.fwd = 0; teleop.th = 0; teleop.x = 0
        teleop.y = 0; teleop.z = 0; teleop.grp = 0
        pub.publish(teleop)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
