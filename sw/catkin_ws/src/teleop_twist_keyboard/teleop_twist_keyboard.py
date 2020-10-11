#!/usr/bin/env python
#
# script for manual control of the robot
#
# input from keyboard
#
# publishing topic /cmd_vel
#
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import sys, curses, select, termios, tty
import time

global x
global z
global errorCount

msg = """
Manual control for dacep robot
---------------------------
Move bindings
        i
   j    k    l
        ,

CTRL-C for quit
"""
'''
moveBindings = {
        '8':(1,0), #dopredu
        '2':(-1,0), #dozadu
        'o':(1,-1),
        '6':(0,-1),
        '4':(0,1),
        'u':(1,1),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }
'''

moveBindings = {
        'i':(1,0), #dopredu
        ',':(-1,0), #dozadu
        'o':(1,-1),
        'l':(0,-1),
        'j':(0,1),
        'u':(1,1),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }


'''
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
'''

class NonBlockingConsole(object):
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    def get_data(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return 0

def bumperCallback(data):
    global errorCount
    errorCount = 0
    global bumperDetect
    bumperDetect = data.data

def bumperListener():
    ppp = rospy.Subscriber("bumperStatus", Int32, bumperCallback)

speed = .1
turn = .1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)



if __name__=="__main__":
    global bumperDetect
    global errorCount
    bumperDetect = 0
    errorCount = 0
    settings = termios.tcgetattr(sys.stdin)
    #nbc = NonBlockingConsole()

    pub = rospy.Publisher('/cmd_vel', Twist,queue_size=5)
    rospy.init_node('teleop_twist_keyboard')


    x = 0
    th = 0
    status = 0


#   try:

    print msg
    print vels(speed,turn)
    with NonBlockingConsole() as nbc:
        while not rospy.is_shutdown():
            errorCount += 1
            #sc.nodelay(1)
            #key = sc.getch()
            #key = getKey()
            #print(key)
            key = nbc.get_data()
            #print(key)
            bumperListener()
            #~ if (bumperDetect > 0) or (errorCount > 100):
                #~ key = 'k'
            if key in moveBindings.keys():
                x += moveBindings[key][0]
                th += moveBindings[key][1]

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15

            elif key == 'k':
                x = 0
                th = 0
                if (key == '\x03'):
                    break


            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            if (twist.linear.x < -0.8):
                twist.linear.x = -0.8
            if (twist.linear.x > 0.8):
                twist.linear.x = 0.8
            if (twist.angular.x < -3.0):
                twist.angular.x = -3.0
            if (twist.angular.x > 3.0):
                twist.angular.x = 3.0
            pub.publish(twist)

            time.sleep(0.05)

'''
    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
'''



