#!/usr/bin/env python
#
#tento scrip byl prevzat z rosu a prepsan do me podoby a obstarava cteni z klavesnice
#a pote posilat data typu Twist na tema cmd_vel odkud si to odebira uzel ovld_bender a
#prepocitava a posila instrukce mbede jak ma bendr jed
#
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Cteni z klavesnice a data posilana ve formatu Twist!
---------------------------
Klavesy pro ovladani:
        8       
   4    5    6
        2     

CTRL-C pro Ukonceni
"""

moveBindings = {
	'8':(1,0), #dopredu
	'2':(-1,0), #dozadu
	'o':(1,-1),
	'6':(0,1),
	'4':(0,-1),
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

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .1
turn = .1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('pid_change', Twist,queue_size=5)
	rospy.init_node('teleop_twist_PID')

	x = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				th = moveBindings[key][1]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


