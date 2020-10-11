#!/usr/bin/env python
# license removed for brevity

import rospy
import serial
import struct

from geometry_msgs.msg import Twist

PORT = '/dev/ttyUSB0'
BAUDRATE = 9600
TIMEOUT = 0.1

class serial_class:
    enk_left = 0
    enk_right = 0
    enk = Twist()

    def __init__(self):
        self.port = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)

    def callback(self,data):
        print "haha"

    def read_enk(self):
        self.port.write([0x00,0x25])

        try:
            enk_raw = self.port.read(8)
        except:
            pass

        #~ print len(enk_raw)
        #~ for i in range(len(enk_raw)):
            #~ self.enk.append(enk_raw[i].encode('hex'))
        #~ print self.enk

        self.enk_left = struct.unpack('>I',enk_raw[0:4])[0]
        self.enk_right = struct.unpack('>I',enk_raw[4:9])[0]

        self.enk.linear.x = self.enk_left
        self.enk.linear.y = self.enk_right

    def main(self):
        self.pub_enk = rospy.Publisher('enk_actual', Twist, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)

        rospy.init_node('serial', anonymous=True)

        # reset
        self.port.write([0x00,0x35])
        while not rospy.is_shutdown():
            self.read_enk()
            self.pub_enk.publish(self.enk)

            self.read_enk()
            while(self.enk_right < 350):
                #go forward
                self.port.write([0x00,0x32,0x8F])
                self.read_enk()
                rospy.sleep(0.05)

            self.port.write([0x00,0x32,0x80])
            rospy.sleep(0.1)

if __name__ == '__main__':
    ser = serial_class()
    try:
        ser.main()
    except rospy.ROSInterruptException:
        pass


