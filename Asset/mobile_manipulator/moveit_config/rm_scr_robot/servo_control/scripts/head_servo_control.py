#!/usr/bin/env python
import rospy
import serial
from rm_msgs.msg import Servo_Move, Servo_GetAngle

ros_ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
sBUFFERSIZE = 10


def callback_servoMove(msg):
    s_buffer = bytearray(sBUFFERSIZE)
    s_buffer[0] = 0x55
    s_buffer[1] = 0x55
    s_buffer[2] = 0x08
    s_buffer[3] = 0x03
    s_buffer[4] = 0x01
    s_buffer[5] = 0xE8
    s_buffer[6] = 0x03
    s_buffer[7] = msg.servo_id
    s_buffer[8] = msg.angle & 0xFF
    s_buffer[9] = (msg.angle >> 8) & 0xFF
    ros_ser.write(s_buffer)
    rospy.loginfo("Control Servo Move")


def callback_servoGetAngle(msg):
    s_buffer = bytearray(sBUFFERSIZE)
    s_buffer[0] = 0x55
    s_buffer[1] = 0x55
    s_buffer[2] = 0x04
    s_buffer[3] = 0x15
    s_buffer[4] = 0x01
    s_buffer[5] = msg.servo_id
    ros_ser.write(s_buffer)
    rospy.loginfo("Get Servo Angle")


def main():
    rospy.init_node("my_serial_node", anonymous=True)
    rospy.Subscriber("/servo_control/move", Servo_Move, callback_servoMove)
    rospy.Subscriber("/servo_control/get_angle", Servo_GetAngle, callback_servoGetAngle)
    rate = rospy.Rate(10)
    rospy.loginfo("Servo controller start")
    while not rospy.is_shutdown():
        if ros_ser.inWaiting():
            rospy.loginfo("Reading from serial port")
            data = ros_ser.read(ros_ser.inWaiting())
            print(" ".join("{:02x}".format(ord(c)) for c in data))

        rate.sleep()

    ros_ser.close()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
