#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket
from geometry_msgs.msg import Twist


def agvCtr_callback(msg):
    global sock_cli, servaddr

    strControldata = (
        "/api/joy_control?angular_velocity="
        + str(msg.angular.z)
        + "&linear_velocity="
        + str(msg.linear.x)
    )
    print("send msg", strControldata)

    try:
        sock_cli.sendall(strControldata.encode())
    except socket.error as e:
        print("send msg error:", str(e))
        sock_cli.close()
        sock_cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_cli.connect(servaddr)


if __name__ == "__main__":
    rospy.init_node("yunjiAgv_moveTwist")

    ip = rospy.get_param("~ser_ip", "192.168.10.10")
    port = rospy.get_param("~ser_port", 31001)

    sock_cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    servaddr = (ip, port)

    while True:
        try:
            sock_cli.connect(servaddr)
            break
        except socket.error as e:
            print("connect", ip, "failed! Attempt to reconnect...")
            rospy.sleep(2.0)

    print("connect", ip, "succeed!")

    rospy.Subscriber("/agv_control/agv_move_twist", Twist, agvCtr_callback)

    rospy.spin()

    sock_cli.close()
