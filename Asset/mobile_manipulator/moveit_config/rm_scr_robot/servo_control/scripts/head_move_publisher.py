#!/usr/bin/env python

import rospy
from rm_msgs.msg import Servo_Move


def servo_move_publisher():
    # 初始化节点
    rospy.init_node("servo_move_publisher", anonymous=True)

    # 创建发布器
    pub = rospy.Publisher("/servo_control/move", Servo_Move, queue_size=10)

    # 设置发送频率
    rate = rospy.Rate(0.5)  # 0.5 Hz

    # 初始化舵机id和角度值
    servo_id = 1  # TODO: Servo ID
    angle = 500

    while not rospy.is_shutdown():
        # 创建消息
        move_msg = Servo_Move()
        move_msg.servo_id = servo_id
        move_msg.angle = angle

        # 发布消息
        pub.publish(move_msg)
        rospy.loginfo(
            "Published servo move command. ID: %d, Angle: %d", servo_id, angle
        )

        # 切换角度
        if angle == 400:
            angle = 600
        else:
            angle = 400

        # 按照指定频率休眠
        rate.sleep()


if __name__ == "__main__":
    try:
        servo_move_publisher()
    except rospy.ROSInterruptException:
        pass
