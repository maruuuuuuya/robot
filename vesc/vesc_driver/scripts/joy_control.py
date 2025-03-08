#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# ボタンのインデックス
BUTTON_A = 0  # Aボタン: 右旋回
BUTTON_X = 1  # Xボタン: 前進
BUTTON_B = 2  # Bボタン: 後進
BUTTON_Y = 3  # Yボタン: 左旋回
BUTTON_ZR = 15  # ZRボタン (トリガーで高速化)

# 動作パラメータ
LINEAR_SPEED = 0.5  # 通常時の前進・後進の速度 (m/s)
ANGULAR_SPEED = 1.0  # 通常時の回転速度 (rad/s)
BOOST_MULTIPLIER = 3.0  # ZRボタン押下時のスピード倍率

# グローバル変数
cmd_vel_pub = None
current_cmd = Twist()  # 現在の速度指令

def joy_callback(data):
    global current_cmd
    # ZRボタンが押されているか判定
    is_boost = data.buttons[BUTTON_ZR] == 1

    # 通常速度または3倍速を決定
    linear_speed = LINEAR_SPEED * (BOOST_MULTIPLIER if is_boost else 1.0)
    angular_speed = ANGULAR_SPEED * (BOOST_MULTIPLIER if is_boost else 1.0)

    # ボタン入力に基づいて速度を設定
    if data.buttons[BUTTON_X] == 1:  # Xボタン: 前進
        current_cmd.linear.x = linear_speed
        current_cmd.angular.z = 0.0
        rospy.loginfo("Button: X -> Action: Forward{}".format(" (Boost)" if is_boost else ""))
    elif data.buttons[BUTTON_B] == 1:  # Bボタン: 後進
        current_cmd.linear.x = -linear_speed
        current_cmd.angular.z = 0.0
        rospy.loginfo("Button: B -> Action: Backward{}".format(" (Boost)" if is_boost else ""))
    elif data.buttons[BUTTON_A] == 1:  # Aボタン: 右旋回
        current_cmd.linear.x = 0.0
        current_cmd.angular.z = -angular_speed
        rospy.loginfo("Button: A -> Action: Turn Right{}".format(" (Boost)" if is_boost else ""))
    elif data.buttons[BUTTON_Y] == 1:  # Yボタン: 左旋回
        current_cmd.linear.x = 0.0
        current_cmd.angular.z = angular_speed
        rospy.loginfo("Button: Y -> Action: Turn Left{}".format(" (Boost)" if is_boost else ""))
    else:
        # ボタンを離した場合、速度を0にする
        current_cmd.linear.x = 0.0
        current_cmd.angular.z = 0.0
        rospy.loginfo("No button pressed -> Action: Stop")

def publish_cmd():
    global cmd_vel_pub, current_cmd
    rate = rospy.Rate(10)  # 10Hzで送信
    while not rospy.is_shutdown():
        cmd_vel_pub.publish(current_cmd)  # 現在の指令を送信
        rate.sleep()

def main():
    global cmd_vel_pub

    rospy.init_node('joy_control')

    # Joyメッセージのサブスクライバ
    rospy.Subscriber('/joy', Joy, joy_callback)

    # /cmd_velトピックのパブリッシャ
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.loginfo("Joy-Con control node started.")
    publish_cmd()

if __name__ == '__main__':
    main()