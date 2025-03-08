#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# パラメータ設定
WHEEL_RADIUS = 0.275 / 2  # タイヤの半径 (m)
WHEEL_BASE = 0.37         # タイヤ間の幅 (m)

def twist_callback(msg):
    # 入力されたTwistメッセージから線速度と角速度を取得
    linear_velocity = msg.linear.x    # 前進・後退の速度 (m/s)
    angular_velocity = msg.angular.z  # 回転速度 (rad/s)

    # 左右の車輪の線速度を計算
    left_wheel_velocity = linear_velocity - (angular_velocity * WHEEL_BASE / 2)
    right_wheel_velocity = linear_velocity + (angular_velocity * WHEEL_BASE / 2)

    # 線速度をRPMに変換
    left_wheel_rpm = (left_wheel_velocity / (2 * 3.141592 * WHEEL_RADIUS)) * 60
    left_wheel_rpm = left_wheel_rpm /2
    right_wheel_rpm = (right_wheel_velocity / (2 * 3.141592 * WHEEL_RADIUS)) * 60
    right_wheel_rpm = right_wheel_rpm /2

    # 結果を出力トピックにパブリッシュ
    left_motor_pub.publish(Float64(left_wheel_rpm))
    right_motor_pub.publish(Float64(right_wheel_rpm))

def main():
    rospy.init_node('twist_to_rpm')

    # サブスクライバを設定
    rospy.Subscriber('/cmd_vel', Twist, twist_callback)

    # パブリッシャを設定
    global left_motor_pub, right_motor_pub
    left_motor_pub = rospy.Publisher('VESCLEFT/commands/motor/speed', Float64, queue_size=10)
    right_motor_pub = rospy.Publisher('VESCRIGHT/commands/motor/speed', Float64, queue_size=10)

    rospy.loginfo("Twist to RPM node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
