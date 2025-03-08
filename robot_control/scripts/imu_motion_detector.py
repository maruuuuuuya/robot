#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import math

# 動き検知のしきい値（適宜調整）
ACCEL_THRESHOLD = 0.5  # m/s²

# 直前の加速度データ
prev_accel = None

# 動きを検知したかどうかのフラグ
movement_detected = False

def imu_callback(data):
    global prev_accel, movement_detected

    # 現在の加速度データを取得
    accel_x = data.linear_acceleration.x
    accel_y = data.linear_acceleration.y
    accel_z = data.linear_acceleration.z

    # 初回だけ現在の値を記録
    if prev_accel is None:
        prev_accel = (accel_x, accel_y, accel_z)
        return

    # 変化量を計算
    delta_x = abs(accel_x - prev_accel[0])
    delta_y = abs(accel_y - prev_accel[1])
    delta_z = abs(accel_z - prev_accel[2])

    total_change = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

    # しきい値を超えたら動きを検知
    if total_change > ACCEL_THRESHOLD:
        rospy.loginfo("⚠️ 動き検知をしました！")
        movement_detected = True
    else:
        movement_detected = False

    # 現在の値を記録
    prev_accel = (accel_x, accel_y, accel_z)

def check_no_movement(event):
    """ 1秒ごとに動きがない場合に「動きはありません」を出力 """
    if not movement_detected:
        rospy.loginfo("動きはありません")

def main():
    rospy.init_node("imu_motion_detector", anonymous=True)

    # IMUのデータを購読
    rospy.Subscriber("/handsfree/imu", Imu, imu_callback)

    # 1秒ごとに動きをチェックするタイマー
    rospy.Timer(rospy.Duration(1.0), check_no_movement)

    rospy.spin()

if __name__ == "__main__":
    main()
