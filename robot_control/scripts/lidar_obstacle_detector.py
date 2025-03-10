#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy  # ROS用のPythonライブラリ
from sensor_msgs.msg import LaserScan  # LiDARのデータ型を扱う
import math  # 角度計算のためにmathライブラリを使用

def callback(data):
    """
    LiDARのスキャンデータを受け取り、前後左右の距離を取得して表示する
    """
    angle_min = data.angle_min  # LiDARのスキャン開始角度（ラジアン）
    angle_increment = data.angle_increment  # 1データごとの角度の増加量（ラジアン）
    ranges = data.ranges  # LiDARの距離データ（単位：メートル）

    # 各方向のインデックス（配列の位置）を計算
    front_index = 0  # 180°（正面）
    back_index = len(ranges) // 2  # 0°（背面）
    right_index = int(len(ranges) * 3/4)  # 90°（右）
    left_index = int(len(ranges) * 1/4)  # -90°（左）

    # 各方向の距離を取得（範囲外のデータは inf にする）
    # LiDARの最小測定距離 15cm以上、最大測定距離 10m以下の範囲内のデータだけ取得
    front_dist = ranges[front_index] if 0.15 < ranges[front_index] < 10 else float('inf')
    back_dist = ranges[back_index] if 0.15 < ranges[back_index] < 10 else float('inf')
    right_dist = ranges[right_index] if 0.15 < ranges[right_index] < 10 else float('inf')
    left_dist = ranges[left_index] if 0.15 < ranges[left_index] < 10 else float('inf')

    # 障害物の距離情報をログとして表示（ROSの標準出力）
    rospy.loginfo("前: {:.2f}m | 後: {:.2f}m | 右: {:.2f}m | 左: {:.2f}m".format(front_dist, back_dist, right_dist, left_dist))

def listener():
    """
    ROSノードを初期化し、LiDARのデータを購読（Subscribe）する
    """
    rospy.init_node('lidar_observer', anonymous=True)  # ノードを初期化
    rospy.Subscriber("/scan", LaserScan, callback)  # LiDARのトピック"/scan"を購読し、callback関数を実行
    rospy.spin()  # ノードが終了しないように待機（無限ループ）

if __name__ == '__main__':
    listener()  # メイン処理を実行
