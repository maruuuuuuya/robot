#!/usr/bin/python
# -*- coding: utf-8 -*-

#!/usr/bin/env python

import rospy
import can
import struct
from mavros_msgs.msg import RCOut

# fmap関数と同様の処理
def fmap(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class CANMotorController:
    def __init__(self):
        # CANインターフェースの初期化
        can_interface = 'can0'
        self.bus = can.interface.Bus(can_interface, bustype='socketcan')

        # CAN送受信IDの設定
        self.tx_id = 0x200
        self.rx_ids = [0x201, 0x202]  # フィードバックID

        # モーターの制御データ
        self.motor1_current_A = 0.0
        self.motor2_current_A = 0.0

        # 制御間隔
        self.prev_time = rospy.Time.now()

        # RCOutトピック購読
        rospy.Subscriber('/mavros/rc/out', RCOut, self.rc_out_callback)
        rospy.loginfo("Initialized CAN Motor Controller Node with RCOut topic")

    def rc_out_callback(self, msg):
        """
        /mavros/rc/outトピックのコールバック
        """
        try:
            if len(msg.channels) > 3:  # 少なくとも1番目と3番目のチャンネルが存在する
                self.motor1_current_A = fmap(msg.channels[1], 1000, 2000, -10, 10)  # 1番目のチャンネル
                self.motor2_current_A = fmap(msg.channels[3], 1000, 2000, -10, 10)  # 3番目のチャンネル
                rospy.loginfo("Updated Motor Currents: Motor1={0}A, Motor2={1}A".format(
                    self.motor1_current_A, self.motor2_current_A
                ))
        except Exception as e:
            rospy.logerr("Error in RCOut callback: {0}".format(e))

    def send_motor_commands(self):
        """
        モーターへの制御コマンドを送信
        """
        try:
            # モーター1の制御値を計算
            motor1_current_byte = fmap(self.motor1_current_A, 0, 20, 0, 16384)

            # モーター2の制御値を計算
            motor2_current_byte = fmap(self.motor2_current_A, 0, 20, 0, 16384)

            # 送信データ作成 (モーター1、モーター2の制御値を順に配置)
            tx_data = [
                (motor1_current_byte >> 8) & 0xFF,  # モーター1の上位バイト
                motor1_current_byte & 0xFF,         # モーター1の下位バイト
                (motor2_current_byte >> 8) & 0xFF,  # モーター2の上位バイト
                motor2_current_byte & 0xFF,         # モーター2の下位バイト
                0x00, 0x00, 0x00, 0x00              # その他のデータ
            ]

            # 20msごとにデータ送信
            current_time = rospy.Time.now()
            if (current_time - self.prev_time).to_sec() > 0.02:
                msg = can.Message(arbitration_id=self.tx_id, data=tx_data, extended_id=False)
                self.bus.send(msg)
                rospy.loginfo("Sent CAN Data: {0}".format(tx_data))
                self.prev_time = current_time

        except Exception as e:
            rospy.logerr("Error in sending motor commands: {0}".format(e))

    def listen_and_control(self):
        """
        CANデータを受信しながらモーターを制御
        """
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            try:
                # モーター制御コマンド送信
                self.send_motor_commands()

                # CANデータ受信
                rx_msg = self.bus.recv(timeout=0.01)  # 10ms待機
                if rx_msg is not None and rx_msg.arbitration_id in self.rx_ids:
                    # データを解析
                    rx_data = rx_msg.data
                    motor_id = rx_msg.arbitration_id
                    angle = struct.unpack('>h', rx_data[0:2])[0]
                    rpm = struct.unpack('>h', rx_data[2:4])[0]
                    amp = struct.unpack('>h', rx_data[4:6])[0]
                    temp = struct.unpack('b', rx_data[6:7])[0]

                    rospy.loginfo("Received from Motor ID 0x{0:03X}: Angle={1}, RPM={2}, Amp={3}, Temp={4}".format(
                        motor_id, angle, rpm, amp, temp
                    ))
            except Exception as e:
                rospy.logerr("Error in CAN reception or control: {0}".format(e))
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('can_motor_controller', anonymous=True)
        controller = CANMotorController()
        controller.listen_and_control()
    except rospy.ROSInterruptException:
        pass
