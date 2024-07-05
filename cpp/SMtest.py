#!/usr/bin/env python

import rospy
from sfdt.msg import SensorMsg, MotorAngle
from std_msgs.msg import Bool
import csv
import os
from datetime import datetime
import math
import time

# 设置CSV文件路径
home_directory = os.path.expanduser("~")
csv_filename = os.path.join(home_directory, "sensor_data.csv")

# 数据数组
data_array = []

# 初始化发布器和订阅器
pub = None
angles_msg = MotorAngle()

def sensor_data_callback(msg):
    global data_array
    # 获取当前时间戳
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    
    # 打印接收到的传感器数据
    rospy.loginfo("Received sensor data:")
    rospy.loginfo("Fx: %f, Fy: %f, Fz: %f", msg.Fx1, msg.Fy1, msg.Fz1)
    rospy.loginfo("Tx: %f, Ty: %f, Tz: %f", msg.Tx1, msg.Ty1, msg.Tz1)
    rospy.loginfo("Fx2: %f, Fy2: %f, Fz2: %f", msg.Ffx1, msg.Ffy1, msg.Ffz1)
    rospy.loginfo("Fx2: %f, Fy2: %f, Fz2: %f", msg.Tfx1, msg.Tfy1, msg.Tfz1)


    # 保存数据到数组中
    data_array.append([timestamp, angles_msg.angles[0], angles_msg.angles[1], angles_msg.angles[2],
                       msg.Fx, msg.Fy, msg.Fz, msg.Tx, msg.Ty, msg.Tz])
def servo_control():
    global pub, angles_msg
    # 初始化 ROS 节点
    rospy.init_node('controller', anonymous=True)

    # 创建一个发布器，将消息发布到名为 "motor_control_all" 的主题上
    pub = rospy.Publisher('motor_control_all', MotorAngle, queue_size=10)

    # 创建一个订阅器，订阅 "sensor_data" 主题
    rospy.Subscriber('sensor_data', SensorMsg, sensor_data_callback)

    # 设置循环频率为 20Hz
    rate = rospy.Rate(20)
    angle1_accumulator = 0.0
    angle2_accumulator = 0.0
    omega = 5 * math.pi / 5
    offset = 180
    amplitude = 30
    theta1 = 0
    theta2 = 0
    timestart = time.time()
    
    while not rospy.is_shutdown():
        try:
            timenow = time.time()
            angle1_accumulator += 10
            angle2_accumulator += 10

            # 计算新的角度值
            theta1 = 180 + 10 * math.sin(math.radians(angle1_accumulator))
            theta2 = 180 + 10 * math.sin(math.radians(angle2_accumulator))
            
            angle1 = float(theta1)
            angle2 = float(theta1)
            angle3 = float(theta2)

            # 创建 MotorAngles 类型的消息，将角度值发布到主题上
            angles_msg = MotorAngle()
            angles_msg.angles = [angle1, angle2, angle3]
            pub.publish(angles_msg)

            timep = timenow - timestart
            # 打印发布的角度值和时间戳
            rospy.loginfo("发布舵机角度值: %f, %f, %f, %f", timep, angle1, angle2, angle3)
        except ValueError as e:
            rospy.loginfo("请输入一个有效的浮点数值: %s", e)

        # 等待一段时间，以达到设定的发布频率
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('controller', anonymous=True)

        # 创建发布器
        pub_zero = rospy.Publisher('set_zero', Bool, queue_size=10)
        pub_receive = rospy.Publisher('start_receiving', Bool, queue_size=10)

        # 设零
        # pub_zero.publish(True)
        print("1")
        rospy.sleep(0.5)  # 等待 0.5 秒
        
        # 启动舵机控制逻辑
        # 开始接收数据
        # pub_receive.publish(True)
        print("2")
        servo_control()


    except rospy.ROSInterruptException:
        pass
    finally:
        # 将数据写入CSV文件
        with open(csv_filename, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['timestamp', 'pub_angle1', 'pub_angle2', 'pub_angle3', 
                                 'Fx1', 'Fy1', 'Fz1', 'Tx1', 'Ty1', 'Tz1'])
            csv_writer.writerows(data_array)

