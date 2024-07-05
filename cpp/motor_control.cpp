#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <sfdt/MotorAngle.h>
#include <sfdt/Motorfeedback.h>

class ServoMotor {
private:
    serial::Serial ser;
    const uint8_t DeepSea_SERVO_FRAME_HEADER = 0xff;
    const uint8_t DeepSea_SERVO_POS_TIME_WRITE = 3;
    const uint8_t DeepSea_SERVO_POS_TIME_WRITE_MemAddr = 42;
    const uint8_t DeepSea_SERVO_POS_Instruction = 0x83;
    const uint8_t DeepSea_SERVO_READ = 2;
    const uint8_t DeepSea_SERVO_READ_POS_MemAddr = 56;
    const uint8_t DeepSea_SERVO_READ_TIME_MemAddr = 58;
    const uint8_t DeepSea_SERVO_ID_MemAddr = 5;
    const uint8_t DeepSea_SERVO_MIN_ANGLE_LIMIT = 9;
    const uint8_t DeepSea_SERVO_MAX_ANGLE_LIMIT = 11;  //0x10 4096
    const uint8_t DeepSea_SERVO_MAX_TORQUE_LIMIT = 16; //0x03 800
    const uint8_t One_date_length = 4;
    const uint8_t Total_data_length = 19;  //(one_date_length+1)*number_of_motor+4

    uint8_t GET_LOW_BYTE(int A) {
        return static_cast<uint8_t>(A);
    }

    uint8_t GET_HIGH_BYTE(int A) {
        return static_cast<uint8_t>(A >> 8);
    }

    uint8_t DeepSeaCheckSum(uint8_t *buf, int length) {
        uint8_t temp = 0;
        for (int i = 2; i < length - 1; ++i) {
            temp += buf[i];
        }
        return ~temp;
    }

    void DeepSeaSerialServoMove(int id, int position, int time) {
        uint8_t buf[11];
        buf[0] = buf[1] = DeepSea_SERVO_FRAME_HEADER;
        buf[2] = static_cast<uint8_t>(id);
        buf[3] = 7;
        buf[4] = DeepSea_SERVO_POS_TIME_WRITE;
        buf[5] = DeepSea_SERVO_POS_TIME_WRITE_MemAddr;
        buf[6] = GET_HIGH_BYTE(position);
        buf[7] = GET_LOW_BYTE(position);
        buf[8] = GET_HIGH_BYTE(time);
        buf[9] = GET_LOW_BYTE(time);
        buf[10] = DeepSeaCheckSum(buf, 11);

        ser.write(buf, sizeof(buf));
    }

    void DeepSeaReceiveHandle(uint8_t *data, int length) {
        ser.read(data, length);
    }

    int BYTE_TO_HW(uint8_t low_byte, uint8_t high_byte) {
        return (static_cast<int>(high_byte) << 8) | static_cast<int>(low_byte);
    }

    void DeepSeaSerialServoMoveall(serial::Serial& ser, const std::vector<uint8_t>& ids, const std::vector<int>& positions, int time) {
        uint8_t buf[23];
        buf[0] = buf[1] = DeepSea_SERVO_FRAME_HEADER;
        buf[2] = 0xfe;
        buf[3] = Total_data_length;
        buf[4] = DeepSea_SERVO_POS_Instruction;
        buf[5] = DeepSea_SERVO_POS_TIME_WRITE_MemAddr;
        buf[6] = One_date_length;

        buf[7] = ids[0];
        buf[8] = GET_HIGH_BYTE(positions[0]);
        buf[9] = GET_LOW_BYTE(positions[0]);
        buf[10] = GET_HIGH_BYTE(time);
        buf[11] = GET_LOW_BYTE(time);

        buf[12] = ids[1];
        buf[13] = GET_HIGH_BYTE(positions[1]);
        buf[14] = GET_LOW_BYTE(positions[1]);
        buf[15] = GET_HIGH_BYTE(time);
        buf[16] = GET_LOW_BYTE(time);

        buf[17] = ids[2];
        buf[18] = GET_HIGH_BYTE(positions[2]);
        buf[19] = GET_LOW_BYTE(positions[2]);
        buf[20] = GET_HIGH_BYTE(time);
        buf[21] = GET_LOW_BYTE(time);

        buf[22] = DeepSeaCheckSum(buf, 23);
        ser.write(buf, sizeof(buf));
    }

public:
    ServoMotor(const std::string& port, int baud_rate = 115200) {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);

        try {
            ser.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port: " << e.what());
            ros::shutdown();
        }

        if (ser.isOpen()) {
            ROS_INFO_STREAM("Serial port initialized");
        } else {
            ROS_ERROR_STREAM("Failed to initialize serial port");
        }
    }

    ~ServoMotor() {
        if (ser.isOpen()) {
            ser.close();
        }
    }

    void moveServo(int id, float angle, int speed) {
        int position = static_cast<int>(angle * 4096 / 360.0);
        DeepSeaSerialServoMove(id, position, speed);
    }

    void moveServos(const std::vector<int>& ids, const std::vector<float>& angles, int speed) {
        std::vector<int> positions(ids.size());
        for (size_t i = 0; i < ids.size(); ++i) {
            positions[i] = static_cast<int>(angles[i] * 4096 / 360.0);
        }
        std::vector<uint8_t> id_bytes(ids.begin(), ids.end());
        DeepSeaSerialServoMoveall(ser, id_bytes, positions, speed);
    }

    float DeepSeaServoReadPosition(int id) {
        uint8_t bufff[8] = {0};
        uint8_t data_angle[2] = {0};

        // 设置通信协议的帧头、伺服ID、长度等信息
        bufff[0] = bufff[1] = DeepSea_SERVO_FRAME_HEADER;
        bufff[2] = id;
        bufff[3] = 4;  // 数据长度
        bufff[4] = DeepSea_SERVO_READ;  // 读取指令
        bufff[5] = 0x38;  // 要读取的地址
        bufff[6] = 0x02;  // 要读取的数据长度

        // 计算并设置校验和
        bufff[7] = DeepSeaCheckSum(bufff, 8);

        // 发送指令包
        ser.write(bufff, sizeof(bufff));
        usleep(1000);  // 休眠100微秒

        // 接收并处理返回的数据
        DeepSeaReceiveHandle(data_angle, sizeof(data_angle));
        int ang = BYTE_TO_HW(data_angle[0], data_angle[1]);
        float ang1 = ang * 360.0 / 4096.0;
        return ang1;
    }

    void angleCallbackall(const sfdt::MotorAngle::ConstPtr& msg) {
        std::vector<int> ids = {7, 8, 9};  // 假设电机ID是1, 2, 3
        std::vector<float> angles = msg->angles;
        std::vector<float> mid_values = {180, 180, 180};
        bool valid = true;
        for (int j = 0; j < 3; ++j) {
            float mid = mid_values[j];
            if (angles[j] < mid - 30 || angles[j] > mid + 30) {
                valid = false;
                ROS_WARN("Received angle %f for motor %d out of range [%f, %f]", angles[j], ids[j], mid - 30, mid + 30);
                break;
            }
        }
        if (valid) {
            moveServos(ids, angles, 0);
        } else {
            ROS_WARN("Skipped invalid angles for motor group %d", 1);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_servo_controller");
    ros::NodeHandle nh;

    // Initialize servo motor with the correct USB port
    ServoMotor servo("/dev/ttyUSB0");

    ros::Publisher servofeedback_pub = nh.advertise<sfdt::Motorfeedback>("Motorfeedback", 10);

    // Set up subscribers for each motor
    ros::Subscriber sub = nh.subscribe<sfdt::MotorAngle>("motor_control_all", 10, &ServoMotor::angleCallbackall, &servo);
    ros::spin();
    // ros::Rate loop_rate(20); // 设置循环频率为10Hz
    // while (ros::ok()) {
    //     sfdt::Motorfeedback feedback_msg;
    //     feedback_msg.angle1 = servo.DeepSeaServoReadPosition(7);
    //     feedback_msg.angle2 = servo.DeepSeaServoReadPosition(8);
    //     feedback_msg.angle3 = servo.DeepSeaServoReadPosition(9);
    //     servofeedback_pub.publish(feedback_msg);

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}

