#include <ros/ros.h>
#include <serial/serial.h>
#include <cmath>
#include <iostream>

#define DeepSea_SERVO_FRAME_HEADER 0xff
#define DeepSea_SERVO_POS_TIME_WRITE 3
#define DeepSea_SERVO_POS_TIME_WRITE_MemAddr 42
#define DeepSea_SERVO_READ 2
#define DeepSea_SERVO_READ_POS_MemAddr 56

serial::Serial ser;

uint8_t GET_LOW_BYTE(int A) {
    return A & 0xFF;
}

uint8_t GET_HIGH_BYTE(int A) {
    return (A >> 8) & 0xFF;
}

uint16_t BYTE_TO_HW(uint8_t A, uint8_t B) {
    return ((A << 8) | B) & 0xFFFF;
}

uint8_t DeepSeaCheckSum(uint8_t *buf, int length) {
    uint8_t temp = 0;
    for (int i = 2; i < length - 1; i++) {
        temp += buf[i];
    }
    temp = ~temp;
    return temp & 0xFF;
}

void DeepSeaSerialServoMove(int id, int position, int time) {
    uint8_t buf[11];
    buf[0] = buf[1] = DeepSea_SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = 7;
    buf[4] = DeepSea_SERVO_POS_TIME_WRITE;
    buf[5] = DeepSea_SERVO_POS_TIME_WRITE_MemAddr;
    buf[6] = GET_HIGH_BYTE(position);
    buf[7] = GET_LOW_BYTE(position);
    buf[8] = GET_HIGH_BYTE(time);
    buf[9] = GET_LOW_BYTE(time);
    buf[10] = DeepSeaCheckSum(buf, 11);
    ser.write(buf, 11);
}

float DeepSeaServoReadPosition(int id) {
    uint8_t buf[8];
    uint8_t data[2];

    buf[0] = buf[1] = DeepSea_SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = 4;
    buf[4] = DeepSea_SERVO_READ;
    buf[5] = DeepSea_SERVO_READ_POS_MemAddr;
    buf[6] = 2;
    buf[7] = DeepSeaCheckSum(buf, 8);

    ser.write(buf, 8);
    size_t bytesRead = ser.read(data, 2);
    if (bytesRead < 2) {
        // TODO: Handle error
        return -1;
    }

    int ang = BYTE_TO_HW(data[0], data[1]);
    float ang1 = ang * 360.0 / 4096.0;
    return ang1;
}

void ServoMove(int ID, float angle, int speed) {
    int dpos = int(angle * 4096.0 / 360.0);
    DeepSeaSerialServoMove(ID, dpos, speed);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_controller");
    ros::NodeHandle nh;

    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Successfully opened port.");
    } else {
        return -1;
    }

    float simplefrequency = 100;
    float offset = 90;
    float f_n = 1.0 / 10.0;
    float A_n = 2.0 / 1000.0;
    float st = A_n * f_n / 1.0;
    st = 0;
    ros::Time time_start = ros::Time::now();
    
    ros::Rate loop_rate(simplefrequency);

    while (ros::ok()) {
        ros::Duration time_gap = ros::Time::now() - time_start;
        float t = time_gap.toSec();
        float theta_half = std::asin(A_n / 2.0);
        float amplitude = theta_half * 360.0 / (2.0 * M_PI);
        float omega = 2.0 * M_PI * f_n;
        float theta1 = round(offset + 60.0 * std::sin(omega * t));
        ServoMove(1, theta1, 10);
        // float angle1 = DeepSeaServoReadPosition(1);
        // ROS_INFO_STREAM("Theta1: " << theta1 << ", Angle1: " << angle1);
        ROS_INFO_STREAM("Theta1: " << theta1 << ", Angle1: " );
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}