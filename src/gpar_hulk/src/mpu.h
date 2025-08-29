/**
 * \brief MPU class definition
 */

#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "ros/ros.h"

#include <sstream>

#ifndef GPAR_HULK_INCLUDE_MPU_H
#define GPAR_HULK_INCLUDE_MPU_H

#define PWR_MGMT_1   0x6B
#define WHO_AM_I     0x75
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define TEMP_OUT_H   0x41
#define GYRO_SENSITIVITY 131.0f
#define ACCEL_SENSITIVITY 16384.0f
#define GRAVITY_CTE 9.80665f
#define DEG2RAD 3.141516/180.0

struct data_axis
 {
    double x, y, z;
 };

 class mpu {
 public:
    mpu(uint8_t address = 0x68);
    ~mpu();

    void wakeUp();

    bool isConnected();

    void read_all();

    struct data_axis accel() {
        return accel_;
    };

    struct data_axis gyro() {
        return gyro_;
    };

 private:
    int file_;
    uint8_t address_;

    data_axis accel_ = {0.0, 0.0, 0.0};
    data_axis gyro_ = {0.0, 0.0, 0.0};

    void writeRegister(uint8_t reg, uint8_t value);
    void readRegisters(uint8_t reg, uint8_t* buffer, int length);

};

#endif


mpu::mpu(uint8_t address) : address_(address), file_(-1) {
    const char* bus = "/dev/i2c-1";
    if ((file_ = open(bus, O_RDWR)) < 0) {
        ROS_FATAL("Error opening I2C bus");
        return;
    }

    std::stringstream ss;
    if (ioctl(file_, I2C_SLAVE, address_) < 0) {
        ss << "Error communicating with MPU at address " << std::hex << (int)address_;
        ROS_FATAL(ss.str().c_str());
        close(file_);
        file_ = -1;
    } else {
        ss << "MPU at " << std::hex << (int)address_ << " is ready";
        ROS_INFO(ss.str().c_str());
    }
}

 mpu::~mpu() {
    if (file_ != -1) {
        close(file_);
    }
 }

void mpu::wakeUp() {
    writeRegister(PWR_MGMT_1, 0x00);
}

bool mpu::isConnected() {
    uint8_t who_am_i_val;
    readRegisters(WHO_AM_I, &who_am_i_val, 1);
    return who_am_i_val == address_;
}

void mpu::read_all() {
    uint8_t buffer[14];
    readRegisters(ACCEL_XOUT_H, buffer, 14);

    accel_.x = static_cast<double>( ((int16_t)(buffer[0] << 8 | buffer[1])) ) / ACCEL_SENSITIVITY * GRAVITY_CTE;
    accel_.y = static_cast<double>( ((int16_t)(buffer[2] << 8 | buffer[3])) ) / ACCEL_SENSITIVITY * GRAVITY_CTE;
    accel_.z = static_cast<double>( ((int16_t)(buffer[4] << 8 | buffer[5])) ) / ACCEL_SENSITIVITY * GRAVITY_CTE;

    gyro_.x = static_cast<double>( ((int16_t)(buffer[8] << 8 | buffer[9])) ) / GYRO_SENSITIVITY * DEG2RAD;
    gyro_.y = static_cast<double>( ((int16_t)(buffer[10] << 8 | buffer[11])) ) / GYRO_SENSITIVITY * DEG2RAD;
    gyro_.z = static_cast<double>( ((int16_t)(buffer[12] << 8 | buffer[13])) ) / GYRO_SENSITIVITY * DEG2RAD;
}

 void mpu::writeRegister(uint8_t reg, uint8_t value) {
    if (file_ == -1) return;
    uint8_t buffer[2] = {reg, value};
    if (write(file_, buffer, 2) != 2) {
        std::stringstream ss;
        ss << "Error communicating with MPU at address " << std::hex << (int)address_;
        ROS_ERROR(ss.str().c_str());
    }
}

void mpu::readRegisters(uint8_t reg, uint8_t* buffer, int length) {
    if (file_ == -1) return;

    if (write(file_, &reg, 1) != 1) {
        std::stringstream ss;
        ss << "Error starting to read MPU at address  " << std::hex << (int)address_;
        ROS_ERROR(ss.str().c_str());
    }

    if (read(file_, buffer, length) != length) {
        std::stringstream ss;
        ss << "Error reading MPU data at address " << std::hex << (int)address_;
        ROS_ERROR(ss.str().c_str());
    }
}