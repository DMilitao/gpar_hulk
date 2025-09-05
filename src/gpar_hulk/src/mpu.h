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

/**
 * \brief Struct to store data for an axis (x, y, z).
 */
struct data_axis {
    double x;
    double y;
    double z;
};

/**
 * \brief Class to interface with an MPU (Motion Processing Unit) sensor.
 *
 * Provides methods to initialize, read from, and calibrate the accelerometer
 * and gyroscope sensors of an MPU device connected via I2C.
 */
class mpu {
public:
    /**
     * \brief Constructor for the mpu class.
     *
     * Initializes the I2C communication with the MPU device at the specified address.
     * \param address The I2C address of the MPU device. Defaults to 0x68.
     */
    mpu(uint8_t address = 0x68);

    /**
     * \brief Destructor for the mpu class.
     *
     * Closes the connection to the I2C device.
     */
    ~mpu();

    /**
     * \brief Wakes the MPU from sleep mode.
     *
     * Configures the power management register to activate the sensor.
     */
    void wakeUp();

    /**
     * \brief Checks if the communication with the MPU is established.
     *
     * \return True if the connection is successful, False otherwise.
     */
    bool isConnected();

    /**
     * \brief Reads all sensor data (accelerometer and gyroscope).
     *
     * Updates the internal 'accel_' and 'gyro_' variables with new readings,
     * applying the bias correction.
     */
    void read_all();

    /**
     * \brief Gets the latest accelerometer reading.
     *
     * \return A 'data_axis' struct containing the accelerometer data.
     */
    struct data_axis accel() {
        return accel_;
    };

    /**
     * \brief Gets the latest gyroscope reading.
     *
     * \return A 'data_axis' struct containing the gyroscope data.
     */
    struct data_axis gyro() {
        return gyro_;
    };

    /**
     * \brief Sets the bias (offset) value for the accelerometer.
     *
     * \param bias_accel A 'data_axis' struct with the bias values
     * for the x, y, and z axes of the accelerometer.
     */
    void set_bias_accel(data_axis bias_accel) {
        bias_accel_ = bias_accel;
    }

    /**
     * \brief Sets the bias (offset) value for the gyroscope.
     *
     * \param bias_gyro A 'data_axis' struct with the bias values
     * for the x, y, and z axes of the gyroscope.
     */
    void set_bias_gyro(data_axis bias_gyro) {
        bias_gyro_ = bias_gyro;
    }

    /**
     * \brief Calculates and stores the sensor bias.
     *
     * Performs a series of readings while the sensor is at rest to determine
     * the average offset for the accelerometer and gyroscope and stores it internally.
     */
    void fix_bias();

private:
    int file_;                          //!> File descriptor for the I2C device.
    uint8_t address_;                   //!> I2C address of the MPU device.
    data_axis accel_ = {0.0, 0.0, 0.0}; //!> Stores the latest accelerometer data.
    data_axis gyro_ = {0.0, 0.0, 0.0};  //!> Stores the latest gyroscope data.
    data_axis bias_accel_ = {0, 0, 0};  //!> Stores the accelerometer bias (offset).
    data_axis bias_gyro_ = {0, 0, 0};   //!> Stores the gyroscope bias (offset).

    /**
     * \brief Writes an 8-bit value to a specific MPU register.
     *
     * \param reg The address of the register to write to.
     * \param value The 8-bit value to be written.
     */
    void writeRegister(uint8_t reg, uint8_t value);

    /**
     * \brief Reads a sequence of bytes from MPU registers.
     *
     * \param reg The starting register address from which to read.
     * \param buffer Pointer to the buffer where the read data will be stored.
     * \param length The number of bytes to read.
     */
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

    accel_.x = static_cast<double>( ((int16_t)(buffer[0] << 8 | buffer[1])) ) / ACCEL_SENSITIVITY * GRAVITY_CTE - bias_accel_.x;
    accel_.y = static_cast<double>( ((int16_t)(buffer[2] << 8 | buffer[3])) ) / ACCEL_SENSITIVITY * GRAVITY_CTE - bias_accel_.y;
    accel_.z = static_cast<double>( ((int16_t)(buffer[4] << 8 | buffer[5])) ) / ACCEL_SENSITIVITY * GRAVITY_CTE - bias_accel_.z;

    gyro_.x = static_cast<double>( ((int16_t)(buffer[8] << 8 | buffer[9])) ) / GYRO_SENSITIVITY * DEG2RAD - bias_gyro_.x;
    gyro_.y = static_cast<double>( ((int16_t)(buffer[10] << 8 | buffer[11])) ) / GYRO_SENSITIVITY * DEG2RAD - bias_gyro_.y;
    gyro_.z = static_cast<double>( ((int16_t)(buffer[12] << 8 | buffer[13])) ) / GYRO_SENSITIVITY * DEG2RAD - bias_gyro_.z;
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

void mpu::fix_bias(){
    data_axis bias_accel;
	data_axis bias_gyro;
	int number_samples = 100;

	for (int i = 0; i < number_samples; i++) {
		read_all();
		data_axis accel_read = accel();
		data_axis gyro_read = gyro();

		bias_accel.x += accel_read.x;
		bias_accel.y += accel_read.y;
		bias_accel.z += accel_read.z;
		bias_gyro.x += gyro_read.x;
		bias_gyro.y += gyro_read.y;
		bias_gyro.z += gyro_read.z;
	}

	bias_accel.x = bias_accel.x / (float)number_samples;
	bias_accel.y = bias_accel.y / (float)number_samples;
	bias_accel.z = bias_accel.z / (float)number_samples - GRAVITY_CTE;
	bias_gyro.x = bias_gyro.x / (float)number_samples;
	bias_gyro.y = bias_gyro.y / (float)number_samples;
	bias_gyro.z = bias_gyro.z / (float)number_samples;

	set_bias_accel(bias_accel);
	set_bias_gyro(bias_gyro);
}