/*
 * IMU_LSM6DSO.h
 * Copyright (c) 2024, ZHAW
 * All rights reserved.
 */

#ifndef IMU_LSM6DSO_H_
#define IMU_LSM6DSO_H_

#include <mbed.h>

/**
 * IMU class for LSM6DSO inertial measurement unit.
 */
class IMU_LSM6DSO {
public:
    IMU_LSM6DSO(SPI &spi, DigitalOut &cs);
    virtual ~IMU_LSM6DSO();

    void configure(void);

    void update(void);

    float get_AccelerationX();
    float get_AccelerationY();
    float get_AccelerationZ();
    float get_GyroX();
    float get_GyroY();
    float get_GyroZ();


private:
    // Control registers
    static const char CTRL1_XL = 0x10; // Accelerometer control: ODR 6667 Hz, Full Scale ±2g
    static const char CTRL2_G = 0x11;  // Gyroscope control: ODR 6667 Hz, Full Scale 2000 dps
    static const char CTRL3_C = 0x12;  // Disable interrupt generation
    static const char CTRL4_C = 0x13;  // Disable low power mode, disable high pass filter
    static const char CTRL5_C = 0x14;  // Enable gyro in all 3 axes
    static const char CTRL6_C = 0x15;  // No decimation, enable accelerometer in all 3 axes
    static const char CTRL7_G = 0x16;  // High res mode disabled, filter bypassed
    static const char CTRL8_XL = 0x17; // 4-wire SPI interface, LSB at lower address
    static const char CTRL9_XL = 0x18; // Disable gyro sleep mode, disable I2C interface, disable FIFO
    static const char CTRL10_C = 0x19; // Self test disabled

    // Gyroscope output registers
    static const char OUTX_L_G = 0x22; // Gyro X-axis (low byte)
    static const char OUTX_H_G = 0x23; // Gyro X-axis (high byte)
    static const char OUTY_L_G = 0x24; // Gyro Y-axis (low byte)
    static const char OUTY_H_G = 0x25; // Gyro Y-axis (high byte)
    static const char OUTZ_L_G = 0x26; // Gyro Z-axis (low byte)
    static const char OUTZ_H_G = 0x27; // Gyro Z-axis (high byte)

    // Accelerometer output registers
    static const char OUTX_L_A = 0x28; // Acceleration X-axis (low byte)
    static const char OUTX_H_A = 0x29; // Acceleration X-axis (high byte)
    static const char OUTY_L_A = 0x2A; // Acceleration Y-axis (low byte)
    static const char OUTY_H_A = 0x2B; // Acceleration Y-axis (high byte)
    static const char OUTZ_L_A = 0x2C; // Acceleration Z-axis (low byte)
    static const char OUTZ_H_A = 0x2D; // Acceleration Z-axis (high byte)

    float AccelerationX = 0.0;
    float AccelerationY = 0.0;
    float AccelerationZ = 0.0;
    float GyroX = 0.0;
    float GyroY = 0.0;
    float GyroZ = 0.0;
    

    SPI &spi;
    DigitalOut &cs;


    float readAccelerationX();
    float readAccelerationY();
    float readAccelerationZ();
    float readGyroX();
    float readGyroY();
    float readGyroZ();

    void writeRegister(char address, char value);
    char readRegister(char address);
};

#endif /* IMU_LSM6DSO_H_ */
