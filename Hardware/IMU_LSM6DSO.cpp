#include "IMU_LSM6DSO.h"

IMU_LSM6DSO::IMU_LSM6DSO(SPI &spi, DigitalOut &cs) : spi(spi), cs(cs) {
    cs = 1;
    
    // Initialisierung des Sensors
    writeRegister(CTRL1_XL, 0x60); // 416 Hz, ±2g
    writeRegister(CTRL2_G, 0x60);  // 416 Hz, ±250 dps
}

IMU_LSM6DSO::~IMU_LSM6DSO() {}

void IMU_LSM6DSO::update()
{
    this->AccelerationX = readAccelerationX();
    this->AccelerationY = readAccelerationY();
    this->AccelerationZ = readAccelerationZ();
    this->GyroX = readGyroX();
    this->GyroY = readGyroY();
    this->GyroZ = readGyroZ();
}

float IMU_LSM6DSO::get_AccelerationX()
{
    return this->AccelerationX;
}
float IMU_LSM6DSO::get_AccelerationY()
{
    return this->AccelerationY;
}
float IMU_LSM6DSO::get_AccelerationZ()
{
    return this->AccelerationZ;
}
float IMU_LSM6DSO::get_GyroX()
{
    return this-> GyroX;
}
float IMU_LSM6DSO::get_GyroY()
{
    return this->GyroY;
}
float IMU_LSM6DSO::get_GyroZ()
{
    return this->GyroZ;
}

void IMU_LSM6DSO::writeRegister(char address, char value) {
    cs = 0;
    spi.write(address);
    spi.write(value);
    cs = 1;
}

char IMU_LSM6DSO::readRegister(char address) {
    cs = 0;
    spi.write(0x80 | address);
    char value = spi.write(0x00);
    cs = 1;
    return value;
}


void IMU_LSM6DSO::configure(void)
{
    // Initialisierung des Sensors
    writeRegister(CTRL1_XL, 0x60); // 416 Hz, ±2g
    writeRegister(CTRL2_G, 0x60);  // 416 Hz, ±250 dps

}

float IMU_LSM6DSO::readAccelerationX() {
    char low = readRegister(OUTX_L_A);
    char high = readRegister(OUTX_H_A);
    short value = (high << 8) | low;
    return (float)value / 32768.0f * 2.0f * 9.81f;
}

float IMU_LSM6DSO::readAccelerationY() {
    char low = readRegister(OUTY_L_A);
    char high = readRegister(OUTY_H_A);
    short value = (high << 8) | low;
    return (float)value / 32768.0f * 2.0f * 9.81f;
}

float IMU_LSM6DSO::readAccelerationZ() {
    char low = readRegister(OUTZ_L_A);
    char high = readRegister(OUTZ_H_A);
    short value = (high << 8) | low;
    return (float)value / 32768.0f * 2.0f * 9.81f;
}

float IMU_LSM6DSO::readGyroX() {
    char low = readRegister(OUTX_L_G);
    char high = readRegister(OUTX_H_G);
    short value = (high << 8) | low;
    return (float)value / 32768.0f * 250.0f * 3.14159265f / 180.0f;
}

float IMU_LSM6DSO::readGyroY() {
    char low = readRegister(OUTY_L_G);
    char high = readRegister(OUTY_H_G);
    short value = (high << 8) | low;
    return (float)value / 32768.0f * 250.0f * 3.14159265f / 180.0f;
}

float IMU_LSM6DSO::readGyroZ() {
    char low = readRegister(OUTZ_L_G);
    char high = readRegister(OUTZ_H_G);
    short value = (high << 8) | low;
    return (float)value / 32768.0f * 250.0f * 3.14159265f / 180.0f;
}
