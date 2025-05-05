#ifndef HARDWARE_H
#define HARDWARE_H

#include "mbed.h"
#include <cstdint>
#include "EncoderCounter.h"
#include "LM35DZ.h"
#include "IMU_LSM6DSO.h"
#include "PWMGenerator.h"
#include "U_Battery.h"
#include "controller.h"
#include "LinearCharacteristics.h"





class Hardware {
public:
    Hardware(float ts = TS);
    ~Hardware();

    void update_1();
    void update_2();

    void reset();

    //##### Encoder 1
    float M1_get_rad_s();
    float M1_get_rad();
    float M1_get_revolutions_s();
    float M1_get_revolutions();
    float M1_get_revolutions_min();
    float M1_get_mm();
    float M1_get_mm_s();

    //##### Motor 1
    void M1_set_U(float Spannung);

    float M1_get_temperature();

    //##### Encoder 2
    float M2_get_rad_s();
    float M2_get_rad();
    float M2_get_revolutions_s();
    float M2_get_revolutions();
    float M2_get_revolutions_min();
    float M2_get_mm();
    float M2_get_mm_s();
    
    //##### Motor 2
    void M2_set_U(float Spannung);


    float M2_get_temperature();

    float get_Uin();


    float get_IMU1_Ax();
    float get_IMU1_Ay();
    float get_IMU1_Az();

    float get_IMU1_gyro_x();
    float get_IMU1_gyro_y();
    float get_IMU1_gyro_z();

    float get_IMU2_Ax();
    float get_IMU2_Ay();
    float get_IMU2_Az();

    float get_IMU2_gyro_x();
    float get_IMU2_gyro_y();
    float get_IMU2_gyro_z();

    void set_position(float x, float y, float speed = 100.0);
    
    void set_velocity(float x, float y);

    void set_reset_position(float x, float y);

    float get_position_mm(int axis);



    DigitalOut Motor1_EN;
    DigitalOut Motor2_EN;
    
    DigitalOut LED_Red;
    DigitalOut LED_Green;

    DigitalOut Fan_M1;
    DigitalOut Fan_M2;

    DigitalIn Endstop_X;
    DigitalIn Endstop_Y;

    DigitalIn Emergency_Stop;

    enum Control_mode {
        VELOCITY,POSITION

    }; Control_mode control_mode;

    PWMGenerator Motor_pwm;

    float Spannung_M1 = 0.0;
    float Spannung_M2 = 0.0;




private:
    EncoderCounter EN1;
    EncoderCounter EN2;

    LinearCharacteristics U_to_pwm_M1;
    LinearCharacteristics U_to_pwm_M2;
    
    Controller velocity_controller_M1;
    Controller position_controller_M1;
    Controller velocity_controller_M2;
    Controller position_controller_M2;


    float velocity_M1 = 0.0; //mm_s
    float velocity_M2 = 0.0; //mm_s


    float position_X = 0.0; //mm
    float position_y= 0.0; //mm

    float position_M1 = 0.0; //mm
    float position_M2= 0.0; //mm

    float Max_speed = 0.0;



    bool pos_or_velocity_Control = true;
    

    
    struct CoreXYPosition 
    {
        float motor_1; 
        float motor_2; 
    };  

    float coreXY_to_motor(float x, float y,int Motor);
    float motor_to_coreXY(float en1, float en2, int axis);
    
    


    LM35DZ Temp_M1;
    LM35DZ Temp_M2;

    SPI spi;
    DigitalOut cs_1;
    DigitalOut cs_2;
    IMU_LSM6DSO imu_1;
    IMU_LSM6DSO imu_2;

    U_Battery Akku;

    float Uin = 0.0;
};

#endif // HARDWARE_H
