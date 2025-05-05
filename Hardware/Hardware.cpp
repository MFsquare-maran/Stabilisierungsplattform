#include "Hardware.h"
#include <cstdint>
#include <cstdio>

Hardware::Hardware(float ts)
    : EN1(PA_6, PA_7, PA_15, GEAR_1, ts, 0),
      EN2(PC_6, PC_7, PC_12, GEAR_2, ts, 0),
      Motor_pwm(),
      Temp_M1(PA_0),
      Temp_M2(PA_1),
      spi(PC_1, PC_2, PB_10),  // MOSI, MISO, SCK
      cs_1(PB_4),
      cs_2(PB_5),
      imu_1(spi, cs_1),
      imu_2(spi, cs_2),
      Akku(PA_4,ts),
      Motor1_EN(PC_11),
      Motor2_EN(PC_10),

      Fan_M1(PC_0),
      Fan_M2(PC_3),

      LED_Red(PC_4),
      LED_Green(PB_13),

      Endstop_X(PB_14),
      Endstop_Y(PB_15),
      Emergency_Stop(PB_1),

     velocity_controller_M1(M1_V_KP,M1_V_KI,M1_V_KD,ts),
     position_controller_M1(M1_P_KP,M1_P_KI,M1_P_KD,ts),
     velocity_controller_M2(M2_V_KP,M2_V_KI,M2_V_KD,ts),
     position_controller_M2(M2_P_KP,M2_P_KI,M2_P_KD,ts),
     U_to_pwm_M1(-24.0,24.0,0.0,100.0),
     U_to_pwm_M2(-24.0,24.0,0.0,100.0)
{
    EN1.reset();
    EN2.reset();

    ThisThread::sleep_for(100ms);
    imu_1.configure();
    ThisThread::sleep_for(100ms);
    imu_2.configure();
    ThisThread::sleep_for(100ms);

    //velocity_controller_M1.setIntegralLimits(-5.0, 5.0);
    //velocity_controller_M2.setIntegralLimits(-5.0, 5.0);
    //position_controller_M1.setIntegralLimits(-5.0, 5.0);
    //position_controller_M2.setIntegralLimits(-5.0, 5.0);
    
    Motor1_EN = 0;
    Motor2_EN = 0;
    LED_Green = 0;
    LED_Red = 0;

    Fan_M1 = 0;
    Fan_M2 = 0;

}

Hardware::~Hardware() {
    // Aufräumarbeiten falls nötig
}

void Hardware::update_1() 
{
     // Sensoren aktualisieren
    EN2.update();
    EN1.update();
    
    imu_1.update();
    imu_2.update();

    // Akkuspannung lesen
    Uin = Akku.get_input_voltage();

}

void Hardware::update_2() 
{
   
    // --- Positionsregelung (Außenregelkreis) ---
    // Positionssollwert ist z.B. position_M1 (definierst du woanders)

    if(control_mode == POSITION)
    {
        float velocity_M1_c = position_controller_M1.compute(position_M1, EN1.get_rad());
        float velocity_M2_c = position_controller_M2.compute(position_M2, EN2.get_rad());


        if(velocity_M1_c >= Max_speed){velocity_M1_c = Max_speed;}
        if(velocity_M2_c >= Max_speed){velocity_M2_c = Max_speed;}
        if(velocity_M1_c <= -Max_speed){velocity_M1_c = -Max_speed;}
        if(velocity_M2_c <= -Max_speed){velocity_M2_c = -Max_speed;}

        Spannung_M1 = velocity_controller_M1.compute(velocity_M1_c, EN1.get_rad_s());
        Spannung_M2 = velocity_controller_M2.compute(velocity_M2_c, EN2.get_rad_s());
    }

    if(control_mode == VELOCITY)
    {

        Spannung_M1 = velocity_controller_M1.compute(velocity_M1, EN1.get_rad_s());
        Spannung_M2 = velocity_controller_M2.compute(velocity_M2, EN2.get_rad_s());

    }


    Motor_pwm.setDutyCycle_Pwm1(U_to_pwm_M1.evaluate(Spannung_M1));
    Motor_pwm.setDutyCycle_Pwm2(U_to_pwm_M2.evaluate(Spannung_M2));
 
}

void Hardware::reset() 
{
    velocity_controller_M1.reset();
    velocity_controller_M2.reset();
    position_controller_M1.reset();
    position_controller_M2.reset();
    EN1.reset(0.0);
    EN2.reset(0.0);
    position_M1 = 0.0;
    position_M2 = 0.0;
    velocity_M1 = 0.0;
    velocity_M2 = 0.0;
    Motor_pwm.setDutyCycle_Pwm1(50.0);
    Motor_pwm.setDutyCycle_Pwm2(50.0);
    Spannung_M1 = 0.0;
    Spannung_M2 = 0.0;
    



    
}


float Hardware::coreXY_to_motor(float x, float y,int Motor) {
   if(Motor == 0)
   {
       return x + y;
   }
   if(Motor == 1)
   {
       return  x - y;
   }

   return 0.0;
}

float Hardware::motor_to_coreXY(float en1, float en2, int axis) 
{
    
    if(axis == 0)
    {
        return (en1 + en2) / 2.0f;
    }
    if(axis == 1)
    {
        return (en1 - en2) / 2.0f;
    }

    return 0.0;
}



void Hardware::set_position(float x, float y,float speed)
{
    control_mode = POSITION;
    Max_speed = 2.0f * speed / D_PULLEY;
    position_M1 = 2.0f * coreXY_to_motor(x,y,0) / D_PULLEY;
    position_M2 = 2.0f * coreXY_to_motor(x,y,1) / D_PULLEY;
    

}

void Hardware::set_velocity(float x, float y)
{
    control_mode = VELOCITY;

    velocity_M1 = 2.0f * coreXY_to_motor(x,y,0) / D_PULLEY;//coreXY_to_motor(x,y,1);
    velocity_M2 = 2.0f * coreXY_to_motor(x,y,1) / D_PULLEY;//coreXY_to_motor(x,y,0);


}

void Hardware::set_reset_position(float x, float y)
{
    
    EN1.reset(2.0f * coreXY_to_motor(x,y,0) / D_PULLEY);
    EN2.reset(2.0f * coreXY_to_motor(x,y,1) / D_PULLEY);
     

}

float Hardware::get_position_mm(int axis)
{
    
    if(axis == 0)
    {
        return motor_to_coreXY(EN1.get_mm(), EN2.get_mm(), axis);
    }
    if(axis == 1)
    {
        return  motor_to_coreXY(EN1.get_mm(), EN2.get_mm(), axis);
    }
     
    return 0.0;
}


//##### Encoder 1
float Hardware::M1_get_rad_s() {
    return EN1.get_rad_s();
}

float Hardware::M1_get_rad() {
    return EN1.get_rad();
}

float Hardware::M1_get_revolutions_s() {
    return EN1.get_revolutions_s();
}

float Hardware::M1_get_revolutions() {
    return EN1.get_revolutions();
}

float Hardware::M1_get_revolutions_min() {
    return EN1.get_revolutions_min();
}

float Hardware::M1_get_mm() {
    return EN1.get_mm();
}

float Hardware::M1_get_mm_s() {
    return EN1.get_mm_s();
}
//##### Motor 1
void Hardware::M1_set_U(float Spannung) {
    
    Spannung_M1 = Spannung;
}

float Hardware::M1_get_temperature() {
    
    return Temp_M1.read_temperature();
}

//##### Encoder 2
float Hardware::M2_get_rad_s() {
    return EN2.get_rad_s();
}

float Hardware::M2_get_rad() {
    return EN2.get_rad();
}

float Hardware::M2_get_revolutions_s() {
    return EN2.get_revolutions_s();
}

float Hardware::M2_get_revolutions() {
    return EN2.get_revolutions();
}

float Hardware::M2_get_revolutions_min() {
    return EN2.get_revolutions_min();
}

float Hardware::M2_get_mm() {
    return EN2.get_mm();
}

float Hardware::M2_get_mm_s() {
    return EN2.get_mm_s();
}

//##### Motor 2
void Hardware::M2_set_U(float Spannung) 
{
    Spannung_M2 = Spannung;
}


float Hardware::M2_get_temperature() {

    return Temp_M2.read_temperature();
}

float Hardware::get_Uin() {

    return Uin;
}



float Hardware::get_IMU1_Ax() {

    return imu_1.get_AccelerationX();
}
float Hardware::get_IMU1_Ay() {

    return imu_1.get_AccelerationY();
}
float Hardware::get_IMU1_Az() {

    return imu_1.get_AccelerationZ();
}

float Hardware::get_IMU1_gyro_x() {

    return imu_1.get_AccelerationX();
}
float Hardware::get_IMU1_gyro_y() {

    return imu_1.get_AccelerationY();
}
float Hardware::get_IMU1_gyro_z() {

    return imu_1.get_AccelerationZ();
}





float Hardware::get_IMU2_Ax() {

    return imu_2.get_AccelerationX();
}
float Hardware::get_IMU2_Ay() {

    return imu_2.get_AccelerationY();
}
float Hardware::get_IMU2_Az() {

    return imu_2.get_AccelerationZ();
}

float Hardware::get_IMU2_gyro_x() {

    return imu_2.get_AccelerationX();
}
float Hardware::get_IMU2_gyro_y() {

    return imu_2.get_AccelerationY();
}
float Hardware::get_IMU2_gyro_z() {

    return imu_2.get_AccelerationZ();
}

