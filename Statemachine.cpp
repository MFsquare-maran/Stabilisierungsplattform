// StateMachine.cpp
#include "Statemachine.h"
#include "env.h"
#include "LowpassFilter.h"
#include "PrintfUart.h"

extern Hardware Stabilisierungsplattform;
extern LowpassFilter Filter_IMU1_X_Plot, Filter_IMU1_Y_Plot;
extern float ax_IMU1_f1, ay_IMU1_f1, ax_IMU1_f_old, ay_IMU1_f_old;
extern float M1_temp, M2_temp;
extern bool stabilisierung;
extern uint32_t cnt_stabilisierung_aus;
extern PrintfUart Print_U;

extern float X_soll;
extern float Y_soll;


void Home_sequenz() {
    switch (process) {
        case y:
            Stabilisierungsplattform.set_velocity(0.0, INIT_SPEED);
            if (Stabilisierungsplattform.Endstop_Y == 0) {
                ThisThread::sleep_for(100ms);
                Stabilisierungsplattform.reset();
                ThisThread::sleep_for(100ms);
                Stabilisierungsplattform.set_position(0.0, 0.4, INIT_SPEED);
                ThisThread::sleep_for(1000ms);
                ThisThread::sleep_for(100ms);
                Stabilisierungsplattform.reset();
                ThisThread::sleep_for(100ms);
                process = x;
            }
            break;

        case x:
            Stabilisierungsplattform.set_velocity(-INIT_SPEED, 0.0);
            if (Stabilisierungsplattform.Endstop_X == 0) {
                process = home;
                Stabilisierungsplattform.reset();
                ThisThread::sleep_for(200ms);
            }
            break;

        case home:
            Stabilisierungsplattform.set_position(POS_ZERO_X, POS_ZERO_Y, INIT_SPEED * 10.0f);
            break;
    }
}

void StateMachine_task() {
    while (true) {
        if (Stabilisierungsplattform.Emergency_Stop == 0) {
            state = ERROR_MODE;
            stabilisierung = false;
        }
        if (M1_temp > TEMP_THRESHOLD_EMERGENCY || M2_temp > TEMP_THRESHOLD_EMERGENCY) {
            state = ERROR_MODE;
            stabilisierung = false;
        }

        switch (state) {
            case SELFTEST:

                if (abs(Stabilisierungsplattform.get_IMU1_Ax()) >= ACCELERATION_THRESHOLD_XY) {
                    state = ERROR_MODE;
                    Print_U.print("Malfunction of Ax in IMU\n");
                    ThisThread::sleep_for(2000ms);
                    break;
                }
                if (abs(Stabilisierungsplattform.get_IMU1_Ay()) >= ACCELERATION_THRESHOLD_XY) {
                    state = ERROR_MODE;
                    Print_U.print("Malfunction of Ay in IMU\n");
                    ThisThread::sleep_for(2000ms);
                    break;
                }
                if (abs(Stabilisierungsplattform.get_IMU1_Az()) >= ACCELERATION_THRESHOLD_Z) {
                    state = ERROR_MODE;
                    Print_U.print("Malfunction of Az in IMU\n");
                    ThisThread::sleep_for(2000ms);
                }
                if (Stabilisierungsplattform.get_Uin() <= MIN_VOLTAGE) {
                    state = ERROR_MODE;
                    Print_U.print("Min Voltage of battery reached!\n ");
                    ThisThread::sleep_for(2000ms);
                    break;
                }

                if (Stabilisierungsplattform.M1_get_temperature() >= TEMP_THRESHOLD_EMERGENCY) {
                    state = ERROR_MODE;
                    Print_U.print("MAX Temp of M2 reached!\n");
                    ThisThread::sleep_for(2000ms);
                    break;
                }
                if (Stabilisierungsplattform.M2_get_temperature() >= TEMP_THRESHOLD_EMERGENCY) {
                    state = ERROR_MODE;
                    Print_U.print("MAX Temp of M2 reached!\n");
                    ThisThread::sleep_for(2000ms);
                    break;
                }
                Stabilisierungsplattform.LED_Red  = 1;
                Stabilisierungsplattform.LED_Green = 1;
                state = CALIBRATION;
                process = y;
                break;

            case CALIBRATION:
                Stabilisierungsplattform.Motor1_EN = 1;
                Stabilisierungsplattform.Motor2_EN = 1;

                Home_sequenz();

                if (process == home &&
                    abs(Stabilisierungsplattform.get_position_mm(0) - POS_ZERO_X) <= 0.5f &&
                    abs(Stabilisierungsplattform.get_position_mm(1) - POS_ZERO_Y) <= 0.5f) {
                    state = RUN_MODE;
                    ThisThread::sleep_for(2000ms);
                    Stabilisierungsplattform.reset();
                    Stabilisierungsplattform.set_position(0.0, 0.0, 1000.0);
                }
                break;

            case RUN_MODE:


                Stabilisierungsplattform.LED_Green = 1;
                Stabilisierungsplattform.LED_Red = 0;

                Stabilisierungsplattform.Motor1_EN = 1;
                Stabilisierungsplattform.Motor2_EN = 1;

                
                #ifdef STEP

                    X_soll = -40.0;
                    Y_soll = 0.0;

                    Stabilisierungsplattform.set_position(X_soll, Y_soll,1000.0);


                    ThisThread::sleep_for(2000ms);
                    
                    X_soll = 40.0;
                    Y_soll = 0.0;

                    Stabilisierungsplattform.set_position(X_soll, Y_soll,1000.0);

                    ThisThread::sleep_for(2000ms);

                #endif
                #ifdef STABILISIERUNG
      
                    
                    stabilisierung = true;

                    if (abs(ax_IMU1_f1 - ax_IMU1_f_old) <= 0.005 && abs(ay_IMU1_f1 - ay_IMU1_f_old) <= 0.005) {
                        cnt_stabilisierung_aus++;
                        if (cnt_stabilisierung_aus >= CNT_SLEEP_MODE) {
                            state = SLEEPMODE;
                            cnt_stabilisierung_aus = 0;
                        }
                    } else {
                        cnt_stabilisierung_aus = 0;
                    }

                    ax_IMU1_f_old = ax_IMU1_f1;
                    ay_IMU1_f_old = ay_IMU1_f1;

                #endif

                break;

            case SLEEPMODE:
                stabilisierung = false;
                Stabilisierungsplattform.LED_Green = 0;
                Stabilisierungsplattform.LED_Red = 0;
                Stabilisierungsplattform.Motor1_EN = 0;
                Stabilisierungsplattform.Motor2_EN = 0;

                if (abs(ax_IMU1_f1 - ax_IMU1_f_old) >= 0.005 || abs(ay_IMU1_f1 - ay_IMU1_f_old) >= 0.005) {
                    state = RUN_MODE;
                }

                ax_IMU1_f_old = ax_IMU1_f1;
                ay_IMU1_f_old = ay_IMU1_f1;
                break;

            case ERROR_MODE:
                Stabilisierungsplattform.LED_Red = 1;
                Stabilisierungsplattform.LED_Green = 0;
                Stabilisierungsplattform.Motor1_EN = 0;
                Stabilisierungsplattform.Motor2_EN = 0;
                Stabilisierungsplattform.reset();

                if (Stabilisierungsplattform.Emergency_Stop == 1) {
                    state = SELFTEST;
                }
                break;
        }

        ThisThread::sleep_for(10ms);
    }
}
