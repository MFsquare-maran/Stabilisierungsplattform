#include "mbed.h"
#include "env.h"
#include "Hardware.h"
#include "LowpassFilter.h"
#include "Statemachine.h"
#include "PrintfUart.h"

// Globale Objekte
Hardware Stabilisierungsplattform(TS);


PrintfUart Print_U;

// Filterobjekte und Variablen
LowpassFilter Filter_IMU1_X(1), Filter_IMU1_Y(1), Filter_IMU1_Z(4);
LowpassFilter Filter_IMU2_X(1), Filter_IMU2_Y(1), Filter_IMU2_Z(1);

LowpassFilter Filter_IMU1_X_f1(1), Filter_IMU1_Y_f1(1), Filter_IMU1_Z_f1(1);
LowpassFilter Filter_IMU2_X_f1(1), Filter_IMU2_Y_f1(1), Filter_IMU2_Z_f1(1);

LowpassFilter Filter_T1(1), Filter_T2(1);


State state = SELFTEST;
Process process = y;

float M1_temp = 0.0;
float M2_temp = 0.0;
bool stabilisierung = false;
uint32_t cnt_stabilisierung_aus = 0;


float X_soll = 0.0;
float Y_soll = 0.0;
 
float ax_IMU1_f = 0.0, ay_IMU1_f = 0.0, az_IMU1_f = 0.0;
float ax_IMU2_f = 0.0, ay_IMU2_f = 0.0, az_IMU2_f = 0.0;
float ax_IMU1_f1 = 0.0, ay_IMU1_f1 = 0.0, az_IMU1_f1 = 0.0;
float ax_IMU2_f1 = 0.0, ay_IMU2_f1 = 0.0, az_IMU2_f1 = 0.0;
float ax_IMU1_f_old = 0.0, ay_IMU1_f_old = 0.0;

Controller controller_ax(44.0f, 0.0f, 0.0f, TS);
Controller controller_ay(4.0f, 0.0f, 0.0f, TS);

// Threads
Thread realtime_thread(200);
Thread print_thread(osPriorityNormal);
Thread statemachine_thread(osPriorityHigh);
Thread temperatur_thread(osPriorityNormal);

// Aufgaben
void temperature_task() {
    while (true) {
        M1_temp = Filter_T1.filter(Stabilisierungsplattform.M1_get_temperature() - 8.685);
        M2_temp = Filter_T2.filter(Stabilisierungsplattform.M2_get_temperature() - 0.727);

        if (M1_temp > TEMP_THRESHOLD_O) {
            Stabilisierungsplattform.Fan_M1 = 1;
        }
        if (M1_temp < TEMP_THRESHOLD_U) {
            Stabilisierungsplattform.Fan_M1 = 0;
        }

        if (M2_temp > TEMP_THRESHOLD_O) {
            Stabilisierungsplattform.Fan_M2 = 1;
        }
        if (M2_temp < TEMP_THRESHOLD_U) {
            Stabilisierungsplattform.Fan_M2 = 0;
        }

        ThisThread::sleep_for(TS_thread_Temp);
    }
}

void realtime_task() {
    while (true) 
    {

        Stabilisierungsplattform.update_1();

        if (state != SLEEPMODE)
        {
            Stabilisierungsplattform.update_2();
        }
            

        ax_IMU1_f1 = Filter_IMU1_X_f1.filter(Stabilisierungsplattform.get_IMU1_Ax() - OFFSET_IMU1_X);
        ay_IMU1_f1 = Filter_IMU1_Y_f1.filter(Stabilisierungsplattform.get_IMU1_Ay() - OFFSET_IMU1_Y);
        az_IMU1_f1 = Filter_IMU1_Z_f1.filter(Stabilisierungsplattform.get_IMU1_Az() - OFFSET_IMU1_Z);

        ax_IMU2_f1 = Filter_IMU2_X_f1.filter(Stabilisierungsplattform.get_IMU2_Ax() - OFFSET_IMU2_X);
        ay_IMU2_f1 = Filter_IMU2_Y_f1.filter(Stabilisierungsplattform.get_IMU2_Ay() - OFFSET_IMU2_Y);
        az_IMU2_f1 = Filter_IMU2_Z_f1.filter(Stabilisierungsplattform.get_IMU2_Az() - OFFSET_IMU2_Z);

        ax_IMU1_f = Filter_IMU1_X.filter(Stabilisierungsplattform.get_IMU1_Ax() - OFFSET_IMU1_X);
        ay_IMU1_f = Filter_IMU1_Y.filter(Stabilisierungsplattform.get_IMU1_Ay() - OFFSET_IMU1_Y);
        az_IMU1_f = Filter_IMU1_Z.filter(Stabilisierungsplattform.get_IMU1_Az() - OFFSET_IMU1_Z);

        ax_IMU2_f = Filter_IMU2_X.filter(Stabilisierungsplattform.get_IMU2_Ax() - OFFSET_IMU2_X);
        ay_IMU2_f = Filter_IMU2_Y.filter(Stabilisierungsplattform.get_IMU2_Ay() - OFFSET_IMU2_Y);
        az_IMU2_f = Filter_IMU2_Z.filter(Stabilisierungsplattform.get_IMU2_Az() - OFFSET_IMU2_Z);

        #ifdef USE_IMU1

      
        if( stabilisierung == true )
        {
            float phi_x = atan(ax_IMU1_f / 9.81);
            float phi_y = atan(ay_IMU1_f / 9.81);

            float pos_x = R_S * sin(phi_x);
            float pos_y = R_S * sin(phi_y);

            if(pos_x >= X_MAX){pos_x = X_MAX;}
            if(pos_x <= X_MIN){pos_x = X_MIN;}
            if(pos_y >= Y_MAX){pos_y = Y_MAX;}
            if(pos_y <= Y_MIN){pos_y = Y_MIN;}


            Stabilisierungsplattform.set_position(-pos_x, -pos_y,1000.0);
        
        }

        #endif
        
        #ifdef USE_IMU2

        if (stabilisierung == true)
        {
            float pos_x = controller_ax.compute(0.0f, ax_IMU2_f);  // Regelt ax → 0
            float pos_y = controller_ay.compute(0.0f, -ay_IMU2_f);  // Regelt ay → 0

            // Begrenzung
            if(pos_x > X_MAX) pos_x = X_MAX;
            if(pos_x < X_MIN) pos_x = X_MIN;
            if(pos_y > Y_MAX) pos_y = Y_MAX;
            if(pos_y < Y_MIN) pos_y = Y_MIN;

            Stabilisierungsplattform.set_position(pos_x, pos_y, 1000.0);
        }

        #endif

        ThisThread::sleep_for(TS_thread);
    }
}

void print_task() {

    while(true) {
        
        #ifdef POS_PRINT

            Print_U.print("X_soll:%f Y_soll:%f X_ist:%f Y_ist:%f\n",X_soll,Y_soll,Stabilisierungsplattform.get_position_mm(0),Stabilisierungsplattform.get_position_mm(1));
        
        #endif

        #ifdef ACCEL_PRINT

            float ax_IMU1 = Filter_IMU1_X_f1.filter(Stabilisierungsplattform.get_IMU1_Ax() - OFFSET_IMU1_X);
            float ay_IMU1 = Filter_IMU1_Y_f1.filter(Stabilisierungsplattform.get_IMU1_Ay() - OFFSET_IMU1_Y);
            float az_IMU1 = Filter_IMU1_Z_f1.filter(Stabilisierungsplattform.get_IMU1_Az() - OFFSET_IMU1_Z);

            float ax_IMU2 = Filter_IMU2_X_f1.filter(Stabilisierungsplattform.get_IMU2_Ax() - OFFSET_IMU2_X);
            float ay_IMU2 = Filter_IMU2_Y_f1.filter(Stabilisierungsplattform.get_IMU2_Ay() - OFFSET_IMU2_Y);
            float az_IMU2 = Filter_IMU2_Z_f1.filter(Stabilisierungsplattform.get_IMU2_Az() - OFFSET_IMU2_Z);

            Print_U.print("ax1:%f ay1:%f az1:%f ax2:%f ay2:%f az2:%f \n",ax_IMU1, ay_IMU1, az_IMU1, ax_IMU2, -ay_IMU2, -az_IMU2);
             
        #endif
        ThisThread::sleep_for(5ms);
    }

}

int main() {
    Print_U.init();

    Filter_IMU1_X.setPeriod(TS);   Filter_IMU1_X.setFrequency(Filter_IMU);
    Filter_IMU1_Y.setPeriod(TS);   Filter_IMU1_Y.setFrequency(Filter_IMU);
    Filter_IMU1_Z.setPeriod(TS);   Filter_IMU1_Z.setFrequency(Filter_IMU);
    Filter_IMU2_X.setPeriod(TS);   Filter_IMU2_X.setFrequency(Filter_IMU);
    Filter_IMU2_Y.setPeriod(TS);   Filter_IMU2_Y.setFrequency(Filter_IMU);
    Filter_IMU2_Z.setPeriod(TS);   Filter_IMU2_Z.setFrequency(Filter_IMU);

    Filter_IMU1_X_f1.setPeriod(TS);  Filter_IMU1_X_f1.setFrequency(FILTER_1);
    Filter_IMU1_Y_f1.setPeriod(TS);  Filter_IMU1_Y_f1.setFrequency(FILTER_1);
    Filter_IMU1_Z_f1.setPeriod(TS);  Filter_IMU1_Z_f1.setFrequency(FILTER_1);
    Filter_IMU2_X_f1.setPeriod(TS);  Filter_IMU2_X_f1.setFrequency(FILTER_1);
    Filter_IMU2_Y_f1.setPeriod(TS);  Filter_IMU2_Y_f1.setFrequency(FILTER_1);
    Filter_IMU2_Z_f1.setPeriod(TS);  Filter_IMU2_Z_f1.setFrequency(FILTER_1);

    Filter_T1.setPeriod(TS_Temp); Filter_T1.setFrequency(1.0);
    Filter_T2.setPeriod(TS_Temp); Filter_T2.setFrequency(1.0);

    print_thread.start(print_task);
    temperatur_thread.start(temperature_task);
    realtime_thread.start(realtime_task);
    statemachine_thread.start(StateMachine_task);

    while (true) {
        ThisThread::sleep_for(200000ms);
    }
}
