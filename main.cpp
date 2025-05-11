#include "mbed.h"
#include "env.h"
#include "Hardware.h"
#include <cmath> // für fabsf()
#include <cstdint>
#include "LowpassFilter.h"

BufferedSerial pc(USBTX, USBRX, 115200);  // USB-Serielle Schnittstelle


Hardware Stabilisierungsplattform(TS);


LowpassFilter Filter_IMU1_X;
LowpassFilter Filter_IMU1_Y;
LowpassFilter Filter_IMU1_Z;
LowpassFilter Filter_IMU2_X;
LowpassFilter Filter_IMU2_Y;
LowpassFilter Filter_IMU2_Z;


LowpassFilter Filter_IMU1_X_Plot;
LowpassFilter Filter_IMU1_Y_Plot;
LowpassFilter Filter_IMU1_Z_Plot;
LowpassFilter Filter_IMU2_X_Plot;
LowpassFilter Filter_IMU2_Y_Plot;
LowpassFilter Filter_IMU2_Z_Plot;


LowpassFilter Filter_T1;
LowpassFilter Filter_T2;




enum State {
    SELFTEST,
    CALIBRATION,
    SLEEPMODE,
    RUN_MODE,
    ERROR_MODE
}; 

State state;

enum Process {
    y,
    x,
    home
};

Process process;

// Thread-Objekte
Thread realtime_thread(200);
Thread print_thread(osPriorityNormal);
Thread statemachine_thread(osPriorityHigh);
Thread temperatur_thread(osPriorityNormal);


bool stabilisierung = false;

float M1_temp = 0.0;
float M2_temp = 0.0;




float ax_IMU1_f = 0.0;
float ay_IMU1_f = 0.0;
float az_IMU1_f = 0.0;

float ax_IMU2_f = 0.0;
float ay_IMU2_f = 0.0;
float az_IMU2_f = 0.0;

float ax_IMU1_f1 = 0.0;
float ay_IMU1_f1 = 0.0;
float az_IMU1_f1 = 0.0;

float ax_IMU2_f1 = 0.0;
float ay_IMU2_f1 = 0.0;
float az_IMU2_f1 = 0.0;

float ax_IMU1_f_old = 0.0;
float ay_IMU1_f_old = 0.0;

uint32_t cnt_stabilisierung_aus = 0;



float X_soll = 0.0;
float Y_soll = 0.0;




DigitalIn userButton(PC_13); // interner PullUp meist aktiv

bool detectFallingEdge()
{
    static bool lastState = true; // Taster ist nicht gedrückt bei Start (High)
    bool currentState = userButton.read();

    bool fallingEdge = (lastState == true && currentState == false);
    lastState = currentState;
    return fallingEdge;
}



void temperature_task()
{
    while(true)
    {

        M1_temp =  Filter_T1.filter(Stabilisierungsplattform.M1_get_temperature()-8.685); 

        M2_temp =  Filter_T2.filter(Stabilisierungsplattform.M2_get_temperature()-0.727);

        

        if(M1_temp > TEMP_THRESHOLD_O)
        {
            Stabilisierungsplattform.Fan_M1 = 1;
            
        }

        if(M1_temp < TEMP_THRESHOLD_U)
        {
            Stabilisierungsplattform.Fan_M1 = 0;
            
        }

        if(M2_temp > TEMP_THRESHOLD_O)
        {
            Stabilisierungsplattform.Fan_M2 = 1;
            
        }

        if(M2_temp < TEMP_THRESHOLD_U)
        {
            Stabilisierungsplattform.Fan_M2 = 0;
            
        }

    
        /*
        if (detectFallingEdge()) 
        {

            Stabilisierungsplattform.Fan_M1 = !Stabilisierungsplattform.Fan_M1;
            Stabilisierungsplattform.Fan_M2 = !Stabilisierungsplattform.Fan_M2;

        }
*/

        ThisThread::sleep_for(TS_thread_Temp);

    }

    
    
}



void realtime_task() {
    while (true) {




        Stabilisierungsplattform.update_1();

        if(state != SLEEPMODE)
        {
            Stabilisierungsplattform.update_2();
        }


        ax_IMU1_f1 = Filter_IMU1_X_Plot.filter(Stabilisierungsplattform.get_IMU1_Ax()-OFFSET_IMU1_X);
        ay_IMU1_f1 = Filter_IMU1_Y_Plot.filter(Stabilisierungsplattform.get_IMU1_Ay()-OFFSET_IMU1_Y);
        az_IMU1_f1 = Filter_IMU1_Z_Plot.filter(Stabilisierungsplattform.get_IMU1_Az()-OFFSET_IMU1_Z);

        ax_IMU2_f1 = Filter_IMU2_X_Plot.filter(Stabilisierungsplattform.get_IMU2_Ax()-OFFSET_IMU2_X);
        ay_IMU2_f1 = Filter_IMU2_Y_Plot.filter(Stabilisierungsplattform.get_IMU2_Ay()-OFFSET_IMU2_Y);
        az_IMU2_f1 = Filter_IMU2_Z_Plot.filter(Stabilisierungsplattform.get_IMU2_Az()-OFFSET_IMU2_Z);

        ax_IMU1_f = Filter_IMU1_X.filter(Stabilisierungsplattform.get_IMU1_Ax()-OFFSET_IMU1_X);
        ay_IMU1_f = Filter_IMU1_Y.filter(Stabilisierungsplattform.get_IMU1_Ay()-OFFSET_IMU1_Y);
        az_IMU1_f = Filter_IMU1_Z.filter(Stabilisierungsplattform.get_IMU1_Az()-OFFSET_IMU1_Z);

        ax_IMU2_f = Filter_IMU2_X.filter(Stabilisierungsplattform.get_IMU2_Ax()-OFFSET_IMU2_X);
        ay_IMU2_f = Filter_IMU2_Y.filter(Stabilisierungsplattform.get_IMU2_Ay()-OFFSET_IMU2_Y);
        az_IMU2_f = Filter_IMU2_Z.filter(Stabilisierungsplattform.get_IMU2_Az()-OFFSET_IMU2_Z);


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
                
        
        ThisThread::sleep_for(TS_thread);  // Echtzeitzyklus mit 1ms (TS)
        
    }
}

void Home_sequenz()
{
     switch (process) {

        case y:

        Stabilisierungsplattform.set_velocity(0.0,INIT_SPEED);

        if(Stabilisierungsplattform.Endstop_Y == 0)
        {

            ThisThread::sleep_for(100ms);
            Stabilisierungsplattform.reset();
            ThisThread::sleep_for(100ms);
            Stabilisierungsplattform.set_position(0.0, 0.4,INIT_SPEED);
            ThisThread::sleep_for(1000ms);
            ThisThread::sleep_for(100ms);
            Stabilisierungsplattform.reset();
            ThisThread::sleep_for(100ms);
            process = x;
            
        }
        
        break;
        
 

        case x:

        
        Stabilisierungsplattform.set_velocity(-INIT_SPEED,0.0);

        if(Stabilisierungsplattform.Endstop_X == 0)
        {
            process = home;
            

            Stabilisierungsplattform.reset();
            ThisThread::sleep_for(200ms);

            

        }

        break;

        case home:

        Stabilisierungsplattform.set_position(POS_ZERO_X, POS_ZERO_Y,INIT_SPEED*10.0f);
        

        

        break;

     }

         
}



void StateMachine_task() {
    while (true) {

        if(Stabilisierungsplattform.Emergency_Stop == 0)
        {
            state = ERROR_MODE;
            stabilisierung = false;
        }
        if(M1_temp > TEMP_THRESHOLD_EMERGENCY || M2_temp > TEMP_THRESHOLD_EMERGENCY)
        {
            state = ERROR_MODE;
            stabilisierung = false;
        }

        switch (state) {

            case SELFTEST:

                Stabilisierungsplattform.LED_Red  = 1;
                Stabilisierungsplattform.LED_Green = 1;

                
                if(abs(Stabilisierungsplattform.get_IMU1_Ax())>= ACCELERATION_THRESHOLD_XY)
                {
                    
                    printf("Malfunction of Ax in IMU");
                    break;
                }
                if(abs(Stabilisierungsplattform.get_IMU1_Ay())>= ACCELERATION_THRESHOLD_XY)
                {
                    state = ERROR_MODE;
                    printf("Malfunction of Ay in IMU");
                    break;
                }
                if(abs(Stabilisierungsplattform.get_IMU1_Az())>= ACCELERATION_THRESHOLD_Z)
                {
                    state = ERROR_MODE;
                    printf("Malfunction of Az in IMU");
                    break;
                }

                if(Stabilisierungsplattform.get_Uin()<= MIN_VOLTAGE)
                {
                    state = ERROR_MODE;
                    printf("Min Voltage of battery reached! ");
                    break;
                }
                
                state = CALIBRATION;
                process = y;

                break;

            case CALIBRATION:

                Stabilisierungsplattform.Motor1_EN = 1;
                Stabilisierungsplattform.Motor2_EN = 1;
                
               Home_sequenz();

                
                if (process == home && abs(Stabilisierungsplattform.get_position_mm(0) - POS_ZERO_X) <= 0.5f && abs(Stabilisierungsplattform.get_position_mm(1) - POS_ZERO_Y) <= 0.5f)
                {

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

                /*

                X_soll = -50.0;
                Y_soll = 0.0;

                Stabilisierungsplattform.set_position(X_soll, Y_soll,1000.0);


                ThisThread::sleep_for(2000ms);
                
                X_soll = 50.0;
                Y_soll = 0.0;

                Stabilisierungsplattform.set_position(X_soll, Y_soll,1000.0);

                ThisThread::sleep_for(2000ms);

*/
                

                stabilisierung = true;


                if(abs(ax_IMU1_f1-ax_IMU1_f_old) <= 0.005 && abs(ay_IMU1_f1-ay_IMU1_f_old) <= 0.005)
                {
                    cnt_stabilisierung_aus++;

                    if(cnt_stabilisierung_aus >= CNT_SLEEP_MODE)
                    {
                        state = SLEEPMODE;
                        cnt_stabilisierung_aus = 0;
                    }

                }
                else 
                {
                    cnt_stabilisierung_aus = 0;
                }

                ax_IMU1_f_old = ax_IMU1_f1;
                ay_IMU1_f_old = ay_IMU1_f1; 

                
                
                
                break;
            
            case SLEEPMODE:
                
                stabilisierung = false;

                Stabilisierungsplattform.LED_Green = 0;
                Stabilisierungsplattform.LED_Red = 0;

                
                Stabilisierungsplattform.Motor1_EN = 0;
                Stabilisierungsplattform.Motor2_EN = 0;

                if(abs(ax_IMU1_f1-ax_IMU1_f_old) >= 0.005 || abs(ay_IMU1_f1-ay_IMU1_f_old) >= 0.005)
                {
                    state = RUN_MODE;
                }

                ax_IMU1_f_old = ax_IMU1_f1;
                ay_IMU1_f_old = ay_IMU1_f1; 
             
                break;

            case ERROR_MODE:

                
                
                Stabilisierungsplattform.LED_Red  = 1;
                Stabilisierungsplattform.LED_Green = 0;

                Stabilisierungsplattform.Motor1_EN = 0;
                Stabilisierungsplattform.Motor2_EN = 0;

                Stabilisierungsplattform.reset();

                if(Stabilisierungsplattform.Emergency_Stop == 1)
                {
                    state = SELFTEST;
                }

 

            break;
                
        }

        ThisThread::sleep_for(10ms);
    }
}
// Funktion für den Realtime-Thread


// Funktion für Druckausgabe (optional)
void print_task() {
    while (true) {

    
        // Print Temp
        //printf("t1:%f t2:%f\n", M1_temp,M2_temp);

        //Print Beschleunigungen
        printf("ax1:%f ay1:%f az1:%f ax2:%f ay2:%f az2:%f \n",ax_IMU1_f1,ay_IMU1_f1,az_IMU1_f1,ax_IMU2_f1,-ay_IMU2_f1,-az_IMU2_f1);


        //Print Position
        //printf("X_soll:%f Y_soll:%f X_ist:%f Y_ist:%f\n",X_soll,Y_soll,Stabilisierungsplattform.get_position_mm(0),Stabilisierungsplattform.get_position_mm(1));


        
/*  
        printf("pos M1 = %f mm\n", Stabilisierungsplattform.M1_get_mm());
        printf("pos M2 = %f mm\n", Stabilisierungsplattform.M2_get_mm());
        printf("pos X = %f mm\n", Stabilisierungsplattform.get_position_mm(0));
        printf("pos Y = %f mm\n", Stabilisierungsplattform.get_position_mm(1));
        
        printf("pos M1 = %f rot\n", Stabilisierungsplattform.M1_get_revolutions());
        printf("pos M2 = %f rot\n", Stabilisierungsplattform.M2_get_revolutions());
        printf("v M1 = %f mm/s\n", Stabilisierungsplattform.M1_get_mm_s());
        printf("v M2 = %f mm/s\n", Stabilisierungsplattform.M2_get_mm_s());
        //printf("vr M1 = %f r/s\n", Stabilisierungsplattform.M1_get_revolutions_s());
        //printf("vr M2 = %f r/s\n", Stabilisierungsplattform.M2_get_revolutions_s());
        printf("U M1 = %f V\n", Stabilisierungsplattform.Spannung_M1);
        printf("U M2 = %f V\n", Stabilisierungsplattform.Spannung_M2);
        printf("Timer = %d\n ",cnt_stabilisierung_aus);
        printf("######################\n");

        */

        ThisThread::sleep_for(5ms);
    }
}

int main() {
    // Threads starten
    
    Filter_IMU1_X.setPeriod(TS);
    Filter_IMU1_Y.setPeriod(TS);
    Filter_IMU1_Z.setPeriod(TS);

    Filter_IMU1_X.setFrequency(Filter_IMU);
    Filter_IMU1_Y.setFrequency(Filter_IMU);
    Filter_IMU1_Z.setFrequency(Filter_IMU);

    Filter_IMU2_X.setPeriod(TS);
    Filter_IMU2_Y.setPeriod(TS);
    Filter_IMU2_Z.setPeriod(TS);

    Filter_IMU2_X.setFrequency(Filter_IMU);
    Filter_IMU2_Y.setFrequency(Filter_IMU);
    Filter_IMU2_Z.setFrequency(Filter_IMU);



    Filter_IMU1_X_Plot.setPeriod(TS);
    Filter_IMU1_Y_Plot.setPeriod(TS);
    Filter_IMU1_Z_Plot.setPeriod(TS);

    Filter_IMU1_X_Plot.setFrequency(PLOT_FILTER);
    Filter_IMU1_Y_Plot.setFrequency(PLOT_FILTER);
    Filter_IMU1_Z_Plot.setFrequency(PLOT_FILTER);

    Filter_IMU2_X_Plot.setPeriod(TS);
    Filter_IMU2_Y_Plot.setPeriod(TS);
    Filter_IMU2_Z_Plot.setPeriod(TS);

    Filter_IMU2_X_Plot.setFrequency(PLOT_FILTER);
    Filter_IMU2_Y_Plot.setFrequency(PLOT_FILTER);
    Filter_IMU2_Z_Plot.setFrequency(PLOT_FILTER);
    

    Filter_T1.setPeriod(TS_Temp);
    Filter_T2.setPeriod(TS_Temp);
    Filter_T1.setFrequency(1.0);
    Filter_T2.setFrequency(1.0);


    //print_thread.start(print_task);
    temperatur_thread.start(temperature_task);
    realtime_thread.start(realtime_task);
    statemachine_thread.start(StateMachine_task);

    while (true) {
        

        /*

        
        Stabilisierungsplattform.set_position(20.0, 0.0, 60.0);
     

        ThisThread::sleep_for(2000ms);

        Stabilisierungsplattform.set_position(20.0, 20.0, 60.0);

        
        ThisThread::sleep_for(2000ms);

        Stabilisierungsplattform.set_position(0.0, 20.0, 60.0);

        

        ThisThread::sleep_for(2000ms);

        Stabilisierungsplattform.set_position(0.0, 0.0, 60.0);
        
*/
        printf("ax1:%f ay1:%f az1:%f ax2:%f ay2:%f az2:%f \n",ax_IMU1_f1,ay_IMU1_f1,az_IMU1_f1,ax_IMU2_f1,-ay_IMU2_f1,-az_IMU2_f1);
        ThisThread::sleep_for(5ms);




    }
}
