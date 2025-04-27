#include "mbed.h"
#include "env.h"
#include "Hardware.h"
#include <cmath> // für fabsf()
#include "LowpassFilter.h"

BufferedSerial pc(USBTX, USBRX, 115200);  // USB-Serielle Schnittstelle


Hardware Stabilisierungsplattform(TS);


LowpassFilter Filter_X;
LowpassFilter Filter_Y;




enum State {
    SELFTEST,
    CALIBRATION,
    SLEEPMODE,
    RUN_MODE,
    ERROR_MODE
}; 

State state;

enum Process {
    y1,
    y2,
    x,
    home
};

Process process;

// Thread-Objekte
Thread realtime_thread(200);
Thread print_thread(osPriorityLow);
Thread statemachine_thread(osPriorityNormal);
/*

Thread temperatur_thread(osPriorityRealtime);

*/

bool stabilisierung = true;

float M1_temp = 0.0;
float M2_temp = 0.0;

void temperature_task()
{
     printf("TEMP");
    M1_temp =  Stabilisierungsplattform.M1_get_temperature();

    M2_temp =  Stabilisierungsplattform.M2_get_temperature();

    

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


    
    ThisThread::sleep_for(5000ms);
    
}



void realtime_task() {
    while (true) {




        Stabilisierungsplattform.update();

        if( stabilisierung == true )
        {
            float phi_x = atan(Stabilisierungsplattform.get_IMU1_Ax() / Stabilisierungsplattform.get_IMU1_Az());
            float phi_y = atan(Stabilisierungsplattform.get_IMU1_Ay() / Stabilisierungsplattform.get_IMU1_Az());

            Stabilisierungsplattform.set_position(Filter_X.filter(R_S * sin(phi_x)), Filter_Y.filter(R_S * sin(phi_y)),1000.0);
        
        }
                
        
        ThisThread::sleep_for(TS_thread);  // Echtzeitzyklus mit 1ms (TS)
        
    }
}

void Home_sequenz()
{
     switch (process) {

        case y1:

        Stabilisierungsplattform.set_velocity(0.0,INIT_SPEED);

        if(Stabilisierungsplattform.Endstop_Y == 0)
        {

            ThisThread::sleep_for(100ms);
            Stabilisierungsplattform.reset();
            ThisThread::sleep_for(100ms);
            process = y2;
        }
        
        break;
        
        case y2:


        
        ThisThread::sleep_for(100ms);

        Stabilisierungsplattform.set_position(0.0,-1.0,INIT_SPEED);

        ThisThread::sleep_for(200ms);

        process = x;
        
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
                process = y1;

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
                
               stabilisierung = true;
                
                break;
            
            case SLEEPMODE:
                // TODO: Warten auf Auslenkung
             
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

        ThisThread::sleep_for(100ms);
    }
}
// Funktion für den Realtime-Thread


// Funktion für Druckausgabe (optional)
void print_task() {
    while (true) {

        printf("######################\n");
        //printf("Uin = %f\n", Stabilisierungsplattform.get_Uin());
        printf("AX = %f\n", Stabilisierungsplattform.get_IMU1_Ax());
        printf("AY = %f\n", Stabilisierungsplattform.get_IMU1_Ay());
        printf("AZ = %f\n", Stabilisierungsplattform.get_IMU1_Az());
        printf("t1 =  %f\n", Stabilisierungsplattform.M1_get_temperature());
        printf("t2 =  %f\n", Stabilisierungsplattform.M2_get_temperature());
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
        printf("######################\n");

        ThisThread::sleep_for(500ms);
    }
}

int main() {
    // Threads starten
    
    Filter_Y.setPeriod(TS);
    Filter_X.setPeriod(TS);
    Filter_Y.setFrequency(50.0);
    Filter_X.setFrequency(50.0);
    

    print_thread.start(print_task);
    //temperatur_thread.start(temperature_task);
    realtime_thread.start(realtime_task);

    statemachine_thread.start(StateMachine_task);

    //Stabilisierungsplattform.Motor1_EN = 1;
    //Stabilisierungsplattform.Motor2_EN = 1;



    //Stabilisierungsplattform.M1_set_U(-24.0);
    //Stabilisierungsplattform.M2_set_U(24.0);
    // Hauptthread tut nichts mehr, bleibt aktiv
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
        ThisThread::sleep_for(2000ms);




    }
}
