#ifndef ENV
#define ENV


#define USE_IMU1

#define STABILISIERUNG
#define ACCEL_PRINT

//#define STEP
//#define POS_PRINT

// === Zeitsysteme ===
#define TS               0.005f      // Grundzeit (Regelung etc.)
#define TS_thread        5ms         // Encoder-Abtastzeit
#define TS_Temp          0.05f       // Temperaturfilter-Zeitbasis
#define TS_thread_Temp   50ms        // Temperatur-Abtastrate

// === PWM & Encoder ===
#define PWMFREQUENZ      50000
#define TICKS_PER_REVOLUTION 1024

// === Sleep-Logik ===
#define CNT_SLEEP_MODE   300

// === IMU Offsets ===
#define OFFSET_IMU1_X    0.074191
#define OFFSET_IMU1_Y   -0.075453
#define OFFSET_IMU1_Z    0.201874

#define OFFSET_IMU2_X   -0.205112
#define OFFSET_IMU2_Y    0.073742
#define OFFSET_IMU2_Z    0.182435

// === Beschleunigungsschwellen (für Selbsttest) ===
#define ACCELERATION_THRESHOLD_XY  1.5
#define ACCELERATION_THRESHOLD_Z   10.0

// === Temperaturgrenzen ===
#define TEMP_THRESHOLD_O         35.0f   // Obergrenze
#define TEMP_THRESHOLD_U         30.0f   // Untergrenze
#define TEMP_THRESHOLD_EMERGENCY 60.0f   // Notabschaltung

// === Position & Mechanik ===
#define GEAR_1      15.5833
#define GEAR_2      15.5844
#define D_PULLEY    14.55

#define INIT_SPEED  15.0f
#define POS_ZERO_X  47.8f
#define POS_ZERO_Y -66.188

#define X_MAX       55.0
#define X_MIN      -55.0
#define Y_MAX       55.0
#define Y_MIN      -55.0

// === Spannungslimit ===
#define MIN_VOLTAGE 20.0

// ===Filterfrequenzen ===
#define FILTER_1 2.0
#define Filter_IMU  3.0

// === Konstanten ===
#define PI  3.14159265359
#define R_S 445.71

// === Reglerparameter ===
// M1 – Velocity
#define M1_V_KP 0.41236
#define M1_V_KI 7.2902
#define M1_V_KD 0.00026581

// M1 – Position
#define M1_P_KP 39.92
#define M1_P_KI 0.0
#define M1_P_KD 0.0

// M2 – Velocity
#define M2_V_KP 0.41236
#define M2_V_KI 7.2902
#define M2_V_KD 0.00026581

// M2 – Position
#define M2_P_KP 39.92
#define M2_P_KI 0.0
#define M2_P_KD 0.0

#endif // ENV
