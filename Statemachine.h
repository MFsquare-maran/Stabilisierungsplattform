// StateMachine.h
#pragma once

#include "mbed.h"
#include "Hardware.h"

enum State { SELFTEST, CALIBRATION, SLEEPMODE, RUN_MODE, ERROR_MODE };
enum Process { y, x, home };

extern State state;
extern Process process;

void StateMachine_task();
void Home_sequenz();
