/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

tasks.h

Description
===========

Task interface. Main code wrapped into individual functions. Designed to be easily implemented into threads if needed.

Code Citations
==============

References
==========

"_ftime, _ftime32, _ftime64 from MSDN"
https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64


*/

#ifndef TASKS_H
#define TASKS_H

#include "crpi_robot.h"
#include "posemath.h"

using namespace crpi_robot;

void lynx_task_code(void* args);//Contains various tests for vehicle communications
void ur5_task_code_control(void* args);//Contains various tests for controlling manipulator arm.
void performance_test(void* args);//Main performance test routine
void performance_test_auto(void* args);//Automatic version of performance test routine that control vehicle in addition to manipulator.

#endif