#ifndef BIOTAC_SENSOR

#define BIOTAC_SENSOR

#include "cheetah.h"
#include "biotac.h"
#include <conio.h>
#include <windows.h>

#include <getopt.h>
#include <assert.h>
#include <iostream>
//#include <vector>
#include <queue>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/random.hpp>
#include <boost/bind.hpp>

// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdh/dsa.h"
#include "sdhoptions.h"
#include "dsaboost.h"
#include "sdh/simpletime.h"
#include <boost/date_time/posix_time/posix_time.hpp>


//cSDH hand;



int bt_main(void);

//int decide_grasping(char 

void control(char* filename, char* start, char* end);

int grasp_listener();

void run(int argc, char** argv, char* filename, char* start, char* end,double force_mag_des, std::string finger, bool engage_manipulation, bool use_biotacs);

#endif