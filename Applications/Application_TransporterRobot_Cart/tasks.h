/**
*\file tasks.h
*\brief Task interface. Main code wrapped into individual functions. Designed to be easily implemented into threads if needed. \n
*
*References:\n
*"_ftime, _ftime32, _ftime64 from MSDN":\n
*https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64 \n\n
*
*\author Omar Aboul-Enein
*\date 2017-06-05
*/


#include "ulapi.h"
#include "cart_comm_client.h"
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "cart_config.h"

using namespace crpi_robot;

#ifndef TASKS_H
#define TASKS_H

/**
*\public
*A structure encapsulating all data structures to be shared between threads.
*/

struct task_args
{
	/**
	*\public
	*A ulapi semaphore used to control a sleep and wakeup signal for the actuation_test thread.
	*/
	void* actuate;

	/**
	*\public
	*A data structure containing socket connection infor for connecting to a vehicle server process and status information about the cart.
	*/
	cart_comm_client* cart_client;

	/**
	*\public
	*A CRPI datastructure that manages the connection to UR5 script. Allows for control of the UR5 robot.
	*/
	CrpiRobot<CrpiUniversal>* ur5;

	cart_config* saved_config;
};

/**
*\public
*Used to test various functions related to manipulator arm pose transformations and positioning.
*Can be used to manually initiate the staging and stowing of the arm, as well as test registration methods.
*\param[in] args	Pointer to data structure containing passed arguments. No arguments are required for this task, thus NULL should be passed.
*/
void ur5_task_code_control(void* args);

/**
*\public
*Background thread for managing connection to vehicle server process, forwarding information to the vehicle server process, and updating cart status information when when requested by thew vehicle server process.
*\param[in] args	Pointer to data structure containing passed arguments. A pointer to a task_args structure should be passed.
*/
void cart_client_connect(void* args);

/**
*\public
*Thread used to test the actuation of the manipulator arm when the cart_client_connect thread updates the cart status to busy. (Essentially performs the coordinate registration and verification steps)
*This version of the function uses bisect registration.
*\param[in] args	Pointer to data structure containing passed arguments. A pointer to the task_args structure that was passed to the cart_client_connect function should be passed to this function.
*/
void actuation_test(void* args);

/**
*\public
*Thread used to test the actuation of the manipulator arm when the cart_client_connect thread updates the cart status to busy. (Essentially performs the coordinate registration and verification steps).\n
*This version of the function uses bisect registration and also dynamically updates the initial search location of the large reflectors as the large reflectors are localized.
*\param[in] args	Pointer to data structure containing passed arguments. A pointer to the task_args structure that was passed to the cart_client_connect function should be passed to this function.
*/
void actuation_test_auto_update(void* args);


/**
*\public
*Thread used to test the actuation of the manipulator arm when the cart_client_connect thread updates the cart status to busy. (Essentially performs the coordinate registration and verification steps).\n
*This version of the function uses edge registration.
*\param[in] args	Pointer to data structure containing passed arguments. A pointer to the task_args structure that was passed to the cart_client_connect function should be passed to this function.
*/
void actuation_test_edge(void* args);

void actuation_test_edge_cont(void* args);

void actuation_test_edge_cont2(void* args);

#endif