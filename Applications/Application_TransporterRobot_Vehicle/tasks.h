/**
*\file tasks.h

*\brief Task interface. Main code wrapped into individual functions. Designed to be easily implemented into threads if needed.
*References:\n
*_ftime, _ftime32, _ftime64 from MSDN \n
*https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64

*\author Omar Aboul-Enein
*\date	2018-06-14
*/

#include "ulapi.h"
#include "cart_comm.h"

#ifndef TASKS_H
#define TASKS_H

/**
*\public
*A structure encapsulating all data structures to be shared between threads.
*/
struct task_args
{
	ulapi_mutex_struct* mutex;
	cart_comm* manager;
};

//ARCL MESSAGE CONSTANTS
//The following constants are messages sent from ld Core. Used to determine when to start robot arm searches.

/**
*This message is generated and sent from the ld core when the vehicle completes the process of docking with a cart.
*/
const char cart_cap[] = "Completed doing task cartCapture";

/**
*This message is generated and sent from the ld core when the vehicle completes the process of undocking with a cart.
*/
const char cart_release[] = "Completed doing task cartRelease";

/**
*This message is generated and sent from the ld core when the vehicle completes a "move" command. The move command uses specific parameters to allow the\n
*vehicle to safely undock with an object directly in front of its laser sensors. The clearance is set so that rotation of the mobile robot does not cause 
*its side sensors to strike the docking structure that the vehicle is trying to move away from.
*/
const char move_back[] = "Completed doing task move -1000 150 25 25 30 0 0 0 0 0 1";

/**
*This message is generated and sent from the ld core when the vehicle completes a "move" command. The move command uses specific parameters to allow the\n
*vehicle to dock with an object directly in front of its laser sensors without the sensors preventing a safety stop in the vehicle.
*/
const char move_forward[] = "Completed doing task move 1000 150 25 25 30 0 0 0 0 0 1";


/**
*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
*/
const char square_goal1[] = "Arrived at Goal3";

/**
*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
*/
const char square_goal2[] = "Arrived at Goal5";

/**
*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
*/
const char square_route1[] = "Finished patrolling route Route-Goal3";

/**
*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
*/
const char square_route2[] = "Finished patrolling route Route-Goal5";

/**
*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
*/
const char stage_route[] = "Finished patrolling route Route-Stage";

/**
*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
*/
const char dock_route[] = "Finished patrolling route Route-DockHelp";


/**
*\public
*Contains code for menu options used to test various functions related to the ld ARCL connections independently from the iterprocess communciations of the sofwtare. \n
*Currently used to test the Outgoing ARCL Connection, specifically
*\param[in] args	Void pointer to allow code to be modified to accept arguments. Currently, NULL should be passed.
*/
void ld_task_code(void* args);

/**
*\public
*Contains code for menu options used to test various functions related to the ld ARCL connections.\n
*Currently used to test the Outgoing ARCL Connection and ARCL server for sending commands, specifically
*\param[in] args	Void pointer to allow code to be modified to accept arguments. Currently, NULL should be passed.
*/
void ld_client_cmd_code(void* args);

/**
*\public
*Contains code for menu options used for running interprocess communications tests with one or more cart processes. The code defined here\n
*allows the user to test the communications for the software independent from the communications with the vehicle hardware.
*\param[in] args Void pointer to a cart_comm object.
*/
void cart_comm_test(void* args);

/**
*\public
*Multithreaded version of the interprocess communications tests. Intended to be run with cart_connect_t in background to connect new cart clients.
*\param[in] args Void pointer to a cart_comm process.
*/
void cart_comm_test_t(void* args);

/**
*\public
*Connects new cart clients as a background thread.
*\param[in] args Void pointer to a cart_comm process.
*/
void cart_connect_t(void* args);

/**
*This function implements the vehicle portion of the transporter robot performance test. In this version of the test, the vehicle always undocks\n
*with a cart immediately before scheduling its next task. This includes if the vehicle schedules the cart it just dropped off to be picked up again.
*\param[in] A void pointer to a task_args structure.
*/
void cart_dock_test_t(void* args);

/**
*This function implements the vehicle portion of the transporter robot performance test. In this version of the test, the vehicle delays the act of undocking \n
*from a cart (also referred to as unlatching) until after the vehicle schedules its next task. This allows the vehicle to function as mobile manipulator if only one cart is being serviced \n
*and eliminates extra moves if the cart that was just taken to its next location is also the next one to be scheduled to be taken to another location.
*\param[in] A void pointer to a task_args structure.
*/
void cart_latch_delayed_test_t(void* args);
#endif