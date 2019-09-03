/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

mobile_robot.cpp

Description
===========

Main function to implement performance test of mobile robot. Calls task code functions.
*/

#include <iostream>
#include "ulapi.h"
#include "tasks.h"

using namespace std;

int main()
{
	int err;
	int user_in;

//Initialize the ulapi library
	if ((err = ulapi_init()) < 0)
	{
		cout << "Error " << err << " : could not initialize ulapi" << endl;
		exit(err);
	}//end if

	cout << "==========================================================" << endl;
	cout << "Mobile Robot Performance Test" << endl;
	cout << "==========================================================" << endl;
	cout << "0. Performance Test Auto" << endl;
	cout << "1. Performance Test Manual" << endl;
	cout << "2. UR5 Control Options" << endl;
	cout << "3. Lynx Comm Tests" << endl;
	cout << "Please select an option: " << endl;
	cin >> user_in;

	switch (user_in)
	{
		case 3:
			//Submenu for testing lynx status monitoring
			lynx_task_code(NULL);
			break;
		case 2:
			//Submenu for testing UR5 control
			ur5_task_code_control(NULL);
			break;
		case 1:
			//Runs full performance test
			performance_test(NULL);
			break;
		case 0:
			performance_test_auto(NULL);
		default:
			cout << "Invalid Choice Entered" << endl;
			break;

	};//end switch

	exit(0);
}//end main