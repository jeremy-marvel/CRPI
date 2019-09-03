/**
*\file transporter_vehicle.cpp

*\brief Main function to implement performance test of mobile robot. Calls task code functions. \n

*References: \n
*Based on ultest by F. Proctor

*\author Omar Aboul-Enein
*\date 2017-06-05
*/

/**
*\mainpage
*\section network Network Diagram
*\image html NetworkDiagramDoxy.png
*\section overview Control Algorithm Overview
*\image html Slide40.jpg
*\image html Slide27.jpg
*\image html Slide28.jpg
*\image html Slide29.jpg
*\section config Project Configuration
*(Note: Be sure to compile for 32-bit architectures)
*\subsection incl Include Directories
*../../../Libraries/ulapi/src; \n
*../../../Libraries/; \n
*../../../Libraries/CRPI; \n
*../posemath; \n
*../../../Libraries/MotionPrims/ \n\n
*\subsection lnk Linker Input
*Winmm.lib; \n
*../MotionPrims.lib; \n
*../CRPI_Library.lib; \n
*../dlfuncs.lib; \n
*ws2_32.lib; \n
*../ulapi.lib; \n
*kernel32.lib; \n
*user32.lib; \n
*gdi32.lib; \n
*winspool.lib; \n
*comdlg32.lib; \n
*advapi32.lib; \n
*shell32.lib; \n
*ole32.lib; \n
*oleaut32.lib; \n
*uuid.lib; \n
*odbc32.lib; \n
*odbccp32.lib; \n
*%(AdditionalDependencies)
*\subsection rsrc Resource Files
*The following compiled libraries should be added to the same directory that contains the source files:\n
*CRPI_Library.lib \n
*dlfuncs.lib \n
*MotionPrims.lib \n
*ulapi.lib
*/

#include <iostream>
#include "ulapi.h"
#include "tasks.h"

using namespace std;

int main()
{
	int err;
	int user_in;
	ulapi_id ulapi_key = 6;

	ulapi_task_struct* cart_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
	ulapi_task_struct* cart_comm_test_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
	ulapi_task_struct* ld_dock_test_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

	task_args* args = (task_args*)malloc(sizeof(task_args));

	cart_comm* ld_server;

	//Initialize the ulapi library
	if ((err = ulapi_init()) < 0)
	{
		cout << "Error " << err << " : could not initialize ulapi" << endl;
		exit(err);
	}//end if

	cout << "==========================================================" << endl;
	cout << "Manpiulator on Cart Test: Vehicle" << endl;
	cout << "==========================================================" << endl;
	cout << "0. LD Comm Test" << endl;
	cout << "1. LD Comm Test with Cart Commands" << endl;
	cout << "2. Cart Client Communications Test (Single Thread)" << endl;
	cout << "3. Cart Client Communications Test (Multi-Thread)" << endl;
	cout << "4. LD Docking Test" << endl;
	cout << "5. LD Delayed Latch Test" << endl;
	cout << "Please select an option: " << endl;
	cin >> user_in;

	//Submenu for testing UR5 control
	if (user_in == 0)
	{
		ld_task_code(NULL);
	}//end if

	if (user_in == 1)
	{
		ld_client_cmd_code(NULL);
	}//end if

	else if (user_in == 2)
	{
		cart_comm_test(NULL);
	}//end switch

	else if (user_in == 3)
	{
		ld_server = new cart_comm(5352, 3, 33, 34);

		if ((err = ld_server->cart_comm_init()) < 0)
			exit(err);

		if ((cart_connect_task = ulapi_task_new()) == NULL)
		{
			cout << "Error: could not create ulapi task" << endl;
			exit(-1);
		}//end if

		if ((cart_comm_test_task = ulapi_task_new()) == NULL)
		{
			cout << "Error: could not create ulapi task" << endl;
			exit(-1);
		}//end if

		if ((err = ulapi_task_init(cart_connect_task)) < 0)
		{
			cout << "Error: could not initialize ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_init(cart_comm_test_task)) < 0)
		{
			cout << "Error: could not initialize ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_start(cart_connect_task, cart_connect_t, (void*)ld_server, ulapi_prio_lowest(), 0)) < 0)
		{
			cout << "Error: could not start ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_start(cart_comm_test_task, cart_comm_test_t, (void*)ld_server, ulapi_prio_lowest(), 0)) < 0)
		{
			cout << "Error: could not start ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_join(cart_comm_test_task, NULL)) < 0)
		{
			cout << "Error: could not join ulapi task" << endl;
			exit(err);
		}//end if

	}//end else if

	else if (user_in == 4)
	{
		ld_server = new cart_comm(5352, 3, 33, 34);

		if ((err = ld_server->cart_comm_init()) < 0)
			exit(err);

		if ((cart_connect_task = ulapi_task_new()) == NULL)
		{
			cout << "Error: could not create ulapi task" << endl;
			exit(-1);
		}//end if

		if ((ld_dock_test_task = ulapi_task_new()) == NULL)
		{
			cout << "Error: could not create ulapi task" << endl;
			exit(-1);
		}//end if

		if ((err = ulapi_task_init(cart_connect_task)) < 0)
		{
			cout << "Error: could not initialize ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_init(ld_dock_test_task)) < 0)
		{
			cout << "Error: could not initialize ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_start(cart_connect_task, cart_connect_t, (void*)ld_server, ulapi_prio_lowest(), 0)) < 0)
		{
			cout << "Error: could not start ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_start(ld_dock_test_task, cart_dock_test_t, (void*)ld_server, ulapi_prio_lowest(), 0)) < 0)
		{
			cout << "Error: could not start ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_join(ld_dock_test_task, NULL)) < 0)
		{
			cout << "Error: could not join ulapi task" << endl;
			exit(err);
		}//end if

	}//end else if

	else if (user_in == 5)
	{
		ld_server = new cart_comm(5352, 3, 33, 34);

		if ((err = ld_server->cart_comm_init()) < 0)
			exit(err);

		if ((cart_connect_task = ulapi_task_new()) == NULL)
		{
			cout << "Error: could not create ulapi task" << endl;
			exit(-1);
		}//end if

		if ((ld_dock_test_task = ulapi_task_new()) == NULL)
		{
			cout << "Error: could not create ulapi task" << endl;
			exit(-1);
		}//end if

		if ((err = ulapi_task_init(cart_connect_task)) < 0)
		{
			cout << "Error: could not initialize ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_init(ld_dock_test_task)) < 0)
		{
			cout << "Error: could not initialize ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_start(cart_connect_task, cart_connect_t, (void*)ld_server, ulapi_prio_lowest(), 0)) < 0)
		{
			cout << "Error: could not start ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_start(ld_dock_test_task, cart_latch_delayed_test_t, (void*)ld_server, ulapi_prio_lowest(), 0)) < 0)
		{
			cout << "Error: could not start ulapi task" << endl;
			exit(err);
		}//end if

		if ((err = ulapi_task_join(ld_dock_test_task, NULL)) < 0)
		{
			cout << "Error: could not join ulapi task" << endl;
			exit(err);
		}//end if

	}//end else if

	exit(0);
}//end main