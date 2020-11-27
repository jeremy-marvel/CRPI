/**
*\file tasks.cpp

*\brief Main task code. (e.g. thread function definitions) \n
*Also corresponds to the actual tasks each robot performs during registration.

*References: \n
*Based on mobmanmain.cpp by S. Legowik \n
*Based on unittest.cpp by J. Marvel \n

*"_ftime, _ftime32, _ftime64 from MSDN" https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64

*\author Omar Aboul-Enein
*\date 2017-06-05

*/

#include <iostream>
#include <fstream>
#include <string>
#include <sys\timeb.h>
#include "ulapi.h"
#include "ld_comm.h"
#include "ld_comm_client.h"
#include "cart_comm.h"
#include "tasks.h"

using namespace std;

void ld_task_code(void* args)
{
	int err;
	ld_msg_pose cur_pose;
	int user_in;
	const int iter = 50;
	_timeb recv_time;
	int i = 0;
	char* msg_buf;
	char* goal_1 = (char*) square_goal1;
	
	ld_comm* mobile_robot;
	mobile_robot = new ld_comm(5353, TRUE);

	if ((err = mobile_robot->ld_connect()) < 0)
	{
		mobile_robot->~ld_comm();
		exit(err);
	}//end if

	mobile_robot->log_comment("seconds since epoch (s), robot_x (mm), robot_y (mm), robot_th (degrees), recv time (s), x recv time (s), y recv time (s), th recv time (s), time recv time (s)");

	while(1)
	{
		cout << endl;
		cout << "1. ld Pose Stream" << endl;
		cout << "2. ARCL Message Stream" << endl;
		cout << "3. Arrival Message Stream" << endl;
		cout << "Enter -1 to Quit" << endl;
		cout << "Please select a pose to transmit:" << endl;
		cin >> user_in;

		switch (user_in)
		{
			case 1:
				mobile_robot->log_comment("RobotPose");
				break;
		}//end switch

		switch(user_in)
		{
			case 1:
				while(1)
				{
					if ((err = mobile_robot->arcl_dsfv_pose(&cur_pose)) < 0)
					{
						mobile_robot->~ld_comm();
						exit(err);
					}//end if


					cout << i << ", ";
					i++;
					_ftime_s(&recv_time);
					cur_pose.server_time = recv_time;
					mobile_robot->print_pose(&cur_pose);
					mobile_robot->log_pose(&cur_pose, -1, "TEST");

				}//end while

				break;

			case 2:

				while (1)
				{

					if ((err = mobile_robot->arcl_read(&msg_buf)) < 0)
					{
						mobile_robot->~ld_comm();
						exit(err);
					}//end if

					cout << msg_buf << endl;

				}//end while

				break;

			case 3:

				while (1)
				{

					if ((err = mobile_robot->arcl_read_status(goal_1)) < 0)
					{
						mobile_robot->~ld_comm();
						exit(err);
					}//end if

				}//end while

				break;

			case -1:
				mobile_robot->~ld_comm();
				exit(err);

			default:
				cout << "Invalid option: Please try again" << endl;
				break;
		}//end switch
	}//end while

}//end ld_control

void ld_client_cmd_code(void* args)
{
	int err;
	ld_msg_pose cur_pose;
	int user_in;
	const int iter = 50;
	_timeb recv_time;
	int i = 0;
	char* msg_buf;
	char* goal_1 = (char*)square_goal1;
	char* cart_c = (char*)cart_cap;
	char* cart_r = (char*)cart_release;

	ld_comm* mobile_robot;
	ld_comm_client* mobile_robot_cmd;

	mobile_robot = new ld_comm(5353, TRUE);
	mobile_robot_cmd = new ld_comm_client("192.168.160.36", 7575, "adept", FALSE);

	if ((err = mobile_robot->ld_connect()) < 0)
	{
		mobile_robot->~ld_comm();
		exit(err);
	}//end if

	if ((err = mobile_robot_cmd->ld_connect()) < 0)
	{
		mobile_robot_cmd->~ld_comm_client();
		exit(err);
	}//end if

	cout << "FINISHED CONNECTING TO ARCL SERVER" << endl;

	mobile_robot->log_comment("seconds since epoch (s), robot_x (mm), robot_y (mm), robot_th (degrees), recv time (s), x recv time (s), y recv time (s), th recv time (s), time recv time (s)");

	while (1)
	{
		cout << endl;
		cout << "1. ld Pose Stream" << endl;
		cout << "2. ARCL Message Stream" << endl;
		cout << "3. Arrival Message Stream" << endl;
		cout << "4. cartCapture Test" << endl;
		cout << "5. cartRelease Test" << endl;
		cout << "Enter -1 to Quit" << endl;
		cout << "Please select a pose to transmit:" << endl;
		cin >> user_in;

		switch (user_in)
		{
		case 1:
			mobile_robot->log_comment("RobotPose");
			break;
		}//end switch

		switch (user_in)
		{
		case 1:
			while (1)
			{
				if ((err = mobile_robot->arcl_dsfv_pose(&cur_pose)) < 0)
				{
					mobile_robot->~ld_comm();
					mobile_robot_cmd->~ld_comm_client();
					exit(err);
				}//end if


				cout << i << ", ";
				i++;
				_ftime_s(&recv_time);
				cur_pose.server_time = recv_time;
				mobile_robot->print_pose(&cur_pose);
				mobile_robot->log_pose(&cur_pose, -1, "TEST");

			}//end while

			break;

		case 2:

			while (1)
			{

				if ((err = mobile_robot->arcl_read(&msg_buf)) < 0)
				{
					mobile_robot->~ld_comm();
					mobile_robot_cmd->~ld_comm_client();
					exit(err);
				}//end if

				cout << msg_buf << endl;

			}//end while

			break;

		case 3:

			while (1)
			{

				if ((err = mobile_robot->arcl_read_status(goal_1)) < 0)
				{
					mobile_robot->~ld_comm();
					mobile_robot_cmd->~ld_comm_client();
					exit(err);
				}//end if

			}//end while

			break;

		case 4:

			mobile_robot_cmd->arcl_write("doTask cartCapture\n");
			mobile_robot->arcl_read_status(cart_c);

			break;

		case 5:

			mobile_robot_cmd->arcl_write("doTask cartRelease\n");
			mobile_robot->arcl_read_status(cart_r);

			break;

		case -1:
			mobile_robot->~ld_comm();
			mobile_robot_cmd->~ld_comm_client();
			exit(err);

		default:
			cout << "Invalid option: Please try again" << endl;
			break;
		}//end switch
	}//end while

}//end ld_control

void cart_comm_test_t(void* args)
{
	int err;
	int user_in = 0;
	int user_disconnect = 0;
	cart_comm* vehicle_server = (cart_comm*)args;
	char* cart_client_buf;

	ld_msg_pose test_pose;
	test_pose.robot_x = -8708;
	test_pose.robot_y = 20705;
	test_pose.robot_th = 180;

	while (1)
	{
		cout << endl;
		cout << "1. Send test data to all cart clients" << endl;
		cout << "2. Update cart client status" << endl;
		cout << "3. Change cart client status to Ready (0)" << endl;
		cout << "4. Change cart client status to Waiting (1)" << endl;
		cout << "5. Change cart client status to Busy (2)" << endl;
		cout << "6. Disconnect a cart client" << endl;
		cout << "7. Print cart client list" << endl;
		cout << "8. Print cart client status" << endl;
		cout << "Enter -1 to Quit" << endl;
		cout << "Please select an option:" << endl;
		cin >> user_in;

		switch (user_in)
		{
		case 1:
			for (int i = 0; i<vehicle_server->get_client_count(); i++)
				vehicle_server->send("Hello Cart, I am Vehicle", vehicle_server->get_client_id(i));
			break;
		case 2:
			vehicle_server->update_status(&test_pose);
			break;
		case 3:
			for (int i = 0; i<vehicle_server->get_client_count(); i++)
				vehicle_server->send("rcupdate0", vehicle_server->get_client_id(i));
			break;
		case 4:
			for (int i = 0; i<vehicle_server->get_client_count(); i++)
				vehicle_server->send("rcupdate1", vehicle_server->get_client_id(i));
			break;
		case 5:
			for (int i = 0; i < vehicle_server->get_client_count(); i++)
			{
				vehicle_server->send("rcupdate2", vehicle_server->get_client_id(i));
				vehicle_server->update_previous_dock(i);
			}//end for
			break;
		case 6:
			if (vehicle_server->get_client_count() > 0)
			{
				cout << "Please enter a cart client id: " << endl;
				cin >> user_disconnect;
				vehicle_server->client_disconnect(user_disconnect);
			}//end if
			else
				cout << "No Clients are connected" << endl;
			break;
		case 7:
			vehicle_server->print_client_ids();
			break;
		case 8:
			vehicle_server->print_client_status();
			break;
		case -1:
			return;
		}//end switch
	}//end while

	return;
}//end cart_comm_test_t

void cart_comm_test(void* args)
{
	int err;
	int user_in = 0;
	int user_disconnect = 0;
	cart_comm* manager = new cart_comm(5352, 3, 33, 34);
	
	if ((err = manager->cart_comm_init()) < 0)
		exit(err);

	while(1)
	{
		cout << endl;
		cout << "1. Send test data to all cart clients" << endl;
		cout << "2. Connect a new cart client" << endl;
		cout << "3. Disconnect a cart client" << endl;
		cout << "4. Print cart client list" << endl;
		cout << "Please select an option:" << endl;
		cout << "Enter -1. to Quit" << endl;
		cin >> user_in;

		switch (user_in)
		{
			case 1:
				for(int i=0; i<manager->get_client_count(); i++)
					manager->send("Hello Cart, I am Vehicle", manager->get_client_id(i));
				break;
			case 2:
				cout << "Waiting for cart client..." << endl;
				manager->cart_connect();
				break;
			case 3:
				cout << "Please enter a cart client id: " << endl;
				cin >> user_disconnect;
				manager->client_disconnect(user_disconnect);
				break;
			case 4:
				manager->print_client_ids();
				break;
			case -1:
				return;
		}//end switch
	}//end while

	return;
}//end cart_comm_test

void cart_dock_test_t(void* args)
{
	int err;
	int user_in = 0;
	int user_disconnect = 0;
	cart_comm* vehicle_server = (cart_comm*)args;
	char* cart_client_buf = NULL;
	char* cart_client_buf2 = NULL;
	cart_status_copy* selected_client = NULL;
	int scheduled_id = -1;
	int prev_id = -1;

	ld_msg_pose cur_pose;

	char* msg_buf_w;
	char* msg_buf_r;
	char* cart_c = (char*)cart_cap;
	char* cart_r = (char*)cart_release;
	char* move_f = (char*)move_forward;
	char* move_b = (char*)move_back;

	ld_comm* mobile_robot;
	ld_comm_client* mobile_robot_cmd;

	mobile_robot = new ld_comm(5353, TRUE);
	mobile_robot_cmd = new ld_comm_client("192.168.160.36", 7575, "adept", FALSE);

	if ((err = mobile_robot->ld_connect()) < 0)
	{
		mobile_robot->~ld_comm();
		exit(err);
	}//end if

	if ((err = mobile_robot_cmd->ld_connect()) < 0)
	{
		mobile_robot_cmd->~ld_comm_client();
		exit(err);
	}//end if

	while (1)
	{
		vehicle_server->update_status();
		vehicle_server->print_client_ids();
		vehicle_server->print_client_status();

		//**CODE TO SCHEDULE TASK GOES HERE**
		if (scheduled_id > 0)
			prev_id = scheduled_id;

		scheduled_id = vehicle_server->simple_scheduler();
		///////////////////////////////////////

		cout << "SCHEDULED ID: " << scheduled_id << endl;

		if (vehicle_server->get_client_count() > 0 && scheduled_id != -1)
		{

			vehicle_server->send("rcupdate1", scheduled_id);
			vehicle_server->update_status();
			vehicle_server->select_client(scheduled_id, &selected_client);

			cout << "SELECTED CLIENT" << endl;
			cout << "select_client!=NULL: " << (selected_client != NULL) << endl;
			cout << "selected_client->get_status() == 1" << (selected_client->get_status() == 1) << endl;

			if (selected_client != NULL && selected_client->get_status() == 1)
			{
				cout << "CHECKED STATUS" << endl;

				selected_client->get_current_dock(&cart_client_buf);
				selected_client->get_previous_dock(&cart_client_buf2);

				if (cart_client_buf != NULL && cart_client_buf2 != NULL)
				{
					cout << "GOT NEXT DOCK" << endl;
					//CAPTURE CART AT GOAL
					msg_buf_w = (char*)malloc(sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1));

					strcpy_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), "patrolOnce ");
					strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), cart_client_buf2);
					strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), "_Route\n");

					msg_buf_r = (char*)malloc(sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1));

					strcpy_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), "Finished patrolling route ");
					strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), cart_client_buf2);
					strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), "_Route");

					cout << "msg_buf_w:" << msg_buf_w << ":end" << endl;
					cout << "msg_buf_r:" << msg_buf_r << ":end" << endl;

					if (scheduled_id != prev_id)
					{
						mobile_robot_cmd->arcl_write(msg_buf_w);
						mobile_robot->arcl_read_status(msg_buf_r);
					}//end if

					if (selected_client->get_no_cart() == 0)
					{
						mobile_robot_cmd->arcl_write("doTask cartCapture\n");
						mobile_robot->arcl_read_status(cart_c);
					}//end if

					mobile_robot_cmd->arcl_write("doTask move -1000 150 25 25 30 0 0 0 0 0 1\n");
					mobile_robot->arcl_read_status(move_b);

					free(msg_buf_w);
					free(msg_buf_r);

					//RELEASE CART AT DESTINATION

					msg_buf_w = (char*)malloc(sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1));

					strcpy_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), "patrolOnce ");
					strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), cart_client_buf);
					strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), "_Route\n");

					msg_buf_r = (char*)malloc(sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1));

					strcpy_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), "Finished patrolling route ");
					strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), cart_client_buf);
					strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), "_Route");

					mobile_robot_cmd->arcl_write(msg_buf_w);
					mobile_robot->arcl_read_status(msg_buf_r);


					mobile_robot_cmd->arcl_write("doTask move 1000 150 25 25 30 0 0 0 0 0 1\n");
					mobile_robot->arcl_read_status(move_f);

					cout << "GETTING POSE" << endl;
					mobile_robot->arcl_dsfv_pose(&cur_pose);
					cout << "LOGGING POSE" << endl;
					mobile_robot->log_pose(&cur_pose, selected_client->get_id(), cart_client_buf);

					if (selected_client->get_no_cart() == 0)
					{
						cout << "SENDING CART RELEASE COMMAND" << endl;
						mobile_robot_cmd->arcl_write("doTask cartRelease\n");
						cout << "SENT CART RELEASE COMMAND" << endl;
						mobile_robot->arcl_read_status(cart_r);
					}//end if

					free(msg_buf_w);
					free(msg_buf_r);

					//UPDATE STATUS //TODAY!!!! NOTICED THAT I DON"T UPDATE THE INTERNAL DATA STRUCTURES BEFORE SENDING THE COMMAND AND POSE
					vehicle_server->update_status(&cur_pose);
					vehicle_server->send("rcupdate2", scheduled_id);
					vehicle_server->update_previous_dock(scheduled_id);
					vehicle_server->update_status(&cur_pose);
				}//end if
			}//end if
		}//end if
		ulapi_sleep(0.1);
	}//end while

}//end cart_dock_test_t

void cart_latch_delayed_test_t(void* args)
{
	int err;
	int user_in = 0;
	int user_disconnect = 0;
	cart_comm* vehicle_server = (cart_comm*)args;
	char* cart_client_buf = NULL;
	char* cart_client_buf2 = NULL;
	cart_status_copy* selected_client = NULL;
	cart_status_copy* prev_client = NULL;
	int scheduled_id = -1;
	int prev_id = -1;

	ld_msg_pose cur_pose;

	char* msg_buf_w;
	char* msg_buf_r;
	char* cart_c = (char*)cart_cap;
	char* cart_r = (char*)cart_release;
	char* move_f = (char*)move_forward;
	char* move_b = (char*)move_back;

	ld_comm* mobile_robot;
	ld_comm_client* mobile_robot_cmd;

	mobile_robot = new ld_comm(5353, TRUE);
	mobile_robot_cmd = new ld_comm_client("192.168.160.36", 7575, "adept", FALSE);

	if ((err = mobile_robot->ld_connect()) < 0)
	{
		mobile_robot->~ld_comm();
		exit(err);
	}//end if

	if ((err = mobile_robot_cmd->ld_connect()) < 0)
	{
		mobile_robot_cmd->~ld_comm_client();
		exit(err);
	}//end if

	while (1)
	{
			vehicle_server->update_status();
			vehicle_server->print_client_ids();
			vehicle_server->print_client_status();

			//**CODE TO SCHEDULE TASK GOES HERE**
			//prev_id = scheduled_id;
			scheduled_id = vehicle_server->simple_scheduler();
			///////////////////////////////////////

			cout << "SCHEDULED ID: " << scheduled_id << endl;

			if (vehicle_server->get_client_count() > 0 && scheduled_id != -1)
			{

				vehicle_server->send("rcupdate1", scheduled_id);
				vehicle_server->update_status();
				vehicle_server->select_client(scheduled_id, &selected_client);

				cout << "SELECTED CLIENT" << endl;
				cout << "select_client!=NULL: " << (selected_client != NULL) << endl;
				cout << "selected_client->get_status() == 1" << (selected_client->get_status() == 1) << endl;

				if (selected_client != NULL && selected_client->get_status() == 1)
				{
					cout << "CHECKED STATUS" << endl;

					selected_client->get_current_dock(&cart_client_buf);
					selected_client->get_previous_dock(&cart_client_buf2);

					if (cart_client_buf != NULL && cart_client_buf2 != NULL)
					{
						cout << "GOT NEXT DOCK" << endl;
						//CAPTURE CART AT GOAL
						msg_buf_w = (char*)malloc(sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1));

						strcpy_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), "patrolOnce ");
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), cart_client_buf2);
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), "_Route\n");

						msg_buf_r = (char*)malloc(sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1));

						strcpy_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), "Finished patrolling route ");
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), cart_client_buf2);
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), "_Route");

						cout << "msg_buf_w:" << msg_buf_w<<":end"<< endl;
						cout << "msg_buf_r:" << msg_buf_r << ":end" << endl;
						mobile_robot_cmd->arcl_write(msg_buf_w);
						mobile_robot->arcl_read_status(msg_buf_r);

						if (selected_client->get_no_cart() == 0)
						{
							mobile_robot_cmd->arcl_write("doTask cartCapture\n");
							mobile_robot->arcl_read_status(cart_c);
						}//end if

						mobile_robot_cmd->arcl_write("doTask move -1000 150 25 25 30 0 0 0 0 0 1\n");
						mobile_robot->arcl_read_status(move_b);

						free(msg_buf_w);
						free(msg_buf_r);

						//RELEASE CART AT DESTINATION

						msg_buf_w = (char*)malloc(sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1));

						strcpy_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), "patrolOnce ");
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), cart_client_buf);
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), "_Route\n");

						msg_buf_r = (char*)malloc(sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1));

						strcpy_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), "Finished patrolling route ");
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), cart_client_buf);
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), "_Route");

						mobile_robot_cmd->arcl_write(msg_buf_w);
						mobile_robot->arcl_read_status(msg_buf_r);

						
						mobile_robot_cmd->arcl_write("doTask move 1000 150 25 25 30 0 0 0 0 0 1\n");
						mobile_robot->arcl_read_status(move_f);

						free(msg_buf_w);
						free(msg_buf_r);

						//UPDATE STATUS
						mobile_robot->arcl_dsfv_pose(&cur_pose);
						vehicle_server->send("rcupdate2", scheduled_id);
						vehicle_server->update_previous_dock(scheduled_id);
						vehicle_server->update_status(&cur_pose);
						break;
					}//end if
				}//end if
			}//end if
	}//end while

	while (1)
	{
		vehicle_server->update_status();
		vehicle_server->print_client_ids();
		vehicle_server->print_client_status();

		if (prev_client != NULL)
		{
			delete prev_client;
			prev_client = NULL;
		}//end if

		//**CODE TO SCHEDULE TASK GOES HERE**
		if (scheduled_id > 0)
			prev_id = scheduled_id;

		scheduled_id = vehicle_server->simple_scheduler();
		///////////////////////////////////////

		cout << "SCHEDULED ID: " << scheduled_id << endl;

		if (vehicle_server->get_client_count() > 0 && scheduled_id != -1 && prev_id!=-1)
		{

			vehicle_server->send("rcupdate1", scheduled_id);
			vehicle_server->update_status();

			if(selected_client!=NULL)
				prev_client = new cart_status_copy(selected_client);

			vehicle_server->select_client(scheduled_id, &selected_client);

			cout << "SELECTED CLIENT" << endl;
			cout << "select_client!=NULL: " << (selected_client != NULL) << endl;
			cout << "selected_client->get_status() == 1" << (selected_client->get_status() == 1) << endl;

			if (selected_client != NULL && prev_client!=NULL && selected_client->get_status() == 1)
			{
				cout << "CHECKED STATUS" << endl;

				selected_client->get_current_dock(&cart_client_buf);
				selected_client->get_previous_dock(&cart_client_buf2);

				if (cart_client_buf != NULL && cart_client_buf2 != NULL)
				{
					cout << "GOT NEXT DOCK" << endl;

					if (scheduled_id != prev_id)
					{
						//CAPTURE CART AT GOAL
						msg_buf_w = (char*)malloc(sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1));

						strcpy_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), "patrolOnce ");
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), cart_client_buf2);
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf2) + strlen("_Route\n") + 1), "_Route\n");

						msg_buf_r = (char*)malloc(sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1));

						strcpy_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), "Finished patrolling route ");
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), cart_client_buf2);
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf2) + strlen("_Route") + 1), "_Route");

						if (prev_client->get_no_cart() == 0)
						{
							mobile_robot_cmd->arcl_write("doTask cartRelease\n");
							mobile_robot->arcl_read_status(cart_r);
						}//end if

						mobile_robot_cmd->arcl_write("doTask move -1000 150 25 25 30 0 0 0 0 0 1\n");
						mobile_robot->arcl_read_status(move_b);

						cout << "msg_buf_w:" << msg_buf_w << ":end" << endl;
						cout << "msg_buf_r:" << msg_buf_r << ":end" << endl;
						mobile_robot_cmd->arcl_write(msg_buf_w);
						mobile_robot->arcl_read_status(msg_buf_r);

						if (selected_client->get_no_cart() == 0)
						{
							mobile_robot_cmd->arcl_write("doTask cartCapture\n");
							mobile_robot->arcl_read_status(cart_c);
						}//end if

						mobile_robot_cmd->arcl_write("doTask move -1000 150 25 25 30 0 0 0 0 0 1\n");
						mobile_robot->arcl_read_status(move_b);

						free(msg_buf_w);
						free(msg_buf_r);

						//RELEASE CART AT DESTINATION

						msg_buf_w = (char*)malloc(sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1));

						strcpy_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), "patrolOnce ");
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), cart_client_buf);
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), "_Route\n");

						msg_buf_r = (char*)malloc(sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1));

						strcpy_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), "Finished patrolling route ");
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), cart_client_buf);
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), "_Route");

						mobile_robot_cmd->arcl_write(msg_buf_w);
						mobile_robot->arcl_read_status(msg_buf_r);

						mobile_robot_cmd->arcl_write("doTask move 1000 150 25 25 30 0 0 0 1 0 1\n");
						mobile_robot->arcl_read_status(move_f);

						free(msg_buf_w);
						free(msg_buf_r);
					}//end if

					else if (scheduled_id == prev_id)
					{

						mobile_robot_cmd->arcl_write("doTask move -1000 150 25 25 30 0 0 0 0 0 1\n");
						mobile_robot->arcl_read_status(move_b);

						//RELEASE CART AT DESTINATION

						msg_buf_w = (char*)malloc(sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1));

						strcpy_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), "patrolOnce ");
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), cart_client_buf);
						strcat_s(msg_buf_w, sizeof(char)*(strlen("patrolOnce ") + strlen(cart_client_buf) + strlen("_Route\n") + 1), "_Route\n");

						msg_buf_r = (char*)malloc(sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1));

						strcpy_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), "Finished patrolling route ");
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), cart_client_buf);
						strcat_s(msg_buf_r, sizeof(char)*(strlen("Finished patrolling route ") + strlen(cart_client_buf) + strlen("_Route") + 1), "_Route");

						mobile_robot_cmd->arcl_write(msg_buf_w);
						mobile_robot->arcl_read_status(msg_buf_r);

						mobile_robot_cmd->arcl_write("doTask move 1000 150 25 25 30 0 0 0 0 0 1\n");
						mobile_robot->arcl_read_status(move_f);

						free(msg_buf_w);
						free(msg_buf_r);
					}//end else if

					//UPDATE STATUS
					mobile_robot->arcl_dsfv_pose(&cur_pose);
					vehicle_server->update_status(&cur_pose);
					vehicle_server->send("rcupdate2", scheduled_id);
					vehicle_server->update_previous_dock(scheduled_id);
					vehicle_server->update_status(&cur_pose);
				}//end if
			}//end if
		}//end if
	}//end while

}//end cart_dock_test_t

void cart_connect_t(void* args)
{

	cart_comm* vehicle_server = (cart_comm*)args;

	while (1)
	{
		cout <<"CLIENT_COUNT="<< vehicle_server->get_client_count() << endl;
		vehicle_server->cart_connect();
		ulapi_sleep(0.1);
	}//end while

}//end cart_connect_t