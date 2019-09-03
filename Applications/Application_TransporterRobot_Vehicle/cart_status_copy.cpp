/**
*\file cart_status_copy.cpp
*\brief Definitions for data structure storing cart service status information.
*\author Omar Aboul-Enein
*\date 2018-06-14
*/

#include "ulapi.h"
#include "cart_status_copy.h"
#include <stdlib.h>
#include <iostream>

using namespace std;

cart_status_copy::cart_status_copy(ulapi_integer new_client, int new_stat, int no_cart_mode, char* first_dock, char* start_dock)
{
	client_id = new_client;
	status = new_stat;
	no_cart = no_cart_mode;

	current_dock = (char*)malloc(sizeof(char)*(strlen(first_dock)+1));
	strcpy_s(current_dock, sizeof(char)*(strlen(first_dock) + 1), first_dock);

	previous_dock = (char*)malloc(sizeof(char)*(strlen(start_dock) + 1));
	strcpy_s(previous_dock, sizeof(char)*(strlen(start_dock) + 1), start_dock);

	current_pose.robot_x = 0;
	current_pose.robot_y = 0;
	current_pose.robot_th = 0;

}//end constructor

cart_status_copy::cart_status_copy(const cart_status_copy* stat)
{
	client_id = stat->client_id;
	status = stat->status;
	no_cart = stat->no_cart;
	
	current_dock = (char*)malloc(sizeof(char)*(strlen(stat->current_dock) + 1));
	strcpy_s(current_dock, sizeof(char)*(strlen(stat->current_dock) + 1), stat->current_dock);

	previous_dock = (char*)malloc(sizeof(char)*(strlen(stat->previous_dock) + 1));
	strcpy_s(previous_dock, sizeof(char)*(strlen(stat->previous_dock) + 1), stat->previous_dock);

	current_pose.robot_x = stat->current_pose.robot_x;
	current_pose.robot_y = stat->current_pose.robot_y;
	current_pose.robot_th = stat->current_pose.robot_th;
}//end cart_status_copy

cart_status_copy::~cart_status_copy()
{
	if (current_dock != NULL)
	{
		free(current_dock);
		current_dock = NULL;
	}//end if

	if (previous_dock != NULL)
	{
		free(previous_dock);
		previous_dock = NULL;
	}//end if
}//end destructor

void cart_status_copy::update_status_init(char* stat_str)
{
	char* next;
	char* current_str;
	char* start_str;

	if (stat_str != NULL && strlen(stat_str) > 0)
	{
		if (stat_str[0] == '0')
			status = 0;
		else if (stat_str[0] == '1')
			status = 1;
		else if (stat_str[0] == '2')
			status = 2;
		else
			status = -1;

		if (stat_str[1] == '0')
			no_cart = 0;
		else if (stat_str[1] == '1')
			no_cart = 1;
		else
			no_cart = -1;

		start_str = strtok_s(stat_str + 2, ":", &next);
		current_str = strtok_s(NULL, ":\n", &next);

		if (current_dock != NULL)
			free(current_dock);

		previous_dock = (char*)malloc(sizeof(char)*(strlen(start_str) + 1));
		strcpy_s(previous_dock, sizeof(char)*(strlen(start_str) + 1), start_str);

		current_dock = (char*)malloc(sizeof(char)*(strlen(current_str) + 1));
		strcpy_s(current_dock, sizeof(char)*(strlen(current_str) + 1), current_str);
	}//end if

	return;
}//end update_status_init

void cart_status_copy::update_status(char* stat_str)
{

	int count = 0;

	if (stat_str != NULL && strlen(stat_str) > 0)
	{
		if (stat_str[0] == '0')
			status = 0;
		else if (stat_str[0] == '1')
			status = 1;
		else if (stat_str[0] == '2')
			status = 2;
		else
			status = -1;

		if (current_dock != NULL)
			free(current_dock);
		cout << "RECIEVED: '" << stat_str + 1 << "'" << endl;

		current_dock = (char*)malloc(sizeof(char)*strlen(stat_str+1));
		
		for (int i = 1; i < strlen(stat_str + 1); i++)
		{
			current_dock[count] = stat_str[i];
			count++;
		}//end for

		current_dock[count] = '\0';

		cout << "RECIEVED2: '" << current_dock << "'" << endl;
	}//end if

	return;
	
}//end update status

void cart_status_copy::update_pose(ld_comm* vehicle_arcl)
{
	vehicle_arcl->arcl_dsfv_pose(&current_pose);
}//end update_pose

void cart_status_copy::update_pose(ld_msg_pose* pose_msg)
{
	current_pose.robot_x = pose_msg->robot_x;
	current_pose.robot_y = pose_msg->robot_y;
	current_pose.robot_th = pose_msg->robot_th;

	return;
}//end update_pose

ulapi_integer cart_status_copy::get_id()
{
	return client_id;
}//end get_client_id

int cart_status_copy::get_status()
{
	return status;
}//end get_status

int cart_status_copy::get_no_cart()
{
	return no_cart;
}//end get_not_cart

void cart_status_copy::get_current_dock(char** stat_buf)
{
	*stat_buf = (char*)malloc(sizeof(char)*(strlen(current_dock)+1));
	strcpy_s(*stat_buf, sizeof(char)*(strlen(current_dock) + 1), current_dock);
	return;
}//end get_current_status

void cart_status_copy::get_previous_dock(char** stat_buf)
{
	*stat_buf = (char*)malloc(sizeof(char)*(strlen(previous_dock) + 1));
	strcpy_s(*stat_buf, sizeof(char)*(strlen(previous_dock) + 1), previous_dock);
	return;
}//end get_current_status

void cart_status_copy::update_previous_dock()
{
	if (previous_dock != NULL)
	{
		free(previous_dock);
		previous_dock = NULL;
	}//end if

	previous_dock = (char*)malloc(sizeof(char)*(strlen(current_dock) + 1));
	strcpy_s(previous_dock, sizeof(char)*(strlen(current_dock) + 1), current_dock);

	return;
}//end get_current_status

void cart_status_copy::print_status()
{
	cout << "Cart Client Status:" << endl;
	cout << "-------------------" << endl;
	cout << "id: " << client_id << endl;
	cout << "status: " << status << endl;

	if (no_cart == 0)
		cout << "cart mode: present" << endl;
	else if(no_cart == 1)
		cout << "cart mode: absent" << endl;

	cout << "current dock: " <<current_dock<< endl;
	cout << "previous dock: " << previous_dock << endl;
	cout << "current pose (x, y, th): (" << current_pose.robot_x << ", " << current_pose.robot_y << ", " << current_pose.robot_th << ")" << endl;
	
}//end print_status

char* cart_status_copy::pose_to_string()
{
	char* temp_pose = NULL;

	char temp_x[256];
	char temp_y[256];
	char temp_th[256];
	sprintf_s(temp_x, 256, "%f", current_pose.robot_x);
	sprintf_s(temp_y, 256, "%f", current_pose.robot_y);
	sprintf_s(temp_th, 256, "%f", current_pose.robot_th);

	temp_pose = (char*)malloc(sizeof(char)*(strlen(temp_x) + strlen(temp_y) + strlen(temp_th) + 3));
	sprintf_s(temp_pose, sizeof(char)*(strlen(temp_x) + strlen(temp_y) + strlen(temp_th) + 3), "%s,%s,%s", temp_x, temp_y, temp_th);

	return temp_pose;
}//end pose_to_string