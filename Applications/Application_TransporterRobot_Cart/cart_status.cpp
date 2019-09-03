/**
*\file cart_status.cpp
*\brief Interface for data structure used to store and monitor the cart service status.\n
*
*References:\n
*sprintf_s function reference\n
*msdn.microsoft.com/en-us/library/ce3zzk1k.aspx\n
*memory access exception messages:\n
*https://stackoverflow.com/questions/19203604/heap-corruption-detected-after-normal-block \n
*https://stackoverflow.com/questions/38072605/crtisvalidheappointer-error-when-trying-to-free-an-element-of-a-linked-list \n
*About safe string functions:\n
*https://stackoverflow.com/questions/11752705/does-stdstring-contain-null-terminator \n
*https://stackoverflow.com/questions/19196301/buffer-is-too-small-when-doing-strcpy-s \n
*docs.microsoft.com/en-us/cpp/c-runtime-library/reference/strcpy-s-wcscpy-s-mbscpy-s \n
*strtok function reference:\n
*www.cpluplus.com/reference/cstring/strtok \n
*msdn.microsoft.com/en-us/library/ftsafwz3.aspx \n
*
*\author Omar Aboul-Enein
*\date 2017-06-05
*/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "cart_status.h"
#include "posemath.h"
#include "ur5_control.h"

using namespace std;

cart_status::cart_status(int stat, int no_cart_mode, char* start_str, int* no_arm_list, char** task_list, ld_msg_pose** goal_list, PM_CARTESIAN** large_point1_list, PM_CARTESIAN** large_point2_list, PM_CARTESIAN** edge_start_list, int len, int num_runs)
{

	status = stat;

	no_cart = no_cart_mode;

	start_dock = (char*)malloc(sizeof(char)*strlen(start_str) + 1);
	strcpy_s(start_dock, sizeof(char)*strlen(start_str) + 1, start_str);

	current_no_arm = no_arm_list[0];
	
	current_dock = (char*)malloc(sizeof(char)*(strlen(task_list[0])+1));
	strcpy_s(current_dock, sizeof(char)*(strlen(task_list[0]) + 1), task_list[0]);

	current_goal = new PM_POSE(PM_CARTESIAN(goal_list[0]->robot_x, goal_list[0]->robot_y, 0), PM_RPY(0, 0, goal_list[0]->robot_th * TO_RAD));
	current_large_point1 = new PM_CARTESIAN(large_point1_list[0]->x, large_point1_list[0]->y, large_point1_list[0]->z);
	current_large_point2 = new PM_CARTESIAN(large_point2_list[0]->x, large_point2_list[0]->y, large_point2_list[0]->z);
	current_edge_start = new PM_CARTESIAN(edge_start_list[0]->x, edge_start_list[0]->y, edge_start_list[0]->z);

	no_arm_buf = new int[len];

	for (int i = 0; i < len; i++)
		no_arm_buf[i] = no_arm_list[i];

	dock_buf = (char**)malloc(sizeof(char*)*len);
	goal_buf = (ld_msg_pose**)malloc(sizeof(char*)*len);
	large_point1_buf = (PM_CARTESIAN**)malloc(sizeof(char*)*len);
	large_point2_buf = (PM_CARTESIAN**)malloc(sizeof(char*)*len);
	edge_start_buf = (PM_CARTESIAN**)malloc(sizeof(char*)*len);
	
	for (int i = 0; i < len; i++)
	{
		dock_buf[i] = (char*)malloc(sizeof(char)*strlen(task_list[i])+1);
		strcpy_s(dock_buf[i], sizeof(char)*strlen(task_list[i])+1, task_list[i]);
	}//end for

	for (int i = 0; i < len; i++)
	{
		goal_buf[i] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
		goal_buf[i]->robot_th = goal_list[i]->robot_th;
		goal_buf[i]->robot_x = goal_list[i]->robot_x;
		goal_buf[i]->robot_y = goal_list[i]->robot_y;
	}//end for

	for (int i = 0; i < len; i++)
	{
		large_point1_buf[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
		large_point1_buf[i]->x = large_point1_list[i]->x;
		large_point1_buf[i]->y = large_point1_list[i]->y;
		large_point1_buf[i]->z = large_point1_list[i]->z;
	}//end for

	for (int i = 0; i < len; i++)
	{
		large_point2_buf[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
		large_point2_buf[i]->x = large_point2_list[i]->x;
		large_point2_buf[i]->y = large_point2_list[i]->y;
		large_point2_buf[i]->z = large_point2_list[i]->z;
	}//end for

	for (int i = 0; i < len; i++)
	{
		edge_start_buf[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
		edge_start_buf[i]->x = edge_start_list[i]->x;
		edge_start_buf[i]->y = edge_start_list[i]->y;
		edge_start_buf[i]->z = edge_start_list[i]->z;
	}//end for

	dock_count = len;
	runs = num_runs;
	progress = 0;

	cout << "Constructor: " << status << "," << current_dock << endl;
}//end constructor

cart_status::~cart_status()
{
	if(current_dock != NULL)
		free(current_dock);

	for (int i = 0; i < dock_count; i++)
	{
		if(dock_buf[i]!=NULL)
			free(dock_buf);//check this line!!
		dock_buf[i] = NULL;
	}//end for

	if (no_arm_buf != NULL)
		delete[] no_arm_buf;

	if(dock_buf!=NULL)
		free(dock_buf);

	if(current_goal != NULL)
		delete current_goal;

	if(current_large_point1!= NULL)
		delete current_large_point1;

	if(current_large_point2 != NULL)
		delete current_large_point2;

	if (current_edge_start != NULL)
		delete current_edge_start;

	if(goal_buf != NULL)
		delete[] goal_buf;

	if(large_point1_buf!= NULL)
		delete[] large_point1_buf;

	if(large_point2_buf != NULL)
		delete[] large_point2_buf;

	if (edge_start_buf != NULL)
		delete[] edge_start_buf;

	current_goal = NULL;
	current_large_point1 = NULL;
	current_large_point2 = NULL;
	current_edge_start = NULL;
	no_arm_buf = NULL;
	dock_buf = NULL;
	goal_buf = NULL;
	large_point1_buf = NULL;
	large_point2_buf = NULL;
	edge_start_buf = NULL;

}//end destructor

void cart_status::set_status(int new_stat)
{
	if (status == 0 || status == 1 || status == 2)
		status = new_stat;
}//end set_status

void cart_status::update_current_dock()
{
	char* temp_dock;
	ld_msg_pose temp_goal;
	PM_CARTESIAN temp_large1;
	PM_CARTESIAN temp_large2;
	PM_CARTESIAN temp_edge;
	int temp_no_arm;

	if (dock_count > 1)
	{
		cout << "Updating current docking task..." << endl;

		if (runs > 0)
		{
			temp_dock = (char*)malloc(sizeof(char)*(strlen(current_dock) + 1));
			strcpy_s(temp_dock, sizeof(char)*(strlen(current_dock) + 1), current_dock);
			
			temp_goal.robot_x = goal_buf[0]->robot_x;
			temp_goal.robot_y = goal_buf[0]->robot_y;
			temp_goal.robot_th = goal_buf[0]->robot_th;

			temp_large1.x = current_large_point1->x;
			temp_large1.y = current_large_point1->y;
			temp_large1.z = current_large_point1->z;

			temp_large2.x = current_large_point2->x;
			temp_large2.y = current_large_point2->y;
			temp_large2.z = current_large_point2->z;

			temp_edge.x = current_edge_start->x;
			temp_edge.y = current_edge_start->y;
			temp_edge.z = current_edge_start->z;

			temp_no_arm = current_no_arm;
		}//end if

		//https://stackoverflow.com/questions/38072605/crtisvalidheappointer-error-when-trying-to-free-an-element-of-a-linked-list
		if (current_dock != NULL)
		{
			free(current_dock);
			current_dock = NULL;
		}//end current task

		if (dock_buf[0] != NULL)
		{
			free(dock_buf[0]);
			dock_buf[0] = NULL;
		}//end if

		if (current_goal != NULL)
		{
			delete current_goal;
			current_goal = NULL;
		}//end current task

		if (goal_buf[0] != NULL)
		{
			free(goal_buf[0]);
			goal_buf[0] = NULL;
		}//end if

		if (current_large_point1 != NULL)
		{
			delete current_large_point1;
			current_large_point1 = NULL;
		}//end current task

		if (large_point1_buf[0] != NULL)
		{
			free(large_point1_buf[0]);
			large_point1_buf[0] = NULL;
		}//end if

		if (current_large_point2 != NULL)
		{
			delete current_large_point2;
			current_large_point2 = NULL;
		}//end current task

		if (large_point2_buf[0] != NULL)
		{
			free(large_point2_buf[0]);
			large_point2_buf[0] = NULL;
		}//end if

		if (current_edge_start != NULL)
		{
			delete current_edge_start;
			current_edge_start = NULL;
		}//end current task

		if (edge_start_buf[0] != NULL)
		{
			free(edge_start_buf[0]);
			edge_start_buf[0] = NULL;
		}//end if

		//https://stackoverflow.com/questions/19203604/heap-corruption-detected-after-normal-block
		current_dock = (char*)malloc(sizeof(char)*(strlen(dock_buf[1])+1));
		strcpy_s(current_dock, sizeof(char)*(strlen(dock_buf[1]) + 1), dock_buf[1]);

		current_no_arm = no_arm_buf[1];
		current_goal = new PM_POSE(PM_CARTESIAN(goal_buf[1]->robot_x, goal_buf[1]->robot_y, 0), PM_RPY(0, 0, goal_buf[1]->robot_th * TO_RAD));
		current_large_point1 = new PM_CARTESIAN(large_point1_buf[1]->x, large_point1_buf[1]->y, large_point1_buf[1]->z);
		current_large_point2 = new PM_CARTESIAN(large_point2_buf[1]->x, large_point2_buf[1]->y, large_point2_buf[1]->z);
		current_edge_start = new PM_CARTESIAN(edge_start_buf[1]->x, edge_start_buf[1]->y, edge_start_buf[1]->z);

		for (int i = 0; i < dock_count - 1; i++)
		{
			no_arm_buf[i] = no_arm_buf[i + 1];

			dock_buf[i] = (char*)malloc(sizeof(char)*(strlen(dock_buf[i + 1]) + 1));
			strcpy_s(dock_buf[i], sizeof(char)*(strlen(dock_buf[i + 1]) + 1), dock_buf[i + 1]);
			free(dock_buf[i + 1]);
			dock_buf[i + 1] = NULL;

			goal_buf[i] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_buf[i]->robot_x = goal_buf[i + 1]->robot_x;
			goal_buf[i]->robot_y = goal_buf[i + 1]->robot_y;
			goal_buf[i]->robot_th = goal_buf[i + 1]->robot_th;
			free(goal_buf[i + 1]);
			goal_buf[i + 1] = NULL;

			large_point1_buf[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			large_point1_buf[i]->x = large_point1_buf[i + 1]->x;
			large_point1_buf[i]->y = large_point1_buf[i + 1]->y;
			large_point1_buf[i]->z = large_point1_buf[i + 1]->z;
			free(large_point1_buf[i + 1]);
			//new PM_CARTESIAN(large_point1_buf[i + 1]->x, large_point1_buf[i + 1]->y, large_point1_buf[i + 1]->z);
			//delete large_point1_buf[i + 1];
			large_point1_buf[i + 1] = NULL;

			large_point2_buf[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			large_point2_buf[i]->x = large_point2_buf[i + 1]->x;
			large_point2_buf[i]->y = large_point2_buf[i + 1]->y;
			large_point2_buf[i]->z = large_point2_buf[i + 1]->z;
			free(large_point2_buf[i + 1]);
			//new PM_CARTESIAN(large_point2_buf[i + 1]->x, large_point2_buf[i + 1]->y, large_point2_buf[i + 1]->z);
			//delete large_point2_buf[i + 1];
			large_point2_buf[i + 1] = NULL;

			edge_start_buf[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			edge_start_buf[i]->x = edge_start_buf[i + 1]->x;
			edge_start_buf[i]->y = edge_start_buf[i + 1]->y;
			edge_start_buf[i]->z = edge_start_buf[i + 1]->z;
			free(edge_start_buf[i + 1]);
			//new PM_CARTESIAN(edge_start_buf[i + 1]->x, edge_start_buf[i + 1]->y, edge_start_buf[i + 1]->z);
			//delete edge_start_buf[i + 1];
			edge_start_buf[i + 1] = NULL;

		}//end for

		if (runs > 0)
		{
			dock_buf[dock_count - 1] = (char*)malloc(sizeof(char)*(strlen(temp_dock) + 1));
			strcpy_s(dock_buf[dock_count-1], sizeof(char)*(strlen(temp_dock) + 1), temp_dock);

			goal_buf[dock_count - 1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_buf[dock_count - 1]->robot_x = temp_goal.robot_x;
			goal_buf[dock_count - 1]->robot_y = temp_goal.robot_y;
			goal_buf[dock_count - 1]->robot_th = temp_goal.robot_th;

			large_point1_buf[dock_count - 1] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			large_point1_buf[dock_count - 1]->x = temp_large1.x;
			large_point1_buf[dock_count - 1]->y = temp_large1.y;
			large_point1_buf[dock_count - 1]->z = temp_large1.z;

			large_point2_buf[dock_count - 1] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			large_point2_buf[dock_count - 1]->x = temp_large2.x;
			large_point2_buf[dock_count - 1]->y = temp_large2.y;
			large_point2_buf[dock_count - 1]->z = temp_large2.z;

			edge_start_buf[dock_count - 1] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			edge_start_buf[dock_count - 1]->x = temp_edge.x;
			edge_start_buf[dock_count - 1]->y = temp_edge.y;
			edge_start_buf[dock_count - 1]->z = temp_edge.z;

			no_arm_buf[dock_count - 1] = temp_no_arm;

			if (progress >= dock_count-1)
			{
				runs--;
				progress = 0;
			}//end if

			else
				progress++;

		}//end if
		

		else
			dock_count--;

		cout << "Printing docking task lists" << endl;

		print_dock_list();
		print_goal_list();
		print_large_point1_list();
		print_large_point2_list();
		print_edge_start_list();

	}//end if

	else
	{
		cout << "Updating to last docking task..." << endl;
		if (current_dock != NULL)
		{
			free(current_dock);
			current_dock = NULL;
		}//end current task

		if (dock_buf[0] != NULL)
		{
			free(dock_buf[0]);
			dock_buf[0] = NULL;
		}//end if

		if (current_goal != NULL)
		{
			delete current_goal;
			current_goal = NULL;
		}//end current task

		if (goal_buf[0] != NULL)
		{
			free(goal_buf[0]);
			goal_buf[0] = NULL;
		}//end if

		if (current_large_point1 != NULL)
		{
			delete current_large_point1;
			current_large_point1 = NULL;
		}//end current task

		if (large_point1_buf[0] != NULL)
		{
			free(large_point1_buf[0]);
			large_point1_buf[0] = NULL;
		}//end if

		if (current_large_point2 != NULL)
		{
			delete current_large_point2;
			current_large_point2 = NULL;
		}//end current task

		if (large_point2_buf[0] != NULL)
		{
			free(large_point2_buf[0]);
			large_point2_buf[0] = NULL;
		}//end if

		if (current_edge_start != NULL)
		{
			delete current_edge_start;
			current_edge_start = NULL;
		}//end current task

		if (edge_start_buf[0] != NULL)
		{
			free(edge_start_buf[0]);
			edge_start_buf[0] = NULL;
		}//end if

		dock_count--;

	}//end else

	return;

}//end update_current_dock

void cart_status::print_dock_list()
{
	for (int i = 0; i < dock_count-1; i++)
		cout << dock_buf[i] << ", ";

	cout << dock_buf[dock_count - 1] << endl;

	return;
}//end print_task_list

void cart_status::print_no_arm_list()
{
	for (int i = 0; i < dock_count - 1; i++)
		cout << no_arm_buf[i] << ", ";

	cout << no_arm_buf[dock_count - 1] << endl;

	return;
}//end print_task_list

void cart_status::print_goal_list()
{
	for (int i = 0; i < dock_count - 1; i++)
		cout << "(" << goal_buf[i]->robot_x << ", " << goal_buf[i]->robot_y << ", " << goal_buf[i]->robot_th << "), ";

	cout << "(" << goal_buf[dock_count - 1]->robot_x << ", " << goal_buf[dock_count - 1]->robot_y << ", " << goal_buf[dock_count - 1]->robot_th << ")" << endl;

	return;
}//end print_task_list

void cart_status::print_large_point1_list()
{
	for (int i = 0; i < dock_count - 1; i++)
		cout <<"("<< large_point1_buf[i]->x << ", "<<large_point1_buf[i]->y << ", "<<large_point1_buf[i]->z << "), ";

	cout << "(" << large_point1_buf[dock_count - 1]->x << ", " << large_point1_buf[dock_count - 1]->y << ", " << large_point1_buf[dock_count - 1]->z << ")" << endl;

	return;
}//end print_task_list

void cart_status::print_large_point2_list()
{
	for (int i = 0; i < dock_count - 1; i++)
		cout << "(" << large_point2_buf[i]->x << ", " << large_point2_buf[i]->y << ", " << large_point2_buf[i]->z << "), ";

	cout << "(" << large_point2_buf[dock_count - 1]->x << ", " << large_point2_buf[dock_count - 1]->y << ", " << large_point2_buf[dock_count - 1]->z << ")" << endl;

	return;
}//end print_task_list

void cart_status::print_edge_start_list()
{
	for (int i = 0; i < dock_count - 1; i++)
		cout << "(" << edge_start_buf[i]->x << ", " << edge_start_buf[i]->y << ", " << edge_start_buf[i]->z << "), ";

	cout << "(" << edge_start_buf[dock_count - 1]->x << ", " << edge_start_buf[dock_count - 1]->y << ", " << edge_start_buf[dock_count - 1]->z << ")" << endl;

	return;
}//end print_task_list

char* cart_status::to_string_init()
{
	char* stat_string = NULL;

	if (current_dock != NULL)
	{
		stat_string = (char*)malloc(sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5));

		if (status == 0)
			strcpy_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), "0");
		else if (status == 1)
			strcpy_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), "1");
		else if (status == 2)
			strcpy_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), "2");

		if (no_cart == 0)
			strcat_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), "0");
		else if (no_cart == 1)
			strcat_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), "1");

		strcat_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), start_dock);
		strcat_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), ":");
		strcat_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), current_dock);
		strcat_s(stat_string, sizeof(char)*(strlen(start_dock) + strlen(current_dock) + 5), "\n");
	}//end to_string

	return stat_string;
}//end to string

char* cart_status::to_string()
{
	char* stat_string = NULL;

	if (current_dock != NULL)
	{
		stat_string = (char*)malloc(sizeof(char)*(strlen(current_dock) + 3));

		if (status == 0)
			strcpy_s(stat_string, sizeof(char)*(strlen(current_dock) + 3), "0");
		else if (status == 1)
			strcpy_s(stat_string, sizeof(char)*(strlen(current_dock) + 3), "1");
		else if (status == 2)
			strcpy_s(stat_string, sizeof(char)*(strlen(current_dock) + 3), "2");

		strcat_s(stat_string, sizeof(char)*(strlen(current_dock) + 3), current_dock);
		strcat_s(stat_string, sizeof(char)*(strlen(current_dock) + 3), "\n");
	}//end to_string

	return stat_string;
}//end to string

int cart_status::get_status()
{
	return status;
}//end get_status

void cart_status::string_to_pose(char* pose_msg)
{
	char* next;
	char* x_str;
	char* y_str;
	char* th_str;

	if (pose_msg != NULL)
	{

		x_str = strtok_s(pose_msg, ",", &next);
		y_str = strtok_s(NULL, ",", &next);
		th_str = strtok_s(NULL, ",", &next);

		cout << "x=" << x_str << endl;
		cout << "y=" << y_str << endl;
		cout << "th=" << th_str << endl;

		current_pose.robot_x = atof(x_str);
		current_pose.robot_y = atof(y_str);
		current_pose.robot_th = atof(th_str);
	}//end if

	return;

}//end string_to_pose

int cart_status::get_dock_count()
{
	return dock_count;
}//end get_dock_count

void cart_status::get_current_pose(ld_msg_pose& copy_pose)
{
	copy_pose.robot_x = current_pose.robot_x;
	copy_pose.robot_y = current_pose.robot_y;
	copy_pose.robot_th = current_pose.robot_th;

	return;
}//end get_current_pose

int cart_status::get_no_cart()
{
	return no_cart;
}//end get_current_dock

int cart_status::get_current_no_arm()
{
	return current_no_arm;
}//end get_current_dock

char* cart_status::get_current_dock()
{
	char* temp;
	temp = (char*)malloc(sizeof(char)*strlen(current_dock)+1);
	strcpy_s(temp, sizeof(char)*strlen(current_dock)+1, current_dock);

	return temp;
}//end get_current_dock

int cart_status::get_current_goal(PM_POSE& copy_pose)
{

	if (current_goal != NULL)
	{
		copy_pose.rot = current_goal->rot;
		copy_pose.tran = current_goal->tran;
		return 0;
	}//end if

	return -1;
}//end get_current_dock

int cart_status::get_current_large_point1(PM_CARTESIAN& copy_point)
{

	if (current_large_point1 != NULL)
	{
		copy_point.x = current_large_point1->x;
		copy_point.y = current_large_point1->y;
		copy_point.z = current_large_point1->z;
		return 0;
	}//end if

	return -1;
}//end get_current_dock

int cart_status::get_current_large_point2(PM_CARTESIAN& copy_point)
{
	if (current_large_point2 != NULL)
	{
		copy_point.x = current_large_point2->x;
		copy_point.y = current_large_point2->y;
		copy_point.z = current_large_point2->z;
		return 0;
	}//end if

	return -1;
}//end get_current_dock

int cart_status::get_current_edge_start(PM_CARTESIAN& copy_point)
{
	if (current_edge_start != NULL)
	{
		copy_point.x = current_edge_start->x;
		copy_point.y = current_edge_start->y;
		copy_point.z = current_edge_start->z;
		return 0;
	}//end if

	return -1;
}//end get_current_dock

void cart_status::print_status()
{
	cout << "Cart Status" << endl;
	cout << "-----------" << endl;
	cout << "Runs remaining: " << runs << endl;
	cout << "status: " << status << endl;
	cout << "current no arm: " << current_no_arm << endl;
	cout << "current dock: " << current_dock << endl;
	cout << "current goal: (" << current_goal->tran.x << ", " << current_goal->tran.y << ")" << endl;
	cout << "current large point 1: (" << current_large_point1->x << ", " << current_large_point1->y << ", " << current_large_point1->z << ")" << endl;
	cout << "current large point 2: (" << current_large_point2->x << ", " << current_large_point2->y << ", " << current_large_point2->z << ")" << endl;
	cout << "current edge start: (" << current_edge_start->x << ", " << current_edge_start->y << ", " << current_edge_start->z << ")" << endl;
	cout << "no arm buf: "; print_no_arm_list();
	cout << "dock buf: "; print_dock_list();
	cout << "goal buf: "; print_goal_list();
	cout << "large point 1 buf: "; print_large_point1_list();
	cout << "large point 2 buf: "; print_large_point2_list();
	cout << "edge start buf: "; print_edge_start_list();
	cout << "current pose in vehicle map: " << "( " << current_pose.robot_x << ", " << current_pose.robot_y << ", " << current_pose.robot_th << ")" << endl;
}//end print_status