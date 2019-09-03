/**
*\file cart_config.cpp
*\brief This file provides the implementation for a configuration file used for saving and loading test parameters (like initial reflector search positions, no arm mode, etc.)
*\author Omar Aboul-Enein
*\date 2018-09-10
*/

#include "cart_config.h"
#include <iostream>
using namespace std;

int check_num(char* c_num)
{

	int check = 0;

	for (int i = 0; i < strlen(c_num); i++)
	{
		if (!isdigit(c_num[i]) && c_num[i] != '.' && c_num[i] != '-')
			check = 1;
	}//end for

	return check;
}//end check num

config_data::config_data()
{
	task_count = 0;
	runs = 0;

	start = NULL;
	dock_list = NULL;
	goal_list = NULL;
	large_point1_list = NULL;
	large_point2_list = NULL;
	edge_start_list = NULL;
	no_arm_list = NULL;
	sum1 = NULL;
	sum2 = NULL;
	update_count = NULL;

}//end config_data

config_data::~config_data()
{
	if (start != NULL)
	{
		free(start);
		start = NULL;
	}//end if

	if (dock_list != NULL)
	{
		for (int i = 0; i < task_count; i++)
		{

			if (dock_list[i] != NULL)
			{
				free(dock_list[i]);
				dock_list[i] = NULL;
			}//end if

		}//end for

		free(dock_list);
		dock_list = NULL;
	}//end if

	if (goal_list != NULL)
	{
		for (int i = 0; i < task_count; i++)
		{

			if (goal_list[i] != NULL)
			{
				free(goal_list[i]);
				goal_list[i] = NULL;
			}//end if

		}//end for

		free(goal_list);
		goal_list = NULL;
	}//end if

	if (large_point1_list != NULL)
	{
		for (int i = 0; i < task_count; i++)
		{

			if (large_point1_list[i] != NULL)
			{
				free(large_point1_list[i]);
				large_point1_list[i] = NULL;
			}//end if

		}//end for

		free(large_point1_list);
		large_point1_list = NULL;
	}//end if

	if (large_point2_list != NULL)
	{
		for (int i = 0; i < task_count; i++)
		{

			if (large_point2_list[i] != NULL)
			{
				free(large_point2_list[i]);
				large_point2_list[i] = NULL;
			}//end if

		}//end for

		free(large_point2_list);
		large_point2_list = NULL;
	}//end if

	if (edge_start_list != NULL)
	{
		for (int i = 0; i < task_count; i++)
		{

			if (edge_start_list[i] != NULL)
			{
				free(edge_start_list[i]);
				edge_start_list[i] = NULL;
			}//end if

		}//end for

		free(edge_start_list);
		edge_start_list = NULL;
	}//end if

	if (no_arm_list != NULL)
	{
		free(no_arm_list);
		no_arm_list = NULL;
	}//end if

	if (sum1!= NULL)
	{
		for (int i = 0; i < task_count; i++)
		{

			if (sum1[i] != NULL)
			{
				free(sum1[i]);
				sum1[i] = NULL;
			}//end if

		}//end for

		free(sum1);
		sum1 = NULL;
	}//end if

	if (sum2 != NULL)
	{
		for (int i = 0; i < task_count; i++)
		{

			if (sum2[i] != NULL)
			{
				free(sum2[i]);
				sum2[i] = NULL;
			}//end if

		}//end for

		free(sum2);
		sum2 = NULL;
	}//end if

	if (update_count != NULL)
	{
		free(update_count);
		update_count= NULL;
	}//end if

}//end destructor

void config_data::print_data()
{

	cout << "Configuration Copy: " << endl;
	cout << "Runs: " << runs << endl;
	cout << "Task count: " << task_count << endl;
	cout << "Start: " << start << endl;

	cout << "Dock list: ";

	for (int i = 0; i < task_count; i++)
	{
		cout << dock_list[i];

		if (i < task_count - 1)
			cout << ", ";
	}//end for

	cout << endl;

	cout << "Goal list: ";

	for (int i = 0; i < task_count; i++)
	{
		cout << "(" << goal_list[i]->robot_x << ", " << goal_list[i]->robot_y << ", " << goal_list[i]->robot_th << ")";

		if (i < task_count - 1)
			cout << ", ";
	}//end for

	cout << endl;

	cout << "Large Point 1 List: ";

	for (int i = 0; i < task_count; i++)
	{
		cout << "(" << large_point1_list[i]->x << ", " << large_point1_list[i]->y << ", " << large_point1_list[i]->z << ")";

		if (i < task_count - 1)
			cout << ", ";
	}//end for

	cout << endl;

	cout << "Large Point 2 list: ";

	for (int i = 0; i < task_count; i++)
	{
		cout << "(" << large_point2_list[i]->x << ", " << large_point2_list[i]->y << ", " << large_point2_list[i]->z << ")";

		if (i < task_count - 1)
			cout << ", ";
	}//end for

	cout << endl;

	cout << "Edge Start list: ";

	for (int i = 0; i < task_count; i++)
	{
		cout << "(" << edge_start_list[i]->x << ", " << edge_start_list[i]->y << ", " << edge_start_list[i]->z << ")";

		if (i < task_count - 1)
			cout << ", ";
	}//end for

	cout << endl;

	cout << "No arm list: ";

	for (int i = 0; i < task_count; i++)
	{
		cout << no_arm_list[i];

		if (i < task_count - 1)
			cout << ", ";
	}//end for

	cout << endl;

}//end print_file


cart_config::cart_config(int select_path)
{
	ifstream* get_path;
	char temp[128];

	runs = -1;
	task_count = -1;

	if (select_path == 0)
		get_path = new ifstream("..\\Config\\config_path.txt");

	else
		get_path = new ifstream("..\\Config\\config_path2.txt");

	if (get_path->is_open())
	{

		get_path->getline(temp, 128);

		file_name = (char*)malloc(sizeof(char)*strlen(temp) + 1);
		strcpy_s(file_name, sizeof(char)*strlen(temp) + 1, temp);

		config_file = new fstream(file_name, fstream::in | fstream::out);

		get_path->close();

		if (get_path != NULL)
		{
			delete get_path;
			get_path = NULL;
		}//end if

		if(config_file->is_open())
		{
			config_file->getline(temp, 128);

			config_file->getline(temp, 128);
			start = (char*)malloc(sizeof(char)*strlen(temp) + 1);
			strcpy_s(start, sizeof(char)*strlen(temp) + 1, temp);

			//Read number of runs in file
			config_file->getline(temp, 128);
			runs = atoi(temp);

			//Read number of task units in file
			config_file->getline(temp, 128);
			task_count = atoi(temp);

			first_task = config_file->tellg();

			dock_list = (char**)malloc(sizeof(char*)*task_count);
			goal_list = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*)*task_count);
			large_point1_list = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
			large_point2_list = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
			edge_start_list = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
			no_arm_list = new int[task_count];

			sum1 = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
			sum2 = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
			update_count = new int[task_count];

			for (int i = 0; i < task_count; i++)
			{
				dock_list[i] = NULL;
				goal_list[i] = NULL;
				large_point1_list[i] = NULL;
				large_point2_list[i] = NULL;
				edge_start_list[i] = NULL;

				sum1[i] = NULL;
				sum2[i] = NULL;
			}//end for

			err = 0;
		}//end if

		else
		{

			cout << "Config File Error: Could not open file at location " << file_name << endl;
			err = 1;

			if (config_file != NULL)
			{
				delete config_file;
				config_file = NULL;
			}//end if

			start = NULL;
			dock_list = NULL;
			goal_list = NULL;
			large_point1_list = NULL;
			large_point2_list = NULL;
			edge_start_list = NULL;
			no_arm_list = NULL;

			if (file_name != NULL)
			{
				free(file_name);
				file_name = NULL;
			}//end if
			
		}//end else
	}//end if
	else
	{

		cout << "Config File Error: Could not open file at location .\\Config\\config_path.txt" << endl;
		err = 1;

		config_file = NULL;
		start = NULL;
		dock_list = NULL;
		goal_list = NULL;
		large_point1_list = NULL;
		large_point2_list = NULL;
		edge_start_list = NULL;
		no_arm_list = NULL;
		file_name = NULL;
		
	}//end else

}//end constructor

cart_config::~cart_config()
{
	if (err != 1)
	{
		config_file->close();

		if (file_name != NULL)
		{
			free(file_name);
			file_name = NULL;
		}//end if

		if (start != NULL)
		{
			free(start);
			start = NULL;
		}//end if

		if (dock_list != NULL)
		{
			for (int i = 0; i < task_count; i++)
			{

				if (dock_list[i] != NULL)
				{
					free(dock_list[i]);
					dock_list[i] = NULL;
				}//end if

			}//end for

			free(dock_list);
			dock_list = NULL;
		}//end if

		if (goal_list != NULL)
		{
			for (int i = 0; i < task_count; i++)
			{

				if (goal_list[i] != NULL)
				{
					free(goal_list[i]);
					goal_list[i] = NULL;
				}//end if

			}//end for

			free(goal_list);
			goal_list = NULL;
		}//end if

		if (large_point1_list != NULL)
		{
			for (int i = 0; i < task_count; i++)
			{

				if (large_point1_list[i] != NULL)
				{
					free(large_point1_list[i]);
					large_point1_list[i] = NULL;
				}//end if

			}//end for

			free(large_point1_list);
			large_point1_list = NULL;
		}//end if

		if (large_point2_list != NULL)
		{
			for (int i = 0; i < task_count; i++)
			{

				if (large_point2_list[i] != NULL)
				{
					free(large_point2_list[i]);
					large_point2_list[i] = NULL;
				}//end if

			}//end for

			free(large_point2_list);
			large_point2_list = NULL;
		}//end if

		if (edge_start_list != NULL)
		{
			for (int i = 0; i < task_count; i++)
			{

				if (edge_start_list[i] != NULL)
				{
					free(edge_start_list[i]);
					edge_start_list[i] = NULL;
				}//end if

			}//end for

			free(edge_start_list);
			edge_start_list = NULL;
		}//end if

		if (no_arm_list != NULL)
		{
			free(no_arm_list);
			no_arm_list = NULL;
		}//end if

		if (sum1 != NULL)
		{
			for (int i = 0; i < task_count; i++)
			{

				if (sum1[i] != NULL)
				{
					free(sum1[i]);
					sum1[i] = NULL;
				}//end if

			}//end for

			free(sum1);
			sum1 = NULL;
		}//end if

		if (sum2 != NULL)
		{
			for (int i = 0; i < task_count; i++)
			{

				if (sum2[i] != NULL)
				{
					free(sum2[i]);
					sum2[i] = NULL;
				}//end if

			}//end for

			free(sum2);
			sum2 = NULL;
		}//end if

		if (update_count != NULL)
		{
			free(update_count);
			update_count = NULL;
		}//end if
	}//end if

}//end destructor

void cart_config::clear_data()
{
	if (err == 0)
	{

		for (int i = 0; i < task_count; i++)
		{

			if (dock_list[i] != NULL)
			{
				free(dock_list[i]);
				dock_list[i] = NULL;
			}//end if

		}//end for

		for (int i = 0; i < task_count; i++)
		{

			if (goal_list[i] != NULL)
			{
				free(goal_list[i]);
				goal_list[i] = NULL;
			}//end if

		}//end for

		for (int i = 0; i < task_count; i++)
		{

			if (large_point1_list[i] != NULL)
			{
				free(large_point1_list[i]);
				large_point1_list[i] = NULL;
			}//end if

		}//end for

		for (int i = 0; i < task_count; i++)
		{

			if (large_point2_list[i] != NULL)
			{
				free(large_point2_list[i]);
				large_point2_list[i] = NULL;
			}//end if

		}//end for

		for (int i = 0; i < task_count; i++)
		{

			if (edge_start_list[i] != NULL)
			{
				free(edge_start_list[i]);
				edge_start_list[i] = NULL;
			}//end if

		}//end for

	}//end if

}//end clear_data

void cart_config::read_file()
{

	char temp[128];
	char* next;
	char* tok_dock = NULL;
	char* tok_th = NULL;
	char* tok_x = NULL;
	char* tok_y = NULL;
	char* tok_no = NULL;

	clear_data();

	if (err == 0)
	{
		config_file->seekg(first_task);

		for (int i = 0; i < task_count; i++)
		{

			config_file->getline(temp, 128);

			for (int j = 0; j < 9; j++)
			{
				config_file->getline(temp, 128);

				if (j == 0)
				{
					if (tok_dock != NULL)
						tok_dock = NULL;

					tok_dock = strtok_s(temp, "\t", &next);
					dock_list[i] = (char*)malloc(sizeof(char)*strlen(tok_dock) + 1);
					strcpy_s(dock_list[i], sizeof(char)*strlen(tok_dock) + 1, tok_dock);
				}//end if

				else if (j == 1)
				{
					goal_list[i] = new ld_msg_pose;

					if (tok_th != NULL)
						tok_th = NULL;

					if (tok_x != NULL)
						tok_x = NULL;

					if (tok_y != NULL)
						tok_y = NULL;

					tok_th = strtok_s(temp, "\t, ", &next);
					tok_x = strtok_s(NULL, "\t, ", &next);
					tok_y = strtok_s(NULL, "\t, ", &next);

					if (check_num(tok_th) == 0 && check_num(tok_x) == 0 && check_num(tok_y) == 0)
					{
						goal_list[i]->robot_th = atof(tok_th);
						goal_list[i]->robot_x = atof(tok_x) + 1000 * cos(goal_list[i]->robot_th * TO_RAD);
						goal_list[i]->robot_y = atof(tok_y) + 1000 * sin(goal_list[i]->robot_th * TO_RAD);
					}//end if

					else
					{
						cout << "Config File Error: File format is incorrect. Detected non-numerical characters when a number was expected. Please check the configuration file." << endl;
						err = 2;
					}//end else if

				}//end else if

				else if (j == 2)
				{

					if (tok_x != NULL)
						tok_x = NULL;

					if (tok_y != NULL)
						tok_y = NULL;

					tok_x = strtok_s(temp, "\t, ", &next);
					tok_y = strtok_s(NULL, "\t, ", &next);

					if (check_num(tok_x) == 0 && check_num(tok_y) == 0)
						large_point1_list[i] = new PM_CARTESIAN(atof(tok_x), atof(tok_y), sensor_height);

					else
					{
						cout << "Config File Error: File format is incorrect. Detected non-numerical characters when a number was expected. Please check the configuration file." << endl;
						err = 2;
					}//end else if

				}//end else if

				else if (j == 3)
				{
					if (tok_x != NULL)
						tok_x = NULL;

					if (tok_y != NULL)
						tok_y = NULL;

					tok_x = strtok_s(temp, "\t, ", &next);
					tok_y = strtok_s(NULL, "\t, ", &next);

					if (check_num(tok_x) == 0 && check_num(tok_y) == 0)
						large_point2_list[i] = new PM_CARTESIAN(atof(tok_x), atof(tok_y), sensor_height);
					
					else
					{
						cout << "Config File Error: File format is incorrect. Detected non-numerical characters when a number was expected. Please check the configuration file." << endl;
						err = 2;
					}//end else if

				}//end else if

				else if (j == 4)
				{
					if (tok_x != NULL)
						tok_x = NULL;

					if (tok_y != NULL)
						tok_y = NULL;

					tok_x = strtok_s(temp, "\t, ", &next);
					tok_y = strtok_s(NULL, "\t, ", &next);

					if (check_num(tok_x) == 0 && check_num(tok_y) == 0)
						edge_start_list[i] = new PM_CARTESIAN(atof(tok_x), atof(tok_y), sensor_height);

					else
					{
						cout << "Config File Error: File format is incorrect. Detected non-numerical characters when a number was expected. Please check the configuration file." << endl;
						err = 2;
					}//end else if

				}//end else if

				else if(j == 5)
				{
					if (tok_no != NULL)
						tok_no = NULL;

					tok_no = strtok_s(temp, "\t, ", &next);

					if (check_num(tok_no) == 0 && (atoi(tok_no) == 1 || atoi(tok_no) == 0))
						no_arm_list[i] = atoi(tok_no);
					
					else
					{
						cout << "Config File Error: File format is incorrect. Detected non-numerical characters when a number was expected. Please check the configuration file." << endl;
						err = 2;
					}//end else if

				}//end else

				else if (j == 6)
				{
					if (tok_x != NULL)
						tok_x = NULL;

					if (tok_y != NULL)
						tok_y = NULL;

					tok_x = strtok_s(temp, "\t, ", &next);
					tok_y = strtok_s(NULL, "\t, ", &next);

					if (check_num(tok_x) == 0 && check_num(tok_y) == 0)
						sum1[i] = new PM_CARTESIAN(atof(tok_x), atof(tok_y), sensor_height);

					else
					{
						cout << "Config File Error: File format is incorrect. Detected non-numerical characters when a number was expected. Please check the configuration file." << endl;
						err = 2;
					}//end else if

				}//end else if

				else if (j == 7)
				{
				if (tok_x != NULL)
					tok_x = NULL;

				if (tok_y != NULL)
					tok_y = NULL;

				tok_x = strtok_s(temp, "\t, ", &next);
				tok_y = strtok_s(NULL, "\t, ", &next);

				if (check_num(tok_x) == 0 && check_num(tok_y) == 0)
					sum2[i] = new PM_CARTESIAN(atof(tok_x), atof(tok_y), sensor_height);

				else
				{
					cout << "Config File Error: File format is incorrect. Detected non-numerical characters when a number was expected. Please check the configuration file." << endl;
					err = 2;
				}//end else if

				}//end else if

				else if (j == 8)
				{
					if (tok_no != NULL)
						tok_no = NULL;

					tok_no = strtok_s(temp, "\t, ", &next);

					if (check_num(tok_no) == 0 && atoi(tok_no) >= 1)
						update_count[i] = atoi(tok_no);

					else
					{
						cout << "Config File Error: File format is incorrect. Detected non-numerical characters when a number was expected. Please check the configuration file." << endl;
						err = 2;
					}//end else if

				}//end else

			}//end for

			config_file->getline(temp, 128);

		}//end for
	}//end if

	else
	{
		cout << "Config File Error: Could not read from file because prior error state detected. Please see error output." << endl;
	}//end else

}//end for

void cart_config::write_file()
{

	char temp[128];

	if (err == 0)
	{

		config_file->close();

		config_file->open(file_name, std::fstream::trunc|std::fstream::in|std::fstream::out);

		if (config_file->is_open() && task_count > 0)
		{
			config_file->seekp(0);

			config_file->write("NEWCONFIG\n", sizeof(char)*strlen("NEWCONFIG\n"));
			config_file->write(start, sizeof(char)*strlen(start));
			config_file->write("\n", sizeof(char) * 1);

			sprintf_s(temp, "%d", runs);
			config_file->write(temp, sizeof(char)*strlen(temp));
			config_file->write("\n", sizeof(char) * 1);

			sprintf_s(temp, "%d", task_count);
			config_file->write(temp, sizeof(char)*strlen(temp));
			config_file->write("\n", sizeof(char) * 1);


			for (int i = 0; i < task_count; i++)
			{
				config_file->write("NEWTASK", sizeof(char)*strlen("NEWTASK"));
				sprintf_s(temp, "%d", i);
				config_file->write(temp, sizeof(char)*strlen(temp));
				config_file->write("\n", sizeof(char)*strlen("\n"));


				for (int j = 0; j < 9; j++)
				{
					config_file->write("\t", sizeof(char) * 1);

					if (j == 0)
						config_file->write(dock_list[i], sizeof(char)*strlen(dock_list[i]));
					else if (j == 1)
					{
						for (int k = 0; k < 3; k++)
						{
							if (k == 0)
								sprintf_s(temp, "%f, ", goal_list[i]->robot_th);
							else if (k == 1)
								sprintf_s(temp, "%f, ", goal_list[i]->robot_x - 1000 * cos(goal_list[i]->robot_th * TO_RAD));
							else if (k == 2)
								sprintf_s(temp, "%f", goal_list[i]->robot_y - 1000 * sin(goal_list[i]->robot_th * TO_RAD));

							config_file->write(temp, sizeof(char)*strlen(temp));
						}//end for
					}//end else if

					else if (j == 2)
					{
						for (int k = 0; k < 2; k++)
						{
							if (k == 0)
								sprintf_s(temp, "%f, ", large_point1_list[i]->x);
							else if (k == 1)
								sprintf_s(temp, "%f", large_point1_list[i]->y);

							config_file->write(temp, sizeof(char)*strlen(temp));
						}//end for
					}//end else if

					else if (j == 3)
					{
						for (int k = 0; k < 2; k++)
						{
							if (k == 0)
								sprintf_s(temp, "%f, ", large_point2_list[i]->x);
							else if (k == 1)
								sprintf_s(temp, "%f", large_point2_list[i]->y);

							config_file->write(temp, sizeof(char)*strlen(temp));
						}//end for
					}//end else if

					else if (j == 4)
					{
						for (int k = 0; k < 2; k++)
						{
							if (k == 0)
								sprintf_s(temp, "%f, ", edge_start_list[i]->x);
							else if (k == 1)
								sprintf_s(temp, "%f", edge_start_list[i]->y);

							config_file->write(temp, sizeof(char)*strlen(temp));
						}//end for
					}//end else if

					else if (j == 5)
					{
						sprintf_s(temp, "%d", no_arm_list[i]);
						config_file->write(temp, sizeof(char)*strlen(temp));
					}//end else if

					else if (j == 6)
					{
						for (int k = 0; k < 2; k++)
						{
							if (k == 0)
								sprintf_s(temp, "%f, ", sum1[i]->x);
							else if (k == 1)
								sprintf_s(temp, "%f", sum1[i]->y);

							config_file->write(temp, sizeof(char)*strlen(temp));
						}//end for
					}//end else if

					else if (j == 7)
					{
						for (int k = 0; k < 2; k++)
						{
							if (k == 0)
								sprintf_s(temp, "%f, ", sum2[i]->x);
							else if (k == 1)
								sprintf_s(temp, "%f", sum2[i]->y);

							config_file->write(temp, sizeof(char)*strlen(temp));
						}//end for
					}//end else if

					else if (j == 8)
					{
						sprintf_s(temp, "%d", update_count[i]);
						config_file->write(temp, sizeof(char)*strlen(temp));
					}//end else if

					config_file->write("\n", sizeof(char) * 1);

				}//end for

				config_file->write("ENDTASK", sizeof(char)*strlen("ENDTASK"));
				sprintf_s(temp, "%d", i);
				config_file->write(temp, sizeof(char)*strlen(temp));
				config_file->write("\n", sizeof(char)*strlen("\n"));


			}//end for
			config_file->write("ENDCONFIG", sizeof(char)*strlen("ENDCONFIG"));
		}//end if
		else
		{
			cout << "Config File Error: Could not reopen the file for writing." << endl;
			err = 3;
		}//end else
	}//end if

	else
	{
		cout << "Config File Error: Could not write to file because prior error state detected. Please see error output." << endl;
	}//end else

}//end write_file

void cart_config::update_large_points(int dock_num, PM_CARTESIAN large_point1, PM_CARTESIAN large_point2)
{
	if (dock_num >= 0 && dock_num < task_count)
	{
		large_point1_list[dock_num]->x = large_point1.x;
		large_point1_list[dock_num]->y = large_point1.y;

		large_point2_list[dock_num]->x = large_point2.x;
		large_point2_list[dock_num]->y = large_point2.y;

	}//end if

	else
	{
		cout << "Warning: Could not update large reflector initial search positions for dock point " << dock_num << ". Index out of range." << endl;
	}//end else
}//end update_large_points

void cart_config::update_edge_points(int dock_num, PM_CARTESIAN edge_point)
{
	if (dock_num >= 0 && dock_num < task_count)
	{
		edge_start_list[dock_num]->x = edge_point.x;
		edge_start_list[dock_num]->y = edge_point.y;

	}//end if

	else
	{
		cout << "Warning: Could not update edge initial search positions for dock point " << dock_num << ". Index out of range." << endl;
	}//end else
}//end update_large_points

void cart_config::avg_large_points(int dock_num, PM_CARTESIAN large_point1, PM_CARTESIAN large_point2)
{
	if (dock_num >= 0 && dock_num < task_count)
	{
		if (update_count[dock_num] == 1 && update_count[dock_num]!=0)
		{
			sum1[dock_num]->x = large_point1_list[dock_num]->x + large_point1.x;
			sum1[dock_num]->y = large_point1_list[dock_num]->y + large_point1.y;

			sum2[dock_num]->x = large_point2_list[dock_num]->x + large_point2.x;
			sum2[dock_num]->y = large_point2_list[dock_num]->y + large_point2.y;

			update_count[dock_num]++;
		}//end if

		else
		{
			sum1[dock_num]->x += large_point1.x;
			sum1[dock_num]->y += large_point1.y;

			sum2[dock_num]->x += large_point1.x;
			sum2[dock_num]->y += large_point1.y;

			update_count[dock_num]++;
		}//end else

		large_point1_list[dock_num]->x = sum1[dock_num]->x / update_count[dock_num];
		large_point1_list[dock_num]->y = sum1[dock_num]->y / update_count[dock_num];

		large_point2_list[dock_num]->x = sum2[dock_num]->x / update_count[dock_num];
		large_point2_list[dock_num]->y = sum2[dock_num]->y / update_count[dock_num];

	}//end if

	else
	{
		cout << "Warning: Could not update large reflector initial search positions for dock point " << dock_num << ". Index out of range." << endl;
	}//end else
}//end update_large_points

int cart_config::get_count()
{
	return task_count;
}//end get_count

void cart_config::get_config(config_data* copy)
{
	
	if (err == 0)
	{
		copy->~config_data();

		copy->runs = runs;

		copy->task_count = task_count;

		copy->start = (char*)malloc(sizeof(char)*strlen(start) + 1);
		strcpy_s(copy->start, sizeof(char)*strlen(start) + 1, start);

		copy->dock_list = (char**)malloc(sizeof(char*)*task_count);
		copy->goal_list = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*)*task_count);
		copy->large_point1_list = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
		copy->large_point2_list = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
		copy->edge_start_list = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
		copy->no_arm_list = new int[task_count];
		copy->sum1 = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
		copy->sum2 = (PM_CARTESIAN**)malloc(sizeof(PM_CARTESIAN*)*task_count);
		copy->update_count = new int[task_count];

		for (int i = 0; i < copy->task_count; i++)
		{
			copy->dock_list[i] = (char*)malloc(sizeof(char)*strlen(dock_list[i]) + 1);
			strcpy_s(copy->dock_list[i], sizeof(char)*strlen(dock_list[i]) + 1, dock_list[i]);

			copy->goal_list[i] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			copy->goal_list[i]->robot_th = goal_list[i]->robot_th;
			copy->goal_list[i]->robot_x = goal_list[i]->robot_x;
			copy->goal_list[i]->robot_y = goal_list[i]->robot_y;

			copy->large_point1_list[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			copy->large_point1_list[i]->x = large_point1_list[i]->x;
			copy->large_point1_list[i]->y = large_point1_list[i]->y;
			copy->large_point1_list[i]->z = large_point1_list[i]->z;

			copy->large_point2_list[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			copy->large_point2_list[i]->x = large_point2_list[i]->x;
			copy->large_point2_list[i]->y = large_point2_list[i]->y;
			copy->large_point2_list[i]->z = large_point2_list[i]->z;

			copy->edge_start_list[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			copy->edge_start_list[i]->x = edge_start_list[i]->x;
			copy->edge_start_list[i]->y = edge_start_list[i]->y;
			copy->edge_start_list[i]->z = edge_start_list[i]->z;

			copy->no_arm_list[i] = no_arm_list[i];

			copy->sum1[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			copy->sum1[i]->x = sum1[i]->x;
			copy->sum1[i]->y = sum1[i]->y;
			copy->sum1[i]->z = sum1[i]->z;

			copy->sum2[i] = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
			copy->sum2[i]->x = sum2[i]->x;
			copy->sum2[i]->y = sum2[i]->y;
			copy->sum2[i]->z = sum2[i]->z;

			copy->update_count[i] = update_count[i];

		}//end for
	}//end if
	
	else
	{
		cout << "Config File Error: Could not read from file because prior error state detected. Please see error output." << endl;
	}//end else
	
}//end get_config

void cart_config::print_file()
{

	if (err == 0)
	{
		cout << "Configuration: " << endl;
		cout << "Runs: " << runs << endl;
		cout << "Task count: " << task_count << endl;
		cout << "Start: " << start << endl;

		cout << "Dock list: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << dock_list[i];

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;

		cout << "Goal list: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << "(" << goal_list[i]->robot_x << ", " << goal_list[i]->robot_y << ", " << goal_list[i]->robot_th << ")";

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;

		cout << "Large Point 1 List: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << "(" << large_point1_list[i]->x << ", " << large_point1_list[i]->y << ", " << large_point1_list[i]->z << ")";

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;

		cout << "Large Point 2 list: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << "(" << large_point2_list[i]->x << ", " << large_point2_list[i]->y << ", " << large_point2_list[i]->z << ")";

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;

		cout << "Edge Start list: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << "(" << edge_start_list[i]->x << ", " << edge_start_list[i]->y << ", " << edge_start_list[i]->z << ")";

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;

		cout << "No arm list: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << no_arm_list[i];

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;

		cout << "sum1 list: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << "(" << sum1[i]->x << ", " << sum1[i]->y << ", " << sum1[i]->z << ")";

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;

		cout << "sum2 list: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << "(" << sum2[i]->x << ", " << sum2[i]->y << ", " << sum2[i]->z << ")";

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;

		cout << "Update count list: ";

		for (int i = 0; i < task_count; i++)
		{
			cout << update_count[i];

			if (i < task_count - 1)
				cout << ", ";
		}//end for

		cout << endl;
	}//end if

	else
	{
		cout << "Config File Error: Could not write to file because prior error state detected. Please see error output." << endl;
	}//end else

}//end print_file

int cart_config::get_err()
{
	return err;
}//end get_err;