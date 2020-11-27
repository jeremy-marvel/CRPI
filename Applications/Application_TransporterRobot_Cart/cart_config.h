/**
*\file cart_config.h
*\brief This file provides definitions for implementing a configuration file used for saving and loading test parameters (like initial reflector search positions, no arm mode, etc.)
*\author Omar Aboul-Enein
*\date 2018-09-10
*/

#ifndef CART_CONFIG_H
#define CART_CONFIG_H

#include <fstream>
#include "ld_msg.h"
#include "ur5_control.h"

/**
*\public
*A structure used to store a copy of the data contain in the cart_config class.
*/
struct config_data
{

public:

	/**
	*\public
	*Stores the number of times a cart will cycle through a sequence of stops.
	*/
	int runs;

	/**
	*\public
	*Counts the number of task locations that the cart will travel to. The tasks need not be unique stops.
	*/
	int task_count;

	/**
	*\public
	*Stores the goal name in the vehicle map at which the cart is initially parked.
	*/
	char* start;

	/**
	*\public
	*A list of goal names in the vehicle map that the cart will travel to.
	*/
	char** dock_list;

	/**
	*\public
	*A list 2D poses that correspond to the goals listed in dock_list. Used for coordinate system transformations adjusting the registration.
	*/
	//Remember that in the config file, you found out that the vehicle pose should not be the ideal goal pose, but the actual vehicle pose
	//at the time of training the initial search points.
	ld_msg_pose** goal_list;

	/**
	*\public
	*A list of initial search positions for the first large reflector used when bisecting.
	*/
	PM_CARTESIAN** large_point1_list;

	/**
	*\public
	*A list of initial search positions for the second large reflector used when bisecting.
	*/
	PM_CARTESIAN** large_point2_list;

	/*
	*\public
	*A list of starting offsets for edge registration. r1_start offset is stored in the x value and r3_start offset is stored in the y value of each PM_CARTESIAN object. See overloaded arguments for square_edge_short.
	*/
	PM_CARTESIAN** edge_start_list;

	/**
	*\public
	*A list of integer flags indicating whether or not the arm is to be used at a specific docking location. 0 indicates the arm will be used, 1 indicates the arm is disabled.
	*/
	int* no_arm_list;

	/**
	*\public
	*A sum used for computing the average actual position of the first large reflector. Used for adaptive registration.
	*/
	PM_CARTESIAN** sum1;

	/**
	*\public
	*A sum used for computing the average actual position of the second large reflector. Used for adaptive registration.
	*/
	PM_CARTESIAN** sum2;

	/**
	*\public
	*Counts the number of registration points recorded for computing the average actual position of the large reflectors.
	*/
	int* update_count;

	/**
	*\public
	*Constructor
	*/
	config_data();

	/**
	*\public
	*Destructor
	*/
	~config_data();

	/**
	*\public
	*Prints the contents of the data structure.
	*/
	void print_data();
};

/**
*\public
*An object containing data to access the configuration file and store a copy of its configuration values in main memory.
*Also contains functions for opening, reading and writing to the file. As well as functions for managing access and modification of the configuration values.
*/
class cart_config
{

private:


	/**
	*\private
	*Stores the file path of the configuration file to read and write to.
	*/
	char* file_name;

	/**
	*\private
	*Stores the number of times a cart will cycle through a sequence of stops.
	*/
	int runs;

	/**
	*\private
	*Counts the number of task locations that the cart will travel to. The tasks need not be unique stops.
	*/
	int task_count;

	/**
	*\private
	*An error integer indicating if there was an error reading or writing to a file. A value of 0 indicates no error occurred.
	*/
	int err;

	/**
	*\private
	*A pointer to a filestream object used to read the configuration file.
	*/
	fstream* config_file;

	/**
	*\private
	*Pointer to the first task 
	*/
	streampos first_task;

	/**
	*\private
	*Stores the goal name in the vehicle map at which the cart is initially parked.
	*/
	char* start;

	/**
	*\private
	*A list of goal names in the vehicle map that the cart will travel to.
	*/
	char** dock_list;

	/**
	*\private
	*A list 2D poses that correspond to the goals listed in dock_list. Used for coordinate system transformations adjusting the registration.
	*/
	ld_msg_pose** goal_list;

	/**
	*\private
	*A list of initial search positions for the first large reflector used when bisecting.
	*/
	PM_CARTESIAN** large_point1_list;

	/**
	*\private
	*A list of initial search positions for the second large reflector used when bisecting.
	*/
	PM_CARTESIAN** large_point2_list;

	/*
	*\public
	*A list of starting offsets for edge registration. r1_start offset is stored in the x value and r3_start offset is stored in the y value of each PM_CARTESIAN object. See overloaded arguments for square_edge_short.
	*/
	PM_CARTESIAN** edge_start_list;

	/**
	*\private
	*A list of integer flags indicating whether or not the arm is to be used at a specific docking location. 0 indicates the arm will be used, 1 indicates the arm is disabled.
	*/
	int* no_arm_list;
	
	/**
	*\private
	*A sum used for computing the average actual position of the first large reflector. Used for adaptive registration.
	*/
	PM_CARTESIAN** sum1;

	/**
	*\private
	*A sum used for computing the average actual position of the second large reflector. Used for adaptive registration.
	*/
	PM_CARTESIAN** sum2;

	/**
	*\private
	*Counts the number of registration points recorded for computing the average actual position of the large reflectors.
	*/
	int* update_count;

public:

	/**
	*\public
	*Constructor
	*\param[in] select_path		Integer variable to indicate which file path to choose. A value of zero indicates the configuration file path stored in config_path.txt will be opened.\n
	*							While any other value indicates the configuration file path stored in config_path2.txt will be opened.
	*/
	cart_config(int select_path);

	/**
	*\public
	*Destructor
	*/
	~cart_config();

	/**
	*\public
	*Deallocates memory of the data structure while keeping the config file open. Used to clear the structure before re-reading the contents of the file.
	*/
	void clear_data();

	/**
	*\public
	*Reads the information stored in the file and populates the private fields of the data structure with the values.
	*/
	void read_file();

	/**
	*\public
	*Writes the contents of the data structure back into the file.
	*/
	void write_file();

	/**
	*\public
	*Updates the pose of the large reflectors within the data structure.
	*\param[in] dock_num		Index into large_point1_list and large_point2_list to select which point to update.
	*\param[in] large_point1	New point for the first large reflector.
	*\param[in] large_point2	New point for the second large reflector.
	*/
	void update_large_points(int dock_num, PM_CARTESIAN large_point1, PM_CARTESIAN large_point2);


	/**
	*\public
	*Updates the pose of the large reflectors within the data structure.
	*\param[in] dock_num		Index into large_point1_list and large_point2_list to select which point to update.
	*\param[in] edge_point		New point for the edge registration
	*/
	void update_edge_points(int dock_num, PM_CARTESIAN edge_point);

	/**
	*\public
	*Updates the pose of the large reflectors within the data structure using the average of previously recorded actual poses.
	*\param[in] dock_num		Index into large_point1_list and large_point2_list to select which point to update.
	*\param[in] large_point1	New point for the first large reflector to incorperate into the average.
	*\param[in] large_point2	New point for the second large reflector to incorperate into the average.
	*/
	void avg_large_points(int dock_num, PM_CARTESIAN large_point1, PM_CARTESIAN large_point2);

	/**
	*\public
	*Accessor
	*\return	The contents of task_count (Number of assembly tasks stored in the file.
	*/
	int get_count();


	/**
	*\public
	*Accessor
	*\param[out] copy	A copy of all the data structure contents.
	*/
	void get_config(config_data* copy);

	/**
	*\public
	*Prints the contents of the file.
	*/
	void print_file();

	/**
	*Accessor
	*\return The contents of err, which indicates if there was error reading or writing to the file.
	*/
	int get_err();

};

#endif