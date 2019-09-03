/**
*\file cart_status.h
*\brief Defines class and functions for storing and modifying manipulator-on-cart service and status information.
*\author Omar Aboul-Enein
*\date 2018-06-05
*/

#ifndef CART_STATUS_H
#define CART_STATUS_H

#include "ld_msg.h"
#include "posemath.h"

/**
*\class cart_status
*Class for storing and modifying cart service status. Here "service" refers to an LD vehicle pushing the cart to its desired destination.
*/

class cart_status
{
private:

	/**
	*\private
	*Current service status of the manipulator-on-cart. Is restricted to taking on the following values with corresponding meanings:\n
	*0 (Ready): the manipulator-on-cart is waiting for a vehicle to commit to transporting the cart to its next destination. \n
	*1 (Waiting): the manipulator-on-cart a vehicle has committed to moving the cart, but the cart is waiting for it to reach its next destination. \n
	*2 (Busy): the manipulator is performing work and does not need service.
	*/
	int status;


	/**
	*\private
	*Integer flag used to indicate if no cart is actually present (used to test vehicle control of multiple carts when a second cart is not available).\n
	*This flag is sent to the vehicle process so vehicle does not try to dock with a cart that is not physically present.
	*0 (false): Cart present
	*1 (true): Cart not present
	*/
	int no_cart; //0--false, 1--true

	/**
	*\private
	*Specifies the dock point the vehicle starts at when the test begins. This is included because the vehicle begisn the test with no cart attached and thus needs a start point \n
	*needs to know the location of the first cart that is scheduled for pickup.
	*/
	char* start_dock;

	/**
	*\private
	*Integer flag indicating whether or not the arm is to be moved when the cart reaches it's current destination.
	*0 (false): Move arm
	*1 (true): Don't move arm
	*/
	int current_no_arm; //0--false, 1--true

	/**
	*\private
	*Specifies the next dock point, or destination for the manipulator-on-cart. Corresponds to the name of a goal point stored on LD vehicle map.
	*/
	char* current_dock;

	/**
	*\private
	*Commanded pose for next dock point which corresponds to the pose of a goal point stored on LD vehicle map.
	*/
	PM_POSE* current_goal;

	/**
	*\private
	*Specifies the expected, or trained, pose that the manipulator assumes when performing bisection localization at the first bisect reflector.
	*/
	PM_CARTESIAN* current_large_point1;

	/**
	*\private
	*Specifies the expected, or trained, pose that the manipulator assumes when performing bisection localization at the second bisect reflector.
	*/
	PM_CARTESIAN* current_large_point2;

	/**
	*\private
	*Specifies the expected, or trained, pose that the manipulator assumes when performing edge registration. 
	*The x component represents the expected position of the long table edge and the y component represents the expected position of the short table edge.
	*/
	PM_CARTESIAN* current_edge_start;

	/*
	*\private
	*List of flags indicating wherther or not the arm is to move when it reaches the corresponding dock point.
	*/
	int* no_arm_buf;

	/**
	*\private
	*List of dock points. Treated as a queue where dock point at the front (index 0) is the next dock point the cart will travel to.
	*/
	char** dock_buf;

	/**
	*\private
	*List of goal poses. Treated as a queue where pose at the front (index 0) is the next pose the cart will travel to.
	*/
	ld_msg_pose** goal_buf;

	/**
	*\private
	*List of large reflector poses. Treated as a queue where pose at the front (index 0) is the next pose manipulator will assume when performing bisection at first bisect reflector.
	*/
	PM_CARTESIAN** large_point1_buf;
	
	/**
	*\private
	*List of large reflector poses. Treated as a queue where pose at the front (index 0) is the next pose manipulator will assume when performing bisection at second bisect reflector.
	*/
	PM_CARTESIAN** large_point2_buf;

	/**
	*\private
	*List of initial search points for edge registration. Treated as a queue where pose at the front (index 0) is the next pose manipulator will assume when performing edge registration.
	*/
	PM_CARTESIAN** edge_start_buf;

	/**
	*\private
	*Keeps count of current size for all queues.
	*/
	int dock_count;

	/**
	*\private
	*Stores number of repetitions to be performed.
	*/
	int runs;

	/**
	*\private
	*Stores current number of completed tasks within a run.
	*/
	int progress;

	/**
	*\private
	*Stores current position of cart within the LD map. Used to adjust for docking offset when performing coordinate registration. This value is to be updated via messages from the vehicle process.
	*/
	ld_msg_pose current_pose;

public:

	/**
	*\public
	*Constructor.
	*\param[in] stat				Initilizes the status field.
	*\param[in] start_str			Initializes the start_dock field.
	*\param[in] no_arm_list			Initializes the no_arm_buf field. First element also used to initialize current_no_arm.
	*\param[in] task_list			Initializes the dock_buf field. First element is also used to initialize current_dock.
	*\param[in] goal_list			Initializes the goal_buf field. First element is also used to initialize current_goal.
	*\param[in] large_point1_list	Initializes large_point2_buf. First element is also used to initialize current_large_point1.
	*\param[in] large_point2_list	Initializes large_point2_buf. First element is also used to initialize current_large_point2.
	*\param[in] len					Initializes dock_count. Note that all list inputs should be the same and equal to this size.
	*/
	cart_status::cart_status(int stat, int no_cart_mode, char* start_str, int* no_arm_list, char** task_list, ld_msg_pose** goal_list, PM_CARTESIAN** large_point1_list, PM_CARTESIAN** large_point2_list, PM_CARTESIAN** edge_start_list, int len, int num_runs);

	/**
	*\public
	*Destructor
	*/
	~cart_status();

	/**
	*\public
	*Sets the status to new_stat. Mutator function.
	@param[in] new_stat		The new status value.
	*/
	void set_status(int new_stat);

	/**
	*\public
	*Updates the current dock point, goal pose, and large reflector poses to the next one in the queue. Essentially, a single dequeue function for all buf members.
	*/
	void update_current_dock();

	/*
	*\public
	*Prints the contents of no_arm_buf
	*/
	void print_no_arm_list();

	/**
	*\public
	*Prints the contents of dock_buf to the standard output.
	*/
	void print_dock_list();

	/**
	*\public
	*Prints the contents of goal_buf to the standard output.
	*/
	void print_goal_list();

	/**
	*\public
	*Prints the contents of large_point1_buf to the standard output.
	*/
	void print_large_point1_list();

	/**
	*\public
	*Prints the contents of large_point2_buf to the standard output.
	*/
	void print_large_point2_list();


	/**
	*\public
	*Prints the contents of edge_start_buf to the standard output.
	*/
	void print_edge_start_list();


	/**
	*\public
	*Formats contents of status object to a string to be transmitted to the vehicle process using TCP/IP. This function converts additional status info that is needed
	*To initialize the data structures on the server side (such as the initial dock point of the cart).
	*\return	Character array containing status info.
	*/
	char* to_string_init();

	/**
	*\public
	*Formats contents of status object to a string to be transmitted to the vehicle process using TCP/IP.
	*\return	A pointer to a character array containing the formatted message.
	*/
	char* to_string();

	/**
	*\public
	*Accessor function for the current cart status.
	*\return	The contents of status.
	*/
	int get_status();

	/**
	*\public
	*Accessor function for the current queue size.
	*\return	The contents of dock_count.
	*/
	int get_dock_count();

	/**
	*\public
	*Parsing function to store pose info contained in a string format.
	*\param[in]		pose_msg A character array of the format "x, y, th"
	*/
	void string_to_pose(char* pose_msg);

	/*
	*\public
	*Accessor. Copies the contents of current_pose to a new memory location.
	*\param[out]	Data structure that stores the copied pose.
	*/
	void get_current_pose(ld_msg_pose& copy_pose);

	/*
	*\public
	*Accessor. Returns the contents of no_cart.
	*\return	Contents of no_cart.
	*/
	int get_no_cart();

	/*
	*\public
	*Accessor. Returns the contents of current_no_arm.
	*\return	Contents of current_no_arm.
	*/
	int get_current_no_arm();

	/*
	*\public
	*Accessor. Returns the contents of current_dock.
	*\return	Contents of current_dock.
	*/
	char* get_current_dock();

	/*
	*\public
	*Accessor. Returns the contents of current_goal.
	*\param[out]	Data structure that will store copy of current_goal.
	*/
	int get_current_goal(PM_POSE& copy_pose);


	/*
	*\public
	*Accessor. Returns the contents of current_large_point1.
	*\param[out]	Data structure that will store copy of current_large_point1.
	*\return	Error code indicating successful return of object. Returns -1 if current_large_point1 is NULL and zero otherwise.
	*/
	int get_current_large_point1(PM_CARTESIAN& copy_point);

	/*
	*\public
	*Accessor. Returns the contents of current_large_point2.
	*\param[out]	Data structure that will store copy of current_large_point2.
	*\return	Error code indicating successful return of object. Returns -1 if current_large_point1 is NULL and zero otherwise.
	*/
	int get_current_large_point2(PM_CARTESIAN& copy_point);


	/*
	*\public
	*Accessor. Returns the contents of current_edge_start.
	*\param[out]	Data structure that will store copy of current_edge_start.
	*\return	Error code indicating successful return of object. Returns -1 if current_large_point1 is NULL and zero otherwise.
	*/
	int get_current_edge_start(PM_CARTESIAN& copy_point);

	/**
	*\public
	*Prints the contents of the status data structure.
	*/
	void print_status();

};

#endif
