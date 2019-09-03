/**
*\file cart_status_copy.h
*\brief Definitions for data structure storing cart service status information
*\author Omar Aboul-Enein
*\date 2018-06-14
*/

#ifndef CART_STATUS_COPY_H
#define CART_STATUS_COPY_H

#include "ulapi.h"
#include "ld_comm.h"
#include "ld_msg.h"


using namespace std;

/**
*\class
*This class is used to store local copies of the cart client statuses. Statuses are updated via clear-text sent by socket connection.
*/
class cart_status_copy
{

private:
	/**
	*\private
	*Socket Descriptor for connecting to the particular cart client.
	*/
	ulapi_integer client_id;

	/**
	*\private
	*Current service status of a manipulator - on - cart. Is restricted to taking on the following values with corresponding meanings : \n
	* 0 (Ready) : the manipulator - on - cart is waiting for a vehicle to commit to transporting the cart to its next destination. \n
	* 1 (Waiting) : the manipulator - on - cart a vehicle has committed to moving the cart, but the cart is waiting for it to reach its next destination. \n
	* 2 (Busy) : the manipulator is performing work and does not need service.
	*/
	int status;

	/**
	*\private
	*Integer flag used to indicate if no cart is actually present (used to test vehicle control of multiple carts when a second cart is not available).\n
	*This flag is checked so vehicle does not try to dock with a cart that is not physically present.
	*0 (false): Cart present
	*1 (true): Cart not present
	*/
	int no_cart;

	/**
	*\private
	*Specifies the next dock point, or destination for a manipulator-on-cart. Corresponds to the name of a goal point stored on LD vehicle map.
	*/
	char* current_dock;

	/**
	*\private
	*Specifies the last dock point, or destination for a manipulator-on-cart. The vehicle uses this field to return to a cart for pick-up.
	*/
	char* previous_dock;

	/**
	*\private
	*Stores current position of cart within the LD map. Sent to cart process over socket to adjust for docking offset when performing coordinate registration.
	*/
	ld_msg_pose current_pose;

public:
	/**
	*\public
	*Constructor
	*\param[in]	new_client Used to initialize client_id.
	*\param[in]	new_stat Used to initialize status.
	*\param[in] no_cart_mode Used to initialize no_cart_mode.
	*\param[in] first_dock Used to initialize first_dock.
	*\param[in] start_dock Used to initialize start_dock.
	*/
	cart_status_copy(ulapi_integer new_client, int new_stat, int no_cart_mode, char* first_dock, char* start_dock);

	/**
	*\public
	*Copy constructor.
	*\param[in]	stat An object to be copied into a new memory location.
	*/
	cart_status_copy(const cart_status_copy* stat);

	/**
	*\public
	*Destructor.
	*/
	~cart_status_copy();
	

	/**
	*\public
	*Parses cart message that provides the first status update for a cart. This function takes special consideration to parsing the starting location of a cart.
	*\param[in] stat_str A special formatted string that contains the status information to store in the datastructure. Specifically the string format is "%d%d%s:%s"
	*\n where the first integer is the single-digit status code (0, 1, or 2), the second integer is a single-digit flag indicating if a cart is present, the first string
	*is the starting location of the cart (corresponding to a vehicle map goal name), and the second string indicating the next docking position of the cart (corresponding to a vehicle map goal name).
	*/
	void update_status_init(char* stat_str);

	/**
	*\public
	*Parses cart message that provides a status update for a cart.
	*\param[in] stat_str A special formatted string that contains the status information to store in the datastructure. Specifically the string format is "%d%s"
	*\n where the first integer is the single-digit status code (0, 1, or 2), and the first string indicating the next docking position of the cart (corresponding to a vehicle map goal name).
	*/
	void update_status(char* stat_str);

	/**
	*\public
	*Wrapper function to transmit the current vehicle pose from the ARCL server. Result is stored in current_pose.
	*\param[in] An object that contains socket descriptors and function definitions for communicating with the vehilce.
	*/
	void update_pose(ld_comm* vehicle_arcl);

	/**
	*\public
	*Wrapper function to transmit the current vehicle pose from the ARCL server. Result is stored in an external structure.
	*\param[out] A structure that will be populated with the current vehicle pose information.
	*/
	void update_pose(ld_msg_pose* pose_msg);

	/**
	*\public
	*Accessor.
	*\return	The contents of client_id.
	*/
	ulapi_integer get_id();

	/**
	*\public
	*Accessor.
	*\return	The contents of status.
	*/
	int get_status();

	/**
	*\public
	*Accessor.
	*\return	The contents of no_cart.
	*/
	int get_no_cart();

	/**
	*\public
	*Accessor.
	*\param[out]	stat_buf A copy of current_dock. A NULL pointer should be passed for argument.
	*/
	void get_current_dock(char** stat_buf);

	/**
	*\public
	*Accessor.
	*\param[out]	stat_buf A copy of current_dock. A NULL pointer should be passed for argument.
	*/
	void get_previous_dock(char** stat_buf);

	/**
	*\public
	*Mutator. Uppdates the vallue of previous_dock to current_dock.
	*/
	void update_previous_dock();

	/**
	*\public
	*Prints the contents of the status data structure.
	*/
	void print_status();

	/**
	*\public
	*Converts the current pose of the vehicle to a string which can be sent over a socket to a cart client process.
	*/
	char* pose_to_string();

};

#endif
