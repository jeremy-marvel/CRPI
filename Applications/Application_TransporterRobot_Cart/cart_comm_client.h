/**
*\file cart_comm_client.h
*\brief Interface defining data structure used for managing socket connection to Application_TransporterRobot_Vehicle Process.
 *\n Based on lynx_comm_client.h by O Aboul-Enein
*\author Omar Aboul-Enein
*\date 2018-06-14

*/
#ifndef CART_COMM_CLIENT_H
#define CART_COMM_CLIENT_H

#include "ulapi.h"
#include "cart_status.h"
#include "crpi_robot.h"
#include "crpi_universal.h"

using namespace crpi_robot;

class cart_comm_client
{
private:

	/**
	*\private
	*Ulapi socket descriptor for connection to vehicle server process.
	*/
	ulapi_integer client_id;

	/**
	*\private
	*Port number used to initialize the socket connection to the vehicle server process. Should be set to 5352 (the port number of the vehicle server process).
	*/
	ulapi_integer port;

	/**
	*\private
	*IP address of the computer running the vehicle server process. Current set to the loopback address (127.0.0.1) since all process are currently running on the same computer.
	*/
	char* ip_addr;

	/**
	*\private 
	*Data structure containing cart status information such as the current operating status (Ready, waiting, or busy), next dock point etc.
	*/
	cart_status* client_stat;

	/**
	*\private
	*Ulapi mutex used to ensure safe concurrent access to the shared resources of this structure. Shared resources that require mutexing include the client_id, port, ip_addr, and 
	*especially the client_stat structure. Must be careful in locking this variable to avoid mutexing blocking ulapi calls.
	*/
	ulapi_mutex_struct* mutex;

	/*
	*\private
	*Helper function for printing communication errors.
	\param[in] err		Error code returned by ulapi socket
	\param[in] msg		Error message the user desires to print.
	*/
	void comm_err_client(int err, char* msg);


public:

	/**
	*\public
	*Constructor
	*\param[in] no_cart_mode		Used to initialize the client_stat structure.
	*\param[in] server_addr			Used to initialize the ip_addr field.
	*\param[in] server_port			Used to initialize the port field.
	*\param[in] key					Used to initialize the mutex field. Ulapi key needed to create or reference a ulapi mutex structure.
	*\param[in] stat_str			Used to initialize the client_stat structure. See \ref cart_status.h
	*\param[in] no_arm_list			Used to initialize the client_stat structure. See \ref cart_status.h
	*\param[in] task_list			Used to initialize the client_stat structure. See \ref cart_status.h
	*\param[in] goal_list			Used to initialize the client_stat structure. See \ref cart_status.h
	*\param[in] large_point1_list	Used to initialize the client_stat structure. See \ref cart_status.h
	*\param[in] large_point2_list	Used to initialize the client_stat structure. See \ref cart_status.h
	*\param[in] len					Used to initialize the client_stat structure. See \ref cart_status.h
	*/
	cart_comm_client::cart_comm_client(int no_cart_mode, char* server_addr, ulapi_integer server_port, ulapi_id key, char* start_str, int* no_arm_list, char** task_list, ld_msg_pose** goal_list, PM_CARTESIAN** large_point1_list, PM_CARTESIAN** large_point2_list, PM_CARTESIAN** edge_start_list, int len, int num_runs);
	
	/**
	*\public
	*Destructor
	*/
	~cart_comm_client();

//Connection management:

	/**
	*\public
	*Initializes client socket and connects to vehicle server process using the configuration variables stored in this data structure.
	*\return	An integer indicating the error status of the socket. Takes on a negative number if an error occurred while trying to connect.
	*/
	int cart_client_connect();

	/**
	*\public
	*Disconnects client socket from vehicle server process. Typically called after a ulapi socket error occurs.
	*\return	An integer indicating the error status of the socket. Takes on a negative number if an error occurred while trying to close the socket.
	*/
	int cart_client_disconnect();

	/**
	*\public
	*Reads socket and returns the resulting message recieved. Simply wraps the ulapi socket read call with error checking control structures.
	*\param[out] buf	A character buffer to hold the message recieved on the socket.
	*\param[in] len		The size of buf.
	*\return			An integer indicating the error status of the socket. Takes on a negative number an error occurred while trying to read the socket.
	*/
	int recv(char* buf, int len);

	/**
	*\public
	*Sends a message on the socket to be recieved by the vehicle server process. 
	*\param[in] buf		The character buffer containing the message to send.
	*\return			An integer indicating the error status of the socket. Takes on a negative number if an error occurred while tring to send the message on the socket.
	*/
	int send(char* buf);

	/**
	*\public
	*Reads a message sent by the vehicle server process on the socket. This function reads messages character-by-character until a newline is encountered.
	*This function should only be used to recieve character buffers that terminate with the newline character and does not contain the null terminator.
	*\param[out] cart_comm_client_recv_buf	Pointer to a character buffer that will contain the recieved message. Messages returned will include the newline character.
	*/
	int cart_comm_client_read(char** cart_comm_client_recv_buf);

	/**
	*\public
	*Deconstructs message containing both a status update and pose message and passes each part to the corresponding update function.
	*\param[in] msg		A character buffer of the format "%d%s:(%f, %f, %f)" where the first integer is a single digit.
	*\return			An integer indicating the error status of the function. Returns a negative number if an error occurred.
	*/
	int parse_msg(char* msg);

	/**
	*\public
	*Performs an action depending on the message passed to the function. Intended to process messages sent from the vehicle server process.
	*These actions include updating the current operating status of the cart and forwarding information back to the vehicle server process among other things.
	*\param[in] msg		A character buffer where the leading integer is a single digit. Acceptable commands include:\n
	*					"rcstat" -- Forwards current status in the format "%d%s" to the vehicle server process. The integer is the current operating status while the string is the next dock point destination.
	*					"rcstatinit" -- Forwards initial status info of the format "%d%s:%s" where the first integer is the initial operating status, the first string is the dock point that the cart starts at, and the second string is the first dock point destination.
	*					"rcupdate0"-- Updates status of cart to 0 (Ready) \n
	*					"rcupdate1"-- Updates status of cart to 1 (Waiting) \n
	*					"rcupdate2"-- Updates status of cart to 2 (Busy) \n
	*\return			An integer indicating the error status of the function. Returns a negative number if an error occurred.
	*/
	int remote_command(char* cmd);

	/**
	*Performs an action depending on the message passed to the function. Intended to process messages locally stored by this process.
	*These actions include updating the current operating status of the cart, updating the current dock point destination, and forwarding information back to the vehicle server process among other things.
	*\param[in] msg		A character buffer of the format "%d%s" where the leading integer is a single digit.
	*\return			An integer indicating the error status of the function. Returns a negative number if an error occurred.
	*/
	void local_command(char* cmd);

	/**
	*\public
	*Accessor method to get the current operating status of the cart from the client_stat data structure. Also provides mutexing.
	*returns	The current operating status of the cart as a single-digit integer.
	*/
	int get_stat_value();

	/**
	*\public
	*Accessor method to get the socket descriptor of the client socket. Also provides mutexing.
	*returns	Socket descriptor as a ulapi_integer.
	*/
	ulapi_integer get_client_id();

	/**
	*\public
	*Accessor method to get the number of remaining dock points the cart wishes to travel to. Also provides mutexing.
	*returns	The current number of remianing dock points.
	*/
	int get_stat_count();

	/**
	*\public
	*Control code for performance test method. Sends manipulator to an initial stage pose for safe access to the RMMA, then performs pose calculations based on the current vehicle position to adjust for docking offset. \n
	*Finally, calls function to perform bisect registration and spiral search verification.
	*\param[in] ur_robot		A pointer to a CRPI datastructure that manages the connection to the UR5 robot URScript server. Needed to send move commands to control the arm.
	*\param[in] level_pause		Flag to enable pausing after extending feet to allow for level measurements.
	*/
	void test_square(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause);

	/**
	*\public
	*Control code for performance test method. Sends manipulator to an initial stage pose for safe access to the RMMA, then performs pose calculations based on the current vehicle position to adjust for docking offset. \n
	*Finally, calls function to perform bisect registration and spiral search verification.
	*Additionally, records the measured actual position of the bisect reflectors, which can be used to update the initial search positions stored in the configuration file.
	*\param[in] ur_robot		A pointer to a CRPI datastructure that manages the connection to the UR5 robot URScript server. Needed to send move commands to control the arm.
	*\param[out] update1		A pointer to a PM_CARTESIAN datastructure that stores the measured actual position of the first bisect reflector after searching. Is not updated if bisect search fails.
	*\param[out] update2		A pointer to a PM_CARTESIAN datastructure that stores the measured actual position of the second bisect reflector after searching. Is not updated if bisect search fails.
	*\param[out] bisect_stat	A pointer to a integer used for error checking. Set to 1 if bisect is successful or 0 if bisect is unsuccessful.
	*/
	void test_square_auto_update(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN* update1, PM_CARTESIAN* update2, int *bisect_stat);


	/**
	*\public
	*Control code for performance test method. Sends manipulator to an initial stage pose for safe access to the RMMA, then performs pose calculations based on the current vehicle position to adjust for docking offset. \n
	*Finally, calls function to perform edge registration (does not use continuous move, only moves incremented by steps) and spiral search verification.
	*This version was one of several edge registration methods tested with the OTS and was measured on 06-28-2019.
	*\param[in] ur_robot	A pointer to a CRPI datastructure that manages the connection to the UR5 robot URScript server. Needed to send move commands to control the arm.
	*\param[in] level_pause		Flag to enable pausing after extending feet to allow for level measurements.
	*/
	void test_edge(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause);

	/**
	*\public
	*Control code for performance test method. Sends manipulator to an initial stage pose for safe access to the RMMA, then performs pose calculations based on the current vehicle position to adjust for docking offset. \n
	*Finally, calls function to perform edge registration (features a trained, initial search point followed by a single continuous move that is interrupted)  and spiral search verification.
	*This version was one of several edge registration methods tested with the OTS and was measured on 07-01-2019.
	*\param[in] ur_robot	A pointer to a CRPI datastructure that manages the connection to the UR5 robot URScript server. Needed to send move commands to control the arm.
	*\param[in] level_pause		Flag to enable pausing after extending feet to allow for level measurements.
	*/
	void test_edge_cont(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause);

	/**
	*\public
	*Control code for performance test method. Sends manipulator to an initial stage pose for safe access to the RMMA, then performs pose calculations based on the current vehicle position to adjust for docking offset. \n
	*Finally, calls function to perform edge registration (features a single continuous move that is interrupted and NO trained initial search point)  and spiral search verification.
	*This version was one of several edge registration methods tested with the OTS and was measured on 07-03-2019.
	*\param[in] ur_robot	A pointer to a CRPI datastructure that manages the connection to the UR5 robot URScript server. Needed to send move commands to control the arm.
	*\param[in] level_pause		Flag to enable pausing after extending feet to allow for level measurements.
	*/
	void test_edge_cont2(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause);

	void test_edge_cont_bisect(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause);

	/**
	*\public
	*Wrapper that prints the contents of client_stat with mutexing.
	*/
	void print_info();
};

#endif