/**
*\file ld_comm.h
*\brief Interface for managing ARCL connections to the Omron Adept ld mobile robot platform.\n
*Based on:\n
*agv_comm.h by J. Marvel, S. Legowik\n
*References:\n
*"_ftime, _ftime32, _ftime64 from MSDN"
*https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64
*\author Omar Aboul-Enein
*\date 2018-06-14
*/

#ifndef LD_COMM_H
#define LD_COMM_H
#include "ulapi.h"
#include "ld_msg.h"
#include <fstream>

/**
\class ld-comm
*Class for storing and managing connections to an Omron Adept LD Transporter Vehicle. This class establishes this process as the server, with the vehicle acting as the client.\n
*Thus the outgoing ARCL connection feature of the LD vehicle is used in the implementation of this class.
*/
class ld_comm
{
private:
	
	/**
	*\private
	*Descriptor for the ulapi server socket
	*/
	ulapi_integer server_id;

	/**
	*\private
	*Descriptor of the ulapi client socket that establishes the connection from an LD vehicle on the outgoing ARCL connection.
	*/
	ulapi_integer client_id;

	/**
	*\private
	*Port number of the server. Should be configure to use the port that the Outgoing ARCL connection on the vehicle is configured to use.
	*/
	ulapi_integer port; 

	/**
	*\private
	*Log file for recording data from ARCL command responses. Typically used to record pose or timestamps for messages.
	*/
	ofstream ld_log;


	/**
	*\private
	*Helper function for printing communication errors.
	*\param[in] err		Error code returned by ulapi socket
	*\param[in] msg		Error message the user desires to print.
	*/
	void comm_err(int err, char* msg);

public:

	/**
	*\public
	*Constructor. Initializes variables used to establish ulapi server socket.
	*\param[in] arcl_port	Port number to use for the created server socket.
	*\param[in] log			Boolean used to enable or disable logging
	*/
	ld_comm(ulapi_integer arcl_port, bool log);

	/**
	*\public
	*Destructor. Used to safely close server socket.
	*/
	~ld_comm();

//Connection management:
	/**
	*\public
	*Opens the server socket and then waits to accept the ld to connect as a client.
	*/
	int ld_connect();

	/**
	*\public
	*Used to safely disconnect a currently running client connection.
	*/
	int client_disconnect();

	/**
	*\public
	*Reads the results of commands that are automatically executed on the vehicle and pushed to the control program.
	*\param[out] pose	Data structure to hold the pose and timestamp that is returned.
	*/
	int arcl_dsfv_pose(ld_msg_pose* pose);

//Other read and write functions:

	/**
	*\public
	*Repetedly polls the socket until a goal arrival message is recieved from the ARCL server. \n
	*Used to pause the program until a specified broadcast message is received from the ld.
	*\param[in] status_string The broadcast message from the ARCL server to wait for.
	*/
	int arcl_read_status(char* status_string);

	/**
	*\public
	*Reads responses from the ARCL server, returning responses line by line. Reads ARCL server messages delmited by newlines.
	*\param[out] arcl_recv_buf String pointer to hold the response recieved from the ARCL server.
	*/
	int arcl_read(char **arcl_recv_buf);

	//int arcl_write(char* arcl_cmd); //Sends a command to the ARCL server.

//Output and Logging:

	/**
	*\public
	*Outputs the contents of a pose message to the console.
	*\param[in] msg ld_pose_msg structure to be printed.
	*/
	void print_pose(ld_msg_pose* msg);

	/**
	*\public
	*Outputs the contents of a pose message to a log file in CSV format.
	*\param[in] msg ld_pose_msg structure to be printed.
	*/
	void ld_comm::log_pose(ld_msg_pose* msg, ulapi_integer client_id, char* current_dock);

	/**
	*\public
	*Enters additional comments into the log. Useful for adding headers to the CSV file.
	*\param[in] msg Message to be added to the log.
	*/
	void log_comment(char* msg);

};//end ld_comm
#endif