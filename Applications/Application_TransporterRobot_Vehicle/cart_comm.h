/**
*\file cart_comm.h
*\brief Interface and data structure definitions for object used for managing connections to Application_TransporterRobot_Cart processes.\n
*Based on:\n
*lynx_comm.h by O. Aboul-Enein
*\author Omar Aboul-Enein
*\date 2018-06-15
*/

#ifndef CART_COMM_H
#define CART_COMM_H

#include "ulapi.h"
#include "cart_status_copy.h"

/**
*\class cart_comm
*Class for storing the communication configurations and operating/service status of multiple cart client processes.
*/
class cart_comm
{

private:
	/**
	*\private
	*Ulapi socket descirptor for the server socket.
	*/
	ulapi_integer server_id;

	/**
	*\private
	*Port number used to initialize the server socket to which cart client processes connect. Will be set to 5352.
	*/
	ulapi_integer port;

	/**
	*\private
	*Ulapi mutex used to ensure safe concurrent access to the shared resources of this structure. Shared resources that require mutexing include the server_id,
	*max_clients, client_count, and client_arr. Must be careful in locking this variable to avoid mutexing blocking ulapi calls.
	*/
	ulapi_mutex_struct* mutex;

	/**
	*\private
	*Ulapi semaphore structure used to send sleep and wakeup signals to the cart_connect thread that accepts new clients.
	*/
	void* connect_sem;

	/**
	*\private
	*Constant that stores the maximum number clients that are permitted to be connected the server at once.
	*/
	int max_clients;

	/**
	*\private
	*Counter variable that keeps track of the number of clients currently connected to the server.
	*/
	int client_count;

	/**
	*\private
	*Array that stores the connection and status information for each client.
	*/
	cart_status_copy** client_arr;

	/**
	*\private
	*Helper function to print error messages related to ulapi socket failures.
	*\param[in] err		Error code to be printed
	*\param[in] msg		Character array that contains the message to be printed.
	*/
	void comm_err(int err, char* msg);

	/**
	*\private
	*Helper function that shifts the elements of client_arr so no holes in memory occur when a client disconnects and its data is deleted from the array.
	*\param start	Index of deleted element. Starting place in array to begin shifting elements.
	*/
	void shift_clients(int start);

public:
	
	/**
	*\public
	*Constructor
	*\param[in] server_port		Used to initialize port.
	*\param[in] num_clients		Used to initialize max_clients.
	*\param[in] key_m			Ulapi key used to initialize mutex.
	*\param[in] key_s			Ualpi key used to initialize connect_sem
	*/
	cart_comm(ulapi_integer server_port, int num_clients, ulapi_id key_m, ulapi_id key_s);

	/**
	*\public
	*Destructor
	*/
	~cart_comm();

//Connection management

	/**
	*\public
	*Initializes the server socket.
	*\return	Integer indicating the error status of the function. A negative number indicates an error occurred while attempting to initialize the socket.
	*/
	int cart_comm_init();

	/**
	*\public
	*Accepts new client on server, initializes and updates internal data structures. This function is fully mutexed.
	\return		Integer indicating the error status of the function. A negative number indicates an error occurred while attempting to initialize the socket.
	*/
	int cart_connect();

	/**
	*\public
	*Disconnects client from server and updates internal data structures to release memory occupied by client status. This function is fully mutexed.
	*\param[in]		client_id The socket descriptor for the client to be disconnected.
	\return			Integer indicating the error status of the function. A negative number indicates an error occurred while attempting to initialize the socket.
	*/
	int client_disconnect(ulapi_integer client_id);

	/**
	*\public
	*Accessor function to return a client_id at a specific index in client_arr.
	*\param[in]		index Integer denoting the element of client_arr to return the client_id.
	\return			The socket descriptor for the client. Reutrns a negative number if no client exists at the location specified.
	*/
	ulapi_integer get_client_id(int index);

	/**
	*\public
	*Accessor function to return the location of a client in client_arr given a client socket descriptor.
	*\param[in]	client_id	Socket descriptor to search for in client_arr.
	*\return				Index into client_arr where client_id was found. Returns a negative number if the client could not be found.
	*/
	int get_client_index(ulapi_integer client_id);

	/**
	*\public
	*Accessor function to return the current number of connected clients. This function is fully mutexed.
	*\return	The number of currently connected clients.
	*/
	int get_client_count();

	/**
	*\public
	*Scheduling function to select the next client to service. Essentially a First Come First Served scheduling algorithm.
	*\return	The socket descriptor for the next client to service.
	*/
	ulapi_integer simple_scheduler();

	/**
	*\public
	*Copies all the status info for a given client into a separate data structure.
	*\param[in] client_id		The socket descriptor of the client to copy.
	*\param[out] copied_client	The data structure that will store the copied data.
	*/
	void select_client(int client_id, cart_status_copy** copied_client);

	/**
	*\public
	*Prints a list of socket descriptors for all currently connected clients. Fully mutexed.
	*/
	void print_client_ids();

	/**
	*\public
	*Prints the contents of all client status structures for all currently connected clients.
	*/
	void print_client_status();

	/**
	*\public
	*Initializes the status of a new cart client. First sends a command to the cart process and updates the contents the internal data structures of this process using the message recieved from the cart process.\n
	*Sends the "rcstatinit" command to cart process and calls update status to change internal data structures.
	*\param[in]		new_stat Pointer to
	*\returns		Integer indicating the error status of the function. A negative number is returned if an error occurred.
	*/
	int update_status_init(cart_status_copy* new_stat);

	/**
	*\public
	*\deprecated
	*Updates the status of all cart clients. First sends the command "rcstatus" to cart processes, then calls update_status on internal data structures to update the data structure.
	*Use of this function is deprecated as the current pose of each client in the vehicle process's internal data structures is not updated. Mainly used for testing communications.
	*/
	void update_status();

	/**
	*\public
	*Updates the status of all cart clients. First sends the command "rcstatus" to cart processes, then calls update_status on internal data structures to update the data structure.
	*This function also updates the current pose contained in the internal data structures.
	*\param[in] new_pose	Pointer to a structure containing the updated pose information for each client.
	*/
	void update_status(ld_msg_pose* new_pose);

	/**
	*\public
	*Sends command messages to a cart client process appending the current LD pose stored in the status internal structure to the message.\n
	*Messages sent are formatted "%s:(%f, %f, %f)" Where the first string is the command to be sent to the cart client process and each float is the x, y, and theta components\n
	*of the LD vehicle, repsectively. Intended for use with clients that already have been connected and initialized.
	*Note that if a socket error occurs while attempting to send the message, the client will be disconnected.
	*\param[in]		buf Contains the command portion of the message.
	*\param[in]		Socket descriptor for the client the message will be sent to.
	*\return		An integer indicating the error status of the ulapi socket. Returns a negative number if an error occurred while attempting to send the message.
	*/
	int send(char* buf, ulapi_integer client_id);

	/**
	*\public
	*Sends command messages to a cart client process appending the current LD pose stored in the status internal structure to the message.\n
	*Messages sent are formatted "%s:(%f, %f, %f)" Where the first string is the command to be sent to the cart client process and each float is the x, y, and theta components\n
	*of the LD vehicle, repsectively. Same as send process but omits an extra error check for newly connected clients since the socket is opened before the\n
	*data structures tracking the status of the client are added to the vehicle process's internal data structures.\n
	*Note that if a socket error occurs while attempting to send the message, the client will be disconnected.\n
	Intended for use with newly connected clients.
	*\param[in]		buf Contains the command portion of the message.
	*\param[in]		Socket descriptor for the client the message will be sent to.
	*\return		An integer indicating the error status of the ulapi socket. Returns a negative number if an error occurred while attempting to send the message.
	*/
	int send_init(char* buf, cart_status_copy* new_stat);

	/**
	*\public
	*Wraps ulapi calls and error checks to read a message from a client socket.
	\param[out] buf			Character buffer that will store the returned message.
	\param[in] client_id	Socket descriptor of client from which the message will be recieved.
	\param[in] len			Size of buf in bytes.
	\return					An integer indicating the error status of the ulapi socket. Returns a negative number if an error occurred while attempting to recieve the message.
	*/
	int recv(char* buf, ulapi_integer client_id, int len);

	/**
	*Reads responses from the cart clients, expects messages to be delimited by newline with no null terminator. \n
	*\param[out] cart_comm_recv_buf String pointer to hold the response recieved from the cart client
	*\param[in] client_id The socket descriptor for the cart client to receive information from.
	*\return		An integer indicating the error status of the ulapi socket. Returns a negative number if an error occurred while attempting to send the message.
	*/
	int cart_comm_read(char** cart_comm_recv_buf, ulapi_integer client_id);

	/**
	*Reads responses from the cart clients, expects messages to be delimited by newline with no null terminator. \n
	*(Note to self: Not exactly sure how this function differs from cart_comm_read. May remove this).
	*\param[out] cart_comm_recv_buf String pointer to hold the response recieved from the cart client
	*\param[in] client_id The socket descriptor for the cart client to receive information from.
	*\return		An integer indicating the error status of the ulapi socket. Returns a negative number if an error occurred while attempting to send the message.
	*/
	int cart_comm_read_init(char** cart_comm_recv_buf, cart_status_copy* new_stat);

	/**
	*Updates the previous location of a client so that the vehicle may return to that client when scheduling it for service.
	*\param client_id	The socket descriptor of the client to update.
	*/
	void update_previous_dock(ulapi_integer client_id);
};

#endif
