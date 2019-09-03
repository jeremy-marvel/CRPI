/**
*\file cart_comm.cpp

*\brief Function implementation for managing connections from Application_TransporterRobot_Cart processes. \n

*References: \n

*Based on: lynx_comm.h by O. Aboul-Enein \n

*[1] Beginning Linux Programming, Chapter 15, Neil Matthew and Richard Stones \n
*[2] stackoverflow.com/questions/11842416/function-does-not-change-passed-pointer-c \n
*[3] stackoverflow.com/questions/13818960/c-tcp-server-still-reading-data-after-client-disconnects \n
*(SOLUTION TO BLOCKING SOCKET PROBLEM, you need to check for a return value of 0 since a 0 return values means client disconnected! \n
*your loop does not check for zero so it repeats the read call)

*\author Omar Aboul-Enein
*\date 2017-06-05
*/

#include "cart_comm.h"
#include "cart_status_copy.h"
#include "ulapi.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


using namespace std;

/**
*Helper function for printing communication errors.
*\param[in] err Error code returned by ulapi socket
*\param[in] msg Error message the user desires to print.
*/
void cart_comm::comm_err(int err, char* msg)
{
	cout << "Comm Error " << err << " " << msg << endl;
}//end comm_err

/**
*Helper function used to ensure list of client ids is contiguous.
*\param[in] start An index indicating where shifting should start.
*/
void cart_comm::shift_clients(int start)
{

	for (int i = start; i < client_count - 1; i++)
	{
		if (client_arr[i] != NULL)
		{
			delete client_arr[i];
			client_arr[i] = NULL;
		}//end if

		client_arr[i] = new cart_status_copy(client_arr[i + 1]);
	}//end for

	delete client_arr[client_count - 1];
	client_arr[client_count -1] = NULL;

	if (client_count == max_clients)
		ulapi_sem_give(connect_sem);

	client_count--;

	return;
}//end shift ids

cart_comm::cart_comm(ulapi_integer server_port, int num_clients, ulapi_id key_m, ulapi_id key_s)
{
	port = server_port;
	client_count = 0;
	max_clients = num_clients;
	client_arr = (cart_status_copy**)malloc(num_clients * sizeof(cart_status_copy*));

	mutex = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

	if ((mutex = ulapi_mutex_new(key_m)) == NULL)
	{
		cout << "Error: could not create ulapi mutex" << endl;
		exit(-1);
	}//end if

	if ((connect_sem = ulapi_sem_new(key_s)) == NULL)
	{
		cout << "Error: could not create ulapi semaphore" << endl;
		exit(-1);
	}//end if

	ulapi_sem_take(connect_sem);

}//end constructor

cart_comm::~cart_comm()
{
	int err;

	ulapi_mutex_take(mutex);
		for (int i = 0; i < client_count; i++)
		{
			if ((err = ulapi_socket_close(client_arr[i]->get_id())) < 0)
				comm_err(err, "client socket close failure");
		}//end for

		if ((err = ulapi_socket_close(server_id)) < 0)
			comm_err(err, "server socket close failure");

		for (int i = 0; i < client_count; i++)
		{
			if (client_arr[i] != NULL)
			{
				delete client_arr[i];
				client_arr[i] = NULL;
			}//end if

		}//end for

		if (client_arr != NULL)
		{
			free(client_arr);
			client_arr = NULL;
		}//end if

	ulapi_mutex_give(mutex);

	if ((err = ulapi_mutex_delete(mutex)) < 0)
		comm_err(err, "ulapi mutex delete failure");

	if ((err = ulapi_sem_delete(connect_sem)) < 0)
		comm_err(err, "ulapi semaphore delete failure");

}//end destructor

int cart_comm::cart_comm_init()
{
	if ((server_id = ulapi_socket_get_server_id(port)) < 0)
	{
		comm_err(server_id, "ulapi socket creation failure");
		return server_id;
	}//end if

	cout << "Server started on port " << port << " using id " << server_id <<endl;

	return 0;
}//end 

int cart_comm::cart_connect()
{
	int client_id;
	int err;

	if (get_client_count() == max_clients)
	{
		cout << "Maximum number of clients exceeded, connection thread will now sleep" << endl;
		ulapi_sem_take(connect_sem);
	}//end if

	client_id = ulapi_socket_get_connection_id(server_id); //MUST NOT BE MUTEXED BECAUSE CALL WILL BLOCK!

	ulapi_mutex_take(mutex);

		if (client_id >= 0)
		{
				client_arr[client_count] = new cart_status_copy(client_id, -1, -1, "UNKNOWN", "UNKNOWN");
				err = update_status_init(client_arr[client_count]);
				
				//cout << "err = " << err << endl;

				if (err > 0)
				{
					client_count++;
					cout << "Accepted cart client process on port " << port << " assigned id " << client_id << endl;
				}//end if
				else
				{
					delete client_arr[client_count];
					client_arr[client_count] = NULL;
				}//end else
		
		}//end else if
		else
			comm_err(0, "ulapi client socket creation failure");
	ulapi_mutex_give(mutex);

	return 0;

}//end cart_connect

int cart_comm::client_disconnect(ulapi_integer client_id)
{
	int index = -1;
	int err;

	index = get_client_index(client_id);

	ulapi_mutex_take(mutex);
		if (index >= 0 && index < client_count)
		{

			if ((err = ulapi_socket_close(client_id)) < 0)
				comm_err(err, "client socket close failure");

			if (client_count > 1)
				shift_clients(index);

			else
			{
				if (client_arr[index] != NULL)
				{
					delete client_arr[index];
					client_arr[index] = NULL;
				}//end if

				if (client_count==max_clients)
					ulapi_sem_give(connect_sem);

				client_count--;
			}//end else

		}//end if
	ulapi_mutex_give(mutex);

	return err;

}//end client_disconnect

ulapi_integer cart_comm::get_client_id(int index)
{
	ulapi_integer retval = -1;

	ulapi_mutex_take(mutex);
		if(client_arr[index]!=NULL)
			retval = client_arr[index]->get_id();
	ulapi_mutex_give(mutex);

	return retval;
}//end int

int cart_comm::get_client_index(ulapi_integer client_id)
{
	int index = -1;

	ulapi_mutex_take(mutex);
		for (int i = 0; i < client_count; i++)
		{
			if (client_arr[i]->get_id() == client_id)
			{
				index = i;
				break;
			}//end if
		}//end for
	ulapi_mutex_give(mutex);

	return index;

}//end get_client_index

int cart_comm::get_client_count()
{
	int retval;

	ulapi_mutex_take(mutex);
		retval = client_count;
	ulapi_mutex_give(mutex);

	return retval;
		
}//end get_client_count

ulapi_integer cart_comm::simple_scheduler()
{
	ulapi_integer scheduled_id = -1;

	ulapi_mutex_take(mutex);

	int found = 0;
	
	for (int i = 0; i < client_count && found == 0; i++)
	{
		cout << "SEARCING" << endl;
		if (client_arr[i]->get_status() == 0)
		{
			cout << "FOUND" << endl;
			found = 1;
			scheduled_id = client_arr[i]->get_id();
		}//end if
	}//end if

	ulapi_mutex_give(mutex);

	return scheduled_id;
}//end simple_schedule


void cart_comm::select_client(int client_id, cart_status_copy** copied_client) //stackoverflow.com / questions / 11842416 / function - does - not- change - passed - pointer - c
{
	ulapi_mutex_take(mutex);

	int index = get_client_index(client_id);

	cart_status_copy* temp;
	
	if (*copied_client != NULL)
	{
		delete *copied_client;
		*copied_client = NULL;
	}//end if

	if (index >= 0 && index < client_count)
	{
		temp = new cart_status_copy(client_arr[index]);
		cout << "MAKING NEW STATUS COPY at index = "<<index << endl;
		temp->print_status();
	}//end if
	else
	{
		temp = NULL;
		cout << "COULD NOT MAKE NEW STATUS COPY" << endl;
	}//end else

	*copied_client = temp;

	ulapi_mutex_give(mutex);

	return;
}//end select client

int cart_comm::update_status_init(cart_status_copy* new_stat)
{
	char* cart_client_buf = NULL;
	int err = 0;

	//cout << " client_count: " << client_count+1 << " id: " << new_stat->get_id() << endl;
	
	if ((err = send_init("rcstatinit", new_stat)) <= 0)
	{
		//cout << "send_init returned " << err << endl;
		return err;
	}//end if

	if ((err = cart_comm_read_init(&cart_client_buf, new_stat)) <= 0)
	{
		//cout << "cart_comm_read_init returned " << err << endl;
		return err;
	}//end if

	if (new_stat != NULL && cart_client_buf != NULL)
		new_stat->update_status_init(cart_client_buf);

	return err;
}//end update_status

void cart_comm::update_status()
{
	char* cart_client_buf = NULL;
	int err_s = 0;
	int err_r = 0;

	for (int i = 0; i < get_client_count(); i++)
	{
		cout << "i: " << i << " client_count: " << get_client_count() << " id: " << get_client_id(i) << endl;
		err_s = send("rcstat", get_client_id(i));

		if (err_s <= 0)
			i--;

		if(err_s > 0)
			err_r = cart_comm_read(&cart_client_buf, get_client_id(i));

		if (err_s > 0 && err_r > 0)
		{
			ulapi_mutex_take(mutex);
				if(client_arr[i]!=NULL && cart_client_buf != NULL)
					client_arr[i]->update_status(cart_client_buf);
			ulapi_mutex_give(mutex);
		}//end if

	}//end for

	return;
}//end update_status

void cart_comm::update_status(ld_msg_pose* new_pose)
{
	char* cart_client_buf = NULL;
	int err_s = 0;
	int err_r = 0;

	for (int i = 0; i < get_client_count(); i++)
	{
		cout << "i: " << i << " client_count: " << get_client_count() << " id: " << get_client_id(i) << endl;
		err_s = send("rcstat", get_client_id(i));

		if (err_s <= 0)
			i--;

		if (err_s > 0)
			err_r = cart_comm_read(&cart_client_buf, get_client_id(i));

		if (err_s >0 && err_r > 0)
		{
			ulapi_mutex_take(mutex);

				if (client_arr[i] != NULL)
					client_arr[i]->update_status(cart_client_buf);

				if(client_arr[i]!=NULL)
					client_arr[i]->update_pose(new_pose);

			ulapi_mutex_give(mutex);
		}//end if

	}//end for

	return;
}//end update_status

void cart_comm::print_client_ids()
{
	ulapi_mutex_take(mutex);
	if (client_count > 0)
	{
		cout << client_count << "/" << max_clients << " clients connected. " << "List of id's: ";

		for (int i = 0; i < client_count - 1; i++)
			cout << client_arr[i]->get_id() << ", ";

		cout << client_arr[client_count - 1]->get_id() << endl;
	}//end if

	else
		cout << "No clients are connected." << endl;
	ulapi_mutex_give(mutex);

	return;
}//end print_client_ids

void cart_comm::print_client_status()
{
	ulapi_mutex_take(mutex);
		if (client_count > 0)
		{
			for (int i = 0; i < client_count; i++)
				client_arr[i]->print_status();
		}//end if
		else
			cout << "No clients are connected." << endl;
	ulapi_mutex_give(mutex);
}//end print_client_status

int cart_comm::send(char* buf, ulapi_integer client_id)
{
	int err = 0;
	int index = get_client_index(client_id);

	ulapi_mutex_take(mutex);
		char* msg_buf = (char*)(malloc(sizeof(char)*(strlen(buf) + strlen(client_arr[index]->pose_to_string()) + 3)));
		strcpy_s(msg_buf, sizeof(char)*(strlen(buf) + strlen(client_arr[index]->pose_to_string()) + 3), buf);
		strcat_s(msg_buf, sizeof(char)*(strlen(buf) + strlen(client_arr[index]->pose_to_string()) + 3), ":");
		strcat_s(msg_buf, sizeof(char)*(strlen(buf) + strlen(client_arr[index]->pose_to_string()) + 3), client_arr[index]->pose_to_string());
		strcat_s(msg_buf, sizeof(char)*(strlen(buf) + strlen(client_arr[index]->pose_to_string()) + 3), "\n");
	ulapi_mutex_give(mutex);

	//cout << "SEND BUF:" << buf << endl;

	//Note that if client_id is not properly synced that is okay because error will be returned and handeled anyway
	if ((err = ulapi_socket_write(client_id, msg_buf, sizeof(char)*strlen(msg_buf))) <= 0)//Intentionally leaves out null terminator, MUST NOT BE MUTEXED BECAUSE CALL WILL BLOCK!
	{
		ulapi_mutex_take(mutex);
			comm_err(err, "ulapi socket read failure closing socket for client");
		ulapi_mutex_give(mutex);

		client_disconnect(client_id);

		return err;
	}//end if

	return err;
}//end send test

int cart_comm::send_init(char* buf, cart_status_copy* new_stat)
{
	int err = 0;

	ulapi_mutex_take(mutex);
	char* msg_buf = (char*)(malloc(sizeof(char)*(strlen(buf) + strlen(new_stat->pose_to_string()) + 3)));
	strcpy_s(msg_buf, sizeof(char)*(strlen(buf) + strlen(new_stat->pose_to_string()) + 3), buf);
	strcat_s(msg_buf, sizeof(char)*(strlen(buf) + strlen(new_stat->pose_to_string()) + 3), ":");
	strcat_s(msg_buf, sizeof(char)*(strlen(buf) + strlen(new_stat->pose_to_string()) + 3), new_stat->pose_to_string());
	strcat_s(msg_buf, sizeof(char)*(strlen(buf) + strlen(new_stat->pose_to_string()) + 3), "\n");
	ulapi_mutex_give(mutex);

	//cout << "SEND BUF:" << buf << endl;

	//Note that if client_id is not properly synced that is okay because error will be returned and handeled anyway
	if ((err = ulapi_socket_write(new_stat->get_id(), msg_buf, sizeof(char)*strlen(msg_buf))) <= 0)//Intentionally leaves out null terminator, MUST NOT BE MUTEXED BECAUSE CALL WILL BLOCK!
	{
		ulapi_mutex_take(mutex);
		comm_err(err, "ulapi socket read failure closing socket for client");
		ulapi_mutex_give(mutex);

		if ((err = ulapi_socket_close(new_stat->get_id())) < 0)
			comm_err(err, "ulapi socket close failure closing socket for client");

		return err;
	}//end if

	return err;
}//end send test

int cart_comm::recv(char* buf, ulapi_integer client_id, int len)
{
	int err = 0;

	//Note that if client_id is not prperly synced that is okay because error will be returned and handled anyway.
	if ((err = ulapi_socket_read(client_id, buf, len)) <= 0)//MUST NOT BE MUTEXED BECAUSE CALL WILL BLOCK!
	{
		ulapi_mutex_take(mutex);
			comm_err(err, "ulapi socket read failure closing socket for client");
		ulapi_mutex_give(mutex);

		client_disconnect(client_id);

		return err;
	}//end if

	return err;
}//end send test

int cart_comm::cart_comm_read(char** cart_comm_recv_buf, ulapi_integer client_id)
{
	string cart_comm_recv_str = "";
	char* buf = (char*)malloc(1);
	int err = 0;

	//cout << "READ FUNCTION: " << endl;

	do
	{
		if ((err = ulapi_socket_read(client_id, buf, 1)) <= 0)
		{
			ulapi_mutex_take(mutex);
				comm_err(err, "ulapi socket read failure");
			ulapi_mutex_give(mutex);

			client_disconnect(client_id);

			return err;
		}//end if

		//cout << *buf;

		cart_comm_recv_str.push_back(*buf);

	} while (*buf != '\n');

	//cout << endl;

	//arcl_recv_buf.append('\0');


	*cart_comm_recv_buf = (char*)malloc(sizeof(char)*(cart_comm_recv_str.length() + 1));
	strcpy_s(*cart_comm_recv_buf, sizeof(char)*(cart_comm_recv_str.length() + 1), cart_comm_recv_str.c_str());

	free(buf);

	return err;

}//end cart_comm_read

int cart_comm::cart_comm_read_init(char** cart_comm_recv_buf, cart_status_copy* new_stat)
{
	string cart_comm_recv_str = "";
	char* buf = (char*)malloc(1);
	int err = 0;

	//cout << "READ FUNCTION: " << endl;

	do
	{
		if ((err = ulapi_socket_read(new_stat->get_id(), buf, 1)) <= 0)
		{
			//cout << "ulapi_socket_read returned " << err << " ";
			//cout << *buf << endl;

			ulapi_mutex_take(mutex);
			comm_err(err, "ulapi socket read failure");
			ulapi_mutex_give(mutex);

			if ((err = ulapi_socket_close(new_stat->get_id())) < 0)
				comm_err(err, "ulapi socket close failure closing socket for client");

			return err;
		}//end if

		//cout << "ulapi_socket_read returned " << err <<" ";
		//cout << *buf << endl;

		cart_comm_recv_str.push_back(*buf);

	} while (*buf != '\n');

	//cout << endl;

	//arcl_recv_buf.append('\0');


	*cart_comm_recv_buf = (char*)malloc(sizeof(char)*(cart_comm_recv_str.length() + 1));
	strcpy_s(*cart_comm_recv_buf, sizeof(char)*(cart_comm_recv_str.length() + 1), cart_comm_recv_str.c_str());

	free(buf);

	return err;

}//end cart_comm_read

void cart_comm::update_previous_dock(ulapi_integer client_id)
{

	int index = get_client_index(client_id);

	ulapi_mutex_take(mutex);

	if (index >= 0 && index < client_count)
	{
		if (client_arr[index] != NULL)
			client_arr[index]->update_previous_dock();
	}//end if

	ulapi_mutex_give(mutex);
}//end 

