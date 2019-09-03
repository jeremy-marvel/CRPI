#include "AR_Interface.h"

#include <iostream>
#include <string>
#include <sstream>

#include <fstream>

using namespace std;

AR_Interface *AR_Interface::instance = 0;

// Read ULAPI socket to the next new-line
string socket_readline( 
	ulapi_integer socket_id )
{
	char c = 0;
	string line;

	int count = ulapi_socket_read( socket_id, &c, 1 );
	while( c != '\n' )
	{
		line.push_back( c );
		count = ulapi_socket_read( socket_id, &c, 1 );

		if( count != 1 )
		{
			cout << "Error reading socket" << endl;
			ulapi_sleep( 60 );
			exit(1);
		}
	}

	return line;
}

AR_Interface::AR_Interface()
	: initialized( false )
{
	ar_status.new_data = false;
}

AR_Interface* AR_Interface::Get_Instance()
{
	if( !instance )
	{
		instance = new AR_Interface();
	}

	return instance;
}

void AR_Interface::Init( 
	ulapi_prio priority,
	int key )
{
	if( initialized )
	{
		cerr << "AR_Interface can only initialized once!" << endl;
		ulapi_sleep( 60 );
		exit(1);
	}
	initialized = true;

	// create data exclusiong mutex
	mutex = ulapi_mutex_new( key );

	// connect to AR status process
	socket_id = ulapi_socket_get_client_id(9734, "192.168.0.51" );
	if( socket_id < 0 )
	{
		cerr << "Failed to connect to AR toolkit interface" << endl;
		ulapi_sleep( 60 );
		exit( 1 );
	}

	// spawn monitoring task
	task = ulapi_task_new();
	ulapi_task_start( task, AR_Thread, this, priority, 0 );
}

AR_Status AR_Interface::Get_Status()
{
	AR_Status status;

	if( !initialized )
	{
		cerr << "Cannot get status till interface object is initialized" << endl;
		ulapi_sleep( 60 );
		exit(1);
	}

	ulapi_mutex_take( mutex );
	status = ar_status;
	ar_status.new_data = false;
	ulapi_mutex_give( mutex );

	return status;
}

void AR_Interface::AR_Thread( void *ar_interface )
{
	AR_Interface *ar = (AR_Interface*)ar_interface;

	while( 1 )
	{
		// read and parse message from socket
		string line = socket_readline( ar->socket_id );
		stringstream input( line );
		AR_Status status;
		status.new_data = true;
		input >> status.time_stamp >> status.marker_id
			>> status.pose.tran.x >> status.pose.tran.y >> status.pose.tran.z
			>> status.pose.rot.x >> status.pose.rot.y >> status.pose.rot.z >> status.pose.rot.s;
		if( input.good() )
		{
			ulapi_mutex_take( ar->mutex );
			ar->ar_status = status;
			ulapi_mutex_give( ar->mutex );
		}
	}
}