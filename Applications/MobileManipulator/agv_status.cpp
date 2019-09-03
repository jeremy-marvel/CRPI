/* Read position status from AGV at 16 Hz */

#include "agv_status.hh"
#include <iostream>
#include <sstream>

/////////////////////////////////////////////////////////////////////////////
AGV_Status::AGV_Status( ulapi_prio priority, 
	std::string address,
	int port )
	: _verbose( false ),
	_connected( false ),
	_address( address ),
	_port( port )
{
	_mutex = ulapi_mutex_new( AGV_STATE_MUTEX_ID );

	// start interface thread
	task = ulapi_task_new();
	if( ULAPI_OK != ulapi_task_start(task, run_thread, this, priority, 0) )
	{
		std::cout << "Error starting comm process" << std::endl;
		exit(-1);
	}	
}

/////////////////////////////////////////////////////////////////////////////
AGV_Status::~AGV_Status()
{
	if( _connected )
	{
		ulapi_socket_close( _socket );
	}
	ulapi_mutex_delete( _mutex );
}

/////////////////////////////////////////////////////////////////////////////
void AGV_Status::set_verbose( bool value )
{
	_verbose = value;
}

///////////////////////////////////////////////////////////////////////////
// Generate a string corresponding to the ULAPI return value
const char* AGV_Status::ulapi_to_string( ulapi_result retval )
{
	switch( retval )
	{
	case ULAPI_OK:
		return "ULAPI_OK";
	case ULAPI_ERROR:
		return "ULAPI_ERROR";
	case ULAPI_IMPL_ERROR:
		return "ULAPI_IMPL_ERROR";
	case ULAPI_BAD_ARGS:
		return "ULAPI_BAD_ARGS";
	default:
		return "Unknown return value";
	}
}

/////////////////////////////////////////////////////////////////////////////
// Monitor TCP port for status.
void AGV_Status::run()
{
	while( 1 )
	{
		if( !_connected )
		{
			// if not connected,
			// connect to port on AGV
      printf ("port %d address %s\n", _port, _address.c_str());
			_socket = ulapi_socket_get_client_id( _port, _address.c_str() );
			if( _socket < 0 )
			{
				std::cout << "Error connecting to AGV" << std::endl;
				ulapi_sleep( 5.0 );
				continue;
			}

			// subscribe to vehicle status
			char *subscribe_msg = "protocol 2\rsubscribe state\r";
			ulapi_integer count = ulapi_socket_write( _socket, subscribe_msg, strlen(subscribe_msg));
			if( count < 0 )
			{
				std::cout << "Error writing to AGV" << std::endl;
				ulapi_sleep( 5.0 );
				continue;
			}

			_connected = true;
		}
		else
		{
			// read status
			std::string message;
			if( read_message( message ) != ULAPI_OK )
			{
				// a read error occured
				_connected = false;
				ulapi_socket_close( _socket );
				std::cout << "AGV Status read error occured... reconnecting" << std::endl;
			}
			else
			{
				// check for comment
				if( message.compare( 0, 1, "#" ) == 0 )
				{
					// skip over comment
				} 
				else if( message.compare( 0, 5, "state" ) == 0 )
				{
					// parse vehicle state
					std::stringstream msg( message );
					std::string command;
					AGV_State agv_state;
					msg >> command >> agv_state.timestamp 
						>> agv_state.pos_x >> agv_state.pos_y >> agv_state.pos_th 
						>> agv_state.pos_known >> agv_state.navlevel;

					// copy AGV state if parse succeded
					if( !msg.bad() )
					{
						ulapi_mutex_take( _mutex );
						_agv_state = agv_state;
						ulapi_mutex_give( _mutex );
					}
				}
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////
// Read in a single message delimited by the \n characters.  \r 
// characters are ignored, and not placed in the message string.  The return 
// string will not contain the \r or \n characters. The return value is 
// ULAPI_OK if there were no read errors.  
ulapi_integer AGV_Status::read_message( std::string &message )
{
	// clear message contents
	message.clear();

	// read in next message
	char c = 0;
	while( c != '\n' )
	{
		if( (ulapi_socket_read( _socket, &c, 1 )) != 1 )
		{
			_connected = false;
			ulapi_socket_close( _socket );
			return ULAPI_ERROR;
		}
		else
		{
			switch( c )
			{
			case '\r':
			case '\n':
				// don't include in message string
				break;
			default:
				// append character onto message string
				message.push_back( c );
				break;
			}
			
		}
	}

	return ULAPI_OK;
}

//////////////////////////////////////////////////////////////////
AGV_State AGV_Status::get_state()
{
	AGV_State state;

	ulapi_mutex_take( _mutex );
	state = _agv_state;
	ulapi_mutex_give( _mutex );

	return state;
}

/////////////////////////////////////////////////////////////////////
// Static function required to interface with tasking system
void AGV_Status::run_thread( void *ptr )
{
	 AGV_Status *this_ptr = (AGV_Status*) ptr;

	 this_ptr->run();
}