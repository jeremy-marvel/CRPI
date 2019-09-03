/* Interface for communicating with the AGV Controller */

#include "agv_comm.h"

using namespace std;

//////////
// Constructor for AGV interface object
AgvComm::AgvComm( string _address,
	int _port )
	: address( _address ),
	port( _port ),
	verbose( false ),
	connected( false )
{
	/*if( verbose )
		cout << "Starting AGV communication thread " << endl;  
	task = ulapi_task_new();
	mutex = ulapi_mutex_new(100);
	ulapi_prio priority = ulapi_prio_lowest();
	ulapi_task_start((ulapi_task_struct*)task, receiveTask, this, priority, 0);*/
}

//////////
// Destructor for AGV interface object
AgvComm::~AgvComm()
{
	if( connected )
	{
		ulapi_socket_close( remoteID );
	}
}

//////////
//void AgvComm::receiveTask( void *param )
//{
//	AgvComm *agvComm = (AgvComm *) param;
//
//	while(1)
//	{
//		if( agvComm ->isConnected() )
//		{
//			ACI_Msg *msg = ACI_read_message( agvComm ->remoteID );
//			++agvComm->messageCount;
//			
//			ulapi_mutex_take( agvComm->mutex );
//			agvComm ->messageQueue.push_back( msg );
//			ulapi_mutex_give( agvComm->mutex );
//		}
//		else
//		{
//			ulapi_sleep( 5 );
//		}
//	}
//}

//////////
// set verbosity flag
void AgvComm::setVerbose( bool _verbose )
{
	verbose = _verbose;
}

//////////
// Attempts to connect if not already connected.  Returns true if connection attempt succedes
// or if the connection was previously established.
bool AgvComm::isConnected()
{
	if (!connected)
	{
		if( verbose )
			std::cout << endl << "Connecting to AGV controller..." << endl;
		remoteID = ulapi_socket_get_client_id( port, address.c_str() );

		if( remoteID > 0 )
		{
			ulapi_socket_set_nonblocking( remoteID );
			if( verbose )
				std::cout << "connected to AGV controller " << remoteID << endl;
			connected = true;
		}
		else
		{
			if( verbose )
				std::cout << "***AGV controller connect failed" << endl;
		}
	}

	return connected;
}

///////////
// empty all the reply messages out of the queue
//void AgvComm::flushMessages()
//{
//	ulapi_mutex_take( mutex );
//	while( !messageQueue.empty() )
//	{
//		ACI_Msg *msg = messageQueue.front();
//		messageQueue.pop_front();
//		delete msg;
//	}
//	ulapi_mutex_give( mutex );
//}

///////////
ACI_Msg* AgvComm::readMessage()
{
	if( isConnected() )
	{
		return ACI_read_message( remoteID );
	}
	return 0;
}


bool AgvComm::writeMessage( ACI_Msg &msg )
{
	if( isConnected() )
	{
		//ulapi_mutex_take( mutex );
		msg.write( remoteID );
		//ulapi_mutex_give( mutex );
	}

	return isConnected();
}

/////////////
// Disconnect network connection to force a reconnect.
void AgvComm::disconnect()
{
	if( connected )
		ulapi_socket_close( remoteID );
	connected = false;
}

////////////
// Query the AGV for position and target data.  
// Assumes Transport Structure index == 0,  and the TS needs to fill in the 
//     position values from OM.
bool AgvComm::getStatus(double &x, double &y, double &angle, int &target )
{
	// clear old reply messages out of the ACI socket
	//flushMessages();

	// send message requesting local parameter data
	ACI_Local_Parameter_Msg request;

	request.index = 0;  // assume the TS is running under index 0
	request.pcount = 4;
	request.p[0] = 0;  // x position
	request.p[1] = 1;  // y position
	request.p[2] = 2;  // heading
	request.p[3] = 3;  // target id, or 0 for robot stowed

	//writeMessage( request );

	// look for reply to parameter request
	bool done = false;
	bool retry = true;
	while( !done )
	{
		if( retry )
		{
			writeMessage( request );
			retry = false;
		}

		ACI_Msg *msg = readMessage();

		// see if message matches response type
		ACI_Parameter_Area_Contents_Msg *params = dynamic_cast<ACI_Parameter_Area_Contents_Msg*>(msg);
		if( params )
		{
			// return AGV status values
			x = params->p_val[0];
			y = params->p_val[1];
			angle = params->p_val[2];
			target = params->p_val[3];

			done = true;
		}
		else
		{
			ACI_Order_Acknowledge_Msg *order_ack = dynamic_cast<ACI_Order_Acknowledge_Msg*>(msg);
			if( order_ack )
			{
				if( order_ack->status == 14 )
				{
					cout << "getStatus: Transport structure not running, reconnecting" << endl;
					disconnect();
					ulapi_sleep(2.0);
					retry = true;
				}
				else
				{
					cout << "getStatus: Order ACK received but status was not 14, status = " 
						<< (int)order_ack->status << endl;
				}
			}
			else
			{
				cout << "getStatus: Unexpected response received:" << endl;
				//msg->print();
			}
		}

		// reclaim memory used by message structure
		if( msg )
			delete msg;
	}

	return true;
}

// Sets the target local parameter in the transport structure.
// Assumes Transport Structure index == 0, and target is parameter 3
bool AgvComm::setTarget( int target )
{
	ACI_Local_Parameter_Msg msg;
	msg.index = 0;
	//msg.function = ACI_Local_Parameter_Msg::REQUESTED_PARAMETER_UPDATE;
	msg.function = ACI_Local_Parameter_Msg::SPONTANEOUS_UPDATE;
	msg.pcount = 1;
	msg.par_no = 3;
	msg.p[0] = target;

	bool ok = writeMessage( msg );

	return ok;
}
