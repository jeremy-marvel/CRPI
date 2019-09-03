/* Interface for communicating with the AGV Controller */

#ifndef __INC_AGV_COMM_H__
#define __INC_AGV_COMM_H__

#include "ACI_msg.h"

#include <string>

//////////////////////////////////////////////////////////////////////
class AgvComm
{
public:
	AgvComm( std::string ipAddress, int port );
	~AgvComm();

	void setVerbose( bool verbose );
	bool getStatus( double &x, double &y, double &angle, int &target );
	bool setTarget( int target );
	void disconnect();

	bool writeMessage( ACI_Msg &msg );
	//bool readMessage( ACI_Msg *msg, double timeout );
	ACI_Msg* readMessage();

private:
	int port;
	std::string address;
	bool verbose;
	bool connected;
	//int messageCount;

	// guarded interface to message queue
	//ulapi_mutex_struct *mutex;
	//ulapi_task_struct *task;
	//list<ACI_Msg*> messageQueue;
	ulapi_integer remoteID;

	static void receiveTask( void *param );
	bool isConnected();
	//void flushMessages();
};

#endif //__INC_AGV_COMM_H__
