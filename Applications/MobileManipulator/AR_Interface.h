/*
Interface for communicating with AR Toolkit process on Beckoff computer.
*/

#ifndef __INC_AR_INTERFACE_H__
#define __INC_AR_INTERFACE_H__

#include "posemath.h"
#include "ulapi.h"

struct AR_Status
{
	bool new_data;
	double time_stamp;
	int marker_id;
	PM_POSE pose;
};

class AR_Interface
{
public:
	static AR_Interface *Get_Instance();

	void Init( ulapi_prio priority, int key );
	AR_Status Get_Status();

private:
	static AR_Interface *instance;  // singleton class instance

	bool initialized;
	ulapi_task_struct *task;
	ulapi_mutex_struct *mutex;
	ulapi_integer socket_id;
	AR_Status ar_status;
	
	AR_Interface();
	static void AR_Thread( void *ar_interface );
};


#endif
