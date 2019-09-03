#ifndef __AGV_STATUS_HH__
#define __AGV_STATUS_HH__

#include <string>
#include <ulapi.h>

#define DEFAULT_STATUS_PORT 5432
#define DEFAULT_STATUS_ADDRESS "192.168.160.3"

const int AGV_STATE_MUTEX_ID = 55;

struct AGV_State
{
	double timestamp;
	double pos_x;
	double pos_y;
	double pos_th;
	int pos_known;
	int navlevel;
};

class AGV_Status
{
public:
	AGV_Status( ulapi_prio priority, 
		std::string address = DEFAULT_STATUS_ADDRESS, 
		int port = DEFAULT_STATUS_PORT );
	~AGV_Status();

	void set_verbose( bool value );
	AGV_State get_state();
	bool is_connected() { return _connected; }

private:
	bool _verbose;
	bool _connected;
	ulapi_integer _socket;

	std::string _address;
	int _port;
	ulapi_task_struct *task;

	ulapi_mutex_struct *_mutex;
	AGV_State _agv_state;

	static void run_thread( void *this_ptr );
	void run();
	ulapi_result read_message( std::string &message );
	const char* ulapi_to_string( ulapi_result retval );
};



#endif // __AGV_STATUS_HH__

