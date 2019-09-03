#include "cheetah.h"
#include <conio.h>
#include <windows.h>

#include <getopt.h>
#include <assert.h>
#include <iostream>
//#include <vector>
#include <queue>

// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdh/dsa.h"
#include "sdhoptions.h"
#include "dsaboost.h"
#include "simpletime.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/random.hpp>
#include <boost/bind.hpp>

#include "ulapi.h"

USING_NAMESPACE_SDH;
using namespace std;

char const* usage =  "stuff";
char const* __version__   = "v1";
char const* __help__      = "stuff";

#define SDH_USE_BINARY_COMMUNICATION 1

#define REQUEST_MSG_SIZE 2048

cDBG cdbg( false, "red" );

int main(int argc, char** argv)
	{
	SDH_ASSERT_TYPESIZES();

	//SET UP HAND ---------------------------------------------------------
	// handle command line options: set defaults first then overwrite by parsing actual command line

	cSDHOptions options;
	
	int unused_opt_ind;
	unused_opt_ind = options.Parse( argc, argv, __help__, "hand-testing", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
	
	//---------------------
	// initialize debug message printing:
	cDBG cdbg( options.debug_level > 0, "red", options.debuglog );
	g_sdh_debug_log = options.debuglog;

	cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";

	// reduce debug level for subsystems
	options.debug_level-=1;
	

	//---------------------
	
	try
	{
		// cSDH instance "hand" of the class cSDH according to the given options:
		cSDH hand(options.use_fahrenheit, options.debug_level ); //cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
		cdbg << "Successfully created cSDH instance\n";
		
		// Open configured communication to the SDH device
		options.OpenCommunication( hand );
		cdbg << "Successfully opened communication to SDH\n";
		
		cdbg << "Caption:\n";
		if (options.period)
			cdbg << "  times are reported in seconds\n";

		cdbg << "  angles are reported in " << hand.uc_angle->GetName() << "[" << hand.uc_angle->GetSymbol() << "]\n";
		
		//Configure Biotac
		// Create a global instance "ts" (tactile sensor) of the class cDSA according to the given options:

		// overwrite user given value
		options.framerate = 30;
		options.timeout = 1.0; // a real timeout is needed to make the closing of the connections work in case of errors / interrupt
		
		cDSA ts = cDSA( options.debug_level, options.dsaport, options.dsa_rs_device );
		//ts.SetFramerate( 30, true,true );
		//ts.SetFramerateRetries(30,true,true,100,true);

		cout << "SDH Boot Successful" << endl;
		//-------------------------------------------------------------------------------------------------

		

		//MAIN PROGRAM INIT ---------------------------------------------------------
		//Configure variables

		const double PI = 3.14159265;

		//Ready pose
		vector <double> pos(7);
		pos[0] = 60.0;
		pos[1] = -35;
		pos[2] = 35;
		pos[3] = -35;
		pos[4] = 35;
		pos[5] = -35;
		pos[6] = 35;

		vector <double> curr_pos(7); //actual joint position

		//Default closing velocity
		vector <double> vel(7), vel4pos(7);
		vel[0] = 0;
		vel[1] = 0;
		vel[2] = 0;
		vel[3] = 0;
		vel[4] = 0;
		vel[5] = 0;
		vel[6] = 0;

		vel4pos[0] = vel4pos[1] = vel4pos[2] = vel4pos[3] = vel4pos[4] = vel4pos[5] = vel4pos[6] = 83;

		vector <int> axes(7);
		for (unsigned int i=0;i<7;++i)
		{
			axes[i] = i;
		}

		int key=0;
		int num_fingers = 3;
				
		//Socketing---------------------------------------------------------------------------------------------
		ulapi_integer server, client;

		int  get, sent, x;
		char inbuffer[REQUEST_MSG_SIZE], outbuffer[REQUEST_MSG_SIZE];

		cout << "Waiting for connection..." << endl;
		server = ulapi_socket_get_server_id(6009);
		ulapi_socket_set_blocking(server); //ulapi_socket_set_nonblocking(server);
		client = ulapi_socket_get_connection_id(server);

		double touch_force = 0.0; //.3;

		while (1)
		{
			for (x = 0; x < REQUEST_MSG_SIZE; ++x)
			{
				inbuffer[x] = '\0';
			}

			get = ulapi_socket_read(client, inbuffer, REQUEST_MSG_SIZE);

			//! Read and parse joint commands from socket message
			char* pEnd;

			pos[0] = strtod (inbuffer, &pEnd); //get first angle
			for (unsigned short int jj=1;jj<=5;++jj) //gets other angles
			{
				pos[jj] = strtod (pEnd, &pEnd);
			}
			pos[6] = strtod (pEnd, NULL); //get last angle

			//Command robot to joint position
			hand.EmergencyStop();
			hand.SetController(hand.eCT_POSE);
			hand.SetAxisTargetAngle(axes,pos);
			hand.SetAxisTargetVelocity(axes,vel4pos);
			hand.MoveAxis(axes);
			hand.WaitAxis(axes);

			sent = ulapi_socket_write(client, "Success", sizeof(outbuffer));

			cout << "finished joint move" << endl;

			//Indicate 1) intent to grasp, -1) kill hand, 0) just a joint move
			for (x = 0; x < REQUEST_MSG_SIZE; ++x)	{inbuffer[x] = '\0';}
			get = ulapi_socket_read(client, inbuffer, REQUEST_MSG_SIZE);
			if (get > 0) {sent = ulapi_socket_write(client, "Success", sizeof(outbuffer));} // action completed

			if (strcmp(inbuffer,"Stop")==0) {break;} //kill hand

			if (strcmp(inbuffer,"Grasp")==0)
			{
				//Set number of closing fingers
				for (x = 0; x < REQUEST_MSG_SIZE; ++x)
				{
					inbuffer[x] = '\0';
				}
				get = ulapi_socket_read(client, inbuffer, REQUEST_MSG_SIZE);
				if (strcmp(inbuffer,"2")==0) {num_fingers = 2;}
				else if (strcmp(inbuffer,"3")==0) {num_fingers = 3;}
				else {cout << "INCORRECT NUMBER OF FINGERS SELECTION!" << endl; break;}

				cout << "closing" << endl;
				//Switch to closing
				if (num_fingers == 3) {vel[1] = vel[2] = vel[3] = vel[4] = vel[5] = vel[6] = 40;}
				else if (num_fingers == 2) {vel[1] = vel[2] = vel[5] = vel[6] = 40;};
				hand.SetController(hand.eCT_VELOCITY_ACCELERATION);
				hand.SetAxisTargetGetAxisActualVelocity(axes,vel);
				
				while (1)
				{
					ts.SetFramerate(0,true,true); //take only a single frame of data, this prevents previous issues
					ts.UpdateFrame();				

					//Look for sufficient grasp force, and halt when there
					if (ts.GetContactInfo(1).force > touch_force) {vel[2] = 0;}
					if (ts.GetContactInfo(3).force > touch_force && num_fingers == 3) {vel[4] = 0;}
					if (ts.GetContactInfo(5).force > touch_force) {vel[6] = 0;}
					if (ts.GetContactInfo(1).force > touch_force && ts.GetContactInfo(3).force > touch_force && ts.GetContactInfo(5).force > touch_force && num_fingers == 3) 
					{

						sent = ulapi_socket_write(client, "Success", sizeof(outbuffer));
						//Set velocities to 0
						for (unsigned int i=0;i<7;++i)
						{
							vel[i] = 0;
						}
						hand.SetAxisTargetGetAxisActualVelocity(axes,vel);

						break;
					}
					if (ts.GetContactInfo(1).force > touch_force && ts.GetContactInfo(5).force > touch_force && num_fingers == 2) 
					{

						sent = ulapi_socket_write(client, "Success", sizeof(outbuffer));
						//Set velocities to 0
						for (unsigned int i=0;i<7;++i)
						{
							vel[i] = 0;
						}
						hand.SetAxisTargetGetAxisActualVelocity(axes,vel);

						break;
					}

					hand.SetAxisTargetGetAxisActualVelocity(axes,vel);

					//Manual Override
					if (kbhit())
					{
						key = getch();
						if(key == 27)
						{
							break;
						}
					}
				}
			}

			//Manual Override
			if(key == 27)
			{
				break;
			}
		}
		
		ts.Close();
		hand.EmergencyStop();
		hand.Close();
		cout << "Hand deactivated" << endl;

		sent = ulapi_socket_write(client, "Success", sizeof(outbuffer));
	}

	catch ( cSDHLibraryException* e )
	{
		cerr << "main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
		delete e;
	}

	return 0;
}