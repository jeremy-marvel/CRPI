// Hand_Test_v2.cpp : Testing with SDH
//to run: hand_Test_v2 -p 12 --dsaport=13 in working directory
//need to have cheetah.dll and SDHLibrary.dll in Debug folder

#include "SDH_Control.h"
#include "force_controller.h"
#include "COP_Normal.h"
#include "Object_Tracking.h"
#include <stack>
#include "manipulation_control.h"
#include "Weiss_force_COP.h"
#include "force_record.h"
#include "ulapi.h"
#include <armadillo>

#define SDH_USE_BINARY_COMMUNICATION 1

#define REQUEST_MSG_SIZE 2048

std::stack<bt_data> pdc_shared_data;
char grasping_fault[10] = "Go";
boost::mutex pdc_shared_data_mutex;

static boost::thread_specific_ptr<std::vector<bt_data> > data_handler;

char const* usage =  "stuff";
char const* __version__   = "v1";
char const* __help__      = "stuff";

using namespace std;
USING_NAMESPACE_SDH

Cheetah 	bt_cheetah_initialize(const bt_info *);
BioTac 		bt_cheetah_get_properties(Cheetah, int, bt_property *);
BioTac 		bt_cheetah_configure_batch(Cheetah, bt_info *, int);
bt_data*	bt_configure_save_buffer(int num_samples);
void 		bt_cheetah_collect_batch(Cheetah, const bt_info *, bt_data *, BOOL);
void 		bt_cheetah_collect_batch_once(Cheetah, const bt_info *, bt_data *, BOOL);
void		bt_display_errors(BioTac);
void 		bt_save_buffer_data(const char *, const bt_data *, int);
void 		bt_cheetah_close(Cheetah);

cDBG cdbg( false, "red" );

int bt_main(void)
{
	/****************************/
	/* --- Define variables --- */
	/****************************/
	bt_info biotac;
    bt_property biotac_property[MAX_BIOTACS_PER_CHEETAH];
	bt_data *data;
	BioTac bt_err_code;
	Cheetah ch_handle;

	int i;
	int length_of_data_in_second;
	int number_of_samples;
	int number_of_loops;

    /**************************************************************************/
	/* --- Initialize BioTac settings (only default values are supported) --- */
    /**************************************************************************/
	biotac.spi_clock_speed = BT_SPI_BITRATE_KHZ_DEFAULT;
	biotac.number_of_biotacs = 0;
	biotac.sample_rate_Hz = BT_SAMPLE_RATE_HZ_DEFAULT;
	biotac.frame.frame_type = 0;
	biotac.batch.batch_frame_count = BT_FRAMES_IN_BATCH_DEFAULT;
	biotac.batch.batch_ms = BT_BATCH_MS_DEFAULT;

	// Set the duration of the run time
	length_of_data_in_second = 10;
	number_of_samples = 5*44; //ie one per sensor //(int)(BT_SAMPLE_RATE_HZ_DEFAULT * length_of_data_in_second);

	// Check if any initial settings are wrong
	if (MAX_BIOTACS_PER_CHEETAH != 3 && MAX_BIOTACS_PER_CHEETAH != 5)
	{
		bt_err_code = BT_WRONG_MAX_BIOTAC_NUMBER;
		bt_display_errors(bt_err_code);
		exit(1);
	}

    /******************************************/
	/* --- Initialize the Cheetah devices --- */
    /******************************************/
	ch_handle = bt_cheetah_initialize(&biotac);

	
	/*********************************************************/
	/* --- Get and print out properties of the BioTac(s) --- */
	/*********************************************************/

	for (i = 0; i < MAX_BIOTACS_PER_CHEETAH; i++)
	{
		bt_err_code = bt_cheetah_get_properties(ch_handle, i+1, &(biotac_property[i]));
		bt_display_errors(bt_err_code);
		if (biotac_property[i].bt_connected == YES)
		{
			(biotac.number_of_biotacs)++;
		}

		if (bt_err_code)
		{
			bt_display_errors(bt_err_code);
			exit(1);
		}
	}

	// Check if any BioTacs are detected
	if (biotac.number_of_biotacs == 0)
	{
		bt_err_code = BT_NO_BIOTAC_DETECTED;
		bt_display_errors(bt_err_code);
		return bt_err_code;
	}
	else
	{
		printf("\n%d BioTac(s) detected.\n\n", biotac.number_of_biotacs);
	}
	
	// The programs stops here until it accepts [Enter] key hit
	printf("Press [Enter] to continue ...");
	fflush(stdout);
	getchar();

	/*************************************/
	/* --- Configure the save buffer --- */
	/*************************************/
	data = bt_configure_save_buffer(number_of_samples);

	/*******************************/
	/* --- Configure the batch --- */
	/*******************************/
	bt_err_code = bt_cheetah_configure_batch(ch_handle, &biotac, number_of_samples);
	if (bt_err_code < 0)
	{
		bt_display_errors(bt_err_code);
		exit(1);
	}
	else
	{
		printf("\nConfigured the batch\n");
	}
	
	//***************************************************************/
	//* --- Collect the batch and display the data (if desired) --- */
	//***************************************************************/
	number_of_loops = (int)(number_of_samples / ((double)(biotac.frame.frame_size * biotac.batch.batch_frame_count)));
	//number_of_loops = number_of_loops/5; //fixed to gather only 44 samples per batch ie the number of sensors
	printf("Start collecting BioTac data for %d second(s)...\n", length_of_data_in_second);
	while (true) //for (i = 0; i < 20; i++)
	{
		try
		{
			bt_cheetah_collect_batch(ch_handle, &biotac, data, NO);
		}

		catch(boost::thread_interrupted const& )
		{
			//clean resources
			std::cout << "Cleaning up Biotac thread" << std::endl;
			free(data);
			bt_cheetah_close(ch_handle);
			break;
		}
		/*
		int key;
		if (kbhit())
		{
			key = getch();
			if(key == 27)
			{
				cout << "Biotac sensing stopped" << endl;
				break;
			}
		}*/
		
	}
    return 0;
}

int grasp_listener() //Threaded function to report various hand status
{
	ulapi_integer server, client;

	int  get, sent, x;
	char inbuffer[REQUEST_MSG_SIZE], outbuffer[REQUEST_MSG_SIZE];

	cout << "Waiting for connection..." << endl;
	server = ulapi_socket_get_server_id(6008);
	ulapi_socket_set_blocking(server);
	client = ulapi_socket_get_connection_id(server);

	cout << "Client connected!" << endl;

	while (true)
	{
		if (!boost::this_thread::interruption_requested())
		{
			//cout << "inside" << endl;
			while (strcmp(grasping_fault,"Go")==0)
			{
				for (x = 0; x < REQUEST_MSG_SIZE; ++x) {inbuffer[x] = '\0';}

				get = ulapi_socket_read(client, inbuffer, REQUEST_MSG_SIZE);

				//Now that it received something, get a mutex lock so you can write to a shared variable
				//cout << "fault thread locking" << endl;
				while (!pdc_shared_data_mutex.try_lock()) {};

				//! Read data from client
				if (strcmp(inbuffer,"Stop")==0) {strcpy(grasping_fault,"Stop");}

				//sent = ulapi_socket_write(client, "pregrasp successful", sizeof(outbuffer));
				pdc_shared_data_mutex.unlock();
			}
		}
		
		else
		{
			//Kill thread
			std::cout << "Killing grasping status thread" << std::endl;

			break;
		}
	}

	return 0;
}

void run(int argc, char** argv, char* filename, char* start, char* end, double force_mag_des, string finger, bool engage_manipulation, bool use_biotacs)
{
	SDH_ASSERT_TYPESIZES();

    //---------------------
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
        cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
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

        printf( "Connecting to tactile sensor controller. This may take up to 8 seconds..." );
		cout << options.dsaport << endl;
		cout << options.dsa_rs_device << endl;
        cDSA ts = cDSA( options.debug_level, options.dsaport, options.dsa_rs_device );
        printf( "OK\n" );

		//Starts code for movement

		hand.UseRadians();
		
		cSimpleTime timer;

		timer.StoreNow();

		//Initializations-------------------------------------------------------------------------------------------
		cout << "Initializing..." << endl;
		const double PI  =3.141592653589793238463;
		double t=0,pos0,t_dir=0,theta=0, elapsed=0, del_t=0, t_control=0;
		int total = 0;
		//if (use_biotacs==false) {ts.SetFramerate( 30, true );}
		double f_weiss=0;
		
		vector <int> axes(7);
		vector <double> qd_dot_vec(7);
		vector <double> q_dot_vec(7);
		vector <double> angles_vec(7);
		arma::mat f_total;
		f_total.zeros(3,1);

		for (unsigned int i=0;i<7;++i)
		{
			axes[i] = i;
			q_dot_vec[i] = 0;
			qd_dot_vec[i] = 0;
			angles_vec[i] = 0;
		}
		
		double finger0_angles[2]={0,0}; // will have to make this of size 3 and compensate for extra angle
		double finger1_angles[2]={0,0}; // split up
		double finger2_angles[2]={0,0}; // will have to make this of size 3 and compensate for extra angle
		
		//Ready pose
		vector <double> pos(7);
		pos[0] = 60.0*(PI/180);
		pos[1] = -35*(PI/180); //-35 //admittance tranpose controller: -5,15 for J1's and J2's or -10 30
		pos[2] = 35*(PI/180); // -35 35 sphere or cube
		pos[3] = -35*(PI/180);
		pos[4] = 35*(PI/180);
		pos[5] = -35*(PI/180);
		pos[6] = 35*(PI/180);

		//Grasp State
		bool grasped=false;
		int key;
		bool inserted = false;

		//Set up data recorders
		force_record force_record0;
		force_record force_record1;
		force_record force_record2;

		//Initialize class objects regardless if finger selected
		force_control finger0;
		force_control finger1;
		force_control finger2;

		COP_Normal cop0;
		COP_Normal cop1;
		COP_Normal cop2;

		Weiss_force_COP Weiss0;
		Weiss_force_COP Weiss1;
		Weiss_force_COP Weiss2;

		double rc_ic[6] = {0,0,0.150,0,0,0}; //{0,0,0.152,0,0,0}; //{0,0,0.152,0,0,0}; //expressed in coordinate system on middle surface of palm
		Object_Tracking object_pose(rc_ic);
		manipulation_control manipulate(rc_ic);

		//Set up objects of selected fingers
		if (finger == "finger0" || finger == "all")
		{
			finger0.initialize("finger0",force_mag_des,filename, start, end);	
			finger0.constructNN();
		}

		if (finger == "finger1" || finger == "all")
		{
			finger1.initialize("finger1",force_mag_des,filename, start, end);	
			finger1.constructNN();
		}

		if (finger == "finger2" || finger == "all")
		{
			finger2.initialize("finger2",force_mag_des,filename, start, end);	
			finger2.constructNN();
		}

		//cSimpleTime timer;
		timer.StoreNow();

		//Allocate for biotacs
		int biotac0_value[44]={};
		int biotac1_value[44]={};
		int biotac2_value[44]={};

		bool new_data = false;

		//hand.SetController(hand.eCT_VELOCITY);
		//hand.SetController(hand.eCT_VELOCITY_ACCELERATION);
		
		timer.StoreNow();
		
		boost::thread workerThread(bt_main);
		boost::thread workerThread2(grasp_listener);

		if (data_handler.get() == NULL) 
			{
				data_handler.reset(new std::vector<bt_data>());
				data_handler->resize(44);
			}
		t = double(timer.Elapsed_us())/1e6;

		//Socketing---------------------------------------------------------------------------------------------
		ulapi_integer server, client, client_fault, server_fault, server_collision, client_collision;

		int  get, sent, x, sent_fault, sent_collision;
		char inbuffer[REQUEST_MSG_SIZE], outbuffer[REQUEST_MSG_SIZE], outbuffer_collision[REQUEST_MSG_SIZE];

		cout << "Waiting for connection..." << endl;
		server = ulapi_socket_get_server_id(6007);
		server_fault = ulapi_socket_get_server_id(6009);
		server_collision = ulapi_socket_get_server_id(6010);
		ulapi_socket_set_blocking(server);
		ulapi_socket_set_blocking(server_fault);
		ulapi_socket_set_blocking(server_collision);
		client = ulapi_socket_get_connection_id(server);
		client_fault = ulapi_socket_get_connection_id(server_fault);
		client_collision = ulapi_socket_get_connection_id(server_collision);

		cout << "Client connected!" << endl;

		//Create thread for checking grasping commandsv\

		while(1)
		{

		//Setting SDH pregrasp shape----------------------------------------------------------------------------
		hand.SetController(hand.eCT_POSE);

		bool pregrasp=false;
		cout << "Select a pre-grasp" << endl;
		while (pregrasp==false)
		{
			for (x = 0; x < REQUEST_MSG_SIZE; ++x)
			{
			inbuffer[x] = '\0';
			}

			get = ulapi_socket_read(client, inbuffer, REQUEST_MSG_SIZE);
			//! Read data from client
			if (get > 0)
			{

			if (strcmp(inbuffer,"GRIP_TYPE,1")==0)
			{
				cout << "Tripod Chosen" << endl;
				pos[0] = 60.0*(PI/180);
				pos[1] = -35*(PI/180); //-35 
				pos[2] = 35*(PI/180); //35
				pos[3] = -35*(PI/180);
				pos[4] = 35*(PI/180);
				pos[5] = -35*(PI/180);
				pos[6] = 35*(PI/180);
			}

			else if (strcmp(inbuffer,"GRIP_TYPE,2")==0)
			{
				cout << "Lateral Chosen" << endl;
				pos[0] = 0.0*(PI/180);
				pos[1] = -35*(PI/180); //-35 //admittance tranpose controller: -5,15 for J1's and J2's or -10 30
				pos[2] = 35*(PI/180); // -35 35 sphere or cube
				pos[3] = -35*(PI/180);
				pos[4] = 35*(PI/180);
				pos[5] = -35*(PI/180);
				pos[6] = 35*(PI/180);
			}

			if (strcmp(inbuffer,"GRIP_TYPE,3")==0)
			{
				cout << "Acute Tripod Chosen" << endl;
				pos[0] = 60.0*(PI/180);
				pos[1] = -10*(PI/180); //-35 //admittance tranpose controller: -5,15 for J1's and J2's or -10 30
				pos[2] = 30*(PI/180); // -35 35 sphere or cube
				pos[3] = -10*(PI/180);
				pos[4] = 30*(PI/180);
				pos[5] = -10*(PI/180);
				pos[6] = 30*(PI/180);
			}

			hand.SetAxisTargetAngle(axes,pos);
			//cout << hand.GetAxisActualAngle(1) << " " << hand.GetAxisActualAngle(2) << endl;
			//cout << "HEEEEEEEEEEEEEEEEEERE" << endl;
			hand.MoveAxis(axes);
			hand.WaitAxis(axes);
			hand.Stop();

			pregrasp=true;

			sent = ulapi_socket_write(client, "pregrasp successful", sizeof(outbuffer));
			}
		}

		//Wait for command to engage grasping--------------------------------------
		bool engage_grasping=false;
		cout << "Engage Grasping?" << endl;
		while (engage_grasping==false)
		{
			for (x = 0; x < REQUEST_MSG_SIZE; ++x)	{inbuffer[x] = '\0';}

			get = ulapi_socket_read(client, inbuffer, REQUEST_MSG_SIZE);
			//! Read data from client
			if (get > 0)
			{
				if (strcmp(inbuffer,"100")==0)	{engage_grasping=true; sent = ulapi_socket_write(client, "engaging grasping", sizeof(outbuffer));}
				if (strcmp(inbuffer,"-100")==0)	{engage_grasping=true;}
				
			}
		}

		//Check to see if SDH needs to be shut down!!!
		if (strcmp(inbuffer,"-100")==0) {"Shutting down SDH"; break;}

		cout << "Engaging Grasping" << endl;

		//Engage Grasping/Manipulation----------------------------------------------
					
		//--------------Control------------------------------------------------------
		
		//hand.SetController(hand.eCT_VELOCITY);
		hand.SetController(hand.eCT_VELOCITY_ACCELERATION);
		bool grasp=true;
		int force_checking_delay=0;

		///*UNCOMMENT THIS TO GET MANIPULATION BACK
		//Reset controllers & sensory signals
		if (finger == "finger0" || finger == "all") {finger0.reset(); cop0.reset();}
		if (finger == "finger1" || finger == "all") {finger1.reset(); cop1.reset();}
		if (finger == "finger2" || finger == "all") {finger2.reset(); cop2.reset();}
	
		object_pose.reset(rc_ic);
		manipulate.reset(rc_ic);
		//*/

		//Clear faults
		while (!pdc_shared_data_mutex.try_lock()) {};
		strcpy(grasping_fault,"Go");
		pdc_shared_data_mutex.unlock();
		grasped = false;
		manipulate.collision = false;
		inserted = false;
		while(grasp=true)
		{		
			//boost::unique_lock<boost::mutex> lock(pdc_shared_data_mutex);

			if (use_biotacs == true)
			{
				//cout << "control thread locking" << endl;
				while (!pdc_shared_data_mutex.try_lock()) {};


				//cout << "inside size: " << pdc_shared_data.size() <<endl;
				//cin.get();
				if (!pdc_shared_data.empty())
				{
					//cout << "pdc_size: " << pdc_shared_data.size() << endl;
					//cout << "data_size: " << data_handler->size() << endl;
					//cout << "reading" << endl;

					if (pdc_shared_data.size() < 44) {total = pdc_shared_data.size();}
					else {total=44;};

					for (int SIZE=0; SIZE < total; ++SIZE)
					{
						//cout << "pdc size: " << pdc_shared_data.size() << endl;
						(*data_handler)[SIZE]=pdc_shared_data.top();
						pdc_shared_data.pop();
						new_data = true;
					}

					while(!pdc_shared_data.empty())
					{
						//data_handler->push_back(pdc_shared_data.front());
						pdc_shared_data.pop();
					}
					//cout << "pdc_size: " << pdc_shared_data.size() << endl;
					//cout << "data_size: " << data_handler->size() << endl;
				}
				//boost::unique_lock<boost::mutex> unlock(pdc_shared_data_mutex);
				pdc_shared_data_mutex.unlock();

				if (new_data == true)
				{
					for (unsigned short int i=0;i<total;++i)
					{

						switch((*data_handler)[i].channel_id)
						{
						case 17:
							{
								biotac0_value[0] = (*data_handler)[i].d[0].word;
								biotac1_value[0] = (*data_handler)[i].d[1].word;
								biotac2_value[0] = (*data_handler)[i].d[2].word;
								break;
							}
						case 18:
							{
								biotac0_value[1] = (*data_handler)[i].d[0].word;
								biotac1_value[1] = (*data_handler)[i].d[1].word;
								biotac2_value[1] = (*data_handler)[i].d[2].word;
								break;
							}
						case 19:
							{
								biotac0_value[2] = (*data_handler)[i].d[0].word;
								biotac1_value[2] = (*data_handler)[i].d[1].word;
								biotac2_value[2] = (*data_handler)[i].d[2].word;
								break;
							}
						case 20:
							{
								biotac0_value[3] = (*data_handler)[i].d[0].word;
								biotac1_value[3] = (*data_handler)[i].d[1].word;
								biotac2_value[3] = (*data_handler)[i].d[2].word;
								break;
							}
						case 21:
							{
								biotac0_value[4] = (*data_handler)[i].d[0].word;
								biotac1_value[4] = (*data_handler)[i].d[1].word;
								biotac2_value[4] = (*data_handler)[i].d[2].word;
								break;
							}
						case 22:
							{
								biotac0_value[5] = (*data_handler)[i].d[0].word;
								biotac1_value[5] = (*data_handler)[i].d[1].word;
								biotac2_value[5] = (*data_handler)[i].d[2].word;
								break;
							}
						case 23:
							{
								biotac0_value[6] = (*data_handler)[i].d[0].word;
								biotac1_value[6] = (*data_handler)[i].d[1].word;
								biotac2_value[6] = (*data_handler)[i].d[2].word;
								break;
							}
						case 24:
							{
								biotac0_value[7] = (*data_handler)[i].d[0].word;
								biotac1_value[7] = (*data_handler)[i].d[1].word;
								biotac2_value[7] = (*data_handler)[i].d[2].word;
								break;
							}
						case 25:
							{
								biotac0_value[8] = (*data_handler)[i].d[0].word;
								biotac1_value[8] = (*data_handler)[i].d[1].word;
								biotac2_value[8] = (*data_handler)[i].d[2].word;
								break;
							}
						case 26:
							{
								biotac0_value[9] = (*data_handler)[i].d[0].word;
								biotac1_value[9] = (*data_handler)[i].d[1].word;
								biotac2_value[9] = (*data_handler)[i].d[2].word;
								break;
							}
						case 27:
							{
								biotac0_value[10] = (*data_handler)[i].d[0].word;
								biotac1_value[10] = (*data_handler)[i].d[1].word;
								biotac2_value[10] = (*data_handler)[i].d[2].word;
								break;
							}
						case 28:
							{
								biotac0_value[11] = (*data_handler)[i].d[0].word;
								biotac1_value[11] = (*data_handler)[i].d[1].word;
								biotac2_value[11] = (*data_handler)[i].d[2].word;
								break;
							}
						case 29:
							{
								biotac0_value[12] = (*data_handler)[i].d[0].word;
								biotac1_value[12] = (*data_handler)[i].d[1].word;
								biotac2_value[12] = (*data_handler)[i].d[2].word;
								break;
							}
						case 30:
							{
								biotac0_value[13] = (*data_handler)[i].d[0].word;
								biotac1_value[13] = (*data_handler)[i].d[1].word;
								biotac2_value[13] = (*data_handler)[i].d[2].word;
								break;
							}
						case 31:
							{
								biotac0_value[14] = (*data_handler)[i].d[0].word;
								biotac1_value[14] = (*data_handler)[i].d[1].word;
								biotac2_value[14] = (*data_handler)[i].d[2].word;
								break;
							}
						case 32:
							{
								biotac0_value[15] = (*data_handler)[i].d[0].word;
								biotac1_value[15] = (*data_handler)[i].d[1].word;
								biotac2_value[15] = (*data_handler)[i].d[2].word;
								break;
							}
						case 33:
							{
								biotac0_value[16] = (*data_handler)[i].d[0].word;
								biotac1_value[16] = (*data_handler)[i].d[1].word;
								biotac2_value[16] = (*data_handler)[i].d[2].word;
								break;
							}
						case 34:
							{
								biotac0_value[17] = (*data_handler)[i].d[0].word;
								biotac1_value[17] = (*data_handler)[i].d[1].word;
								biotac2_value[17] = (*data_handler)[i].d[2].word;
								break;
							}
						case 35:
							{
								biotac0_value[18] = (*data_handler)[i].d[0].word;
								biotac1_value[18] = (*data_handler)[i].d[1].word;
								biotac2_value[18] = (*data_handler)[i].d[2].word;
								break;
							}
						case 1:
							{
								biotac0_value[19] = (*data_handler)[i].d[0].word;
								biotac1_value[19] = (*data_handler)[i].d[1].word;
								biotac2_value[19] = (*data_handler)[i].d[2].word;
								break;
							}
						}
					}

					//for (unsigned short int n=0;n<20;++n)
					//{
					//	cout << " " << bt_cd_value[n];
					//}
					//t = double(timer.Elapsed_us())/1e6;

					//cout << "Biotac t: " << t << endl;
					//cout << "new data" << endl;

				}
				new_data = false;	
			}

			else //For Weiss sensors, get and set COP, force_mag and normals
			{
				ts.SetFramerate(0,true,true); //take only a single frame of data, this prevents previous issues
				ts.UpdateFrame();
				Weiss0.get_COP(ts.GetContactInfo(1).cog_x,ts.GetContactInfo(1).cog_y);
				Weiss1.get_COP(ts.GetContactInfo(3).cog_x,ts.GetContactInfo(3).cog_y);
				Weiss2.get_COP(ts.GetContactInfo(5).cog_x,ts.GetContactInfo(5).cog_y);
				
				//Brings in force, but also checks for contact based on force threshold
				Weiss0.get_f(ts.GetContactInfo(1).force,1);
				Weiss1.get_f(ts.GetContactInfo(3).force,3);
				Weiss2.get_f(ts.GetContactInfo(5).force,5);

				cop0.touched = Weiss0.touched;
				cop1.touched = Weiss1.touched;
				cop2.touched = Weiss2.touched;
			}

			t = double(timer.Elapsed_us())/1e6;
			//cout << t << endl;
			//Force Code
			if (t > 2)
			{
				//cout << "control start " << t << endl;
				if (t_control == 0) {t_control = double(timer.Elapsed_us())/1e6;};

				//cout << "cutaenous.............." << endl;
				//Use cutaneous sensory data for active fingers
				if (finger == "finger0" || finger == "all")
				{
					if (use_biotacs == true)
					{
						if (finger0.tared == false) {finger0.tare(biotac0_value);};
						if (cop0.tared == false) {cop0.tare(biotac0_value);};
						cop0.run(biotac0_value);
						finger0.fn = cop0.normal_filtered;
						finger0.convert2forces(biotac0_value);
					}

					else // using Weiss
					{
						finger0.fn = Weiss0.fn;
						finger0.f = Weiss0.f;
						finger0.rotate_forces();
					}
				}

				if (finger == "finger1" || finger == "all")
				{
					if (use_biotacs == true)
					{
						if (finger1.tared == false) {finger1.tare(biotac1_value);};
						if (cop1.tared == false) {cop1.tare(biotac1_value);};
						cop1.run(biotac1_value);
						finger1.fn = cop1.normal_filtered;
						finger1.convert2forces(biotac1_value);
					}
					
					else // using Weiss
					{
						finger1.fn = Weiss1.fn;
						finger1.f = Weiss1.f;
						finger1.rotate_forces();
					}
				}

				if (finger == "finger2" || finger == "all")
				{
					if (use_biotacs == true)
					{
						if (finger2.tared == false) {finger2.tare(biotac2_value);};
						if (cop2.tared == false) {cop2.tare(biotac2_value);};
						cop2.run(biotac2_value);
						finger2.fn = cop2.normal_filtered;
						finger2.convert2forces(biotac2_value);
					}

					else
					{
						finger2.fn = Weiss2.fn;
						finger2.f = Weiss2.f;
						finger2.rotate_forces();
					}
				}

				//Get angles for all joints regardless of active fingers, and place into angle arrays
				angles_vec = hand.GetAxisActualAngle(axes);
				//elapsed = double(timer.Elapsed_us())/1e6-t;

				//cout << "assigning joint angles" << endl;
				for (unsigned short int i=1;i<3;++i)
				{
					finger0_angles[i-1] = angles_vec[i];
					finger1_angles[i-1] = angles_vec[i+2];
					finger2_angles[i-1] = angles_vec[i+4];
				}
				
				//For active fingers, process kinematics
				if (finger == "finger0" || finger == "all")
				{
					finger0.set_joint_states(finger0_angles);

					if (use_biotacs == true) {finger0.getJacobian(cop0.COP_loc_filtered,angles_vec[0]);} //this rotates force normals as well
					else {finger0.getJacobian(Weiss0.COP,angles_vec[0]);}
					//cout << finger0.kin.end_effector_palm.t() << endl;
					//cout << finger0.f_palm.t() << endl;
				}

				if (finger == "finger1" || finger == "all")
				{
					finger1.set_joint_states(finger1_angles);

					if (use_biotacs == true) {finger1.getJacobian(cop1.COP_loc_filtered,angles_vec[0]);}
					else {finger1.getJacobian(Weiss1.COP,angles_vec[0]);}
					//cout << finger1.kin.end_effector_palm.t() << endl;
					//cout << finger1.f_palm.t() << endl;
				}

				if (finger == "finger2" || finger == "all")
				{
					finger2.set_joint_states(finger2_angles);

					if (use_biotacs == true) {finger2.getJacobian(cop2.COP_loc_filtered,angles_vec[0]);}
					else {finger2.getJacobian(Weiss2.COP,angles_vec[0]);}
					//cout << finger2.kin.end_effector_palm.t() << endl;
					//cout << finger2.f_palm.t() << endl;
				}

				//For active fingers, make control computations and emit control
				del_t = double(timer.Elapsed_us())/1e6-t_control;
				//cout << del_t << endl;
				if (del_t == 0) {del_t = .0001;}
				t_control = double(timer.Elapsed_us())/1e6;

				//cout << del_t << endl;
				if (engage_manipulation == false || grasped == false) //engage only force controllers
				{
					if (finger == "finger0" || finger == "all")
					{
						force_record0.append(t_control-2,finger0.f_palm);
						force_record0.f_d();
						force_mag_des = force_record0.force_d(1,0);
						finger0.f_d_palm << -force_mag_des << arma::endr << -force_mag_des << arma::endr << 0 << arma::endr; //finger0.f_d_palm << 0 << arma::endr << -force_mag_des << arma::endr << 0 << arma::endr;
						//finger0.f_d_palm = force_record0.force_d;
						finger0.desired_force();
						finger0.calc_control(del_t);
						//cout << finger0.f_palm.t() << endl;
						//cout << "cop: " << finger0.kin.end_effector_palm.t() << endl;
					};

					if (finger == "finger1" || finger == "all")
					{
						force_record1.append(t_control-2,finger1.f_palm);
						force_record1.f_d();
						force_mag_des = force_record1.force_d(1,0);
						finger1.f_d_palm << 0 << arma::endr << force_mag_des << arma::endr << 0 << arma::endr;
						//finger1.f_d_palm = force_record1.force_d;
						finger1.desired_force();
						finger1.calc_control(del_t);
						//cout << finger1.f_palm.t() << endl;
						//cout << "cop: " << finger1.kin.end_effector_palm.t() << endl;
					};

					if (finger == "finger2" || finger == "all")
					{
						force_record2.append(t_control-2,finger2.f_palm);
						force_record2.f_d();
						force_mag_des = force_record2.force_d(1,0);
						finger2.f_d_palm << force_mag_des << arma::endr << -force_mag_des << arma::endr << 0 << arma::endr; //finger2.f_d_palm << 0 << arma::endr << -force_mag_des << arma::endr << 0 << arma::endr;
						//finger2.f_d_palm = force_record2.force_d;
						finger2.desired_force();
						//cout << arma::norm(finger2.f_d,2) << endl;
						finger2.calc_control(del_t);
						//cout << finger2.f_palm.t() << endl;
						//cout << "cop: " << finger2.kin.end_effector_palm.t() << endl;
					};
					/*
					if (t>=5)
					{
						object_pose.run(del_t,finger0.kin.end_effector_palm,finger1.kin.end_effector_palm,finger2.kin.end_effector_palm);
						//cout << "rc: " << object_pose.rc.t() << endl;
					}*/

					//cout << finger0.fn.t() << endl;
					//cout << finger1.fn.t() << endl;
					//cout << finger2.fn.t() << endl;
					//cout << finger0.f_palm.t() << endl;
					//cout << finger1.f_palm.t() << endl;
					//cout << finger2.f_palm.t() << endl;
					//cout << "finger0" << finger0.kin.end_effector_palm.t() << endl;
					//cout << "finger1" << finger1.kin.end_effector_palm.t() << endl;
					//cout << "finger2" << finger2.kin.end_effector_palm.t() << endl;
					//object_pose.run(del_t,finger0.kin.end_effector_palm,finger1.kin.end_effector_palm,finger2.kin.end_effector_palm);
					//cout << object_pose.rc.t() << endl;

				}

				else //engage manipulation controller
				{
					//Update object states
					object_pose.run(del_t,finger0.kin.end_effector_palm,finger1.kin.end_effector_palm,finger2.kin.end_effector_palm);

					//cout << object_pose.rc.t() << endl;

					//Assign object states to manipulation controller internals
					manipulate.rc = object_pose.rc;
					manipulate.rc_dot = object_pose.rc_dot;
					manipulate.fn0_hat = finger0.fn;
					manipulate.fn1_hat = finger1.fn;
					manipulate.fn2_hat = finger2.fn;
					//cout << manipulate.fn0_hat.t() << endl;
					//cout << manipulate.fn1_hat.t() << endl;
					//cout << manipulate.fn2_hat.t() << endl;
					manipulate.f0 = finger0.f_palm;
					manipulate.f1 = finger1.f_palm;
					manipulate.f2 = finger2.f_palm;

					//cout << "f0f: " << finger0.f_palm.t() << endl;
					//cout << "f1f: " << finger1.f_palm.t() << endl;
					//cout << "f2f: " << finger2.f_palm.t() << endl;
					manipulate.p0 = finger0.kin.end_effector_palm.submat(0,0,2,0);
					manipulate.p1 = finger1.kin.end_effector_palm.submat(0,0,2,0);
					manipulate.p2 = finger2.kin.end_effector_palm.submat(0,0,2,0);

					//cout << "p0: " << finger0.kin.end_effector_palm.submat(0,0,2,0).t() << endl;

					//General comments: filter on COP and normals might need to be higher, or just different from each other
					
					//Step Manipulation Controller
					manipulate.calc_control(del_t);


					//Set desired finger forces and step force controllers
					finger0.f_d_palm = manipulate.f0_d;
					//cout << "outside: " << manipulate.f0_d.t() << endl;
					finger1.f_d_palm = manipulate.f1_d;
					finger2.f_d_palm = manipulate.f2_d;

					//Write control signals
					force_record0.force_d = manipulate.f0_d;
					force_record1.force_d = manipulate.f1_d;
					force_record2.force_d = manipulate.f2_d;

					force_record0.append(t_control-2,finger0.f_palm);
					force_record1.append(t_control-2,finger1.f_palm);
					force_record2.append(t_control-2,finger2.f_palm);

					//cout << "f0_d: " << manipulate.f0_d.t() << endl;
					//cout << "f1_d: " << manipulate.f1_d.t() << endl;
					//cout << "f2_d: " << manipulate.f2_d.t() << endl;
					
					finger0.desired_force();
					finger1.desired_force();
					finger2.desired_force();

					//Step force controllers
					finger0.calc_control(del_t);
					finger1.calc_control(del_t);
					finger2.calc_control(del_t);

					//cout << "f0_d_finger: " << finger0.f_d.t() << endl;
					//cout << "f1_d_finger: " << finger1.f_d.t() << endl;
					//cout << "f2_d_finger: " << finger2.f_d.t() << endl;

					//Halt finger motion due to large object errors
					if (arma::norm(manipulate.e1.submat(0,0,2,0),2) > .03 || arma::norm(manipulate.e1.submat(3,0,5,0),2) > .4)
					{
						if (force_checking_delay>=400) //if a few seconds passed since initial touching, then stop motion
						{
							cout << "object tracking error exceeded...assuming object loss or unrecoverable" << endl;
							for (unsigned short int i=1;i<=6;++i) {qd_dot_vec[i]=0;}
							q_dot_vec = hand.SetAxisTargetGetAxisActualVelocity(axes,qd_dot_vec);
							grasp =false;
							sent_fault = ulapi_socket_write(client_fault, "Error", sizeof(outbuffer)); //sends hand fault regarding bad object tracking error
						}
					}
					/*
					//Halt finger motion if finger loss
					if (arma::norm(finger0.f,2) < .5 || arma::norm(finger1.f,2) < .5 || arma::norm(finger2.f,2) < .5)
					{
						if (force_checking_delay>=400) //if a few seconds passed since initial touching, then stop motion
						{
							cout << "One or more finger contact losses" << endl;
							for (unsigned short int i=1;i<=6;++i) {qd_dot_vec[i]=0;}
							q_dot_vec = hand.SetAxisTargetGetAxisActualVelocity(axes,qd_dot_vec);
							grasp =false;
							sent_fault = ulapi_socket_write(client_fault, "Loss", sizeof(outbuffer)); //sends hand fault object loss based on forces
						}
					}
					*/
					
					/* //Use for Peg n Hole
					//Check friction forces for collision detection every second
					f_total = (finger0.f_palm+finger1.f_palm+finger2.f_palm)/3;		
					//cout << f_total(2,0) << " " << manipulate.collision << " " << manipulate.rcd(2,0) << endl;
					if (f_total(2,0) > 2.05 && manipulate.collision == false && manipulate.rc(2,0) < .156)
					{
						cout << f_total(2,0) << endl;
						manipulate.collision = true;
						sent_collision = ulapi_socket_write(client_collision, "inserting", sizeof(outbuffer_collision));
						cout << "Collision Detected! Retracting" << endl;
					}

					else if (f_total(2,0) > 2.5 && manipulate.collision == false && manipulate.rc(2,0) < .158)
					{
						cout << f_total(2,0) << endl;
						manipulate.collision = true;
						sent_collision = ulapi_socket_write(client_collision, "inserting", sizeof(outbuffer_collision));
						cout << "Collision Detected! Retracting" << endl;
					}

					else if (manipulate.rc(2,0) <= .1525 && manipulate.collision == true) //reset collision
					{
						cout << "Now ready to try again" << endl;
						manipulate.collision = false;
						sent_collision = ulapi_socket_write(client_collision, "failed", sizeof(outbuffer_collision));
					}

					else if (manipulate.rc(2,0) >= .158 && inserted == false) //inserted
					{
						cout << "Now ready to try again" << endl;
						manipulate.collision = false;
						inserted = true;
						sent_collision = ulapi_socket_write(client_collision, "inserted", sizeof(outbuffer_collision));
					}
					
					*/
					//Artifical counter for checking stuff
					force_checking_delay+=1;
					//cout << force_checking_delay << endl;
				}

				//Halt finger motion if client asks
				if (strcmp(grasping_fault,"Stop")==0)
				{
					cout << "Grasping halted due to client demand" << endl;
					for (unsigned short int i=1;i<=6;++i) {qd_dot_vec[i]=0;}
					q_dot_vec = hand.SetAxisTargetGetAxisActualVelocity(axes,qd_dot_vec);
					grasp =false;
				}
				
				if (grasp==false) {break;}
				///* UNDO THIS TO GET MANIPULATION BACK
				//Check once grasp status
				//if (arma::norm(finger0.f,2) > 1.5 && arma::norm(finger1.f,2) > 1.5 && arma::norm(finger2.f,2) > 1.5) {grasped = true;}
				if (cop0.touched == true && cop1.touched == true && cop2.touched == true) {grasped = true;}
				//else {grasped = false;}

				if (engage_manipulation == true && grasped == false)
				{
					if (cop0.touched == true) {finger0.qd_dot(0,0)=0;finger0.qd_dot(1,0)=0;}
					if (cop1.touched == true) {finger1.qd_dot(0,0)=0;finger1.qd_dot(1,0)=0;}
					if (cop2.touched == true) {finger2.qd_dot(0,0)=0;finger2.qd_dot(1,0)=0;}
				}
				//*/

				//cout << object_pose.rc.t() << endl;
				
				//t_control = double(timer.Elapsed_us())/1e6;

				//Parse command velocities and submit

				if (finger == "finger0" || finger == "all")
				{
					qd_dot_vec[1]=finger0.qd_dot(0,0);
					qd_dot_vec[2]=finger0.qd_dot(1,0);
				}

				if (finger == "finger1" || finger == "all")
				{
					qd_dot_vec[3]=finger1.qd_dot(0,0);
					qd_dot_vec[4]=finger1.qd_dot(1,0);
				}


				if (finger == "finger2" || finger == "all")
				{
					qd_dot_vec[5]=finger2.qd_dot(0,0);
					qd_dot_vec[6]=finger2.qd_dot(1,0);
				}
				
				//Send velocity profiles
				q_dot_vec = hand.SetAxisTargetGetAxisActualVelocity(axes,qd_dot_vec);

				if (finger == "finger0" || finger == "all")
				{
					finger0.q_dot(0,0) = q_dot_vec[1];
					finger0.q_dot(1,0) = q_dot_vec[2];
				}

				if (finger == "finger1" || finger == "all")
				{
					finger1.q_dot(0,0) = q_dot_vec[3];
					finger1.q_dot(1,0) = q_dot_vec[4];
				}


				if (finger == "finger2" || finger == "all")
				{
					finger2.q_dot(0,0) = q_dot_vec[5];
					finger2.q_dot(1,0) = q_dot_vec[6];
				}
			} 

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

		//Writing collected data and shutting down---------------------------------------------------------------------------------------------------

		hand.EmergencyStop();
		cout << "hand stopped" << endl;

		//Write Data
		if (finger == "finger0" || finger == "all") {force_record0.write_file("finger0"); cout << "Finger 0 Force Data Written" << endl;}
		if (finger == "finger1" || finger == "all") {force_record1.write_file("finger1"); cout << "Finger 1 Force Data Written" << endl;}
		if (finger == "finger2" || finger == "all") {force_record2.write_file("finger2"); cout << "Finger 2 Force Data Written" << endl;}

		cout << "Writing Pose Tracking Data" << endl;
		manipulate.write_file();
		cout << "Done!" << endl;
		
		//-------------End Control-------------
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		cout << "1" << endl;
		//ts.SetFramerate( 0, true,false );
		cout << "2" << endl;
		ts.Close();
		cout << "3" << endl;
		cout << "sensors stopped" << endl;
		hand.Close();
		cout << "hand closed" << endl;
		
		workerThread.interrupt();
		workerThread.join();
		cout << "interrupting fault thread" << endl;
		workerThread2.interrupt();
		workerThread2.join();
		cout << "Everything Closed" << endl;
		sent = ulapi_socket_write(client, "SDH has shut down", sizeof(outbuffer));
    }

    catch ( cSDHLibraryException* e )
    {
        cerr << "main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
	}
}
