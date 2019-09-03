#include "force_controller.h"

//include SDH library here and fix function calls

using namespace arma;
using namespace std;

void force_control::initialize(string finger_in, double force_mag_des, char * DH_file, char* &start_link, char* &end_link)
{
	finger = finger_in;
	Kf.zeros(2,2);
	Kf(0,0)=.002;//.5; //.75;
	Kf(1,1)=.002; //.5; //.75;
	//Bf.zeros(2,2);
	//Bf(0,0) = 0.0005; //10;
	//Bf(1,1) = 0.0005; //10;
	alpha = 1;
	num_hidden_neurons = 240;
	p = 0.5; //dropout
	
	force_d_mag = force_mag_des;

	//Import robot
	robot.import(DH_file);
	robot.convertRobot();

	//Initialize kinematics
	kin.chain_ends_indices(robot.robot_data,start_link,end_link);
	kin.initialize(finger_in);

	//Initial Conditions
	ef.zeros(3,1);
	ef_old.zeros(3,1);
	ef_dot.zeros(3,1);
	f_old.zeros(3,1);
	xd_dot.zeros(2,1);
	ef_old.zeros(3,1);
	q.resize(2,1);
	qd = q;
	q_dot.zeros(2,1);
	qd_dot.zeros(2,1);
	biotacs.resize(21,1);
	biotacs(20,0) = 1; //bias
	hidden.resize(num_hidden_neurons+1,1);
	hidden(num_hidden_neurons,0) = 1; // bias
	f.resize(3,1);
	tared = false;
	biotac_tared.resize(21,1);
	biotac_tared(20,0)=0;
	f_tared.resize(3,1);
	fn.zeros(3,1);

	//Set normalization maxs and mins
	set_mins_maxs();

	//Initialized desired force profile
	f_dir << 0 << endr
		  << 1 << endr
		  << 0 << endr;

	f_dir = f_dir/norm(f_dir,2);

	f_d = force_d_mag*f_dir;

	ef_modifier=0;
}

void force_control::reset()
{
	//Reset
	ef.zeros(3,1);
	ef_old.zeros(3,1);
	f_old.zeros(3,1);
	xd_dot.zeros(2,1);
	ef_old.zeros(3,1);
	tared = false;
	biotac_tared(20,0)=0;

	f_dir << 0 << endr
		  << 1 << endr
		  << 0 << endr;

	f_d = force_d_mag*f_dir;
}

void force_control::set_mins_maxs()
{
	//Set Biotac mins and maxs
	double temp=0;
	mins.reshape(20,1);
	maxs.reshape(20,1);

	//Importing mins
	if (finger == "finger0") {extrema_filename = "Biotac0_extrema.csv";}
	else if (finger == "finger1") {extrema_filename = "Biotac1_extrema.csv";}
	else if (finger == "finger2") {extrema_filename = "Biotac2_extrema.csv";}
	else {cout << "incorrect finger name!" << endl; cin.get();}
	
	ifstream extrema_file(extrema_filename);
	string line;
	unsigned int row=0,col=0;
	while (row < mins.n_rows)
	{
		//cout << row << " " << col << endl;
		getline(extrema_file,line);
		stringstream lineStream(line);
		string cell;
		while (col <= mins.n_cols)
		{
			getline(lineStream,cell,',');
			std::stringstream convertor(cell);
			convertor >> temp;
			if (col==0)	{mins(row,col)=temp;}
			else {maxs(row,col-1)=temp;}
			col+=1;
		}
		col=0;
		row+=1;
	}

	extrema_file.close();


	//cout << "mins: " << mins << endl;
	//cout << "maxs: " << maxs << endl;
	//cin.get();
}

void force_control::constructNN()
{
	double temp=0;
	W.reshape(3,num_hidden_neurons+1);
	V.reshape(num_hidden_neurons,21);
	if (finger == "finger0") {W_filename = "Biotac0_W.csv"; V_filename = "Biotac0_V.csv";}
	else if (finger == "finger1") {W_filename = "Biotac1_W.csv"; V_filename = "Biotac1_V.csv";}
	else if (finger == "finger2") {W_filename = "Biotac2_W.csv"; V_filename = "Biotac2_V.csv";}
	else {cout << "incorrect finger name!" << endl; cin.get();}


	//Importing W
	ifstream W_file(W_filename);
	string line;
	unsigned int row=0,col=0;
	while (row < W.n_rows)
	{
		getline(W_file,line);
		stringstream lineStream(line);
		string cell;
		while (col < W.n_cols)
		{
			getline(lineStream,cell,',');
			std::stringstream convertor(cell);
            convertor >> temp;
			W(row,col)=temp;
			col+=1;
		}
		col=0;
		row+=1;
	}

	W_file.close();

	

	//Importing V
	ifstream V_file(V_filename);
	row=0; col=0;
	while (row < V.n_rows)
	{
		getline(V_file,line);
		//cout << "row: " << row << endl;
		//cout << "col: " << col << endl;
		stringstream lineStream(line);
		string cell;
		while (col < V.n_cols)
		{
			getline(lineStream,cell,',');
			std::stringstream convertor(cell);
            convertor >> temp;
			V(row,col)=temp;
			col+=1;
		}
		col=0;
		row+=1;
	}

	V_file.close();

	//cout << "V: " << V << endl;
	//cout << " " << endl;
	//cout << "W: " << W << endl; 
	//cin.get();

}

void force_control::tare(int * bt_cd_value)
{
	for (unsigned int col=0;col<20;++col)
	{
		biotac_tared(col,0)=bt_cd_value[col]; //bt_cd_value[0];  //set now to create relative impedance measures, not just straight zeroing via taring
	}

	convert2forces(bt_cd_value);
	tared=true;
}

void force_control::normalize()
{
	//Normalizes Biotac readings between -1 and 1 based on training data
	biotacs=biotacs-biotac_tared;
	//cout << biotacs << endl;
	biotacs.submat(0,0,19,0)=2*(biotacs.submat(0,0,19,0)-mins)/(maxs-mins)-1;
}

void force_control::Rz(double angle)
{
	Rot_z << cos(angle) << -sin(angle) << 0 << endr << sin(angle) << cos(angle) << 0 << endr << 0 << 0 << 1 << endr;
}

void force_control::convert2forces(int * bt_cd_value)
{
	for (unsigned int col=0;col<20;++col)
	{
		biotacs(col,0)=bt_cd_value[col];
	}
	

	force_control::normalize();
	hidden.submat(0,0,num_hidden_neurons-1,0)= tanh(p*V*biotacs);
	f = -p*W*hidden;
	//if (tared == false) {f_tared=f;}
	//f = f-f_tared; //TARING FORCE PREDICTIONS
	Rz(q(1,0));
	f = Rot_z*f;
	fn = Rot_z*fn;
	Rz(q(0,0));
	f = Rot_z*f;
	fn = Rot_z*fn;
	f = alpha*f+(1-alpha)*f_old;
	f_old = f;

	//force error modifier for neural net predictions
	
	
	if (finger.compare("finger0") == 0)	
	{
		//cout << "here" << endl;
		//if (f(1,0)<0) {f(1,0) = 0;} //can't have forces in finger's -y
		//f(1,0) = f(1,0); //scaling
		//f(2,0) = 0;
		f = f*.75;
		
	}

	//if (finger.compare("finger1") == 0)	{f(1,0) = 0.55*f(1,0); f(2,0) = 0;} //.65
	//if (finger.compare("finger2") == 0)	{f(1,0) = 0.75*f(1,0); f(2,0) = 0;} //f(0,0) = 0; //.55
	
	//cout << f.t() << endl;
	//Friction not accurate within 2 N, so set to 0.
	if (abs(f(0,0)) < 1.5) {f(0,0) = 0;}
	if (abs(f(1,0)) < 1.5) {f(1,0) = 0;} //y is fairly accurate
	if (abs(f(2,0)) < 1.5) {f(2,0) = 0;}

	/*
	if (f(0,0)<.8 && f(0,0)>-.8 ) { f(0,0)=0;};
	if (f(1,0)<.8 && f(1,0)>-.8 ) { f(1,0)=0;};
	if (f(2,0)<.8 && f(2,0)>-.8 ) { f(2,0)=0;};
	*/
}

//Rotating forces into finger base coordinate system
void force_control::rotate_forces()
{
	Rz(q(1,0));
	f = Rot_z*f;
	fn = Rot_z*fn;
	Rz(q(0,0));
	f = Rot_z*f;
	fn = Rot_z*fn;
}

//need to read in joint positions here
void force_control::set_joint_states(double anglesZ[])
{
	q(0,0)=anglesZ[0];
	q(1,0)=anglesZ[1];
}

//creates Jacobian matrix - kin.J
void force_control::getJacobian(mat COP,double joint0)
{
	//kin.initialize();
	kin.joint_var_pos(q);
	kin.COP_offset = COP;
	kin.Tmatrix();
	kin.T_world_joints(joint0);
	kin.jacobian();
	f_palm = kin.Tpf.submat(0,0,2,2)*f;
	fn = kin.Tpf.submat(0,0,2,2)*fn;	
}

void force_control::desired_force()
{
	//f_d_palm --  set this directly from outside since it is public member -- should be 3x1
	//cout << kin.Tpf << endl;
	f_d = kin.Tpf.submat(0,0,2,2).t()*f_d_palm; //inverse rotation matrix to convert palm force into finger base force
}

void force_control::error_filter()
{
	//cout << "ef: " << ef << endl;
	//cout << "ef_old: " << ef_old << endl;
	
	ef=alpha*ef+(1-alpha)*ef_old;
	ef_dot = f-f_old;
	f_old = f;
	//cout << "ef_filtered: " << ef << endl;
}

//calculates error
void force_control::error()
{
	//cout << f.t() << endl;
	//f = kin.Tpf.submat(0,0,2,2)*f; //rewrite in palm coordinate system
	//cout << f.t() << endl;

	//force error
	Bf = 400;
	//if (finger == "finger0") {cout << "before: " << f_d.t() << endl;}

	//f_d = f_d.submat(0,0,1,0)-Bf*kin.J.submat(0,0,1,1)*q_dot;
	//if (finger == "finger0") {cout << "after: " << f_d.t() << endl;}
	ef = f_d-f; //round(f_d-f);
	//if (abs(ef(0,0)) < 1) {ef(0,0)=0;}
	//if (abs(ef(1,0)) < 1) {ef(1,0)=0;}
}

void force_control::calc_control(double del_t)
{
	//send qd and qd_dot to position controller
	error();
	//error_filter();

	//Controller - averaging scheme
	//cout << norm(ef,2) << endl;
	//if (norm(ef,2) < 1) {ef.zeros(3,1);}
	
	//Kf(0,0) = .002;
	//Kf(1,1) = .002;
	
	Kf(0,0)=.0015; //(tanh(.125*norm(ef,2)))*.001+.002; //(log(norm(ef,2)+1)+1)*.001; //norm(ef,2)*.001; //.001; //norm(ef,2)*.0007; //norm(ef,2)*.002; //ef(0,0)*ef(0,0)*.0002; //floor(norm(ef,2))*.001; //.006; //.002 //these gains can probably go up more once biotac is calibrated better
	Kf(1,1)=.0015; //(tanh(.125*norm(ef,2)))*.001+.002; //(log(norm(ef,2)+1)+1)*.001; //norm(ef,2)*.001; //.001; //norm(ef,2)*.001;//.0015; //norm(ef,2)*.0007; //norm(ef,2)*.002; //ef(1,0)*ef(1,0)*.0002;//norm(ef,2)*.0005; //floor(norm(ef,2))*.001;
	//Bf = .0000003;
	//if (finger == "finger0") {cout << "ef: " << ef.t() << endl; cout << ef_dot.t() << endl;}
	xd_dot=Kf*ef.submat(0,0,1,0); //+Bf*ef_dot.submat(0,0,1,0)/del_t;
	//if (finger == "finger2") {cout << ef.t() << endl; cout << sign(ef.submat(0,0,1,0)) << endl;}
	qd_dot = inv(kin.J.submat(0,0,1,1))*xd_dot; //.5*tanh(inv(kin.J.submat(0,0,1,1))*xd_dot);
	
	/*
	Kf(0,0)= .1; //(tanh(.5*norm(ef,2)))*.1; //norm(ef,2)*.004;//norm(ef,2)*.05;
	Kf(1,1)= .01; //(tanh(.5*norm(ef,2)))*.1; //norm(ef,2)*.004;//norm(ef,2)*.05;
	qd_dot = Kf*kin.J.submat(0,0,1,1).t()*ef.submat(0,0,1,0);
	//qd_dot = .5*(Kf*kin.J.submat(0,0,1,1).t()*ef.submat(0,0,1,0)+qd_dot);
	
	if (finger == "finger0") 
	{
		cout << kin.J.submat(0,0,1,1).t() << endl;
		cout << ef.submat(0,0,1,0) << endl;
		cout << kin.J.submat(0,0,1,1).t()*ef.submat(0,0,1,0) << endl;
	}// cout << f_d.t() << endl;}
	*/
	//Saturate within firmware limits
	if (abs(qd_dot(0,0)) > (2*datum::pi)/3) {cout << "qd1 too high!!" << endl; qd_dot(0,0) = (qd_dot(0,0)/abs(qd_dot(0,0)))*((2*datum::pi)/3);}
	if (abs(qd_dot(1,0)) > (2*datum::pi)/3) {cout << "qd2 too high!!" << endl; qd_dot(1,0) = (qd_dot(1,0)/abs(qd_dot(1,0)))*((2*datum::pi)/3);}
	
	//Prevent fingers from going past straight (singular configuration)

	//if (norm(q,2) < 0.122) {cout << "nearing singularity" << endl;qd_dot(1,0) = 0;}
	//if (q(1,0) < .05 && qd_dot(1,0) < 0) {cout << "avoiding singularity" << endl; qd_dot(1,0) = 0;}

	//cout << "qd_dot: " << qd_dot.t() << endl;

	//cout << "f: " << f.t() << endl;
			
	//if (finger.compare("finger0")==0) { cout << f_d.t() << endl;}
	//cout << f.t() << endl;
}