#include "force_record.h"

using namespace std;
using namespace arma;

force_record::force_record()
{
	time_batch.resize(1,1);
	forces_batch.resize(3,1);
	forces_d_batch.resize(3,1);
	force_d.zeros(3,1);
	t = 0;
};

void force_record::append(double t_in,mat forces)
{
	//append new data
	time_batch.insert_cols(time_batch.n_cols,1);
	time_batch(0,time_batch.n_cols-1) = t_in;
	forces_batch.insert_cols(forces_batch.n_cols,forces);
	forces_d_batch.insert_cols(forces_d_batch.n_cols,force_d);
	t=t_in;
};

void force_record::f_d()
{
	force_d(1,0) = 8;
	//force_d(1,0) = 3*log(sin(2*datum::pi*t/4+(3*datum::pi)/2)+cos(t/4+datum::pi)+3)+1;
}

void force_record::write_file(string finger)
{
	ofstream force_data;

	if (finger == "finger0") {force_data.open("finger0_data.csv");}
	else if (finger == "finger1") {force_data.open("finger1_data.csv");}
	else if (finger == "finger2") {force_data.open("finger2_data.csv");}
	else {cout << "incorrect finger name for force recording" << endl; cin.get();}

	cout << time_batch.n_cols << endl;

    force_data << "Time, Force X (N), Force Y (N), Force Z (N), Force_d X (N), Force_d Y (N), Force_d Z (N)" << endl;
	for (int i=0;i<time_batch.n_cols;++i)
	{
		force_data << time_batch(0,i) << ", " << forces_batch(0,i) << ", " << forces_batch(1,i) << ", " << forces_batch(2,i) << ", " << forces_d_batch(0,i) << ", " << forces_d_batch(1,i) << ", " << forces_d_batch(2,i) << endl;  
		//cout << i << endl;
	}

	force_data.close();
};