#ifndef FORCE_CONTROL
#define FORCE_CONTROL

#include <iostream>
#include <armadillo>
#include "kinematics.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

//#include <time.h> //use one that comes built in with SHD library

class force_control
{
	private:
		arma::mat Kf; //gain
		double Bf; //derivative gain
		double alpha; //low pass filter constant
		std::string finger; //finger selected
		std::string extrema_filename; //Biotac file containing mins and maxes for normalization
		std::string W_filename;
		std::string V_filename;
		unsigned int num_hidden_neurons;
		double ef_modifier;
		double p; //dropout

	public:
		arma::mat f_d; // desired force
		arma::mat f_d_palm; //desired force expressed in palm
		arma::mat f_dir; // force direction
		arma::mat f; //measured force
		arma::mat f_old; //past measurement of force
		arma::mat f_palm; //measured force expressed in palm
		arma::mat fn; //measured force normal - assigned in Biotac, transformed to Palm
		Import_Robot robot;
		Kinematics kin;
		arma::mat q; //measured joint positions
		arma::mat qd; //desired joint positions
		arma::mat qd_dot; //desired joint velocities
		arma::mat xd_dot; //desired Cart Vel
		//std::vector <double> qd_dot_vec;
		arma::mat q_dot; //measured joint velocities
		arma::mat ef; //force error
		arma::mat ef_old;
		arma::mat ef_dot;
		arma::mat W;
		arma::mat V;
		arma::mat Rot_z;
		arma::mat biotacs;
		arma::mat hidden;
		arma::mat maxs;
		arma::mat mins;
		bool tared;
		arma::mat biotac_tared;
		arma::mat f_tared;
		double force_d_mag;
		double del_t; //time step
		//time_t start,end;
		
		void initialize(std::string finger, double force_mag_des, char * DH_file, char* &start_link, char* &end_link);

		void reset();
		
		void force_control::set_mins_maxs();

		void constructNN();

		void tare(int * bt_cd_value);

		void normalize();

		void Rz(double angle);

		void convert2forces(int * bt_cd_value);

		void rotate_forces();

		//need to read in joint positions here
		void set_joint_states(double anglesZ[]);
		
		//creates Jacobian matrix - kin.J
		void getJacobian(arma::mat COP,double joint0);

		void desired_force();
		void error_filter();

		//calculates error
		void error();

		void calc_control(double del_t);
};


#endif