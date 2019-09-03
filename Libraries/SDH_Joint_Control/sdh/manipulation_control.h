#ifndef manipulation
#define manipulation

#include <armadillo>
#include <iostream>
#include <string>
#include <math.h>

class manipulation_control
{
	private:
		//unitized friction force vector of individual fingers
		arma::mat ff01_hat; //this will come from analyzing resultant contact force and normal direction
		arma::mat ff11_hat;
		arma::mat ff21_hat;

		arma::mat ff02_hat; //this will come from cross product of normal and existing friction force directions
		arma::mat ff12_hat;
		arma::mat ff22_hat;

		arma::mat ff0_hat; //this will come from combining and unitizing the previous two
		arma::mat ff1_hat;
		arma::mat ff2_hat;

		//Unitized contact resultant forces
		arma::mat f0_hat;
		arma::mat f1_hat;
		arma::mat f2_hat;
				
		//command magnitudes of the unitized friction force vectors
		arma::mat uf01;
		arma::mat uf02;
		arma::mat uf11;
		arma::mat uf12;
		arma::mat uf21;
		arma::mat uf22;

		//command magnitudes of unitized normal forces
		arma::mat un0;
		arma::mat un1;
		arma::mat un2;

		//Control shaping scalars
		arma::mat gamma1,gamma2,gamma3;

		//Control gains
		double alpha1,alpha2; //filtered error gains
		double Ks; //normal force control gain
		double Kf,betaf; //friction force control gains

		//Grasp matrices
		arma::mat Bf;
		arma::mat Bn;
		arma::mat Bn_dot;
		arma::mat Bn_prev;

		//Command force magnitudes
		arma::mat Un;
		arma::mat Uf;

		//System errors
		arma::mat e2;
		arma::mat r;
		arma::mat w;
		arma::mat Q;
		arma::mat Q_integrand;
		arma::mat Q_integrand_prev;
		arma::mat vf;
		arma::mat vf_integrand;
		arma::mat vf_integrand_prev;

		unsigned int iterator;
		double dt;
		double time;
		arma::mat I;
		bool init;
		arma::mat Z;

		//Coefficient of friction
		double mu_s;

		arma::mat vel_error;

		//Recording
		arma::mat time_batch;
		arma::mat rc_batch;
		arma::mat rcd_batch;

		//Rotation Angles
		double theta1;
		double theta2;
		double theta3;
		arma::mat Tpo; //transformation from object to palm

		//Random matrix
		arma::mat random_matrix;
		bool switch1;
		
	public:
		bool collision;

		//Tracking Error
		arma::mat e1;

		//desired and actual object states
		arma::mat rcd;
		arma::mat rcd_dot;
		arma::mat rc;
		arma::mat rc_dot;
		arma::mat pc;
		
		//Other measured signals
		arma::mat p0; //assumes 3 controllable points of contact
		arma::mat p1;
		arma::mat p2;

		//Desired POC based on initial touch
		arma::mat p0d_fixed;
		arma::mat p1d_fixed;
		arma::mat p2d_fixed;

		arma::mat p0d;
		arma::mat p1d;
		arma::mat p2d;

		arma::mat p0df;
		arma::mat p1df;
		arma::mat p2df;

		//unitized normal force vector of individual fingers
		arma::mat fn0_hat;
		arma::mat fn1_hat;
		arma::mat fn2_hat;

		//contact resultant force for each contact
		arma::mat f0;
		arma::mat f1;
		arma::mat f2;

		//Normal, friction, and resultant desired control forces
		arma::mat fn0_d;
		arma::mat fn1_d;
		arma::mat fn2_d;

		arma::mat ff0_d;
		arma::mat ff1_d;
		arma::mat ff2_d;

		arma::mat f0_d;
		arma::mat f1_d;
		arma::mat f2_d;

		double ff0_d_norm;
		double ff1_d_norm;
		double ff2_d_norm;


		manipulation_control(double rc_ic[]); //constructor with IC on pose

		void reset(double rc_ic[]);

		void calc_friction_directions(); //calculates orthogonal set of friction force directions for each POC

		void calc_artificial_friction();

		void calc_grasp_matrices(); //calculates grasp matrices

		void errors(); //calculates tracking errors

		arma::mat sigmoid(arma::mat input_matrix); // sigmoid function

		void calc_Bn_dot();

		void calc_Un(); //normal force magnitude commands

		void calc_Uf(); //friction force magnitude commands

		void manipulation_control::calc_command_forces(); //calculates normal, friction, and resultant command forces

		void append(); //appends new data
		
		void calc_control(double del_t); //runs one time step of manipulation controller

		void write_file(); //write collected data to file
};


#endif