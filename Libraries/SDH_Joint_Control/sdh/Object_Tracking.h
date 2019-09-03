#ifndef OBJECT
#define OBJECT

#include <iostream>
#include <armadillo>
#include <string>

class Object_Tracking
{
	private:
		unsigned int iterator;
		double dt;
		bool init;

		arma::mat pc;
		arma::mat vc;
		arma::mat omegac;
		arma::mat vc_prev;
		arma::mat omegac_prev;
		arma::mat thetac;
		arma::mat p0;
		arma::mat p1;
		arma::mat p2;
		arma::mat p0_prev;
		arma::mat p1_prev;
		arma::mat p2_prev;
		arma::mat v0;
		arma::mat v1;
		arma::mat v2;
		arma::mat R1;
		arma::mat R2;
		arma::mat V1;
		arma::mat V2;
		arma::mat V;
		arma::mat A;

	public:
		arma::mat rc;
		arma::mat rc_dot;
		
		Object_Tracking(double rc_ic[]); //constructs with IC's of pose

		void reset(double rc_ic[]);

		void COP_vel(); //time derivative of COPs to get COP velocities

		void rel_vel(); //calculates relative velocities between three points, include compiled matrix

		void rel_pos(); //calculates relative positions between three points, include compiled matrix

		void ang_vel(); //calculates angular velocity

		void lin_vel(); //calculates linear velocity of control point on object

		void pose(); //updates pose of object using numerical integration. use quadrature integration

		void run(double del_t, arma::mat &finger0_poc, arma::mat &finger1_poc, arma::mat &finger2_poc);
};

#endif