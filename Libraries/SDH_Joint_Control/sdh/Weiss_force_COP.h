//This should:
//1.) Take Weiss force estimate and correct using linear fit
//2.) Take Weiss COP estimate, and estimate Z component, and remap to Biotac base coordinate system
//3.) Simple static normal force in Biotac base. Later can make better with CAD model

#ifndef WEISSFORCECOP
#define WEISSFORCECOP

#include <armadillo>
#include <iostream>

class Weiss_force_COP
{
	private:
		arma::mat T_Weiss_Base; //transformation from Weiss coordinate system to distal base coordinate system
		int iterator;
	public:
		arma::mat COP; //center of pressure of contact
		arma::mat COP_temp;
		arma::mat f_mag; //contact force magnitude
		arma::mat f; //force magnitude multiplied by force normal
		arma::mat fn; //force normal
		bool touched;

		Weiss_force_COP(); //constructor

		void get_COP(double COP_X, double COP_Y);

		void get_f(double f_mag_in,int matrix);
};


#endif