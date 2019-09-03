#include "Weiss_force_COP.h"

using namespace std;
using namespace arma;

Weiss_force_COP::Weiss_force_COP()
{
	COP_temp.zeros(4,1);
	COP_temp(3,0) = 1;
	fn << 0 << endr << 1 << endr << 0 << endr; //points in distal base positive 'y' axis
	f.zeros(3,1);
	T_Weiss_Base << 0 << -1 << 0 << 0.0408 << endr
				 << 0 << 0 << -1 << 0.0150 << endr
				 << 1 << 0 << 0 << -0.0085 << endr
				 << 0 << 0 << 0 << 1 << endr;
	touched = false;
}

void Weiss_force_COP::get_COP(double COP_X, double COP_Y)
{
	COP_temp(0,0) = COP_X/1000;
	COP_temp(1,0) = COP_Y/1000;
	COP_temp(2,0) = 0;
	COP_temp(3,0) = 1;
	COP_temp = T_Weiss_Base*COP_temp;
	COP = COP_temp.submat(0,0,2,0);
}

void Weiss_force_COP::get_f(double f_mag_in, int matrix)
{
	//Scaling of inherent calibration
	/*if (f_mag_in > 0) 
	{
		if (matrix == 1) {cout << "calibrate matrix first" << endl; cin.get();}
		if (matrix == 3) {f_mag_in = f_mag_in*4.3057+1.8648;}
		if (matrix == 5) {f_mag_in = f_mag_in*6.6323+2.9913;}
	}*/
	f(1,0) = f_mag_in;

	if (f_mag_in > 0.25) {touched = true;}
}