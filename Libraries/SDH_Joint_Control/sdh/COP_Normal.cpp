#include "COP_Normal.h"

using namespace arma;
using namespace std;

COP_Normal::COP_Normal() //creates Biotac's Cartesian position of impedance sensors
{
	biotac_IC.resize(19);
	biotac.resize(19);
	biotac_ranges.resize(19);
	
	COP_loc.zeros(3,1);
	normal.zeros(3,1);

	COP_loc_palm.ones(4,1);

	num_sensors = 3;
	mins.resize(num_sensors);
	index.resize(num_sensors);
	
	cart_pos_imp_sensors << 51.95 << 9.67 << -6.69 << endr //modified X by 3 mm to compensate for new plastic adapters
						 << 49.15 << 12.78 << -4.38 << endr
						 << 45.01 << 11.5 << -4.04 << endr
						 << 43.04 << 7.06 << -6.73 << endr
						 << 40.62 << 9.68 << -4.38 << endr
						 << 38.27 << 5.52 << -6.47 << endr
						 << 56.91 << 13.12 << 0 << endr
						 << 52.86 << 14.39 << -2.8 << endr
						 << 52.86 << 14.39 << 2.8 << endr
						 << 52.05 << 15.09 << 0 << endr
						 << 51.95 << 9.67 << 6.69 << endr
						 << 49.15 << 12.78 << 4.38 << endr
						 << 45.01 << 11.5 << 4.04 << endr
						 << 43.04 << 7.06 << 6.73 << endr
						 << 40.62 << 9.68 << 4.38 << endr
						 << 38.27 << 5.52 << 6.47 << endr
						 << 46.27 << 13.45 << 0 << endr
						 << 39.6 << 11.03 << 0 << endr
						 << 36.25 << 9.6 << 0 << endr;

	cart_pos_imp_sensors = cart_pos_imp_sensors.t();
	/*
	normals_imp_sensors << -0.023450783 << 0.194934631 << -0.980535849 << endr
						<< -0.266356829 << 0.716661745 << -0.644554096 << endr
						<< -0.278711955 << 0.752079879 << -0.597239904 << endr
						<< -0.096906909 << 0.260793594 << -0.960518481 << endr
						<< -0.26739359 << 0.720355408 << -0.639991214 << endr
						<< -0.250550507 << 0.243309163 << -0.937029932 << endr
						<< 0.710053021 << 0.704148214 << 0 << endr
						<< 0.111797188 << 0.901830651 << -0.417376169 << endr
						<< 0.111797188 << 0.901830651 << 0.417376169 << endr
						<< -0.008888538 << 0.999960496 << 0 << endr
						<< -0.023450783 << 0.194934631 << 0.980535849 << endr
						<< -0.266356829 << 0.716661745 << 0.644554096 << endr
						<< -0.278711955 << 0.752079879 << 0.597239904 << endr
						<< -0.096906909 << 0.260793594 << 0.960518481 << endr
						<< -0.26739359 << 0.720355408 << 0.639991214 << endr
						<< -0.250550507 << 0.243309163 << 0.937029932 << endr
						<< -0.348554207 << 0.937288624 << 0 << endr
						<< -0.347930659 << 0.93752027 << 0 << endr
						<< -0.545244542 << 0.838277036 << 0 << endr;
	*/
	
	normals_imp_sensors << -0.16 <<	1.33 <<	0 << endr
						<< -1.81 <<	4.87 <<	0 << endr
						<< -1.89 <<	5.1 <<	0 << endr
						<< -0.68 <<	1.83 <<	0 << endr
						<< -1.83 <<	4.93 <<	0 << endr
						<< -1.73 <<	1.68 <<	0 << endr
						<< 4.81 <<	4.77 <<	0 << endr
						<< 0.75	<< 6.05 <<	0 << endr
						<< 0.75 <<	6.05 <<	0 << endr
						<< -0.06 <<	6.75 <<	0 << endr
						<< -0.16 <<	1.33 <<	0 << endr
						<< -1.81 <<	4.87 <<	0 << endr
						<< -1.89 <<	5.1	<< 0 << endr
						<< -0.68 <<	1.83 <<	0 << endr
						<< -1.83 <<	4.93 <<	0 << endr
						<<-1.73 <<	1.68 <<	0 << endr
						<< -2.38 <<	6.4 <<	0 << endr
						<< -2.39 <<	6.44 <<	0 << endr
						<< -3.74 <<	5.75 <<	0 << endr;
	

	for (iterator=0;iterator<19;++iterator) {normals_imp_sensors.submat(iterator,0,iterator,2) = normals_imp_sensors.submat(iterator,0,iterator,2)/norm(normals_imp_sensors.submat(iterator,0,iterator,2),2);}

	normals_imp_sensors = normals_imp_sensors.t();
	//filter initialization
	alpha = .01; //.1;
	COP_loc_filtered = cart_pos_imp_sensors.col(9)/1e3; //.zeros(1,3);
	normal_filtered = normals_imp_sensors.col(9)/1e3;
	tared=false;
	touched=false;
};

void COP_Normal::reset()
{
	tared=false;
	touched=false;
}

void COP_Normal::tare(int * bt_cd_value)
{
	for (iterator=0;iterator<19;++iterator)
	{
		biotac_IC(iterator) = bt_cd_value[iterator];
		biotac_ranges(iterator) = abs(750 - bt_cd_value[iterator]);
	}
	
	tared = true;
};

void COP_Normal::max_difference() //finds Biotacs with largest change in readings from IC
{
	diff = (biotac-biotac_IC); ///biotac_ranges;

	for (iterator=0; iterator<num_sensors; ++iterator)
	{
		mins(iterator) = diff.min(index(iterator));
		if (mins(iterator) > 0) {mins(iterator)=0;}; //for any positive mins, negate their effect by setting min to 0
		diff(index(iterator)) = 999;
	}
}; 

void COP_Normal::centroid() //calculates weighted centroid of triangle (Cartesian position)
{
	if (norm(mins,2) < .1) //if no significant touch, set to 0
	{
		COP_loc = cart_pos_imp_sensors.col(9)/1e3; //.zeros(1,3);
		normal = normals_imp_sensors.col(9);//.zeros(1,3);
	}
	else
	{
		COP_loc.zeros(3,1);
		normal.zeros(3,1);
		for (iterator=0; iterator<num_sensors; ++iterator)
		{
			COP_loc = mins(iterator)*cart_pos_imp_sensors.col(index(iterator))+COP_loc;
			normal = mins(iterator)*normals_imp_sensors.col(index(iterator))+normal;
		}

		COP_loc = COP_loc/(sum(mins)*1e3);
		normal = normal/sum(mins);
		normal = normal/norm(normal,2);
	}

	COP_loc_filtered = cart_pos_imp_sensors.col(9)/1e3; // alpha*COP_loc+(1-alpha)*COP_loc_filtered; //cart_pos_imp_sensors.col(9)/1e3; //
	normal_filtered = alpha*normal+(1-alpha)*normal_filtered;
	//normal_filtered(0,0) = 0;
	//normal_filtered(1,0) = 1;
	//normal_filtered(2,0) = 0;
};

void COP_Normal::touch()
{
	biotac-biotac_IC;
	diff = biotac-biotac_IC;
	if (mean(abs(diff)) > 10) {touched=true;}
}

void COP_Normal::run(int * bt_cd_value) //reads in new Biotac data, and triangulates COP
{
	//bring over Biotac values into biotac matrix
	for (iterator=0;iterator<19;++iterator)
	{
		biotac(iterator) = bt_cd_value[iterator];
	}

	COP_Normal::touch();
	COP_Normal::max_difference();
	COP_Normal::centroid();
};