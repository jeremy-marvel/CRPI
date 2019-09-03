#ifndef COPDEF
#define COPDEF

#include <iostream>
#include <armadillo>
#include <string>

class COP_Normal
{
	private:
		int iterator;
		int num_sensors;
		arma::vec biotac_IC;
		arma::vec biotac;
		arma::vec diff;
		arma::mat cart_pos_imp_sensors;
		arma::mat normals_imp_sensors;
		arma::uvec index;
		double alpha;
		
		//arma::uword index2;
		//arma::uword index3;
		arma::vec mins;
		arma::vec biotac_ranges;

	public:
		arma::mat COP_loc;
		arma::mat COP_loc_filtered;
		arma::mat COP_loc_palm;
		arma::mat normal;
		arma::mat normal_filtered;
		arma::mat normal_palm;
		bool touched;

		bool tared;

		COP_Normal(); //creates Biotac's Cartesian position of impedance sensors

		void reset();

		void tare(int * bt_cd_value);

		void max_difference(); //finds 3 Biotacs with largest change in readings from IC

		void centroid(); //calculates weighted centroid of triangle (Cartesian position)

		void touch(); //determines whether or not finger is in contact

		void run(int * bt_cd_value); //reads in new Biotac data, and triangulates COP
};

#endif
