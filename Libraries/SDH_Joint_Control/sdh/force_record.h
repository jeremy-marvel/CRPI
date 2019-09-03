#ifndef FORCE_RECORD
#define FORCE_RECORD

#include <iostream>
#include <armadillo>
#include <fstream>
#include <sstream>

class force_record
{
	private:
		arma::mat time_batch;
		arma::mat forces_batch;
		arma::mat forces_d_batch;
		double t;

	public:
		arma::mat force_d;

		force_record();//constructor

		void append(double t_in,arma::mat forces);

		void f_d();

		void write_file(std::string finger);

};

#endif