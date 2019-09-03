#pragma once

#ifndef KINEMATICS
#define KINEMATICS

#include <iostream>
#include <armadillo>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
//#include <vector>


struct data
{
	std::string name;
	bool active;
	//std::vector <double> DH;
	double DH [4];
	double joint_var;
	std::string joint_type; //"prismatic" or "revolute"
};

class Import_Robot
{

	//initializations
	YAML::Node robot;
	data DATA;

	public:
		std::vector <data> robot_data;
		
		//imports and parses yaml file
		void import(char* robot_file);


		//Places YAML data in vector
		void convertRobot();		
};


class Kinematics
{
	std::vector <data> robot_data; //data of robot
	std::string direction; //direction of chain calculation: forward or backward
	unsigned short int start_index;
	unsigned short int end_index;
	unsigned short int num_active_joints;
	unsigned short int active_link;
	arma::mat T; //transformation matrix
	arma::mat T_temp; //temporary transformation matrix
	std::vector <arma::mat> Z_vector; //vector collection of z-axes per joint
	std::vector <arma::mat> origins_vector; //vector collection of origins per joint
	arma::mat Z_static;
	arma::mat origin_static;

	double a,d,alpha,theta;
	unsigned short int i; //iterator

	arma::mat aR; //a rotation matrix
	arma::mat aT; //a translation matrix
	arma::mat Ry_neg90; //a rotation about y for -90 deg

	std::string finger; //finger selection
		
	public:
		arma::mat Tpf; //transform from palm to finger 1 base frame
		arma::mat Tpf_pre; //the fixed part of the transform from the palm to the finger base frame		
		arma::mat J; //jacobian matrix
		arma::mat end_effector; //Cartesian location of the end effector
		arma::mat end_effector_palm; //Cartesian location of end effector expressed in palm coordinate system
		arma::mat COP_offset; //adjustment for end-effector
		arma::mat end_effector_transform;
		
		//pulls indices of robot vector data for starting and ending links of chain
		//determines chain directionality (forward,backward)
		void chain_ends_indices(std::vector <data> &robot_in,char* &start_link,char* &end_link);

		//Sets current joint variable positions of the robot
		void joint_var_pos(arma::mat &joints_in);

		//calculates transformation of link
		void transform(unsigned short int index);

			//initializes Jacobian, origins, and Z's
		void initialize(std::string finger_in);

		arma::mat rotation(char axis, double angle);

		arma::mat Kinematics::translation(char axis, double length);

		//extracts z-axis and joint pos data for jacobian construction
		void extract_jacobian_data(unsigned short int index);
		
		//Generates Transformation matrix from starting to ending links. Also saves relevant data
		//for the construction of the Jacobian
		void Tmatrix();

		void T_world_joints(double joint0);//Transforms for palm coordinate system to fixed, first joint coordinate systems

		//Calculates the linear velocity portion of the Jacobian matrix for select end link
		void jacobian();
};

#endif