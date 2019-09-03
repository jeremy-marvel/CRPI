/*
Author: Karl Van Wyk
Date: 11/1/2017

Analytical inverse kinematics for KUKA LWR 4+
-Assumes third joint (e1) remains at 0 for simplicity
-Performs workspace checks
-Performs joint limit checks

Example logic--------------------------------------

//For calculations
1) use set_TCP_offset method to set the tool offset for robot
2) use set_current_angles_deg to set the current pose of the robot (not necessary, but useful)
3) use target_Cartesian_pose to set the desired Cartesian pose of the robot end-effector

//Returning solutions
1) get_closest_solution method returns the closest valid joint angles to the robot's current joint angles
2) get_all_solutions method returns all valid joint angles

*/

#ifndef KUKA_IK
#define KUKA_IK

#include <math.h>
#include <vector>
#include <armadillo>

#pragma once

using namespace std;

namespace KUKAIK
{
	class _declspec(dllexport) KUKA_AIK
	{
		private:
			double d1, a2, a3;

			double to_radians;

			double to_degrees;

			double pi;

			double r11, r12, r13, r21, r22, r23, r31, r32, r33;

			double roll, pitch, yaw;

			double X, Y, Z;

			double tcp_offset_x, tcp_offset_y, tcp_offset_z;

			double joint_lower_limits[7];

			double joint_upper_limits[7];

			double xc, yc, zc;

			double theta1, theta2, theta4, theta5, theta6, theta7;

			//Multiple joint angle solutions. Some are conditional on others so it is not a full permutation
			//Combined, will yield 8 candidate solutions
			double theta1_1, theta1_2, theta2_1, theta2_2, theta4_1, theta4_2, theta5_1, theta5_2, theta6_1, theta6_2, theta7_1, theta7_2;

			double theta3; //setting e1 joint to zero to simplify math

			//Container for solutions of joint angles
			vector <vector<double>> joint_angles;

			//number of valid solutions
			int num_solutions;

			//Robot's current angles
			vector<double> current_angles;

			//Various transforms
			arma::mat T0tool, T7tool, T07;

			//DH parameters
			vector<vector<double>> DH_params;

		public:

			KUKA_AIK();

			//Sets offsets for robot tool point
			void set_TCP_offset(double x_offset, double y_offset, double z_offset, double roll_offset, double pitch_offset, double yaw_offset);

			//Sets current joint angles for the robot (in degrees) - acts as criterion for selecting valid joint angle
			//solution at target Cartesian pose
			void set_current_angles_deg(double j0, double j1, double j2, double j3, double j4, double j5, double j6);

			//Takes in target/desired Cartesian pose for KUKA end-effector
			//and calculates all possible joint angles
			//Returned int takes on following values:
			//0 - IK calculation was a success
			//1 - failed due to workspace violation
			//2 - failed due to joint limits
			int target_Cartesian_pose(double x_d, double y_d, double z_d, double roll_d, double pitch_d, double yaw_d);

			//Give in degrees
			//Calculates rotation matrix from Euler ZYX
			arma::mat Euler_ZYX_deg(double roll_in, double pitch_in, double yaw_in);

			//Check workspace limits
			//void check_workspace_limits();

			//Check joint angle limits
			//TRUE = valid
			//FALSE = invalid
			bool check_joint_angle_limits(vector<double> angles_in);

			//Access method for returning closest solution
			void get_closest_solution(double * closest_angles);

			//Access method for returning all solutions
			//Returned int is the number of valid solutions
			int get_all_solutions(double * all_angles);

			//Creates a 4x4 homogenous transformation matrix given XYZ
			//and roll pitch yaw (Euler ZYX) components
			arma::mat T_matrix(double x_in, double y_in, double z_in, double roll_in, double pitch_in, double yaw_in);

			//Creates a 4x4 homogenous transformation matrix given DH parameters
			arma::mat T_dh_params(double a, double alpha, double d, double theta);

			~KUKA_AIK();
	};
}

#endif KUKA_IK