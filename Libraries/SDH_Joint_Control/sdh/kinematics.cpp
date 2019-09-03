//This code allows for the calculation of transformation matrices and Jacobians

#include "kinematics.h"
using namespace std;
using namespace arma;

void Import_Robot::import(char* robot_file)
{
	cout << "Loading: " << robot_file << endl;
	ifstream fin(robot_file);
	YAML::Parser parser(fin);
	parser.GetNextDocument(robot);
	
}

void Import_Robot::convertRobot()
{
	string name;
	bool active;
	vector <string> DH_symbols (4);
	DH_symbols[0] = "a";
	DH_symbols[1] = "d";
	DH_symbols[2] = "alpha";
	DH_symbols[3] = "theta";

	for (unsigned short int i=0;i<robot.size();i++)
	{
		robot[i]["name"] >> DATA.name;
		robot[i]["active"] >> DATA.active;
		robot[i]["joint_type"] >> DATA.joint_type;
		double number;
		
		for (unsigned short int j=0;j<4;j++)
		{
			robot[i][DH_symbols[j]] >> number;
			DATA.DH[j] =number;
		}

		robot_data.push_back(DATA);
	}
}

//--------------------------

//pulls indices of robot vector data for starting and ending links of chain
//determines chain directionality (forward,backward)
void Kinematics::chain_ends_indices(vector <data> &robot_in,char* &start_link,char* &end_link)
{
	robot_data = robot_in;
	string starting_link = string(start_link);
	string ending_link = string(end_link);
	num_active_joints = 0;

	for (unsigned short int j=0; j<robot_data.size(); j++)
	{
		if (starting_link == robot_data[j].name)
		{
			start_index = j;
		}
		else if (ending_link == robot_data[j].name)
		{
			end_index = j;
		}

		if (robot_data[j].active == true)
		{
			++num_active_joints;
		}
	}

	if (start_index < end_index)
	{
		direction = "forward";
	}
	else
	{
		direction = "backward";
	}
}

//Sets current joint variable positions of the robot
void Kinematics::joint_var_pos(mat &joints_in)
{
	i=0;
	for (unsigned short int j=0; j<robot_data.size(); j++)
	{
		if (robot_data[j].active == true)
		{
			robot_data[j].joint_var = joints_in(i,0);
			++i;
		}
	}
}

//calculates transformation of link
void Kinematics::transform(unsigned short int index)
{
	//Generate transformation matrix
	a = robot_data[index].DH[0];
	d = robot_data[index].DH[1];
	alpha = robot_data[index].DH[2];
	theta = robot_data[index].DH[3];

	if (robot_data[index].joint_type == "revolute")
	{
		theta = theta + robot_data[index].joint_var;
	}
	else if (robot_data[index].joint_type == "prismatic")
	{
		d = d + robot_data[index].joint_var;
	}
	else
	{
		cout<<"Incorrect joint type labeling"<<endl;
		cin.get();
	}

	T_temp << cos(theta) << -sin(theta)*cos(alpha) << sin(theta)*sin(alpha) << a*cos(theta) << endr
		<< sin(theta) << cos(theta)*cos(alpha) << -cos(theta)*sin(alpha) << a*sin(theta) << endr
		<< 0 << sin(alpha) << cos(alpha) << d << endr
		<< 0 << 0 << 0 << 1 << endr;
}

//inverse of transformation matrix - could write this to be faster than armadillo's inv() function
//		void inverse_transform()
//		{
//		}

//initializes Jacobian, origins, and Z's
void Kinematics::initialize(string finger_in)
{
	active_link = 0;
	J = randu<mat>(3,num_active_joints);
	mat temp = randu<mat>(3,1);
	//mat Z = temp;
	//mat origin = temp;

	//Assumes ground z-axis is [0;0;1] and ground origin is [0;0;0]
	Z_static << 0.0 << endr
		<< 0.0 << endr
		<< 1.0 << endr
		<< 0.0 << endr;

	origin_static << 0.0 << endr
		<< 0.0 << endr
		<< 0.0 << endr
		<< 1.0 << endr;

	end_effector_transform = eye(4,4);

	origins_vector.push_back(origin_static);
	Z_vector.push_back(Z_static);

	for (unsigned short int j=0; j<num_active_joints; j++)
	{
		origins_vector.push_back(temp);
		Z_vector.push_back(temp);
	}

	//Set finger
	finger = finger_in;

	//Calculate static transforms for palm offsets
	if (finger == "finger0")
	{
		Tpf_pre = translation('y',.019053)*translation('x',.033);
	}

	if (finger == "finger1")
	{
		Tpf_pre = rotation('y',-datum::pi/2)*translation('y',-.038105);
		Tpf = Tpf_pre;
	}

	if (finger == "finger2")
	{
		Tpf_pre = translation('y',.019053)*translation('x',-.033);
	}

	Ry_neg90 = rotation('y',-datum::pi/2);
}

mat Kinematics::rotation(char axis, double angle)
{
	if (axis == 'x')
	{
		aR << 1 << 0 << 0 << 0 << endr
		   << 0 << cos(angle) << -sin(angle) << 0 << endr
		   << 0 << sin(angle) << cos(angle) << 0 << endr
		   << 0 << 0 << 0 << 1 << endr;
	}

	if (axis == 'y')
	{
		aR << cos(angle) << 0 << sin(angle) << 0 << endr
		   << 0 << 1 << 0 << 0 << endr
		   << -sin(angle) << 0 << cos(angle) << 0 << endr
		   << 0 << 0 << 0 << 1 << endr;
	}

	if (axis == 'z')
	{
		aR << cos(angle) << -sin(angle) << 0 << 0 << endr
		   << sin(angle) << cos(angle) << 0 << 0 <<  endr
		   << 0 << 0 << 1 << 0 <<  endr
		   << 0 << 0 << 0 << 1 << endr;
	}

	return aR;
}

mat Kinematics::translation(char axis, double length)
{
	if (axis == 'x')
	{
		aT << 1 << 0 << 0 << length << endr
		   << 0 << 1 << 0 << 0 << endr
		   << 0 << 0 << 1 << 0 << endr
		   << 0 << 0 << 0 << 1 << endr;
	}

	if (axis == 'y')
	{
		aT << 1 << 0 << 0 << 0 << endr
		   << 0 << 1 << 0 << length << endr
		   << 0 << 0 << 1 << 0 << endr
		   << 0 << 0 << 0 << 1 << endr;
	}

	if (axis == 'z')
	{
		aT << 1 << 0 << 0 << 0 << endr
		   << 0 << 1 << 0 << 0 << endr
		   << 0 << 0 << 1 << length << endr
		   << 0 << 0 << 0 << 1 << endr;
	}

	return aT;
};

//extracts z-axis and joint pos data for jacobian construction
void Kinematics::extract_jacobian_data(unsigned short int index)
{
	//increment active link number for Jacobian construction purposes
	Z_vector[0] = Z_static;
	origins_vector[0] = origin_static;

	if (robot_data[index].active == true)
	{
		Z_vector[active_link+1] = T.submat(0,2,3,2); //z-axis
		//cout << T.submat(0,2,2,2) << endl;
		origins_vector[active_link+1] = T.submat(0,3,3,3); //origin
		//cout << T.submat(0,3,2,3) << endl;
		++active_link;
	}

	//reset active link number after last link reached. Jacobians aren't made in backward chain
	//directions so only handles forward case
	if (index == end_index)
	{
		active_link = 0;
		end_effector = T.submat(0,3,3,3); //collects end effector position
		//cout << "end: " <<  end_effector << endl;
	}
}

//Generates Transformation matrix from starting to ending links. Also saves relevant data
//for the construction of the Jacobian
void Kinematics::Tmatrix()
{
	T = eye<mat>(4,4);
	//needs to handle different joint types and chain directionality
	if (direction == "forward")
	{
		for (unsigned short int j=start_index; j<=end_index; j++)
		{
			transform(j);
			T = T*T_temp;
			if (j==end_index)
			{
				end_effector_transform.submat(0,3,2,3)=COP_offset;
				//cout << "transform: " <<  end_effector_transform << endl;
				T=T*end_effector_transform;
			}

			extract_jacobian_data(j);
		}
	}
	else
	{
		for (unsigned short int j=end_index; j>=start_index; j--)
		{
			transform(j);
			T=T*inv(T_temp);
			//extract_jacobian_data(j); //Jacobian not really for backwards kinematics
		}
	} 
}

//Transforms for palm coordinate system to fixed, first joint coordinate systems
void Kinematics::T_world_joints(double joint0) 
{
	if (finger == "finger0")
	{
		Tpf = Tpf_pre*rotation('z',datum::pi-joint0)*Ry_neg90;
	}

	if (finger == "finger2")
	{
		Tpf = Tpf_pre*rotation('z',datum::pi+joint0)*Ry_neg90;
	}
};

//Calculates the linear velocity portion of the Jacobian matrix for select end link
void Kinematics::jacobian()
{	
	//cout << "tool tip loc: " << end_effector << endl;

	//Adjust end_effectors
	end_effector_palm = Tpf*end_effector;
	//cout << "end_effector: " << endl;
	//cout << "end_effector " << end_effector.t() << endl;

	for (unsigned short int j=0; j<num_active_joints; j++)
	{
		//Adjust Z_vectors and origins by palm transform offets
		//Z_vector[j] = Tpf*Z_vector[j];
		//origins_vector[j] = Tpf*origins_vector[j];

		//cout << "j " << j << endl;
		//cout << "Z_vector[j] " << Z_vector[j] << endl;
		//cout << "origins_vector[j]" << origins_vector[j] << endl;

		//cout << "Z_vector[j]" << Z_vector[j] << endl;
		//cout << "origins_vector[j]" << origins_vector[j] << endl;
		//cout << "end_effector" << end_effector << endl;
		J.col(j) = cross(Z_vector[j].submat(0,0,2,0),end_effector.submat(0,0,2,0)-origins_vector[j].submat(0,0,2,0));
		//cout << Z_vector[j] << endl;
		//cout << origins_vector[j] << endl;
	}

	//cout << "J: " << J << endl;
}
