#include "Object_Tracking.h"

using namespace arma;
using namespace std;

Object_Tracking::Object_Tracking(double rc_ic[])  //constructs with IC's of pose
{
	rc.zeros(6,1);
	pc.zeros(3,1);
	thetac.zeros(3,1);

	for (iterator=0;iterator<6;++iterator)
	{
		rc(iterator,0)=rc_ic[iterator];
		if (iterator < 3) {pc(iterator,0)=rc_ic[iterator];}
		if (iterator > 2) {thetac(iterator-3,0)=rc_ic[iterator];}
	}

	rc_dot.zeros(6,1);
	vc.zeros(3,1);
	omegac.zeros(3,1);
	vc_prev.zeros(3,1);
	omegac_prev.zeros(3,1);
	v0.zeros(3,1);
	v1.zeros(3,1);
	v2.zeros(3,1);
	R1.zeros(3,1);
	R2.zeros(3,1);
	V1.zeros(3,1);
	V2.zeros(3,1);
	V.zeros(6,1);
	A.zeros(6,3);

	init=false;
};

void Object_Tracking::reset(double rc_ic[])
{
	rc.zeros(6,1);
	pc.zeros(3,1);
	thetac.zeros(3,1);

	for (iterator=0;iterator<6;++iterator)
	{
		rc(iterator,0)=rc_ic[iterator];
		if (iterator < 3) {pc(iterator,0)=rc_ic[iterator];}
		if (iterator > 2) {thetac(iterator-3,0)=rc_ic[iterator];}
	}

	rc_dot.zeros(6,1);
	vc.zeros(3,1);
	omegac.zeros(3,1);
	vc_prev.zeros(3,1);
	omegac_prev.zeros(3,1);
	v0.zeros(3,1);
	v1.zeros(3,1);
	v2.zeros(3,1);
	R1.zeros(3,1);
	R2.zeros(3,1);
	V1.zeros(3,1);
	V2.zeros(3,1);
	V.zeros(6,1);
	A.zeros(6,3);

	init=false;
}

void Object_Tracking::COP_vel() //time derivative of COPs to get COP velocities
{
	v0 = (p0-p0_prev)/dt;
	v1 = (p1-p1_prev)/dt;
	v2 = (p2-p2_prev)/dt;
};

void Object_Tracking::rel_vel() //calculates relative velocities between three points, include compiled matrix
{
	V1 = v0-v1;
	V2 = v1-v2;
	for (iterator=0;iterator<3;++iterator)
	{
		V(iterator,0) = V1(iterator,0);
		V(iterator+3,0) = V2(iterator,0);
	}

	
};

void Object_Tracking::rel_pos() //calculates relative positions between three points, include compiled matrix
{
	R1 = p0-p1;
	R2 = p1-p2;
	A(0,1) = R1(2,0);
	A(0,2) = -R1(1,0);
	A(1,0) = -R1(2,0);
	A(1,2) = R1(0,0);
	A(2,0) = R1(1,0);
	A(2,1) = -R1(0,0);
	A(3,1) = R2(2,0);
	A(3,2) = -R2(1,0);
	A(4,0) = -R2(2,0);
	A(4,2) = R2(0,0);
	A(5,0) = R2(1,0);
	A(5,1) = -R2(0,0);
};

void Object_Tracking::ang_vel() //calculates angular velocity
{
	omegac = pinv(A)*V; //inv(A.t()*A)*A.t()*V;
	//cout << omegac.t() << endl;
	//cout << omegac.t() << endl;
	for (iterator=0;iterator<3;++iterator)
	{
		rc_dot(iterator+3,0) = omegac(iterator,0);
	};
};

void Object_Tracking::lin_vel() //calculates linear velocity of control point on object
{
	vc = v0+cross(omegac,pc-p0);
	vc = vc + v1+cross(omegac,pc-p1);
	vc = vc + v2+cross(omegac,pc-p2);
	vc = vc/3.;
	for (iterator=0;iterator<3;++iterator)
	{
		rc_dot(iterator,0) = vc(iterator,0);
	};
}; 

void Object_Tracking::pose() //updates pose of object using numerical integration. use quadrature integration
{
	thetac = omegac*dt+thetac;
	pc = vc*dt+pc;
	//thetac = (omegac+omegac_prev)*dt/2+thetac;
	//pc = (vc+vc_prev)*dt/2+pc;
	omegac_prev = omegac;
	vc_prev = vc;
	//cout << pc.t() << endl;

	for (iterator=0;iterator<3;++iterator)
	{
		rc(iterator,0) = pc(iterator,0);
		rc(iterator+3,0) = thetac(iterator,0);
		//rc(5,0) = -rc(5,0);
	};
}; 

void Object_Tracking::run(double del_t, mat &finger0_poc, mat &finger1_poc, mat &finger2_poc)
{
	dt=del_t;
	p0 = finger0_poc.submat(0,0,2,0);
	p1 = finger1_poc.submat(0,0,2,0);
	p2 = finger2_poc.submat(0,0,2,0);
	if (init==false)
	{
		p0_prev = finger0_poc.submat(0,0,2,0);
		p1_prev = finger1_poc.submat(0,0,2,0);
		p2_prev = finger2_poc.submat(0,0,2,0);
		init=true;
	}

	else
	{
		rel_pos();
		if (det(A.t()*A)==0)
		{
			cout << p0.t() << endl;
			cout << p1.t() << endl;
			cout << p2.t() << endl;
			rc_dot.zeros(6,1);
			vc.zeros(3,1);
			omegac.zeros(3,1);
		}

		else
		{
			COP_vel();
			rel_vel();
			ang_vel();
			lin_vel();
			pose();
		}
	}

	p0_prev = finger0_poc.submat(0,0,2,0);
	p1_prev = finger1_poc.submat(0,0,2,0);
	p2_prev = finger2_poc.submat(0,0,2,0);
};