#include "manipulation_control.h"

using namespace arma;
using namespace std;

manipulation_control::manipulation_control(double rc_ic[]) //constructor with IC on pose
{
	//Initializing desired and measured states of object
	collision = false;
	init = false;
	rc.zeros(6,1);
	rcd.zeros(6,1);
	pc.zeros(3,1);
	e1.zeros(6,1);
	time = 0;
	vel_error.zeros(6,1);

	for (iterator=0;iterator<6;++iterator)
	{
		rc(iterator,0)=rc_ic[iterator];
		rcd(iterator,0)=rc_ic[iterator];
		if (iterator < 3) {pc(iterator,0)=rc_ic[iterator];}
	}

	rc_dot.zeros(6,1);
	rcd_dot.zeros(6,1);

	//Initializing command forces
	f0_d = zeros(3,1);
	f1_d = zeros(3,1);
	f2_d = zeros(3,1);

	//Initializing grasp matrices
	Bn.zeros(6,3);
	Bf.zeros(6,3); //only 6x3 because fingers are planar

	//Matrix of ones
	I.ones(3,1); //three fingers

	//Set control shaping scalars
	gamma1.zeros(3,3);
	gamma1(0,0) = 12;//8; //3;//10; //1.5; //7;
	gamma1(1,1) = 12;//8;
	gamma1(2,2) = 12;//8;
	gamma2 = 5*gamma2.eye(3,3);
	gamma3.zeros(3,3);
	gamma3(0,0) = 4;//7; //6 - box, 8 - sphere
	gamma3(1,1) = 4;//5; 
	gamma3(2,2) = 4;//5; //5 - box, 6 - sphere 

	//Other initializations
	Q_integrand_prev.zeros(3,1);
	Q.zeros(3,1);
	vf_integrand_prev.zeros(3,1);
	vf.zeros(3,1);
	mu_s = 2;
	Z << 0 << endr << 0 << endr << 1 << endr;

	//Set control gains
	Kf = 2; //100; //10; //40.;
	betaf = .01; //.1; //1;
	Ks = 1; //100; //(10./75)*Kf;
	alpha1 = 2000; //4;
	alpha2 = .01; //4; //45;

	//Data Recording
	time_batch.resize(1,1);
	rc_batch.resize(6,1);
	rcd_batch.resize(6,1);

	p0d.zeros(3,1);
	random_matrix.resize(1,1);
	switch1=0;
};


void manipulation_control::reset(double rc_ic[])
{
	init = false;
	rc.zeros(6,1);
	rcd.zeros(6,1);
	pc.zeros(3,1);
	e1.zeros(6,1);
	time = 0;
	vel_error.zeros(6,1);

	for (iterator=0;iterator<6;++iterator)
	{
		rc(iterator,0)=rc_ic[iterator];
		rcd(iterator,0)=rc_ic[iterator];
		if (iterator < 3) {pc(iterator,0)=rc_ic[iterator];}
	}

	rc_dot.zeros(6,1);
	rcd_dot.zeros(6,1);

	//Initializing command forces
	f0_d = zeros(3,1);
	f1_d = zeros(3,1);
	f2_d = zeros(3,1);

	//Initializing grasp matrices
	Bn.zeros(6,3);
	Bf.zeros(6,3); //only 6x3 because fingers are planar

	//Other initializations
	Q_integrand_prev.zeros(3,1);
	Q.zeros(3,1);
	vf_integrand_prev.zeros(3,1);
	vf.zeros(3,1);
	p0d.zeros(3,1);
	Q_integrand.zeros(3,1);
	vf_integrand.zeros(3,1);
}
void manipulation_control::calc_friction_directions() //calculates orthogonal set of friction force directions for each POC
{
	//scale resultant force
	//f0 = (datum::sqrt2*f0)/norm(f0,2);
	//f1 = (datum::sqrt2*f1)/norm(f1,2);
	//f2 = (datum::sqrt2*f2)/norm(f2,2);

	//unitize resultant force
	if (norm(f0,2) != 0) {f0_hat = f0/norm(f0,2);}
	else {f0_hat = fn0_hat;}
	if (norm(f1,2) != 0) {f1_hat = f1/norm(f1,2);}
	else {f1_hat = fn1_hat;}
	if (norm(f2,2) != 0) {f2_hat = f2/norm(f2,2);}
	else {f2_hat = fn2_hat;}

	//Get first set of friction force directions
	//ff01_hat = f0-fn0_hat;
	//ff11_hat = f1-fn1_hat;
	//ff21_hat = f2-fn2_hat;

	//cout << "ff01_hat: " << ff01_hat.t() << endl;

	//Get second set of friction force directions
	ff02_hat = cross(fn0_hat,Z); //f0); //ff01_hat);
	ff12_hat = cross(fn1_hat,Z); //f1); //ff11_hat);
	ff22_hat = cross(fn2_hat,Z); //f2); //ff21_hat);

	//cout << ff01_hat.t() << endl;

	//ff02_hat = ff02_hat/norm(ff02_hat,2);
	//ff12_hat = ff12_hat/norm(ff12_hat,2);
	//ff22_hat = ff22_hat/norm(ff22_hat,2);

	ff01_hat = cross(ff02_hat,fn0_hat);
	ff11_hat = cross(ff12_hat,fn1_hat);
	ff21_hat = cross(ff22_hat,fn2_hat);

	//ff01_hat = ff01_hat/norm(ff01_hat,2);
	//ff11_hat = ff11_hat/norm(ff11_hat,2);
	//ff21_hat = ff21_hat/norm(ff21_hat,2);

	//cout << "fn1_hat: " << fn1_hat.t() << endl;
	//cout << "ff11_hat: " << ff11_hat.t() << endl;
	//cout << "ff12_hat: " << ff12_hat.t() << endl;

	//cout << fn2_hat.t() << endl;

	//cout << "ff01_hat: " << ff01_hat.t() << endl;

	calc_artificial_friction();
};

void manipulation_control::calc_artificial_friction() //calculates orthogonal set of friction force directions for each POC
{
	if (time < 0.1 && p0d.n_rows < 4)
	{
		p0d_fixed = p0-pc;
		p0d_fixed.insert_rows(p0d_fixed.n_rows,1);
		p0d_fixed(3,0) = 1;
		p1d_fixed = p1-pc;
		p1d_fixed.insert_rows(p1d_fixed.n_rows,1);
		p1d_fixed(3,0) = 1;
		p2d_fixed = p2-pc;
		p2d_fixed.insert_rows(p2d_fixed.n_rows,1);
		p2d_fixed(3,0) = 1;
	}

	//cout << p0.t() << endl;

	//transform from body-fixed to palm coordinate system
	theta1 = rcd(3,0);
	theta2 = rcd(4,0);
	theta3 = rcd(5,0);
	//cout << theta1 << theta2 << theta3 << endl;
	/*
	Tpo << cos(theta2)*cos(theta3) << cos(theta3)*sin(theta1)*sin(theta2) - cos(theta1)*sin(theta3) << sin(theta1)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2) << rcd(0,0) << endr
		<< cos(theta2)*sin(theta3) << cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3) << cos(theta1)*sin(theta2)*sin(theta3) - cos(theta3)*sin(theta1) << rcd(1,0) << endr
		<< -sin(theta2) << cos(theta2)*sin(theta1) << cos(theta1)*cos(theta2) << rcd(2,0) << endr
		<< 0 << 0 << 0 << 1 << endr;
		*/
	Tpo << cos(theta2)*cos(theta3) <<-cos(theta2)*sin(theta3) << sin(theta2) << rcd(0,0) << endr
		<< cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2) << cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3) << -cos(theta2)*sin(theta1) << rcd(1,0) << endr
		<< sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2) << cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3) << cos(theta1)*cos(theta2) << rcd(2,0) << endr
		<< 0 << 0 << 0 << 1 << endr;

	//cout << "Tpo: " << Tpo << endl;
	//cout << "rcd: " << rcd.t() << endl;
	//cout << Tpo << endl;
	p0d = Tpo*p0d_fixed;
	p1d = Tpo*p1d_fixed;
	p2d = Tpo*p2d_fixed;

	//cout << p0d.t() << endl;

	if (norm(p0d.submat(0,0,2,0)-p0,2) > 0) {p0df = (p0d.submat(0,0,2,0)-p0)/norm(p0d.submat(0,0,2,0)-p0,2);}
	else {p0df.zeros(3,1);}
	if (norm(p1d.submat(0,0,2,0)-p1,2) > 0) {p1df = (p1d.submat(0,0,2,0)-p1)/norm(p1d.submat(0,0,2,0)-p1,2);}
	else {p1df.zeros(3,1);}
	if (norm(p2d.submat(0,0,2,0)-p2,2) > 0) {p2df = (p2d.submat(0,0,2,0)-p2)/norm(p2d.submat(0,0,2,0)-p2,2);}
	else {p2df.zeros(3,1);}

	/*
	fn0_hat(0,0) = 0;
	fn0_hat(1,0) = -1;
	fn0_hat(2,0) = 0;

	fn1_hat(0,0) = 0;
	fn1_hat(1,0) = 1;
	fn1_hat(2,0) = 0;

	fn2_hat(0,0) = 0;
	fn2_hat(1,0) = -1;
	fn2_hat(2,0) = 0;
	*/

	ff02_hat = cross(fn0_hat,cross(p0df,fn0_hat));
	ff12_hat = cross(fn1_hat,cross(p1df,fn1_hat));
	ff22_hat = cross(fn2_hat,cross(p2df,fn2_hat));

	//cout << p0d.t() << endl;
	//cout << "actual " << p0.t() << endl;
	//cout << "direction " << p0df.t() << endl;
	//cout << cross(p0df,fn0_hat).t() << endl;
	//cout << ff22_hat.t() << endl;
	//cout << ff11_hat.t() << endl;
	//cout << ff21_hat.t() << endl;

}
void manipulation_control::calc_grasp_matrices() //calculates grasp matrices
{
	//appending unitized normal forces
	/*
	Bn.submat(0,0,2,0) = f0_hat;
	Bn.submat(0,1,2,1) = f1_hat; //(fn1_hat+ff11_hat)/norm(fn1_hat+ff11_hat,2); //f1_hat; //fn1_hat;
	Bn.submat(0,2,2,2) = f2_hat;
	*/
	
	Bn.submat(0,0,2,0) = (fn0_hat+ff02_hat)/norm(fn0_hat+ff02_hat,2); //f0_hat-Z; //(fn0_hat+ff01_hat)/norm(fn0_hat+ff01_hat,2); //f0_hat; //fn0_hat;
	Bn.submat(0,1,2,1) = (fn1_hat+ff12_hat)/norm(fn1_hat+ff12_hat,2); //f1_hat-Z; //(fn1_hat+ff11_hat)/norm(fn1_hat+ff11_hat,2); //f1_hat; //fn1_hat;
	Bn.submat(0,2,2,2) = (fn2_hat+ff22_hat)/norm(fn2_hat+ff22_hat,2); //f2_hat-Z; //(fn2_hat+ff21_hat)/norm(fn2_hat+ff21_hat,2); //f2_hat; //fn2_hat;
	
	//appending moments
	/*
	Bn.submat(3,0,5,0) = cross(p0-pc,fn0_hat); //cross(p0-pc,(fn0_hat+ff01_hat)/norm(fn0_hat+ff01_hat,2)); //cross(p0-pc,fn0_hat);
	Bn.submat(3,1,5,1) = cross(p1-pc,fn1_hat); //cross(p1-pc,(fn1_hat+ff11_hat)/norm(fn1_hat+ff11_hat,2)); //cross(p1-pc,fn1_hat);
	Bn.submat(3,2,5,2) = cross(p2-pc,fn2_hat); 
	*/

	Bn.submat(3,0,5,0) = cross(p0-pc,(fn0_hat+ff02_hat)/norm(fn0_hat+ff02_hat,2)); //cross(p0-pc,fn0_hat-Z); //cross(p0-pc,(fn0_hat+ff01_hat)/norm(fn0_hat+ff01_hat,2)); //cross(p0-pc,fn0_hat);
	Bn.submat(3,1,5,1) = cross(p1-pc,(fn1_hat+ff12_hat)/norm(fn1_hat+ff12_hat,2)); //cross(p1-pc,fn1_hat-Z); //cross(p1-pc,(fn1_hat+ff11_hat)/norm(fn1_hat+ff11_hat,2)); //cross(p1-pc,fn1_hat);
	Bn.submat(3,2,5,2) = cross(p2-pc,(fn2_hat+ff22_hat)/norm(fn2_hat+ff22_hat,2)); //cross(p2-pc,fn2_hat-Z); //cross(p2-pc,(fn2_hat+ff21_hat)/norm(fn2_hat+ff21_hat,2)); //cross(p2-pc,fn2_hat);
	
	//appending unitized friction forces
	Bf.submat(0,0,2,0) = ff01_hat;
	//Bf.submat(0,1,2,1) = ff02_hat;
	Bf.submat(0,1,2,1) = ff11_hat;
	//Bf.submat(0,3,2,3) = ff12_hat;
	Bf.submat(0,2,2,2) = ff21_hat;
	//Bf.submat(0,5,2,5) = ff22_hat;
	//appending moments
	Bf.submat(3,0,5,0) = cross(p0-pc,ff01_hat);
	//Bf.submat(3,1,5,1) = cross(p0-pc,ff02_hat);
	Bf.submat(3,1,5,1) = cross(p1-pc,ff11_hat);
	//Bf.submat(3,3,5,3) = cross(p1-pc,ff12_hat);
	Bf.submat(3,2,5,2) = cross(p2-pc,ff21_hat);
	//Bf.submat(3,5,5,5) = cross(p2-pc,ff22_hat);
};

void manipulation_control::errors() //calculates tracking errors
{
	/*
	if (datum::pi*14 >= time && time > datum::pi*5.5) {rcd(2,0) = -(.01*sin(1*(time-datum::pi*6)))+.14;}
	if (datum::pi*24 >= time && time > datum::pi*15) {rcd(3,0) = -((datum::pi/20)*sin(1*(time-datum::pi*15)));}
	if (datum::pi*34 >= time && time > datum::pi*25) {rcd(4,0) = -((datum::pi/20)*sin(1*(time-datum::pi*25)));}
	*/
	
	if (datum::pi*14 >= time && time > datum::pi*5.5) {rcd(2,0) = -(.0075*sin(1*(time-datum::pi*6)))+.1425;}
	if (datum::pi*23 >= time && time > datum::pi*15) {rcd(3,0) = -((datum::pi/25)*sin(1.25*(time-datum::pi*15)));}
	if (datum::pi*32 >= time && time > datum::pi*24) {rcd(4,0) = -((datum::pi/25)*sin(0.75*(time-datum::pi*24)));}
	
	
	if (datum::pi*41 >= time && time > datum::pi*33) {rcd(2,0) = -(.0075*sin(1*(time-datum::pi*33)))+.1425;}
	if (datum::pi*41 >= time && time > datum::pi*33) {rcd(3,0) = -((datum::pi/25)*sin(1.25*(time-datum::pi*33)));}
	if (datum::pi*41 >= time && time > datum::pi*33) {rcd(4,0) = -((datum::pi/25)*sin(0.75*(time-datum::pi*33)));}
	

	//if (datum::pi*44 >= time && time > datum::pi*35) {rcd(1,0) = -(.01*sin(.5*(time-datum::pi*35)));}
	
	//if (time >= datum::pi*6) {rcd(1,0) = -(.01*sin(.5*(time-datum::pi*6)));}
	
	//cout << rcd.t() << endl;
	//cout << rc.t() << endl;
	//datum::pi*30 >= time && 
	//if (time > datum::pi*6) {rcd(2,0) = -(.005*sin(.5*(time-datum::pi*6)))+.15;}
	
	//if (fmod(time,10) > 5) {rcd(2,0) = .1475;}
	//else {rcd(2,0)=.15;}
	
	//cout << rc(3,0) << " " << rc(4,0) << endl;
	
	/*
	//Dithering
	if (time > 4 && rcd(2,0) < .158 && collision == false)
	{
	if (fmod(time,.05) > .025) 
	{
		if (switch1==0)
		{
			random_matrix.randu();
			rcd(3,0) = .2*random_matrix(0,0)-.1;
			random_matrix.randu();
			rcd(4,0) = .2*random_matrix(0,0)-.1;
			random_matrix.randu();
		
			rcd(2,0) = rcd(2,0)+.0002;
			switch1=1;
		}		
	}

	else 
	{
		if (switch1==1)
		{
			
			//random_matrix.randu();
			//rcd(3,0) = .2*random_matrix(0,0)-.1;
			//random_matrix.randu();
			//rcd(4,0) = .2*random_matrix(0,0)-.1;
			random_matrix.randu();
			
			rcd(2,0) = rcd(2,0)-.0001;
			switch1=0;
		}
	}
	}
	else if (rcd(2,0) > .152 && collision == true && fmod(time,.05) > .025) {rcd(2,0) = rcd(2,0)-.00005;}
	*/
	//else {rcd(3,0) = 0; rcd(4,0) = 0;}
	
	//cout << rcd(2,0) << endl;
	
	//if (time <=15 && time > 10) {rcd(2,0) = .145;}
	//if (time>15) {rcd(2,0) = .15;}
	//if (time<15 && time > 10) {rcd(2,0) = rcd(2,0)-.00002;}
	//if (time < 20 && time >= 15) {rcd(2,0) = rcd(2,0)+.00002;}

	/*
	//spiraling
	if (time > 10)
	{
		rcd(3,0) = .1*sin(.2*(time-10));
		rcd(4,0) = .1*sin(.2*(time-10));
		rcd(5,0) = .1*sin(.2*(time-10));
	}
	*/

	//if (datum::pi*24 >= time && time > datum::pi*4) {rcd(3,0) = -((datum::pi/25)*sin(.5*(time-datum::pi*4)));}
	
	//if (datum::pi*34 >= time && time > datum::pi*24) {rcd(1,0) = -(.02*sin(.5*(time-datum::pi*6))); rcd_dot(2,0) = -(.5*.02*cos(.5*(time-datum::pi*6)));}

	//if (10 >= time && time > 5)  {rcd(2,0) = .135; rcd(0,0)=(randu()-.5)*.01;rcd(1,0)=(randu()-.5)*.01; }
	
	/*
	if (10 < time && time < 20)  {rcd(2,0) = .155;}
	if (20 < time && time < 30)  {rcd(2,0) = .15;}
	if (30 < time && time < 40)  {rcd(2,0) = .155;}
	if (40 < time && time < 50)  {rcd(2,0) = .15;}
	*/
	//if (10 < time && time < 20) {rcd(2,0) = .15+.01*tanh(.2*(time-10));}// {rcd(5,0) = datum::pi/10;}

	//if (time < 30 && time >= 20) {rcd(2,0) = .16-.02*tanh(.2*(time-20));}
	//if (time >= 30) {rcd(2,0) = .14+.01*tanh(.2*(time-30));}
	//cout << rcd(2,0) << endl;
	//rc(5,0) = -rc(5,0);
	//cout << rc.t() << endl;
	e1 = rcd-rc;

	//cout << e1.t() << endl;

	vel_error = rcd_dot-rc_dot;
	vel_error.zeros();

	e2 = vel_error+alpha1*e1;
};

mat manipulation_control::sigmoid(mat input_matrix) //sigmoid function - potentially remove for speed
{
	return 1/(1+exp(-input_matrix));
};

void manipulation_control::calc_Bn_dot()
{
	Bn_dot = (Bn-Bn_prev)/dt;
};

void manipulation_control::calc_Un()
{
	calc_Bn_dot();

	//Need to put artificial limits on Q_integrand and Q?
	Q_integrand = Ks*(-Bn_dot.t()*e2+Bn.t()*alpha2*e2);
	Q = (Q_integrand_prev)*dt+Q;
	Q_integrand_prev = Q_integrand;
	
	for (iterator=0;iterator<3;++iterator)
	{
		if (Q(iterator,0) > 10) {Q(iterator,0)=10; Q_integrand_prev(iterator,0)=0;}
		if (Q(iterator,0) < 0) {Q(iterator,0)=0; Q_integrand_prev(iterator,0)=0;}
	};
	
	
	//cout << "Q: " << Q.t() << endl;
	//cout << (Ks*Bn.t()*e2).t() << endl;

	Q.zeros(3,1);
	w = Ks*Bn.t()*e2+Q;
	//cout << Bn.t() << endl;
	//cout << e2 << endl;
	//cout << "w: " << w.t() << endl;
	//cout << "Q: " << Q.t() << endl;
	Un = gamma1*sigmoid(w-gamma2*I) + gamma3*I;
	cout << Un.t() << endl;
	//cout << e2 << endl;


	//cout << Bn << endl;
	//cout << "pc: " << pc.t() << endl;
	//cout << "e1: " << e1.t() << endl;
	//cout << "e2: " << e2.t() << endl;
	//cout << "Q: " << Q.t() << endl;
	//cout << "w: " << w.t() << endl;
	//cout << "Un: " << Un.t() << endl;
};

void manipulation_control::calc_Uf()
{
	vf_integrand = Kf*alpha2*Bf.t()*e2+betaf*Bf.t()*sign(e2);
	vf = (vf_integrand_prev)*dt+vf;
	vf_integrand_prev = vf_integrand;

	/*
	for (iterator=0;iterator<3;++iterator)
	{
		if (vf(iterator,0) > 3) {vf(iterator,0)=3; vf_integrand_prev(iterator,0)=0;}
		if (vf(iterator,0) < -3) {vf(iterator,0)=-3; vf_integrand_prev(iterator,0)=0;}
	};
	*/

	//cout << "rc: " << rc.t() << endl;
	//cout << "e1: " << e1.t() << endl;
	//cout << "Bf: " << Bf << endl;
	//cout << Uf.t() << endl;

	Uf = Kf*Bf.t()*e2+vf;

	//cout << Bf << endl;
	//cout << "Uf: " << Uf.t() << endl;
};

void manipulation_control::calc_command_forces()
{
	//command normal forces
	fn0_d = fn0_hat*Un(0,0);
	fn1_d = fn1_hat*Un(1,0);
	fn2_d = fn2_hat*Un(2,0);

	//cout << "fn0_d: " << fn0_d.t() << endl;
	//cout << "fn1_d: " << fn1_d.t() << endl;
	//cout << "fn2_d: " << fn2_d.t() << endl;

	//command friction forces
	ff0_d = ff01_hat*Uf(0,0);//+ff02_hat*Uf(1,0);
	ff1_d = ff11_hat*Uf(1,0);//+ff12_hat*Uf(3,0);
	ff2_d = ff21_hat*Uf(2,0);//+ff22_hat*Uf(5,0);

	//ff0_d.zeros(3,1);
	//ff1_d.zeros(3,1);
	//ff2_d.zeros(3,1);

	//cout << "ff0_d: " << ff0_d.t() << endl;
	//cout << "ff1_d: " << ff1_d.t() << endl;
	//cout << "ff2_d: " << ff2_d.t() << endl;

	ff0_d_norm = norm(ff0_d,2);
	ff1_d_norm = norm(ff1_d,2);
	ff2_d_norm = norm(ff2_d,2);

	//Projection scheme for command friction forces
	if (ff0_d_norm > mu_s*Un(0,0)) 
	{
		ff0_d = mu_s*Un(0,0)*(ff0_d/ff0_d_norm); 
		//if (vf(0,0) > 0) {vf(0,0)=mu_s*Un(0,0);}
		//if (vf(0,0) < 0) {vf(0,0)=-mu_s*Un(0,0);}
		vf_integrand_prev(0,0) = 0;
	}
	if (ff1_d_norm > mu_s*Un(1,0)) 
	{
		ff1_d = mu_s*Un(1,0)*(ff1_d/ff1_d_norm); 
		//if (vf(1,0) > 0) {vf(1,0)=mu_s*Un(1,0);}
		//if (vf(1,0) < 0) {vf(1,0)=-mu_s*Un(1,0);}
		vf_integrand_prev(1,0) = 0;
	}
	if (ff2_d_norm > mu_s*Un(2,0)) 
	{
		ff2_d = mu_s*Un(2,0)*(ff2_d/ff2_d_norm); 
		//if (vf(2,0) > 0) {vf(2,0)=mu_s*Un(2,0);}
		//if (vf(2,0) < 0) {vf(2,0)=-mu_s*Un(2,0);}
		vf_integrand_prev(2,0) = 0;
	}

	//cout << "ff0_d: " << ff0_d.t() << endl;
	//cout << "fn0_d: " << fn0_d.t() << endl;

	f0_d = fn0_d + ff0_d;
	f1_d = fn1_d + ff1_d;
	f2_d = fn2_d + ff2_d;

	//cout << norm(ff0_d,2) << endl;
	//cout << fn0_d.t() << endl;
	//cout << "fn2_d: " << fn2_d.t() << endl;
	//cout << "ff2_d: " << ff2_d.t() << endl;

	//cout << "ff21_hat: " << ff21_hat.t() << endl;

	//cout << "Un: " << Un.t() << endl;
	//cout << "e1: " << e1.t() << endl;
};

void manipulation_control::append() //appends new data
{
	time_batch.insert_cols(time_batch.n_cols,1);
	time_batch(0,time_batch.n_cols-1) = time;
	rc_batch.insert_cols(rc_batch.n_cols,rc);
	rcd_batch.insert_cols(rcd_batch.n_cols,rcd);
}

void manipulation_control::write_file()
{
	ofstream pose_data;
	pose_data.open("pose_data.csv");

    pose_data << "Time, X (m), Y (m), Z (m), Rx (rad), Ry (rad), Rz (rad), Xd (m), Yd (m), Zd (m), Rxd (rad), Ryd (rad), Rzd (rad)" << endl;

	for (int i=0;i<time_batch.n_cols;++i)
	{
		pose_data << time_batch(0,i) << ", " << rc_batch(0,i) << ", " << rc_batch(1,i) << ", " << rc_batch(2,i) << ", " << rc_batch(3,i) << ", " << rc_batch(4,i) << ", " << rc_batch(5,i) << ", " << rcd_batch(0,i) << ", " << rcd_batch(1,i) << ", " << rcd_batch(2,i) << ", " << rcd_batch(3,i) << ", " << rcd_batch(4,i) << ", " << rcd_batch(5,i)<< endl;  
		//cout << i << endl;
	}

	pose_data.close();
};

void manipulation_control::calc_control(double del_t) //runs one time step of manipulation controller
{
	//Normal force directions, contact force resultants, points of contact, and object states (rc, rc_dot) are set externally
	dt = del_t;
	time = time + dt;

	//cout << fn0_hat.t() << endl;
	//cout << "fn1_hat: " << fn1_hat.t() << endl;
	//cout << fn2_hat.t() << endl;

	//If any one finger loses contact, set desired force to 0's
	//if (accu(abs(f0))==0 || accu(abs(f1))==0 || accu(abs(f2))==0)
	//{
	//	f0_d.zeros(3,1);
	//	f1_d.zeros(3,1);
	//	f2_d.zeros(3,1);
	//}

	//else
	//{
		if (init == false)
		{
			calc_friction_directions();
			calc_grasp_matrices();
			init=true;
		}
		else
		{
			//Update pc
			for (iterator=0;iterator<3;++iterator) {pc(iterator,0)=rc(iterator,0);}

			//Engage all functions
			errors();
			//cout << "e1: " << e1.t() << endl;
			//cout << "e2: " << e2.t() << endl;
			calc_friction_directions();
			calc_grasp_matrices();
			calc_Un();
			calc_Uf();
			calc_command_forces();
			append();
		}

		Bn_prev = Bn;
	//}
	
	
};