#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>

using namespace std;


///////////////////////////////////////////////////////////////////////////////
//   From "Numerical Recipes in C" for normal-distribution random numbers.   //
//          I did not write these, and cannot take credit for them.          //
///////////////////////////////////////////////////////////////////////////////

#define IA 16807
#define IM 2147483647
#define IQ 127773
#define IR 2836
#define NTAB 32
#define EPS (1.2E-07)
#define MAX(a,b) (a>b)?a:b
#define MIN(a,b) (a<b)?a:b

double ran1(long *idum)
{
  int j,k;
	static int iv[NTAB],iy=0;
	void nrerror();
  static double NDIV = 1.0/(1.0+(IM-1.0)/NTAB);
  static double RNMX = (1.0-EPS);
  static double AM = (1.0/IM);

	if ((*idum <= 0) || (iy == 0))
  {
		*idum = MAX(-*idum,*idum);
    for(j=NTAB+7;j>=0;j--) 
    {
			k = *idum/IQ;
			*idum = IA*(*idum-k*IQ)-IR*k;
			if(*idum < 0) *idum += IM;
			if(j < NTAB) iv[j] = *idum;
		}
		iy = iv[0];
	}
	k = *idum/IQ;
	*idum = IA*(*idum-k*IQ)-IR*k;
	if(*idum<0) *idum += IM;
	j = iy*NDIV;
	iy = iv[j];
	iv[j] = *idum;
	return MIN(AM*iy,RNMX);
}

//! generate random # w/ 0 mean & 1.0 variance
double gasDev(long *idum)
{
  double ran1(long *idum);
  static int iset = 0;
  static double gset;
  double fac, rsq, v1, v2;

  if (*idum < 0)
  {
    iset = 0;
  }
  if (iset == 0)
  {
    do
    {
      v1 = 2.0 * ran1(idum)-1.0;
      v2 = 2.0 * ran1(idum)-1.0;
      rsq = (v1 * v1) + (v2 * v2);
    } while (rsq >= 1.0 || rsq == 0.0);

    fac = sqrt(-2.0 * log(rsq) / rsq);
    gset = v1 * fac;
    iset = 1;
    return (v2 * fac);
  }
  else
  {
    iset = 0;
    return gset;
  }
}

///////////////////////////////////////////////////////////////////////////////


void main()
{
  ifstream train("train.dat");
  ifstream test("test.dat");
  ofstream trout("train1.dat");
  ofstream teout("test1.dat");
  ofstream trscale("train2.dat");
  ofstream tebad("testbad.dat");
  ofstream tegood("testgood.dat");
  ofstream trnoise("train_noisy.dat");
  ofstream tenoise("test_noisy.dat");
  ofstream trnoise2("train1_noisy.dat");
  ofstream tenoise2("test1_noisy.dat");

  int trnum, tenum, x, y, attrib, off = 200;
  double *raw, *out, *scout, *noise;
  static long seed = time(NULL);

  raw = new double[400];
  noise = new double[400];
  out = new double[199];
  scout = new double[3];

  cout << "number of training samples : ";
  cin >> trnum;
  cout << "number of testing samples : ";
  cin >> tenum;

  //! Extract & precompute the training data
  for (x = 0; x < trnum; ++x)
  {
    //! Grab the data from the file
    for (y = 0; y < 400; ++y)
    {
      train >> raw[y];

      noise[y] = raw[y] + (gasDev(&seed) * 0.5);
      if (y >= 200)
        trnoise2 << noise[y] << " ";
      else
        trnoise2 << raw[y] << " ";

      cout << raw[y] << " " << noise[y] << endl;
    }
    train >> attrib;
    trnoise2 << attrib << endl;

    //! Compute the motion metric
    scout[1] = -1;
    scout[2] = 1000;
    for (y = 0; y < 199; ++y)
    {
      out[y] = (raw[y+1] - raw[off+y+1]) / (raw[y] - raw[off+y]);
      trout << out[y] << " ";

      if (out[y] > scout[1])
      {
        scout[1] = out[y];
      }
      if (out[y] < scout[2])
      {
        scout[2] = out[y];
      }
    }
    trout << attrib << endl;
    scout[0] = out[0];
    trscale << scout[0] << " " << scout[1] << " " << scout[2] << " " << attrib << endl;
//    trscale << scout[1] << " " << scout[2] << " " << attrib << endl;

    //! Compute the motion metric with noise
    scout[1] = -1;
    scout[2] = 1000;
    for (y = 0; y < 199; ++y)
    {
//      out[y] = (raw[y+1] - noise[off+y+1]) / (raw[y] - noise[off+y]);
      out[y] = (noise[y+1] - noise[off+y+1]) / (noise[y] - noise[off+y]);

      if (out[y] > scout[1])
      {
        scout[1] = out[y];
      }
      if (out[y] < scout[2])
      {
        scout[2] = out[y];
      }
    }
    scout[0] = out[0];
    trnoise << scout[0] << " " << scout[1] << " " << scout[2] << " " << attrib << endl;
  }


  //! Extract & precompute the testing data
  for (x = 0; x < tenum; ++x)
  {
    //! Grab the data from the file
    for (y = 0; y < 400; ++y)
    {
      test >> raw[y];
      noise[y] = raw[y] + (gasDev(&seed) * 0.5);
      if (y >= 200)
        tenoise2 << noise[y] << " ";
      else
        tenoise2 << raw[y] << " ";
    }
    test >> attrib;
    tenoise2 << attrib << endl;

    //! Compute the motion metric
    scout[1] = -1;
    scout[2] = 1000;
    for (y = 0; y < 199; ++y)
    {
      out[y] = (raw[y+1] - raw[off+y+1]) / (raw[y] - raw[off+y]);
      teout << out[y] << " ";
      if (out[y] > scout[1])
      {
        scout[1] = out[y];
      }
      if (out[y] < scout[2])
      {
        scout[2] = out[y];
      }
    }
    teout << attrib << endl;
    scout[0] = out[0];
    if (attrib == 0)
    {
      tebad << scout[0] << " " << scout[1] << " " << scout[2] << " " << attrib << endl;
//      tebad << scout[1] << " " << scout[2] << " " << attrib << endl;
    }
    else
    {
      tegood << scout[0] << " " << scout[1] << " " << scout[2] << " " << attrib << endl;
//      tegood << scout[1] << " " << scout[2] << " " << attrib << endl;
    }

    //! Compute the motion metric with noise
    scout[1] = -1;
    scout[2] = 1000;
    for (y = 0; y < 199; ++y)
    {
//      out[y] = (raw[y+1] - noise[off+y+1]) / (raw[y] - noise[off+y]);
      out[y] = (noise[y+1] - noise[off+y+1]) / (noise[y] - noise[off+y]);

      if (out[y] > scout[1])
      {
        scout[1] = out[y];
      }
      if (out[y] < scout[2])
      {
        scout[2] = out[y];
      }
    }
    scout[0] = out[0];
    tenoise << scout[0] << " " << scout[1] << " " << scout[2] << " " << attrib << endl;
  }

  delete [] raw;
  delete [] noise;
  delete [] out;
  delete [] scout;
}