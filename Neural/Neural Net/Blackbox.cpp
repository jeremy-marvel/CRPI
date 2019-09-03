///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        Blackbox.cpp
//  Revision:        1.0 - 27 January, 2009
//
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Black box simulator/placeholder that computes an N-dimensional gaussian for
//  testing purposes.
//
///////////////////////////////////////////////////////////////////////////////

#include "Models.h"
#include "../Neuron/Numerical/Numerical.h"
#include "../../Libraries/Math/NumericalMath.h"
#include <cmath>
#include <time.h>

#define DEBUGOUT
#define RANDOMNOISE

using namespace Math;
using namespace std;

//! Explicit instantiations
template class LIBRARY_API NeuralNet::Blackbox<Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::Blackbox<Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::Blackbox<Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::Blackbox<Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::Blackbox<Neuron::LogicNeuron>;

namespace NeuralNet
{
  template <class T> Blackbox<T>::Blackbox (int inFeat,
                                            int interFeat,
                                            int outFeat,
                                            double initWeight,
                                            double initEta) :
    inputFeatures_(inFeat),
    pTrainingPatterns_(0),
    pTestingPatterns_(0),
    noise_(GAUSSIAN)
  {
    //! Allocate memory for value passing & temporary storage
    inputs_ = new double [inputFeatures_ + 1];
    center_ = new double [inputFeatures_ + 1];
    sigma_ = new double [inputFeatures_ + 1];

    try
    {
      for (int x = 0; x <= inputFeatures_; ++x)
      {
        //! Center of gaussian is located at (3.14, 3.14, 3.14, ... , 3.14)
        inputs_[x] = 0.0f; //! Initialize to zero so we don't accidentally use junk
        center_[x] = 3.0f; //JAM:  modified to make later tests easier pi ();
        sigma_[x] = (double)x;
      }
    }
    catch (...)
    {
      //! Exception
    }
  }


  template <class T> Blackbox<T>::~Blackbox ()
  {
    try
    {
      delete [] inputs_;
      delete [] center_;
      delete [] sigma_;
    }
    catch (...)
    {
      //! Exception
    }
  }


  template <class T> void Blackbox<T>::initialize ()
  {
  }


  template <class T> double Blackbox<T>::trainNetwork (int iter,
                                                          bool prune)
  {
    return 0.0f;
  }


  template <class T> void Blackbox<T>::runTests (int k)
  {
  }


  template <class T> void Blackbox<T>::runTestsBest (int top, int k)
  {
  }


  template <class T> double Blackbox<T>::autoOptimize (vector<double>& minVals,
                                                          vector<double>& maxVals,
                                                          vector<double>& best,
                                                          int outWatch,
                                                          bool maximize)
  {
    return 0.0f;
  }


  template <class T> void Blackbox<T>::addTrainingSet (double *in,
                                                       double *out)
  {
    //! Do nothing.  Not used.
  }


  template <class T> void Blackbox<T>::clearTrainingSets ()
  {
  }

  template <class T> void Blackbox<T>::adjustNeighborhood (int method, int size)
  {
    //! Do nothing.  Not used.
  }


  template <class T> void Blackbox<T>::addTestingSet (double *in)
  {
    //! Do nothing.  Noth used.
  }


  template <class T> int Blackbox<T>::parseInputFile (const char *trainPath,
                                                      const char *testPath)
  {
    return 1;
  }


  template <class T> bool Blackbox<T>::readNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> bool Blackbox<T>::writeNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> double Blackbox<T>::getRMSError ()
  {
    return 0.0f;
  }


  template <class T> double Blackbox<T>::getTotalError ()
  {
    return 0.0f;
  }


  template <class T> int Blackbox<T>::getNumEpochs ()
  {
    return 0;
  }


  template <class T> int Blackbox<T>::getNumInputs ()
  {
    return inputFeatures_;
  }


  template <class T> int Blackbox<T>::getNumOutputs ()
  {
    return 1;
  }


  template <class T> void Blackbox<T>::setInputs (double *inputVec)
  {
    inputs_[0] = 1; // bias term
    for (int i = 1; i <= inputFeatures_; ++i)
    {
      inputs_[i] = inputVec[i];
    }
  }


  template <class T> void Blackbox<T>::getRawRange (vector<double>& minInVals,
                                                    vector<double>& maxInVals)
  {
    //! Do nothing.  Not used.
  }


  template <class T> void Blackbox<T>::scaleInput (vector<double> &rawVec,
                                                   double *scaledVec)
  {
    int i;

    for (i = 0; i <= inputFeatures_; ++i)
    {
      scaledVec[i] = rawVec.at (i);
    }
  }


  template <class T> void Blackbox<T>::useNoise (noise n)
  {
    noise_ = n;
  }

  //! A*(e^-( (((x-x_center)^2)/(2*sigma_x^2)) + (((y-y_center)^2)/(2*sigma_y^2)) )
  template <class T> double Blackbox<T>::Gaussian ()
  {
    int i;
    double A = 1.0f, temp = 0.0f;

    for (i = 1; i <= inputFeatures_; ++i)
    {
      temp += (((inputs_[i] - center_[i]) * (inputs_[i] - center_[i])) /
              (2 * sigma_[i] * sigma_[i]));
    }

    return (A * exp(-temp));
  }


  template <class T> double Blackbox<T>::DoubleGaussian ()
  {
    int i;
    double A1 = 1.0f, A2 = 0.5f, temp1 = 0.0f, temp2 = 0.0f, g1, g2;

    for (i = 1; i <= inputFeatures_; ++i)
    {
      temp1 += (((inputs_[i] - center_[i]) * (inputs_[i] - center_[i])) /
                (2 * sigma_[i] * sigma_[i]));
      temp2 += (((inputs_[i] - 6.0) * (inputs_[i] - 6.0)) /
                (2 * sigma_[i] * sigma_[i]));
    }

    g1 = A1 * exp(-temp1);
    g2 = A2 * exp(-temp2);

    return (g1 > g2) ? g1 : g2;
  } 


  template <class T> double Blackbox<T>::Mesa ()
  {
    int i;
    double out = 1.0f;

    for (i = 1; i <= inputFeatures_; ++i)
    {
      out *= (fabs(center_[i] - inputs_[i]) < 1) ? 1.0f : 0.0f;
    }
    return out;
  }


  template <class T> double Blackbox<T>::PureNoise ()
  {
    static bool first = true;

    //! Return a random number, regardless of inputs
    if (first)
    {
      first = false;
      return lRand((long)time(NULL));
    }
    return lRand();
  }


  template <class T> double Blackbox<T>::BumpyPlain ()
  {
    static long seed = (long)time(NULL);
    return ((0.01f) * gRand(&seed));
  }


  template <class T> double Blackbox<T>::SingleVariable ()
  {
    int i, j = 0;
    double out = 0.0f;

    //! Average sigmoid output over the domain, using every 3rd input
    for (i = 1; i <= inputFeatures_; ++i)
    {
      if (((i-1) % 3) == 0)
      {
        out += (2.0 / (1.0 + exp(inputs_[i])));
        ++j;
      }
    }

    out /= (j > 0) ? j : 1.0f;
    
    return out;
  }


  template <class T> void Blackbox<T>::computeOutputs ()
  {
    static long seed = (long)time(NULL);
    static bool first = true;
    double out = 0.0f, temp = 0.0f, A = 1.0f;

    //! Compute the output of our blackbox model
//    actual_ = DoubleGaussian ();
    actual_ = Gaussian ();
    output_ = actual_;

    //! Add random noise to the system
    if (noise_ == GAUSSIAN)
    {
      output_ += ((0.03f) * gRand(&seed));
    }
    else if (noise_ == NORMAL)
    {
      output_ += (0.25f * (first ? lRand((long)time(NULL)) : lRand()));
    }
    //! Do nothing if noise_ == NOISELESS

    first = false;

#ifdef RANDOMNOISE
    
    if (output_ < 0.0)
    {
    //  output_ = 0.0;
    }
    first = false;
#endif
  }


  template <class T> double Blackbox<T>::getOutput (int index)
  {
    if (index < 0)
    {
      return actual_;
    }
    return output_;
  }


  template <class T> void Blackbox<T>::updateEtas ()
  {
  }

} // Blackbox
