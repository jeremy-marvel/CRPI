///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        BioInspired.cpp
//  Revision:        1.0 - 21 July, 2009
//
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
//  Black box simulator/placeholder that computes an N-dimensional gaussian for
//  testing purposes.
//
///////////////////////////////////////////////////////////////////////////////

#include "BioInspired.h"
#include "../Neuron/Numerical/Numerical.h"
#include "../../Libraries/Math/NumericalMath.h"
#include <cmath>
#include <time.h>

#define DEBUGOUT
#define RANDOMNOISE

using namespace Math;
using namespace std;

//! Explicit instantiations
template class LIBRARY_API NeuralNet::BioInspired<Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::BioInspired<Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::BioInspired<Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::BioInspired<Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::BioInspired<Neuron::LogicNeuron>;

namespace NeuralNet
{
  template <class T> BioInspired<T>::BioInspired (int inFeat,
                                                  int interFeat,
                                                  int outFeat,
                                                  double initWeight,
                                                  double initEta) :
    inputFeatures_(inFeat),
    pTrainingPatterns_(0),
    pTestingPatterns_(0)
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
        center_[x] = 3.141592654;
        sigma_[x] = (double)x;
      }
    }
    catch (...)
    {
      exception ("BioInspired::BioInspired", "Error setting initial parameters");
    }
  }


  template <class T> BioInspired<T>::~BioInspired ()
  {
    try
    {
      delete [] inputs_;
      delete [] center_;
      delete [] sigma_;
    }
    catch (...)
    {
      exception ("BioInspired::~BioInspired", "Error reclaiming used memory");
    }
  }


  template <class T> void BioInspired<T>::initialize ()
  {
  }


  template <class T> double BioInspired<T>::trainNetwork (int iter,
                                                          bool prune)
  {
    return 0.0f;
  }


  template <class T> void BioInspired<T>::runTests (int k)
  {
  }


  template <class T> void BioInspired<T>::runTestsBest (int top, int k)
  {
  }


  template <class T> double BioInspired<T>::autoOptimize (vector<double>& minVals,
                                                          vector<double>& maxVals,
                                                          vector<double>& best,
                                                          int outWatch,
                                                          bool maximize)
  {
    return 0.0f;
  }


  template <class T> void BioInspired<T>::addTrainingSet (double *in,
                                                          double *out)
  {
    //! Do nothing.  Not used.
  }


  template <class T> void BioInspired<T>::clearTrainingSets ()
  {
  }

  template <class T> void BioInspired<T>::adjustNeighborhood (int method, int size)
  {
    //! Do nothing.  Not used.
  }


  template <class T> void BioInspired<T>::addTestingSet (double *in)
  {
    //! Do nothing.  Noth used.
  }


  template <class T> int BioInspired<T>::parseInputFile (const char *trainPath,
                                                         const char *testPath)
  {
    return 1;
  }


  template <class T> bool BioInspired<T>::readNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> bool BioInspired<T>::writeNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> double BioInspired<T>::getRMSError ()
  {
    return 0.0f;
  }


  template <class T> double BioInspired<T>::getTotalError ()
  {
    return 0.0f;
  }


  template <class T> int BioInspired<T>::getNumEpochs ()
  {
    return 0;
  }


  template <class T> int BioInspired<T>::getNumInputs ()
  {
    return inputFeatures_;
  }


  template <class T> int BioInspired<T>::getNumOutputs ()
  {
    return 1;
  }


  template <class T> void BioInspired<T>::setInputs (double *inputVec)
  {
    inputs_[0] = 1; // bias term
    for (int i = 1; i <= inputFeatures_; ++i)
    {
      inputs_[i] = inputVec[i];
    }
  }


  template <class T> void BioInspired<T>::getRawRange (vector<double>& minInVals,
                                                       vector<double>& maxInVals)
  {
    //! Do nothing.  Not used.
  }


  template <class T> void BioInspired<T>::scaleInput (vector<double> &rawVec,
                                                      double *scaledVec)
  {
    int i;

    for (i = 0; i <= inputFeatures_; ++i)
    {
      scaledVec[i] = rawVec.at (i);
    }
  }


  template <class T> void BioInspired<T>::computeOutputs ()
  {
    static long seed = (long)time(NULL);
    static bool first = true;
    double out = 0.0f, temp = 0.0f, A = 1.0f;
    double t1, t2, t3;
    int i;

    //! A*(e^-( (((x-x_center)^2)/(2*sigma_x^2)) + (((y-y_center)^2)/(2*sigma_y^2)) )
    for (i = 1; i <= inputFeatures_; ++i)
    {
      t1 = inputs_[i];
      t2 = center_[i];
      t3 = sigma_[i];
      temp += (((inputs_[i] - center_[i]) * (inputs_[i] - center_[i])) /
              (2 * sigma_[i] * sigma_[i]));
    }

    actual_ = A * exp(-temp);
    output_ = actual_;
#ifdef RANDOMNOISE
//    output_ += ((0.25f) * gRand(&seed));
    output_ += (0.25f * (first ? lRand((long)time(NULL)) : lRand()));
    if (output_ < 0.0)
    {
    //  output_ = 0.0;
    }
    first = false;
#endif
  }


  template <class T> double BioInspired<T>::getOutput (int index)
  {
    if (index < 0)
    {
      return actual_;
    }
    return output_;
  }


  template <class T> void BioInspired<T>::updateEtas ()
  {
  }

/*
  template <class T> void BioInspired<T>::snapshot (NNDisplayAdapter &display)
  {
  }
*/

  template <class T> void BioInspired<T>::exception (char *where, char *what)
  {
    static char message[1024];
    sprintf (message, "Exception in \" %s \" : %s", where, what);
    printf(message);
  }
} // BioInspired
