///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        Naysayer.cpp
//  Revision:        1.0 - 30 December, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Sanity check "neural net" that always generates outputs of 0.
//
///////////////////////////////////////////////////////////////////////////////

#include "Models.h"
#include "../../types.h"
#include "../Neuron/Numerical/Numerical.h"
#include <cmath>

#define DEBUGOUT
//#define NSY

#ifdef NSY
#include <iostream>
#endif

using namespace std;

//! Explicit instantiations
template class LIBRARY_API NeuralNet::NaySayer<Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::NaySayer<Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::NaySayer<Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::NaySayer<Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::NaySayer<Neuron::LogicNeuron>;

namespace NeuralNet
{
  template <class T> NaySayer<T>::NaySayer (int inFeat,
                                            int interFeat,
                                            int outFeat,
                                            double initWeight,
                                            double initEta) :
    inputFeatures_(inFeat),
    outputFeatures_(outFeat),
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
        sigma_[x] = 1.0;//(double)x;
      }
    }
    catch (...)
    {
      //! Exception
    }
  }


  template <class T> NaySayer<T>::~NaySayer ()
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


  template <class T> void NaySayer<T>::initialize ()
  {
  }


  template <class T> double NaySayer<T>::trainNetwork (int iter,
                                                       bool prune)
  {
    return 0.0f;
  }


  template <class T> void NaySayer<T>::runTests (int k)
  {
  }


  template <class T> void NaySayer<T>::runTestsBest (int top, int k)
  {
  }


  template <class T> double NaySayer<T>::autoOptimize (vector<double>& minVals,
                                                       vector<double>& maxVals,
                                                       vector<double>& best,
                                                       int outWatch,
                                                       bool maximize)
  {
    return 0.0f;
  }


  template <class T> void NaySayer<T>::addTrainingSet (double *in,
                                                       double *out)
  {
  }


  template <class T> void NaySayer<T>::clearTrainingSets ()
  {
  }


  template <class T> void NaySayer<T>::adjustNeighborhood (int method,
                                                           int size)
  {
  }


  template <class T> void NaySayer<T>::addTestingSet (double *in)
  {
  }


  template <class T> int NaySayer<T>::parseInputFile (const char *trainPath,
                                                      const char *testPath)
  {
    return 1;
  }


  template <class T> bool NaySayer<T>::readNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> bool NaySayer<T>::writeNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> double NaySayer<T>::getRMSError ()
  {
    return 0.0f;
  }


  template <class T> double NaySayer<T>::getTotalError ()
  {
    return 0.0f;
  }


  template <class T> int NaySayer<T>::getNumEpochs ()
  {
    return 0;
  }


  template <class T> int NaySayer<T>::getNumInputs ()
  {
    return inputFeatures_;
  }


  template <class T> int NaySayer<T>::getNumOutputs ()
  {
    return 1;
  }


  template <class T> void NaySayer<T>::setInputs (double *inputVec)
  {
  }


  template <class T> void NaySayer<T>::getRawRange (vector<double>& minInVals,
                                                    vector<double>& maxInVals)
  {
  }


  template <class T> void NaySayer<T>::scaleInput (vector<double> &rawVec,
                                                   double *scaledVec)
  {
  }


  template <class T> void NaySayer<T>::computeOutputs ()
  {
  }


  template <class T> double NaySayer<T>::getOutput (int index)
  {
    return -1.0f;
  }

} // NaySayer
