///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        AnalyticTangent.cpp
//  Revision:        1.0 - 11 March, 2009
//
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
//
///////////////////////////////////////////////////////////////////////////////

#include "Models.h"
#include "../Neuron/Numerical/Numerical.h"
#include "../../Libraries/Math/VectorMath.h"
#include <cmath>

#define DEBUGOUT
//#define NSY

using namespace std;
using namespace Math;

//! Explicit instantiations
template class LIBRARY_API NeuralNet::AnalyticTangent<Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::AnalyticTangent<Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::AnalyticTangent<Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::AnalyticTangent<Neuron::LogicNeuron>;

namespace NeuralNet
{
  template <class T> AnalyticTangent<T>::AnalyticTangent (int inFeat,
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
        center_[x] = 3.0f;//pi ();
        sigma_[x] = (double)x;
      }
    }
    catch (...)
    {
      //!  Exception
    }
  }


  template <class T> AnalyticTangent<T>::~AnalyticTangent ()
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


  template <class T> void AnalyticTangent<T>::initialize ()
  {
  }


  template <class T> double AnalyticTangent<T>::trainNetwork (int iter,
                                                          bool prune)
  {
    return 0.0f;
  }


  template <class T> void AnalyticTangent<T>::runTests (int k)
  {
  }


  template <class T> void AnalyticTangent<T>::runTestsBest (int top, int k)
  {
  }


  template <class T> double AnalyticTangent<T>::autoOptimize (vector<double>& minVals,
                                                              vector<double>& maxVals,
                                                              vector<double>& best,
                                                              int outWatch,
                                                              bool maximize)
  {
    return 0.0f;
  }


  template <class T> void AnalyticTangent<T>::addTrainingSet (double *in,
                                                              double *out)
  {
//    double output = 0.0f, temp = 0.0f, A = 1.0f;
    int p;
    double *tempVec1;
    
    tempVec1 = new double[inputFeatures_ + 1];
    for (p = 0; p <= inputFeatures_; ++p)
    {
      tempVec1[p] = in[p];
//      temp += (((in[p] - center_[p]) * (in[p] - center_[p])) /
//              (2 * sigma_[p] * sigma_[p]));
    }
//    output = A * exp(-temp);

    trainingInputs_.push_back (tempVec1);
    tempVec1 = NULL;

    tempVec1 = new double[outputFeatures_ + 1];
    for (p = 0; p <= outputFeatures_; ++p)
    {
//      tempVec1[p] = (p > 0) ? output : 0.0f;
      tempVec1[p] = out[p];
    }
    trainingOutputs_.push_back (tempVec1);
    tempVec1 = NULL;

    pTrainingPatterns_ = trainingInputs_.size ();
  }

  template <class T> void AnalyticTangent<T>::clearTrainingSets ()
  {
    vector<double*>::iterator iter;
    iter = trainingInputs_.begin();
    for (; iter != trainingInputs_.end(); ++iter)
    {
      delete [] *iter;
    }
    trainingInputs_.clear();

    iter = trainingOutputs_.begin();
    for (; iter != trainingOutputs_.end(); ++iter)
    {
      delete [] *iter;
    }
    trainingOutputs_.clear();

    pTrainingPatterns_ = 0;
  }


  template <class T> void AnalyticTangent<T>::adjustNeighborhood (int method,
                                                                  int size)
  {
    switch (method)
    {
    case NEAREST:
      break;
    case RECENT:
      break;
    case FULL:
      break;
    default:
      break;
    }
  }


  template <class T> void AnalyticTangent<T>::addTestingSet (double *in)
  {
  }


  template <class T> int AnalyticTangent<T>::parseInputFile (const char *trainPath,
                                                             const char *testPath)
  {
    return 1;
  }


  template <class T> bool AnalyticTangent<T>::readNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> bool AnalyticTangent<T>::writeNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> double AnalyticTangent<T>::getRMSError ()
  {
    return 0.0f;
  }


  template <class T> double AnalyticTangent<T>::getTotalError ()
  {
    return 0.0f;
  }


  template <class T> int AnalyticTangent<T>::getNumEpochs ()
  {
    return 0;
  }


  template <class T> int AnalyticTangent<T>::getNumInputs ()
  {
    return inputFeatures_;
  }


  template <class T> int AnalyticTangent<T>::getNumOutputs ()
  {
    return 1;
  }


  template <class T> void AnalyticTangent<T>::setInputs (double *inputVec)
  {
    inputs_[0] = 1; // bias term
    for (int i = 1; i <= inputFeatures_; ++i)
    {
      inputs_[i] = inputVec[i];
    }
  }


  template <class T> void AnalyticTangent<T>::getRawRange (vector<double>& minInVals,
                                                           vector<double>& maxInVals)
  {
  }


  template <class T> void AnalyticTangent<T>::scaleInput (vector<double> &rawVec,
                                                          double *scaledVec)
  {
    int i;

    for (i = 0; i <= inputFeatures_; ++i)
    {
      scaledVec[i] = rawVec.at (i);
    }
  }


  template <class T> void AnalyticTangent<T>::computeOutputs ()
  {
    double out = 0.0f, temp = 0.0f, temp2 = 0.0f, dx = 0.0f, A = 1.0f;
    vector<double> dists, ins, tests;
    vector<int> indexes;
    vector<double*>::iterator iter;
    int i;

    if (trainingInputs_.empty())
    {
      output_ = 0.0f;
      return;
    }


    for (i = 0; i <= inputFeatures_; ++i)
    {
      ins.push_back (inputs_[i]);
    }

    //tests.resize (inputFeatures_ + 1);
    iter = trainingInputs_.begin();
    //! Find the N training sets that are closest to the inquiry point
    for (; iter != trainingInputs_.end(); ++iter)
    {
      tests.clear();
      for (i = 0; i <= inputFeatures_; ++i)
      {
        tests.push_back((*iter)[i]);
      }
      temp = eucDist(ins, tests);
      dists.push_back (temp);
    }
    indexes.resize (dists.size());
    mergeSort (dists, indexes);




    //! Get Gaussian output at training point
/*
    for (i = 0; i <= outputFeatures_; ++i)
    {
      temp2 += trainingOutputs_.at(pTrainingPatterns_-1)[i];
    }
*/
/*
    for (i = 1; i <= inputFeatures_; ++i)
    {
      x = trainingInputs_.at(pTrainingPatterns_-1)[i];
      temp += (((x - center_[i]) * (x - center_[i])) /
              (2 * sigma_[i] * sigma_[i]));
    }
    temp2 = A * exp(-temp); //! (Gaussian ouput at train point)
*/

    //! out = train_out + dX(in_X - train_X) + dY(in_Y - train_Y) + ...
    //! dX = (-train_X - 1)/(sigma_X ^ 2) * (Gaussian output at train point)
    for (i = 0; i <= outputFeatures_; ++i)
    {
      out += trainingOutputs_.at(indexes.at(0))[i];
    }
    temp2 = out;
    for (i = 1; i <= inputFeatures_; ++i)
    {
      dx = temp2 * (center_[i]-trainingInputs_.at(indexes.at(0))[i]) / (sigma_[i] * sigma_[i]);
      out += dx * (inputs_[i] - trainingInputs_.at(indexes.at(0))[i]);
    }

    output_ = out;
  }


  template <class T> double AnalyticTangent<T>::getOutput (int index)
  {
    return output_;
  }

} // AnalyticTangent
