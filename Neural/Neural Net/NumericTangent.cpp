///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        NumericTangent.cpp
//  Revision:        1.0 - 27 January, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Numerical tangent neuron definition.
//
///////////////////////////////////////////////////////////////////////////////

#include "Models.h"
#include "../Neuron/Numerical/Numerical.h"
#include "../../Libraries/Math/VectorMath.h"
#include "../../Libraries/Math/MatrixMath.h"
#include <cmath>

#define DEBUGOUT
//#define NSY

#ifdef NSY
#include <iostream>
#endif

using namespace std;
using namespace Math;

//! Explicit instantiations
template class LIBRARY_API NeuralNet::NumericTangent<Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::NumericTangent<Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::NumericTangent<Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::NumericTangent<Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::NumericTangent<Neuron::LogicNeuron>;

namespace NeuralNet
{
  template <class T> NumericTangent<T>::NumericTangent (int inFeat,
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


  template <class T> NumericTangent<T>::~NumericTangent ()
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


  template <class T> void NumericTangent<T>::initialize ()
  {
  }


  template <class T> double NumericTangent<T>::trainNetwork (int iter,
                                                             bool prune)
  {
    return 0.0f;
  }


  template <class T> void NumericTangent<T>::runTests (int k)
  {
  }


  template <class T> void NumericTangent<T>::runTestsBest (int top, int k)
  {
  }


  template <class T> double NumericTangent<T>::autoOptimize (vector<double>& minVals,
                                                             vector<double>& maxVals,
                                                             vector<double>& best,
                                                             int outWatch,
                                                             bool maximize)
  {
    return 0.0f;
  }


  template <class T> void NumericTangent<T>::addTrainingSet (double *in,
                                                             double *out)
  {
    int p;
    double *tempVec1;
    
    tempVec1 = new double[inputFeatures_ + 1];
    tempVec1[0] = 1.0f;
    for (p = 1; p <= inputFeatures_; ++p)
    {
      tempVec1[p] = in[p];
    }

    trainingInputs_.push_back (tempVec1);
    tempVec1 = NULL;

    tempVec1 = new double[outputFeatures_ + 1];
    for (p = 0; p < (outputFeatures_ + 1); ++p)
    {
      tempVec1[p] = out[p];
    }
    trainingOutputs_.push_back (tempVec1);
    tempVec1 = NULL;

    pTrainingPatterns_ = trainingInputs_.size ();
  }


  template <class T> void NumericTangent<T>::clearTrainingSets ()
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


  template <class T> void NumericTangent<T>::adjustNeighborhood (int method,
                                                                 int size)
  {
    switch (method)
    {
    case NEAREST:
      break;
    case RECENT:
      break;
    default:
      break;
    }
  }


  template <class T> void NumericTangent<T>::addTestingSet (double *in)
  {
  }


  template <class T> int NumericTangent<T>::parseInputFile (const char *trainPath,
                                                            const char *testPath)
  {
    return 1;
  }


  template <class T> bool NumericTangent<T>::readNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> bool NumericTangent<T>::writeNetworkFile (const char *netPath)
  {
    return true;
  }


  template <class T> double NumericTangent<T>::getRMSError ()
  {
    return 0.0f;
  }


  template <class T> double NumericTangent<T>::getTotalError ()
  {
    return 0.0f;
  }


  template <class T> int NumericTangent<T>::getNumEpochs ()
  {
    return 0;
  }


  template <class T> int NumericTangent<T>::getNumInputs ()
  {
    return inputFeatures_;
  }


  template <class T> int NumericTangent<T>::getNumOutputs ()
  {
    return 1;
  }


  template <class T> void NumericTangent<T>::setInputs (double *inputVec)
  {
    inputs_[0] = 1.0; // bias term
    for (int i = 1; i <= inputFeatures_; ++i)
    {
      inputs_[i] = inputVec[i];
    }
  }


  template <class T> void NumericTangent<T>::getRawRange (vector<double>& minInVals,
                                                          vector<double>& maxInVals)
  {
  }


  template <class T> void NumericTangent<T>::scaleInput (vector<double> &rawVec,
                                                         double *scaledVec)
  {
    int i;

    for (i = 0; i <= inputFeatures_; ++i)
    {
      scaledVec[i] = rawVec.at (i);
    }
  }


  template <class T> void NumericTangent<T>::computeOutputs ()
  {
    double out = 0.0f, temp = 0.0f, A = 1.0f, old, d;
    int i, j, end;
    unsigned int x;

#ifdef NSY
    int y;
#endif

    vector<double*>::iterator iter;
    vector<double> dists, ins, tests;
    vector<double> line1, line2;
    static matrix mA, mA_T, my, mA_TA, mA_Ty, mA_TAinv, mA_TAinvA_T, mb;
    vector<int> indexes, indx;
    
    if ((int)trainingInputs_.size() < (inputFeatures_+1))
    {
      //! To perform the least squares linear fit, we need at least (inputFeatures_ + 1) 
      //! sample points
      output_ = 0.0f;
      return;
    }

    for (i = 0; i < inputFeatures_; ++i)
    {
      ins.push_back (inputs_[i+1]);
    }

    //tests.resize (inputFeatures_ + 1);
    iter = trainingInputs_.begin();
    //! Find the N training sets that are closest to the inquiry point
    for (; iter != trainingInputs_.end(); ++iter)
    {
      tests.clear();
      for (i = 0; i < inputFeatures_; ++i)
      {
        tests.push_back((*iter)[i+1]);
      }
      temp = eucDist(ins, tests);
      dists.push_back (temp);
    }
    indexes.resize (dists.size());
    mergeSort (dists, indexes);

    indx.clear();
    x = 0;
    i = 0;
    old = -1.0f;
    while (x < dists.size())
    {
      d = dists.at(x);
      while (fabs(d - old) < 0.0000000001)
      {
        ++x;
        if (x >= dists.size())
        {
          break;
        }
        d = dists.at(x);
      }
      if (x >= dists.size())
      {
        break;
      }

      indx.push_back (x);
      ++i;
      old = dists.at(x);
    }

    line1.resize(inputFeatures_);
    line2.resize(1);
    end = ((inputFeatures_ + 1) < (int)indx.size()) ? (inputFeatures_ + 1) : (int)indx.size();

    if (end < inputFeatures_)
    {
      output_ = 0.0f;
      return;
    }

    //! Compute the hyperplane that passes through those N closest points and compute
    //! the projected output of the inquiry point

    //! Create A (matrix1) and y (matrix2)
    mA.resize(end, inputFeatures_+1);

    my.resize(end, 1);
    x = 0;
    old = -1.0f;
    for (i = 0; i < end; ++i)
    {
      for (j = 0; j <= inputFeatures_; ++j)
      {
        mA.at(i, j) = trainingInputs_.at(indexes.at(indx.at(i)))[j];
      }

      my.at(i, 0) = 0.0f;
      for (j = 1; j <= outputFeatures_; ++j)
      {
        old = trainingOutputs_.at(indexes.at(indx.at(i)))[j];
        my.at(i, 0) += trainingOutputs_.at(indexes.at(indx.at(i)))[j];
      }
    }
#ifdef NSY
    cout << "mA" << endl;
    for (y = 0; y < mA.rows; ++y)
    {
      for (x = 0; x < mA.cols; ++x)
      {
        cout << mA.at(y, x) << " ";
      }
      cout << endl;
    }
#endif

    //! Compute the transpose of A
    mA_T = mA.trans();
    if (!mA_T.valid)
    {
      //! Exception
    }

#ifdef NSY
    cout << "mA_T" << endl;
    for (y = 0; y < mA_T.rows; ++y)
    {
      for (x = 0; x < mA_T.cols; ++x)
      {
        cout << mA_T.at(y, x) << " ";
      }
      cout << endl;
    }
#endif

    //! Compute A_T * A
    mA_TA = mA_T * mA;
    if (!mA_TA.valid)
    {
      //! Exception
    }

#ifdef NSY
    cout << "mA_TA" << endl;
    for (y = 0; y < mA_TA.rows; ++y)
    {
      for (x = 0; x < mA_TA.cols; ++x)
      {
        cout << mA_TA.at(y, x) << " ";
      }
      cout << endl;
    }
#endif

    //! Compute A_T * y
    mA_Ty = mA_T * my;
    if (!mA_Ty.valid)
    {
      //! Exception
    }

#ifdef NSY
    cout << "mA_Ty" << endl;
    for (y = 0; y < mA_Ty.rows; ++y)
    {
      for (x = 0; x < mA_Ty.cols; ++x)
      {
        cout << mA_Ty.at(y, x) << " ";
      }
      cout << endl;
    }
#endif

    //! normal equations = (A_T * A) * B = (A_T * y)
    //! Compute (A_T * A)^-1
    mA_TAinv = mA_TA.inv();
    if (!mA_TAinv.valid)
    {
      //! Exception
    }

#ifdef NSY
    cout << "mA_TAinv" << endl;
    for (y = 0; y < mA_TAinv.rows; ++y)
    {
      for (x = 0; x < mA_TAinv.cols; ++x)
      {
        cout << mA_TAinv.at(y, x) << " ";
      }
      cout << endl;
    }
#endif

    //! Compute (A_T * A)^-1 * (A_T * y) <--- solution vector b
    mb = mA_TAinv * mA_Ty;
    if (!mb.valid)
    {
      //! Exception
    }

#ifdef NSY
    cout << "mb" << endl;
    for (y = 0; y < mb.rows; ++y)
    {
      for (x = 0; x < mb.cols; ++x)
      {
        cout << mb.at(y, x) << " ";
      }
      cout << endl;
    }
#endif

    //! Compute output
    out = 0.0f;//mb.at (0, 0);
    for (i = 0; i < inputFeatures_; ++i)
    {
      out += mb.at(i, 0) * inputs_[i];
    }
    output_ = out;
  }


  template <class T> double NumericTangent<T>::getOutput (int index)
  {
    return output_;
  }

} // NumericTangent
