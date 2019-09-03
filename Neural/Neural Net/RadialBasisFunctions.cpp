///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        RadialBasisFunctions.cpp
//  Revision:        1.0 - 7 February, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Implementation of Radial Basis Functions using integrate and fire neurons.
//  Currently, can only use numerical neuron types
//
///////////////////////////////////////////////////////////////////////////////

#include "RadialBasisFunctions.h"
#include "../Neuron/Numerical/Numerical.h"
#include "../../Libraries/Math/VectorMath.h"
#include "../../Clustering/kMeans/kMeansCluster.h"
#include <cmath>
#include <time.h>
#include <fstream>

#define DEBUGOUT

using namespace std;
using namespace Math;


//! Explicit instantiations
//! <T1, TanhNeuron>
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::TanhNeuron,
                                                          Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::SigmoidNeuron,
                                                          Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LinearNeuron,
                                                          Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::RandBinNeuron,
                                                          Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LogicNeuron,
                                                          Neuron::TanhNeuron>;
//! <T1, SigmoidNeuron>
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::TanhNeuron,
                                                    Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::SigmoidNeuron,
                                                    Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LinearNeuron,
                                                    Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::RandBinNeuron,
                                                    Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LogicNeuron,
                                                    Neuron::SigmoidNeuron>;
//! <T1, LinearNeuron>
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::TanhNeuron,
                                                    Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::SigmoidNeuron,
                                                    Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LinearNeuron,
                                                    Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::RandBinNeuron,
                                                    Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LogicNeuron,
                                                    Neuron::LinearNeuron>;
//! <T1, RandBinNeuron>
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::TanhNeuron,
                                                    Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::SigmoidNeuron,
                                                    Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LinearNeuron,
                                                    Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::RandBinNeuron,
                                                    Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LogicNeuron,
                                                    Neuron::RandBinNeuron>;
//! <T1, LogicNeuron>
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::TanhNeuron,
                                                    Neuron::LogicNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::SigmoidNeuron,
                                                    Neuron::LogicNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LinearNeuron,
                                                    Neuron::LogicNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::RandBinNeuron,
                                                    Neuron::LogicNeuron>;
template class LIBRARY_API NeuralNet::RadialBasisFunction<Neuron::LogicNeuron,
                                                    Neuron::LogicNeuron>;

namespace NeuralNet
{
  template <class T1, class T2>
    RadialBasisFunction<T1, T2>::RadialBasisFunction (int inFeat,
                                                      int inter1Feat,
                                                      int inter2Feat,
                                                      int outFeat,
                                                      double initWeight,
                                                      double initEta) :
    inputFeatures_(inFeat),
    interJFeatures_(inter1Feat),
    interKFeatures_(inter2Feat),
    outputFeatures_(outFeat),
    initWeight_(initWeight),
    initEta_(initEta),
    pTrainingPatterns_(0),
    pTestingPatterns_(0),
    nIters_(0),
    randRange_(2.0f),
    scalingOverride_(false),
    useAlpha_(false),
    useBeta_(false),
    useGamma_(false),
    useRawInputs_(false)
  {
    unsigned int j, k, l;

    //! Allocate memory for value passing & temporary storage
    inputs_ = new double [inputFeatures_ + 1];
    inputMaxVals_ = new double [inputFeatures_ + 1];
    inputMinVals_ = new double [inputFeatures_ + 1];

    hiddenJInputs_ = new double [interJFeatures_ + 1];
    hiddenJOutputs_ = new double [interJFeatures_ + 1];
    hiddenKInputs_ = new double [interKFeatures_ + 1];
    hiddenKOutputs_ = new double [interKFeatures_ + 1];

    outputLInputs_ = new double [outputFeatures_ + 1];
    outputs_ = new double [outputFeatures_ + 1];

    outputMaxVals_ = new double [outputFeatures_ + 1];
    outputMinVals_ = new double [outputFeatures_ + 1];

    //! Allocate memory for the neurons
    interJNeurons_ = new T1 [interJFeatures_ + 1];
    alphaLife_ = new int[interJFeatures_ + 1];
    alphaFire_ = new int[interJFeatures_ + 1];
    interKNeurons_ = new T2 [interKFeatures_ + 1];
    outputNeurons_ = new Neuron::LinearNeuron [outputFeatures_ + 1];

    //! Allocate the memory for synaptic connections from layer 0 to layer 1
    synapses0to1_ = new double * [interJFeatures_ + 1];
    for (j = 0; j <= interJFeatures_; ++j)
    {
      alphaLife_[j] = 0;
      alphaFire_[j] = 0;
      synapses0to1_[j] = new double [inputFeatures_ + 1];
    }

    //! Allocate the memory for synaptic connections from layer 1 to layer 2
    synapses1to2_ = new double * [interKFeatures_ + 1];
    for (k = 0; k <= interKFeatures_; ++k)
    {
      synapses1to2_[k] = new double [interJFeatures_ + 1];
    }

    //! Allocate the memory for synaptic connections from layer 2 to layer 3
    synapses2to3_ = new double * [outputFeatures_ + 1];
    dwBetaGamma_ = new double * [outputFeatures_ + 1];
    for (l = 0; l <= outputFeatures_; ++l)
    {
      dwBetaGamma_[l] = new double [interKFeatures_ + 1];
      synapses2to3_[l] = new double [interKFeatures_+1];
    }
  }


  template <class T1, class T2> RadialBasisFunction<T1, T2>::~RadialBasisFunction ()
  {
    vector<double*>::iterator iter1, iter2;
    unsigned int j, k, l;

    //! Garbage collection : Training sets
    if (pTrainingPatterns_ > 0)
    {
      iter1 = trainingInputs_.begin();
      iter2 = trainingOutputs_.begin();
      for (; iter1 != trainingInputs_.end(); ++iter1, ++iter2)
      {
        delete *iter1;
        *iter1 = NULL;
        delete *iter2;
        *iter2 = NULL;
      }

      iter1 = rawTrainingInputs_.begin();
      iter2 = rawTrainingOutputs_.begin();
      for (; iter1 != rawTrainingInputs_.end(); ++iter1, ++iter2)
      {
        delete *iter1;
        *iter1 = NULL;
        delete *iter2;
        *iter2 = NULL;
      }

      iter1 = rawTrainingInputsBackup_.begin();
      iter2 = rawTrainingOutputsBackup_.begin();
      for (; iter1 != rawTrainingInputsBackup_.end(); ++iter1, ++iter2)
      {
        delete *iter1;
        *iter1 = NULL;
        delete *iter2;
        *iter2 = NULL;
      }

      trainingInputs_.clear();
      rawTrainingInputs_.clear();
      trainingOutputs_.clear();
      rawTrainingOutputs_.clear();
      rawTrainingInputsBackup_.clear();
      rawTrainingOutputsBackup_.clear();
      pTrainingPatterns_ = 0;
    }

    //! Garbage collection : Testing sets
    if (pTestingPatterns_ > 0)
    {
      iter1 = testingInputs_.begin();
      iter2 = rawTestingInputs_.begin();
      for (; iter1 != testingInputs_.end(); ++iter1, ++iter2)
      {
        delete *iter1;
        delete *iter2;
      }
      testingInputs_.clear();
      rawTestingInputs_.clear();
      pTestingPatterns_ = 0;
    }

    //! Synapse garbage collection
    {
      for (j = 1; j <= interJFeatures_; ++j)
      {
        delete [] synapses0to1_[j];
      }
      delete [] synapses0to1_;

      for (k = 0; k <= interKFeatures_; ++k)
      {
        delete [] synapses1to2_[k];
      }
      delete [] synapses1to2_;

      for (l = 1; l <= outputFeatures_; ++l)
      {
        delete [] synapses2to3_[l];
      }
      delete [] synapses2to3_;
    }

    delete [] hiddenJInputs_;
    delete [] hiddenJOutputs_;
    delete [] hiddenKInputs_;
    delete [] hiddenKOutputs_;
    delete [] outputLInputs_;
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::initialize ()
  {
    //! Initialize the synapses for the network by randomly assigning weights
    initializeSynapses ();
  }


  template <class T1, class T2>
    double RadialBasisFunction<T1, T2>::trainNetwork (int iter, bool prune)
  {
    adjustWeights ();
    return RMSError_;
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::runTests (int k)
  {
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::runTestsBest (int top, int k)
  {
  }


  template <class T1, class T2>
    double RadialBasisFunction<T1, T2>::autoOptimize (vector<double>& minVals,
                                                      vector<double>& maxVals,
                                                      vector<double>& best,
                                                      int outWatch,
                                                      bool maximize)
  {
    double out = 0.0f;
    return out;
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::optiHelper (vector<double>& minVals,
                                                  vector<double>& maxVals,
                                                  vector<double>& current,
                                                  vector<double>& best,
                                                  int element,
                                                  int outWatch,
                                                  double& bestout,
                                                  bool maximize)
  {
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::addTrainingSet (double *in, double *out)
  {
    unsigned int p, j;
    double *tempVec1, *tempVec2, *tempVec3, dist;
    vector<double> distTemp1, distTemp2;

    for (p = 0; p < (inputFeatures_ + 1); ++p)
    {
      distTemp1.push_back (in[p]);
    }

    distTemp2.resize(inputFeatures_ + 1);

    for (j = 0; j < rawTrainingInputsBackup_.size(); ++j)
    {
      for (p = 0; p < (inputFeatures_ + 1); ++p)
      {
        dist = rawTrainingInputsBackup_.at(j)[p];
        distTemp2.at(p) = dist;
      }
      dist = eucDist (distTemp1, distTemp2);
      if (dist < 0.0001)
      {
        distTemp1.clear();
        distTemp2.clear();
        return;
      }
    }
    
    distTemp1.clear();
    distTemp2.clear();

    tempVec1 = new double[inputFeatures_ + 1];
    tempVec2 = new double[inputFeatures_ + 1];
    tempVec3 = new double[inputFeatures_ + 1];
    for (p = 0; p < (inputFeatures_ + 1); ++p)
    {
      tempVec1[p] = tempVec2[p] = tempVec3[p] = in[p];
    }

    //! TODO: Verify we haven't already added this point or one too close to it 

    rawTrainingInputs_.push_back (tempVec1);
    trainingInputs_.push_back (tempVec2);
    rawTrainingInputsBackup_.push_back (tempVec3);

    inputActive_.push_back (true);
    EVals_.push_back(0.0);
    tempVec1 = tempVec2 = tempVec3 = NULL;

    tempVec1 = new double[outputFeatures_ + 1];
    tempVec2 = new double[outputFeatures_ + 1];
    tempVec3 = new double[outputFeatures_ + 1];
    for (p = 0; p < (outputFeatures_ + 1); ++p)
    {
      tempVec1[p] = tempVec2[p] = tempVec3[p] = out[p];
    }
    rawTrainingOutputs_.push_back (tempVec1);
    trainingOutputs_.push_back (tempVec2);
    rawTrainingOutputsBackup_.push_back (tempVec3);
    tempVec1 = tempVec2 = tempVec3 = NULL;

    pTrainingPatterns_ = trainingInputs_.size ();
    pRawTrainingPatternsBackup_ = rawTrainingInputsBackup_.size();

    //! Find the minimum and maximum raw input values
    findInputMinMaxVals ();

    //! Find the minimum and maximum raw output values
    findOutputMinMaxVals ();

    //! Scale values
    scaleAllInputs ();
    scaleAllOutputs ();
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::clearTrainingSets ()
  {
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::adjustNeighborhood (int method, unsigned int size)
  {
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::addTestingSet (double *in)
  {
  }


  template <class T1, class T2>
    int RadialBasisFunction<T1, T2>::parseInputFile (const char *trainPath,
                                                     const char *testPath)
  {
    unsigned int nvals = 0, tvals=0;
    char message[256];
    double inval;
    double *tempVec1 = NULL, *tempVec2 = NULL;
    ifstream trainingFile, testingFile;
    unsigned x, y, i;
    int okay = 0;
    bool flag;
    vector<double> vals;

    if (trainPath != NULL)
    {
      flag = true;
      trainingFile.open(trainPath);
      if (!trainingFile)
      { 
        okay -= 1;
        flag = false;
      }

      if (flag)
      {
        while (trainingFile >> inval)
        {
          vals.push_back (inval);
        }
        nvals = vals.size ();

        trainingFile.close ();

        if ((nvals % (inputFeatures_ + outputFeatures_)) != 0)
        {
          okay -= 1;
        }
        else
        {
          nvals /= (inputFeatures_ + outputFeatures_);
          tempVec1 = new double[inputFeatures_+1];
          tempVec2 = new double[outputFeatures_+1];

          i = 0;
          for (y = 0; y < nvals; ++y)
          {
            tempVec1[0] = 1.0f;
            for (x = 0; x < inputFeatures_; ++x, ++i)
            {
              tempVec1[x+1] = vals[i];
            }

            tempVec2[0] = 1.0f;
            for (x = 0; x < outputFeatures_; ++x, ++i)
            {
              tempVec2[x+1] = vals[i];
            }
            addTrainingSet (tempVec1, tempVec2);
          }
        } // if ((nvals % (inputFeatures_ + outputFeatures_)) != 0) ... else
      } // if (flag)
      okay += 1;
    } // if (trainPath != NULL)
    vals.clear();

    if (testPath != NULL)
    {
      flag = true;
      testingFile.open (testPath);
      if (!testingFile)
      {
        okay -= 2;
        flag = false;
      }

      if (flag)
      {
        while (testingFile >> inval)
        {
          vals.push_back (inval);
        }
        nvals = vals.size ();

        testingFile.close ();

        if ((nvals % inputFeatures_) != 0)
        {
          okay -= 2;
        }
        else
        {
          nvals /= inputFeatures_;

          i = 0;
          for (y = 0; y < nvals; ++y)
          {
            tempVec1[0] = 1.0f;
            for (x = 0; x < inputFeatures_; ++x, ++i)
            {
              tempVec1[x+1] = vals[i];
            }
            addTestingSet (tempVec1);
          }
        } // if ((nvals % (inputFeatures_ + outputFeatures_)) != 0) ... else
      } // if (flag)
      okay += 2;
    } // if (testPath != NULL)
    vals.clear();

    if (tempVec1 != NULL)
    {
      delete [] tempVec1;
    }
    if (tempVec2 != NULL)
    {
      delete [] tempVec2;
    }
    return okay;
  }


/*
  template <class T> bool FeedForward<T>::readNetworkFile (const char *netPath)
  {
    char message[256];
    ifstream netFile;
    int in, hidden, out, x, y;

    if (netPath == NULL)
    {
      exception ("FeedForward::readNetworkFile", "Null path");
      return false;
    }

    netFile.open (netPath);
    if (!netFile)
    {
      sprintf (message, "Cannot open network description file: %s", netPath);
      exception ("FeedForward::readNetworkFile", message);
      return false;
    }

    netFile >> in;
    netFile >> hidden;
    netFile >> out;

    if (in != inputFeatures_ || hidden != interFeatures_ || out != outputFeatures_)
    {
      exception ("FeedForward::readNetworkFile", "Illegal number of parameters in file");
      return false;
    }

    for (x = 0; x <= inputFeatures_; ++x)
    {
      for (y = 0; y <= interFeatures_; ++y)
      {
        //! Read input-inter weights
        netFile >> synapses0to1_[y][x];
      }
    }

    for (x = 0; x <= interFeatures_; ++x)
    {
      for (y = 0; y <= outputFeatures_; ++y)
      {
        //! Read inter-output weights
        netFile >> synapses1to2_[y][x];
      }
    }

    return true;
  }
*/



  template <class T1, class T2>
    bool RadialBasisFunction<T1, T2>::readNetworkFile (const char *netPath)
  {
    //! TODO
    return true;
  }


/*
  template <class T> bool FeedForward<T>::writeNetworkFile (const char *netPath)
  {
    char message[256];
    ofstream netFile;
    int x, y;

    if (netPath == NULL)
    {
      exception ("FeedForward::writeNetworkFile", "Null path");
      return false;
    }

    netFile.open (netPath);
    if (!netFile)
    {
      sprintf (message, "Cannot open network description file: %s", netPath);
      exception ("FeedForward::writeNetworkFile", message);
      return false;
    }

    netFile << inputFeatures_ << " " << interFeatures_ << " " << outputFeatures_ << endl;

    for (x = 0; x <= inputFeatures_; ++x)
    {
      for (y = 0; y <= interFeatures_; ++y)
      {
        //! Read input-inter weights
        netFile << synapses0to1_[y][x] << " ";
      }
      netFile << endl;
    }

    for (x = 0; x <= interFeatures_; ++x)
    {
      for (y = 0; y <= outputFeatures_; ++y)
      {
        //! Read inter-output weights
        netFile << synapses1to2_[y][x] << " ";
      }
      netFile << endl;
    }

    return true;
  }
*/

  template <class T1, class T2>
    bool RadialBasisFunction<T1, T2>::writeNetworkFile (const char *netPath)
  {
    //! TODO
    return true;
  }


  template <class T1, class T2> double RadialBasisFunction<T1, T2>::getRMSError ()
  {
    return RMSError_;
  }


  template <class T1, class T2> double RadialBasisFunction<T1, T2>::getTotalError ()
  {
    return ETotal_;
  }


  template <class T1, class T2> int RadialBasisFunction<T1, T2>::getNumEpochs ()
  {
    return nIters_;
  }


  template <class T1, class T2> int RadialBasisFunction<T1, T2>::getNumInputs ()
  {
    return inputFeatures_;
  }


  template <class T1, class T2> int RadialBasisFunction<T1, T2>::getNumOutputs ()
  {
    return outputFeatures_;
  }


  template <class T1, class T2> int 
    RadialBasisFunction<T1, T2>::getNumTrainingPatterns()
  {
    return pTrainingPatterns_;
  }


  template <class T1, class T2> double 
    RadialBasisFunction<T1, T2>::getTrainingInput (unsigned int pattern,
                                                   unsigned int input)
  {
    if (pattern > pTrainingPatterns_ || input > inputFeatures_)
    {
      return -1.0f;
    }
    return trainingInputs_.at(pattern)[input];
  }


  template <class T1, class T2> double 
    RadialBasisFunction<T1, T2>::getTrainingOutput (unsigned int pattern,
                                                    unsigned int output)
  {
    if (pattern > pTrainingPatterns_ || output > outputFeatures_)
    {
      return -1.0f;
    }
    return trainingOutputs_.at(pattern)[output];
  }

  template <class T1, class T2> double
    RadialBasisFunction<T1, T2>::getRawTrainingOutput (unsigned int pattern,
                                                       unsigned int output)
  {
    if (pattern > pTrainingPatterns_ || output > outputFeatures_)
    {
      return -1.0f;
    }
    return rawTrainingOutputs_.at(pattern)[output];
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::setInputs (double *inputVec)
  {
    inputs_[0] = 1.0f; // bias term
    for (unsigned int i = 1; i <= inputFeatures_; ++i)
    {
      inputs_[i] = inputVec[i];
    }
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::computeErrors ()
  {
    unsigned int p, l;

    ETotal_ = 0.0f;
    for (p = 0; p < pTrainingPatterns_; ++p)
    {
      setInputs (trainingInputs_[p]);
      computeOutputs ();

      EVals_[p] = 0.0f;

      for (l = 1; l <= outputFeatures_; ++l)
      {
        EVals_[p] += (outputs_[l] - trainingOutputs_[p][l]) *
                     (outputs_[l] - trainingOutputs_[p][l]);
      }
      EVals_[p] *= 0.5f;

      ETotal_ += EVals_[p];
    }
    RMSError_ = sqrt (ETotal_ / pTrainingPatterns_);
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::computeErrorsWithDisable ()
  {
    //! TODO
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::findInputMinMaxVals ()
  {
    unsigned int i, p;

    //! Error: no training patterns have been added, or user has specified a range of
    //! values already
    if (pTrainingPatterns_ <= 0 || scalingOverride_)
    {
      return;
    }

    for (i = 0; i <= inputFeatures_; ++i)
    {
      inputMaxVals_[i] = inputMinVals_[i] = rawTrainingInputsBackup_.at(0)[i];
    }
    for (i = 0; i <= inputFeatures_; ++i)
    {
      for (p = 1; p < pRawTrainingPatternsBackup_; ++p)
      {
        if (rawTrainingInputsBackup_.at(p)[i] > inputMaxVals_[i])
        {
          inputMaxVals_[i] = rawTrainingInputsBackup_.at(p)[i];
        }
        if (rawTrainingInputsBackup_.at(p)[i] < inputMinVals_[i])
        {
          inputMinVals_[i] = rawTrainingInputsBackup_.at(p)[i];
        }
      }
    } // for (i = 0; i <= inputFeatures_; ++i)
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::findOutputMinMaxVals ()
  {
    unsigned int i, p;

    //! Error: no training patterns have been added, or user has specified a range of
    //! values already
    if (pTrainingPatterns_ <= 0 || scalingOverride_)
    {
      return;
    }

    for (i = 0; i <= outputFeatures_; ++i)
    {
      outputMaxVals_[i] = outputMinVals_[i] = rawTrainingOutputsBackup_.at(0)[i];
    }

    for (i = 0; i <= outputFeatures_; ++i)
    {
      for (p = 1; p < pTrainingPatterns_; ++p)
      {
        if (rawTrainingOutputsBackup_.at(p)[i] > outputMaxVals_[i])
        {
          outputMaxVals_[i] = rawTrainingOutputsBackup_.at(p)[i];
        }
        if (rawTrainingOutputsBackup_.at(p)[i] < outputMinVals_[i])
        {
          outputMinVals_[i] = rawTrainingOutputsBackup_.at(p)[i];
        }
      }
    } // for (i = 0; i <= outputFeatures_; ++i)
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::getRawRange (vector<double>& minInVals,
                                                   vector<double>& maxInVals)
  {
    unsigned int i;
    minInVals.resize (inputFeatures_+1);
    maxInVals.resize (inputFeatures_+1);

    minInVals.at (0) = 1.0;
    maxInVals.at (0) = 1.0;
    for (i = 1; i <= inputFeatures_; ++i)
    {
      minInVals.at (i) = inputMinVals_[i];
      maxInVals.at (i) = inputMaxVals_[i];
    }
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::setRawRange (vector<double>& minInVals,
                                                   vector<double>& maxInVals,
                                                   vector<double>& minOutVals,
                                                   vector<double>& maxOutVals)
  {
    unsigned int i;
    for (i = 1; i < inputFeatures_; ++i)
    {
      inputMinVals_[i] = minInVals.at (i);
      inputMaxVals_[i] = maxInVals.at (i);
    }
    scaleAllInputs ();
    for (i = 1; i < outputFeatures_; ++i)
    {
      outputMinVals_[i] = minOutVals.at (i);
      outputMaxVals_[i] = maxOutVals.at (i);
    }
    scaleAllOutputs ();
    scalingOverride_ = true;
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::scaleInput (vector<double> &rawVec,
                                                  double *scaledVec)
  {
    unsigned int i;
    double range;

    scaledVec[0] = 1.0f;
    for (i = 1; i <= inputFeatures_; ++i)
    {
      range = inputMaxVals_[i] - inputMinVals_[i];
      if (fabs(range) <= 0.0000001)
      {
        scaledVec[i] = 1.0f;
      }
      else
      {
        scaledVec[i] = -1.0f + (2.0f * ((rawVec.at (i) - inputMinVals_[i]) / range));
      }
    }
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::scaleAllInputs ()
  {
    unsigned int p, i;
    double range;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
    {
      return;
    }

    //! Scale training patterns
    for (p = 0; p < pTrainingPatterns_; ++p)
    {
      trainingInputs_[p][0] = 1.0f; // bias term is special case

      if (useRawInputs_)
      {
        for (i = 1; i <= inputFeatures_; ++i)
        {
          trainingInputs_[p][i] = rawTrainingInputs_[p][i];
        } // for (i = 1; i <= inputFeatures_; ++i)
      } // if (useRawInputs_)
      else
      {
        for (i = 1; i <= inputFeatures_; ++i)
        {
          range = inputMaxVals_[i] - inputMinVals_[i];
          if (range == 0.0)
          {
            trainingInputs_[p][i] = 1.0f;
          }
          else
          {
            trainingInputs_[p][i] = -1.0f + (2.0f * ((rawTrainingInputs_[p][i] -
                                                      inputMinVals_[i]) / range));
          }
        } // for (i = 1; i <= inputFeatures_; ++i)
      } // if (useRawInputs_) ... else
    }

    //! Scale testing patterns
    if (pTestingPatterns_ > 0)
    {
      for (p = 0; p < pTestingPatterns_; ++p)
      {
        testingInputs_[p][0] = 1.0f; // bias term is special case
        if (useRawInputs_)
        {
          for (i = 1; i <= inputFeatures_; ++i)
          {
            trainingInputs_[p][i] = rawTrainingInputs_[p][i];
          } // for (i = 1; i <= inputFeatures_; ++i)
        } // if (useRawInputs_)
        else
        {
          for (i = 1; i <= inputFeatures_; ++i)
          {
            range = inputMaxVals_[i] - inputMinVals_[i];
            if (range == 0)
            {
              testingInputs_[p][i] = 1.0f;
            }
            else
            {
              testingInputs_[p][i] = -1.0f + 2.0f * (rawTestingInputs_[p][i] -
                                                     inputMinVals_[i]) / range;
            }
          } // for (i = 1; i <= inputFeatures_; ++i)
        } // if (useRawInptus_) ... else
      } // for (p = 0; p < pTestingPatterns_; ++p)
    } // if (pTestingPatterns_ > 0)
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::scaleAllOutputs ()
  {
    unsigned int p, k;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
    {
      return;
    }

    for (p = 0; p < pTrainingPatterns_; ++p)
    {
      trainingOutputs_.at(p)[0] = 1.0; //! bias term is special case
      for (k = 1; k <= outputFeatures_; ++k)
      {
        trainingOutputs_.at(p)[k] = -1.0f + (2.0f * 
          ((rawTrainingOutputs_.at(p)[k] - outputMinVals_[k]) /
          (outputMaxVals_[k] - outputMinVals_[k])));
      }
    }
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::initializeSynapses ()
  {
    unsigned int i, j, k, l, p;

    //! Weights to "0" node are irrelevant since 0 interneuron is bias term.
    for (i = 0; i <= inputFeatures_; ++i)
    {
      synapses0to1_[0][i] = 0.0;
    }
    for (j = 0; j <= interJFeatures_; ++j)
    {
      synapses1to2_[0][j] = 0.0;
    }
    for (k = 0; k <= interKFeatures_; ++k)
    {
      synapses2to3_[0][k] = 0.0;
    }

    //! Put in random values for the alpha-plane weights
    srand( (unsigned)time(NULL) );

    for (j = 1; j <= interJFeatures_; ++j)
    {
      adjustAlphaWeight (j);
    }

    //! Set weights into beta nodes based on specific examples.
    //! Need to compute alpha-plane activations to set these, so
    //! initialize weights with valid values.
    for (j = 0; j <=interJFeatures_; ++j)
    {
      for (k = 0; k <= interKFeatures_; ++k)
      {
        synapses1to2_[k][j] = 0.0f;
      }
    }

    for (l = 0; l <= outputFeatures_; ++l)
    {
      synapses2to3_[l][0] = 0.0f;
    }

    for (k = 1; k <= interKFeatures_; ++k)
    {
      for (l = 1; l <= outputFeatures_; ++l)
      {
        synapses2to3_[l][k] = 1.0f;
      }
    }

    //! Set the alpha-beta synapse weights
    adjustBetaWeights ();

    //! Set gamma synapses based on target values.  Find most responsive
    //! beta node, and try to make it be responsible for output
    for (l = 1; l <= outputFeatures_; ++l)
    {
      for (p = 0; p < pTrainingPatterns_; ++p)
      {
        setInputs (trainingInputs_[p]);
        computeOutputs();
        //! Find the most responsive beta node:
        hiddenKOutputs_[0] = 0.0; // suppress bias for maxnet
        k = maxElement (hiddenKOutputs_, interKFeatures_+1);

        synapses2to3_[l][k] = trainingOutputs_[p][l] / hiddenKOutputs_[k];
      }
    }
  }


  //! This function assumes the input vector has been defined.  It uses the
  //! the synapse0to1_ matrix to compute the net input to each alpha-layer
  //! neuron.  It then computes the respective output of each alpha-layer
  //! neuron and then uses the synapses1to2_ matrix to compute the net input
  //! to each beta-layer neuron and computes the corresponding outputs.  Then
  //! this function uses synapses2to3_ to compute the net input to the gamma-
  //! layer neuron(s) and the respective output(s) of the gamma layer.
  template <class T1, class T2> void RadialBasisFunction<T1, T2>::computeOutputs ()
  {
    unsigned int i, j, k, l;
    const double fireThreshold = 0.5f;

    //! Start with the Alpha (J) layer
    hiddenJOutputs_[0] = 1.0f; //! Bias term for output layer
    hiddenJInputs_[0] = 0.0f;

    for (j = 1; j <= interJFeatures_; ++j)
    {
      //! Compute weighted sum of pattern inputs:
      hiddenJInputs_[j] = 0.0f;

      for (i = 0; i <= inputFeatures_; ++i)
      {
        hiddenJInputs_[j] += synapses0to1_[j][i] * inputs_[i];
      }

      //! Use "net" input to each interneuron to compute respective outputs
      interJNeurons_[j].activate (hiddenJInputs_[j]);
      hiddenJOutputs_[j] = interJNeurons_[j].output ();
      alphaFire_[j] += (fabs(hiddenJOutputs_[j]) > fireThreshold) ? 1 : 0;
      alphaLife_[j] += 1;
    }

    //! Repeat for the beta layer, using alpha outputs to compute net inputs
    hiddenKInputs_[0] = 0.0f;
    hiddenKOutputs_[0] = 1.0f; // Bias term for gamma layer input

    for (k = 1; k <= interKFeatures_; ++k)
    {
      //! Compute weighted sum of  pattern inputs:
      hiddenKInputs_[k] = 0.0f;
      for (j = 0; j <= interJFeatures_; ++j)
      {
        hiddenKInputs_[k] += synapses1to2_[k][j] * hiddenJOutputs_[j];
      }

      //! Use "net" input to each beta neuron to compute respective outputs
      interKNeurons_[k].activate (hiddenKInputs_[k]);
      hiddenKOutputs_[k] = interKNeurons_[k].output ();
    }

    //! Repeat for the gamma layer (may be only 1 intersting output)
    outputLInputs_[0] = 0.0f;
    outputs_[0] = 1.0f; //! Not used
    for (l = 1; l <= outputFeatures_; ++l)
    {
      outputLInputs_[l] = 0.0f;

      //! Compute weighted sum of pattern inputs:
      for (k = 0; k <= interKFeatures_; ++k) // include bias contrib
      {
        outputLInputs_[l] += synapses2to3_[l][k] * hiddenKOutputs_[k];
      }

      //! Use "net" input to each gamma neuron to compute respective outputs
      outputNeurons_[l].activate (outputLInputs_ [l]);
      outputs_[l] = outputNeurons_[l].output ();
    }
  }


  template <class T1, class T2> double RadialBasisFunction<T1, T2>::getOutput (unsigned int index)
  {
    return (index > outputFeatures_) ? -1.0f : outputs_[index];
  }


  template <class T1, class T2> void RadialBasisFunction<T1, T2>::adjustWeights ()
  {
    const double fireMax = 0.80f, fireMin = 0.10f;
    const int lifeMin = 500;
    double origRMSError, temp = 0.0f, popmult = 1.0f;
    unsigned int j = 0, k = 0;
    bool adjusted = false;
    static bool first = true;
    static int oldpop, errcount;

    if (first)
    {
      oldpop = pTrainingPatterns_;
      errcount = 0;
      first = false;
    }

    if (useAlpha_)
    {
      //! Modify Input-Alpha synapses
      for (j = 1; j < (interJFeatures_ + 1); ++j)
      {
        if (alphaLife_[j] > lifeMin)
        {
          temp = (double)alphaFire_[j]/(double)alphaLife_[j];
          if (temp > fireMax || temp < fireMin)
          {
            adjustAlphaWeight (j);
            alphaLife_[j] = alphaFire_[j] = 0;
            adjusted = true;
          }
        }
      }
    } // if (useAlpha)

    if (useBeta_)
    {
      //! Modify Alpha-Beta synapses
      if (adjusted || (pTrainingPatterns_ > (oldpop * popmult)))
      {
        adjustBetaWeights ();
        adjusted = true;
        oldpop = pTrainingPatterns_;
      }
    }

    //! Modify Beta-Gamma synapses
    computeErrors ();
    origRMSError = RMSError_;

    fillRandDWeights (randRange_);
    addDWBetaGammaWeights ();

    computeErrors ();
    if (origRMSError <= RMSError_)
    {
      RMSError_ = origRMSError;
      //! New RMS error no better than old one.  Reject weight changes.
      subDWBetaGammaWeights ();
      ++errcount;
    }
    else
    {
      errcount = 0;
    }

    if (useGamma_)
    {
      if (errcount >= 10)
      {
        randRange_ *= 0.99;
      }
      else if (errcount == 0)
      {
        randRange_ *= 1.1;
      }
      if (randRange_ <= 0.01f)
      {
        randRange_ = 0.01f;
      }
    } // if (useGamma)
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::addDWBetaGammaWeights ()
  {
    unsigned int k, l;
    for (l = 1; l <= outputFeatures_; ++l)
    {
      for (k = 0; k <= interKFeatures_; ++k)
      {
        synapses2to3_[l][k] += dwBetaGamma_[l][k];
      }
    }
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::subDWBetaGammaWeights ()
  {
    unsigned int k, l;
    for (l = 1; l <= outputFeatures_; ++l)
    {
      for (k = 0; k <= interKFeatures_; ++k)
      {
        synapses2to3_[l][k] -= dwBetaGamma_[l][k];
      }
    }
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::fillRandDWeights (double val)
  {
    unsigned int k, l;
    for (l = 1; l <= outputFeatures_; ++l)
    {
      for (k = 0; k <= interKFeatures_; ++k)
      {
        dwBetaGamma_[l][k] = randRange_ * 2.0f *
                             ((double) rand() / ((double)(RAND_MAX + 1)) - 0.5);
      }
    }
  }

/*
  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::snapshot (NNDisplayAdapter &display)
  {
    display.clear();
    vector <double> temp;
    unsigned int i, j, k, l;

    //! Copy the network node values
    for (i = 0; i < inputFeatures_+1; ++i)
    {
      temp.push_back (inputs_[i]);
    }
    display.nodeOuts.push_back (temp);
    temp.clear ();
    for (j = 0; j < interJFeatures_+1; ++j)
    {
      temp.push_back (hiddenJOutputs_[j]);
    }
    display.nodeOuts.push_back (temp);
    temp.clear ();
    for (k = 0; k < interKFeatures_+1; ++k)
    {
      temp.push_back (hiddenKOutputs_[k]);
    }
    display.nodeOuts.push_back (temp);
    temp.clear ();
    for (l = 0; l < outputFeatures_+1; ++l)
    {
      temp.push_back (outputs_[l]);
    }
    display.nodeOuts.push_back (temp);
    temp.clear ();

    //! Copy the network weight values
    for (j = 0; j <= interJFeatures_; ++j)
    {
      for (i = 0; i <= inputFeatures_; ++i)
      {
        temp.push_back (synapses0to1_[j][i]);
      }
    }
    display.linkWeights.push_back (temp);
    temp.clear ();

    for (k = 0; k <= interKFeatures_; ++k)
    {
      for (j = 0; j <= interJFeatures_; ++j)
      {
        temp.push_back (synapses1to2_[k][j]);
      }
    }
    display.linkWeights.push_back (temp);
    temp.clear ();

    for (l = 0; l <= outputFeatures_; ++l)
    {
      for (k = 0; k <= interKFeatures_; ++k)
      {
        temp.push_back (synapses2to3_[l][k]);
      }
    }
    display.linkWeights.push_back (temp);
    temp.clear ();
  }
*/
  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::setDynamicFunctions(bool useAlpha,
                                                          bool useBeta,
                                                          bool useGamma)
  {
    useAlpha_ = useAlpha;
    useBeta_ = useBeta;
    useGamma_ = useGamma;
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::setUseRawInputs(bool useRawInputs)
  {
    useRawInputs_ = useRawInputs;
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::adjustAlphaWeight (int node)
  {
    double sum_w_abs = 0.0f;
    unsigned int i;

    //! Non-bias weights:
    for (i = 1; i <= inputFeatures_; ++i)
    {
      if (useRawInputs_)
      {
        synapses0to1_[node][i] = ((inputMaxVals_[i] - inputMinVals_[i]) / 2.0f) * (inputMaxVals_[i] - inputMinVals_[i]) *
                                 ((double) rand()/((double)(RAND_MAX+1))-0.5f);
      }
      else
      {
        synapses0to1_[node][i] = initWeight_ * 2.0f *
                               ((double) rand()/((double)(RAND_MAX+1))-0.5f);
      }
    }

    //set biases to keep planes "involved"
    sum_w_abs = 0.0f;
    for (i = 1; i <= inputFeatures_; ++i)
    {
      sum_w_abs += fabs (synapses0to1_[node][i]);
    }

    //! If input ranges from -1 to 1, then largest possible weighted
    //! input for plane j is sum_i(w_j,i*sgn(w_j,i)) = sum_w_abs
    //! and smallest possible input is -sum_w_abs.
    //! Keep bias between these values to avoid useless planes.
    if (useRawInputs_)
    {
      synapses0to1_[node][0]= sum_w_abs * (inputMaxVals_[i] - inputMinVals_[i]) *
                              ((double) rand()/((double)(RAND_MAX+1))-0.5f);
    }
    else
    {
      synapses0to1_[node][0]= sum_w_abs * 2.0f *
                              ((double) rand()/((double)(RAND_MAX+1))-0.5f);
    }
  }


  template <class T1, class T2>
    void RadialBasisFunction<T1, T2>::adjustBetaWeights ()
  {
    unsigned int k, j, i, x;
    const int arbitraryLargeIterationLimit = 10000;
    char *path = NULL;
    const bool clustering = true;
    vector<double*> trainSet;
    vector<double*>::iterator trainIter;
    vector<double> trainVal;
    double *input, *copy;
    vector<double> out;

    if (pTrainingPatterns_ == 0)
    {
      //! Train the beta input synapses
      for (x = 0; x <= interKFeatures_; ++x)
      {
        synapses1to2_[x][0] = 0.0f;
        for (j = 1; j <= interJFeatures_; ++j)
        { 
          synapses1to2_[x][j] = 0.0f;
        } // for (j = 1; j <= interJFeatures_; ++j)
      } // for (k = 0; k <= interKFeatures_; ++k)

      return;
    }

    k = (interKFeatures_ <= pTrainingPatterns_) ? interKFeatures_ : pTrainingPatterns_;

    if (clustering)
    {
      //! Add all training patterns to the clustering set
      input = new double[inputFeatures_ + 1];

      Clustering::kMeans kclust (inputFeatures_+1, 1, k, NULL);
      for (i = 0; i < pTrainingPatterns_; ++i)
      {
        for (x = 0; x < (inputFeatures_+1); ++x)
        {
          input[x] = trainingInputs_.at(i)[x];
        }
        trainVal.clear();
        trainVal.push_back(trainingOutputs_.at(i)[1]);
        kclust.addTrainingPattern (input, trainVal);
      }

      delete [] input;

      //! Cluster data
      kclust.seedClusters ();
      x = kclust.recluster ();
      j = 0;
      while (x > 0 && j < arbitraryLargeIterationLimit)
      {
        x = kclust.recluster ();
        ++j;
      }

      copy = new double[inputFeatures_ + 1];
      for (i = 0; i < k; ++i)
      {
        //! Add the cluster centroids to the training set
        input = new double[inputFeatures_ + 1];

        kclust.getClusterInfo (i, copy, out);
        for (j = 0; j < inputFeatures_+1; ++j)
        {
          input[j] = copy[j];
        }
        trainSet.push_back(input);
        input = NULL;
      }
      delete [] copy;
    } // if (clustering)
    else
    {
      bool *used;
      used = new bool[pTrainingPatterns_];
      for (j = 0; j < pTrainingPatterns_; ++j)
      {
        used[j] = false;
      }

      //! Select k random patterns to train the beta-layer neurons with
      for (j = 0; j < k; ++j)
      {
        //! Grab a unique, random pattern
        do
        {
          i = rand() % pTrainingPatterns_;
        } while (used[i]);
        used[i] = true;

        //! Add that pattern to the training set
        input = new double[inputFeatures_ + 1];
        for (x = 0; x < (inputFeatures_+1); ++x)
        {
          input[x] = trainingInputs_.at(i)[x];
        }
        trainSet.push_back(input);
        input = NULL;
      }
      delete [] used;
    } // select random input patterns for training


    //! Train the beta input synapses
    for (x = 0; x < interKFeatures_; ++x)
    {
      if (x < k)
      {
        //! Set sequence to network inputs
        setInputs (trainSet.at(x));

        //! Compute outputs up to alpha layer
        computeOutputs();
      }

      synapses1to2_[x][0] = (double)(-(int)interJFeatures_ + 1)/(double)interJFeatures_;
//      synapses1to2_[x][0] = (double)(-interJFeatures_ + 1);
      for (j = 1; j <= interJFeatures_; ++j)
      { 
        if (x < k)
        {
          //! If beta-layer neuron is active...

          //! Set weight to signum of output of alpha layer nodes
          synapses1to2_[x][j] = sgn(hiddenJOutputs_[j]) * (1.0 / (double)interJFeatures_);;
//          synapses1to2_[x][j] = sgn(hiddenJOutputs_[j]);
        }
        else
        {
          //! If beta-layer neuron is not active...
          //! Set to 0 (no input to the neuron--dead neuron)
          synapses1to2_[x][j] = 0.0f;
        }
        //! See EECS 484 HW5, Experiment #2
      } // for (j = 1; j <= interJFeatures_; ++j)
    } // for (k = 0; k <= interKFeatures_; ++k)

    for (trainIter = trainSet.begin(); trainIter != trainSet.end(); ++trainIter)
    {
      delete [] *trainIter;
    }
    trainSet.clear();
  }


  template <class T1, class T2>
    double RadialBasisFunction<T1, T2>::getGammaWeightRange ()
  {
    return randRange_;
  }


  template <class T1, class T2>
    double RadialBasisFunction<T1, T2>::getMaxOutput (unsigned int output)
  {
    if (output > outputFeatures_)
    {
      return 0.0f;
    }
    return outputMaxVals_[output];
  }

  template <class T1, class T2>
    double RadialBasisFunction<T1, T2>::getMinOutput (unsigned int output)
  {
    if (output > outputFeatures_)
    {
      return 0.0f;
    }
    return outputMinVals_[output];
  }
} // NeuralNet
