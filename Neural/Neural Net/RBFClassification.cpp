///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        RBFClassification.cpp
//  Revision:        1.0 - 7 February, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
//  Currently, can only use numerical neuron types
//
///////////////////////////////////////////////////////////////////////////////

#include "RBFClassification.h"
#include "../Neuron/Numerical/Numerical.h"
#include "../../Libraries/Math/VectorMath.h"
#include "../../Libraries/Math/MatrixMath.h"
#include "../../Clustering/kMeans/kMeansCluster.h"
#include <cmath>
#include <fstream>

#define DEBUGOUT
#define INSTANTANEOUS

using namespace std;
using namespace Math;


//! Explicit instantiations
template class LIBRARY_API NeuralNet::RBFClassification<Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::RBFClassification<Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::RBFClassification<Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::RBFClassification<Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::RBFClassification<Neuron::LogicNeuron>;


namespace NeuralNet
{
  template <class T1> RBFClassification<T1>::RBFClassification (int inFeat,
                                                                int interFeat,
                                                                int outFeat,
                                                                double initWeight,
                                                                double initEta) :
    inputFeatures_(inFeat),
    interJFeatures_(inFeat),
    interKFeatures_(interFeat),
    outputFeatures_(outFeat),
    initWeight_(initWeight),
    initEta_(initEta),
    pTrainingPatterns_(0),
    pTestingPatterns_(0),
    nIters_(0),
    randRange_(0.1f),
    alphaAxis_(1000),
    scalingOverride_(false)
  {
    int j, k, l;

    interJFeatures_ = (alphaAxis_ * inputFeatures_) + 1;

    //! Allocate memory for value passing & temporary storage
    inputs_ = new double [inputFeatures_ + 1];
    inputMaxVals_ = new double [inputFeatures_ + 1];
    inputMinVals_ = new double [inputFeatures_ + 1];

    hiddenJInputs_ = new double [interJFeatures_ + 1];
    hiddenJOutputs_ = new double [interJFeatures_ + 1];
    hiddenKInputs_ = new double [interKFeatures_ + 1];
    weight_ = new double [interKFeatures_ + 1];
    hiddenKOutputs_ = new double [interKFeatures_ + 1];

    outputLInputs_ = new double [outputFeatures_ + 1];
    outputs_ = new double [outputFeatures_ + 1];

    outputMaxVals_ = new double [outputFeatures_ + 1];
    outputMinVals_ = new double [outputFeatures_ + 1];

    //! Allocate memory for the neurons
    interJNeurons_ = new Neuron::LogicNeuron [interJFeatures_ + 1];
    interKNeurons_ = new T1 [interKFeatures_ + 1];
    interKNeuronsActive_ = new bool[interKFeatures_ + 1];
    outputNeurons_ = new Neuron::LinearNeuron [outputFeatures_ + 1];

    //! Allocate the memory for synaptic connections from layer 0 to layer 1
    synapses0to1_ = new double * [interJFeatures_ + 1];
    for (j = 0; j <= interJFeatures_; ++j)
    {
      synapses0to1_[j] = new double [inputFeatures_ + 1];
    }

    //! Allocate the memory for synaptic connections from layer 1 to layer 2
    synapses1to2_ = new double * [interKFeatures_ + 1];
    for (k = 0; k <= interKFeatures_; ++k)
    {
      interKNeuronsActive_[k] = true;
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


  template <class T1> RBFClassification<T1>::~RBFClassification ()
  {
    vector<double*>::iterator iter1, iter2;
    int j, k, l;

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

  }


  template <class T1> void RBFClassification<T1>::initialize ()
  {
    //! Initialize the synapses for the network by randomly assigning weights
    initializeSynapses ();
  }


  template <class T1> double RBFClassification<T1>::trainNetwork (int iter,
                                                                  bool prune)
  {
    adjustWeights ();
    return RMSError_;
  }


  template <class T1> void RBFClassification<T1>::runTests (int k)
  {
  }


  template <class T1> void RBFClassification<T1>::runTestsBest (int top, int k)
  {
  }


  template <class T1> 
    double RBFClassification<T1>::autoOptimize (vector<double>& minVals,
                                                vector<double>& maxVals,
                                                vector<double>& best,
                                                int outWatch,
                                                bool maximize)
  {
    double out = 0.0f;
    return out;
  }


  template <class T1>
    void RBFClassification<T1>::optiHelper (vector<double>& minVals,
                                            vector<double>& maxVals,
                                            vector<double>& current,
                                            vector<double>& best,
                                            int element,
                                            int outWatch,
                                            double& bestout,
                                            bool maximize)
  {
  }


  template <class T1> void RBFClassification<T1>::addTrainingSet (double *in,
                                                                  double *out)
  {
    int p;//, j;
    double *tempVec1, *tempVec2, *tempVec3;//, dist;
    vector<double> distTemp1, distTemp2;

    for (p = 0; p < (inputFeatures_ + 1); ++p)
    {
      distTemp1.push_back (in[p]);
    }

    distTemp2.resize(inputFeatures_ + 1);
/*
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
*/
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


  template <class T1> void RBFClassification<T1>::clearTrainingSets ()
  {
  }


  template <class T1>
    void RBFClassification<T1>::adjustNeighborhood (int method, int size)
  {
  }


  template <class T1>
    void RBFClassification<T1>::addTestingSet (double *in)
  {
  }


  template <class T1>
    int RBFClassification<T1>::parseInputFile (const char *trainPath,
                                               const char *testPath)
  {
    int nvals = 0, tvals=0;
    char message[256];
    double inval;
    double *tempVec1 = NULL, *tempVec2 = NULL;
    ifstream trainingFile, testingFile;
    int okay = 0, x, y, i;
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



  template <class T1>
    bool RBFClassification<T1>::readNetworkFile (const char *netPath)
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

  template <class T1>
    bool RBFClassification<T1>::writeNetworkFile (const char *netPath)
  {
    //! TODO
    return true;
  }


  template <class T1> double RBFClassification<T1>::getRMSError ()
  {
    return RMSError_;
  }


  template <class T1> double RBFClassification<T1>::getTotalError ()
  {
    return ETotal_;
  }


  template <class T1> int RBFClassification<T1>::getNumEpochs ()
  {
    return nIters_;
  }


  template <class T1> int RBFClassification<T1>::getNumInputs ()
  {
    return inputFeatures_;
  }


  template <class T1> int RBFClassification<T1>::getNumOutputs ()
  {
    return outputFeatures_;
  }


  template <class T1> int 
    RBFClassification<T1>::getNumTrainingPatterns()
  {
    return pTrainingPatterns_;
  }


  template <class T1> double 
    RBFClassification<T1>::getTrainingInput (int pattern, int input)
  {
    if (pattern > pTrainingPatterns_ || pattern < 0 ||
        input > inputFeatures_ || input < 0)
    {
      return -1.0f;
    }
    return trainingInputs_.at(pattern)[input];
  }


  template <class T1> double 
    RBFClassification<T1>::getTrainingOutput (int pattern, int output)
  {
    if (pattern > pTrainingPatterns_ || pattern < 0 ||
        output > outputFeatures_ || output < 0)
    {
      return -1.0f;
    }
    return trainingOutputs_.at(pattern)[output];
  }

  template <class T1> double
    RBFClassification<T1>::getRawTrainingOutput (int pattern, int output)
  {
    if (pattern > pTrainingPatterns_ || pattern < 0 ||
        output > outputFeatures_ || output < 0)
    {
      return -1.0f;
    }
    return rawTrainingOutputs_.at(pattern)[output];
  }


  template <class T1> void RBFClassification<T1>::setInputs (double *inputVec)
  {
    inputs_[0] = 1.0f; // bias term
    for (int i = 1; i <= inputFeatures_; ++i)
    {
      inputs_[i] = inputVec[i];
    }
  }


  template <class T1> void RBFClassification<T1>::computeErrors ()
  {
    int p, l;

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


  template <class T1> void RBFClassification<T1>::computeErrorsWithDisable ()
  {
    //! TODO
  }


  template <class T1> void RBFClassification<T1>::findInputMinMaxVals ()
  {
    int i, p;

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


  template <class T1> void RBFClassification<T1>::findOutputMinMaxVals ()
  {
    int i, p;

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


  template <class T1>
    void RBFClassification<T1>::getRawRange (vector<double>& minInVals,
                                                   vector<double>& maxInVals)
  {
    int i;
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


  template <class T1>
    void RBFClassification<T1>::setRawRange (vector<double>& minInVals,
                                             vector<double>& maxInVals,
                                             vector<double>& minOutVals,
                                             vector<double>& maxOutVals)
  {
    int i;
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


  template <class T1>
    void RBFClassification<T1>::scaleInput (vector<double> &rawVec,
                                                  double *scaledVec)
  {
    int i;
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


  template <class T1> void RBFClassification<T1>::scaleAllInputs ()
  {
    int p, i;
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
      }
    }

    //! Scale testing patterns
    if (pTestingPatterns_ > 0)
    {
      for (p = 0; p < pTestingPatterns_; ++p)
      {
        testingInputs_[p][0] = 1.0f; // bias term is special case
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
      } // for (p = 0; p < pTestingPatterns_; ++p)
    } // if (pTestingPatterns_ > 0)
  }


  template <class T1> void RBFClassification<T1>::scaleAllOutputs ()
  {
    int p, k;

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


  template <class T1> void RBFClassification<T1>::initializeSynapses ()
  {
    int i,j,k,l,p;
    double axis;

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



    //! Put a decision plane at regular intervals along each of the input
    //! axes.
    //! First, set all synapses to 0...
    for (i = 0; i <= inputFeatures_; ++i)
    {
      for (j = 0; j <= interJFeatures_; ++j)
      {
        synapses0to1_[j][i] = 0.0f;
      }
    }
    //! Now, for each axis I, set the weights to w_I = 1, w_b = -val_I,
    //! where val_I is the current position along the axis where we want
    //! a decision plane to be set
    /*
    for (i = 1; i <= inputFeatures_; ++i)
    {
      axis = -1.0f;
      for (j = 0; j <= alphaAxis_; ++j, axis += (2.0f/(double)alphaAxis_))
      {
        synapses0to1_[((i-1) * alphaAxis_) + (j+1)][0] = -1.0f/axis; //! Bias
        synapses0to1_[((i-1) * alphaAxis_) + (j+1)][i] = 1.0f;  //! Relevant input
      }
    }
    */
    int count = 1;
    for (i = 1; i <= inputFeatures_; ++i)
    {
      axis = -1.0f;
      for (j = 1; j <= alphaAxis_; ++j, axis += (2.0f/(double)alphaAxis_), ++count)
      {
        synapses0to1_[count][0] = -axis; // Bias
        synapses0to1_[count][i] = 1.0f; // Relevant input
      }
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
//    initializeBetas ();
    initializeBetasSpecial ();
    axis = -1.0f;



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
  template <class T1> void RBFClassification<T1>::computeOutputs ()
  {
    int i, j, k, l;
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
    }

    //! Repeat for the beta layer, using alpha outputs to compute net inputs
    hiddenKInputs_[0] = 1.0f;
    hiddenKOutputs_[0] = 0.0f; // Bias term for gamma layer input

    for (k = 1; k <= interKFeatures_; ++k)
    {
      //! Compute weighted sum of  pattern inputs:
      hiddenKInputs_[k] = 0.0f;
      for (j = 0; j <= interJFeatures_; ++j)
      {
        hiddenKInputs_[k] += synapses1to2_[k][j] * hiddenJOutputs_[j];
      }

      //! Use "net" input to each beta neuron to compute respective outputs
      interKNeurons_[k].activate (hiddenKInputs_[k], weight_[k]);
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
        if (interKNeuronsActive_[k])
        {
          outputLInputs_[l] += synapses2to3_[l][k] * hiddenKOutputs_[k];
        }
      }

      //! Use "net" input to each gamma neuron to compute respective outputs
      outputNeurons_[l].activate (outputLInputs_ [l]);
      outputs_[l] = outputNeurons_[l].output ();
    }
  }


  template <class T1> double RBFClassification<T1>::getOutput (int index)
  {
    return (index > outputFeatures_ || index < 0) ? -1.0f : outputs_[index];
  }


  template <class T1> void RBFClassification<T1>::setBetaActive (int index, bool active)
  {
    if (index > interKFeatures_ || index < 0)
    {
      return;
    }
    interKNeuronsActive_[index] = active;
  }


  template <class T1> double RBFClassification<T1>::getAlphaOutput (int index)
  {
    return (index > interJFeatures_ || index < 0) ? -1.0f : hiddenJOutputs_[index];
  }


  template <class T1> double RBFClassification<T1>::getBetaOutput (int index)
  {
    return (index > interKFeatures_ || index < 0 || !interKNeuronsActive_[index]) ? 0.0f : hiddenKOutputs_[index];
  }


  template <class T1> void RBFClassification<T1>::adjustWeights ()
  {
    /*
    const double fireMax = 0.75f, fireMin = 0.25f;
    const int lifeMin = 100;
    double origRMSError, temp = 0.0f, popmult = 1.0f;
    int j = 0, k = 0;
    static bool first = true;
    static int oldpop, errcount;
    const bool useBeta = true;
    */
    static int errcount;
    double origRMSError;
    static bool first = true;

    if (first)
    {
      errcount = 0;
      first = false;
    }

    //! Modify Beta-Gamma synapses
    computeErrors ();
    origRMSError = RMSError_;

#ifdef INSTANTANEOUS
    setBetaGammaWeights();
    computeErrors();
#else
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
#endif
  }


  template <class T1>
    void RBFClassification<T1>::addDWBetaGammaWeights ()
  {
    int k, l;
    for (l = 1; l <= outputFeatures_; ++l)
    {
      for (k = 0; k <= interKFeatures_; ++k)
      {
        synapses2to3_[l][k] += dwBetaGamma_[l][k];
      }
    }
  }


  template <class T1>
    void RBFClassification<T1>::subDWBetaGammaWeights ()
  {
    int k, l;
    for (l = 1; l <= outputFeatures_; ++l)
    {
      for (k = 0; k <= interKFeatures_; ++k)
      {
        synapses2to3_[l][k] -= dwBetaGamma_[l][k];
      }
    }
  }


  template <class T1>
    void RBFClassification<T1>::fillRandDWeights (double val)
  {
//    qqq
    int k, l;
    for (l = 1; l <= outputFeatures_; ++l)
    {
      for (k = 0; k <= interKFeatures_; ++k)
      {
        dwBetaGamma_[l][k] = randRange_ * 2.0f *
                             ((double) rand() / ((double)(RAND_MAX + 1)) - 0.5);
      }
    }
  }


  template <class T1>
    void RBFClassification<T1>::setBetaGammaWeights ()
  {
    bool okay = true;
    double debug;

    /*
    matrix X(interKFeatures_+1, inputFeatures_+1);
    matrix X_T(inputFeatures_+1, interKFeatures_+1);
    matrix X1(inputFeatures_+1, inputFeatures_+1);
    matrix X_in(inputFeatures_+1, inputFeatures_+1);
    matrix XT(inputFeatures_+1, interKFeatures_+1);
    matrix t(interKFeatures_+1, 1);
    matrix w(interKFeatures_+1, 1);
    */

    //matrix X(pTrainingPatterns_, interKFeatures_+1);
    matrix X(pTrainingPatterns_, interKFeatures_),
           X_T(interKFeatures_, pTrainingPatterns_),
           X1(interKFeatures_, interKFeatures_),
           X_in(interKFeatures_, interKFeatures_),
           XT(interKFeatures_, pTrainingPatterns_), 
           t(pTrainingPatterns_, 1), 
           w(interKFeatures_, 1);
    
    int i, j;

    for (i = 0; i < pTrainingPatterns_; ++i)
    {
      setInputs (trainingInputs_[i]);
      computeOutputs ();

      for (j = 0; j < interKFeatures_; ++j)
      {
        X.at(i, j) = hiddenKOutputs_[j+1];
      }
      t.at (i, 0) = trainingOutputs_[i][1];
    }

    debug = t.at(0,0);
    debug = X.at(0,0);
    X_T = X.trans();
    okay = X_T.valid;
    debug = X_T.at(0,0);
    X1 = X_T * X;
    okay = X1.valid;
    debug = X1.at(0,0);
    X_in = X1.inv();
    okay = X_in.valid;
    debug = X_in.at(0,0);
    XT = X_in * X_T;
    okay = XT.valid;
    debug = XT.at(0,0);
    w = XT * t;
//    okay = matrixMult (*XT, *t, *w);
    debug = w.at(0,0);

    for (j = 0; j < interKFeatures_; ++j)
    {
      synapses2to3_[1][j+1] = w.at(j, 0);
    }
  }

/*
  template <class T1>
    void RBFClassification<T1>::snapshot (NNDisplayAdapter &display)
  {
    display.clear();
    vector <double> temp;
    int i, j, k, l;

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

  template <class T1> void RBFClassification<T1>::initializeBetas ()
  {
    double wght = 10.0f;
    bool option = true; //! true = on edge of plane
                        //! false = evenly-spaced throughout
    int i, j, side, count;
    double axisAdd, *ins;

    ins = new double [inputFeatures_ + 1];
    ins[0] = 1.0f;

    side = (int)sqrt((double)interKFeatures_);

    for (i = 1; i <= inputFeatures_; ++i)
    {
      ins[i] = option ? -1.0f : (-1.0f + (1.0f/(double)(side)));
    }

    count = 1;
    if (option)
    {
      //! Line up beta nodes on edges of parameter plane
      axisAdd = 2.0f/(double)(side-1);

      while (true)
      {
        
        j = inputFeatures_;
        ins[j] += axisAdd;
        if (ins[j] > 1.0000001f)
        {
          do
          {
            ins[j] = -1.0f;
            --j;
            ins[j] += axisAdd;
          } while (ins[j] > 1.0000001f && j > 0);
        }
        
/*
        ins[1] = -1.0f;
        for (int ii = 1; ii <= side; ++ii, ins[1] += axisAdd)
        {
          ins[2] = -1.0f;
          for (int jj = 1; jj <= side; ++jj, ins[2] += axisAdd)
          {
            setAlphaBetaWeights (count, ins);
            ++count;
          }
        }
*/
        setAlphaBetaWeights (count, ins);
        weight_[count] = wght;
        count++;
        if (j <= 0)
        {
          break;
        }
      } // while (true)
    } // if (option)
    else
    {
      //! Place beta nodes entirely within confines of the parameter
      //! plane
      axisAdd = 2.0f/(double)(side);

      while (true)
      {
        j = inputFeatures_;
        ins[j] += axisAdd;
        if (ins[j] > 1.0000001f)
        {
          do
          {
            ins[j] = -1.0f + (0.5f * axisAdd);
            --j;
            ins[j] += axisAdd;
          } while (ins[j] > 1.0000001f && j > 0);
        }
        setAlphaBetaWeights (count, ins);
        weight_[count] = wght;
        ++count;
        if (j <= 0)
        {
          break;
        }
      } // while (true)
    } // if (option) ... else
  }


  template <class T1> void RBFClassification<T1>::initializeBetasSpecial ()
  {
    double wght = 1.0f;
    bool option = true; //! true = on edge of plane
                        //! false = evenly-spaced throughout
    int i, j, side, count, count2;
    double axisAdd, *ins;

    ins = new double [inputFeatures_ + 1];
    ins[0] = 1.0f;

    side = (int)sqrt((double)interKFeatures_);

    for (i = 1; i <= inputFeatures_; ++i)
    {
      ins[i] = option ? -1.0f : (-1.0f + (1.0f/(double)(side)));
    }

    count = count2 = 1;
    if (interKFeatures_ == 24)
    {
      side = 5;
      //! Line up beta nodes on edges of parameter plane
      axisAdd = 2.0f/(double)(side-1);

      while (true)
      {
        
        j = inputFeatures_;
        ins[j] += axisAdd;
        if (ins[j] > 1.0000001f)
        {
          do
          {
            ins[j] = -1.0f;
            --j;
            ins[j] += axisAdd;
          } while (ins[j] > 1.0000001f && j > 0);
        }
        
        if (count2 != 12)
        {
          setAlphaBetaWeights (count, ins);
          weight_[count] = wght;
          count++;
        }
        count2++;
        if (j <= 0)
        {
          break;
        }
      } // while (true)
    } // if (option)
    else if (interKFeatures_ == 25)
    {
      int iii = 2;
      int jjj = 1;
      for (j = 1; j <= interKFeatures_; ++j)
      {
        weight_[j] = wght;
      }

      //! 25
      ins[iii] = 0.625f;
      ins[jjj] = -0.75f;
      setAlphaBetaWeights (1, ins);
      ins[iii] = 0.625f;
      ins[jjj] = -0.50f;
      setAlphaBetaWeights (2, ins);
      ins[iii] = 0.0f;
      ins[jjj] = -0.625f;
      setAlphaBetaWeights (3, ins);
      ins[iii] = -0.625f;
      ins[jjj] = -0.75f;
      setAlphaBetaWeights (4, ins);
      ins[iii] = -0.625f;
      ins[jjj] = -0.5f;
      setAlphaBetaWeights (5, ins);

      ins[iii] = 0.625f;
      ins[jjj] = 0.75f;
      setAlphaBetaWeights (6, ins);
      ins[iii] = 0.625f;
      ins[jjj] = 0.50f;
      setAlphaBetaWeights (7, ins);
      ins[iii] = 0.0f;
      ins[jjj] = 0.625f;
      setAlphaBetaWeights (8, ins);
      ins[iii] = -0.625f;
      ins[jjj] = 0.75f;
      setAlphaBetaWeights (9, ins);
      ins[iii] = -0.625f;
      ins[jjj] = 0.5f;
      setAlphaBetaWeights (10, ins);

      ins[iii] = -0.75f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (11, ins);
      ins[iii] = -0.75f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (12, ins);
      ins[iii] = -0.625f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (13, ins);
      ins[iii] = -0.5f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (14, ins);
      ins[iii] = -0.5f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (15, ins);

      ins[iii] = 0.75f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (16, ins);
      ins[iii] = 0.75f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (17, ins);
      ins[iii] = 0.625f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (18, ins);
      ins[iii] = 0.5f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (19, ins);
      ins[iii] = 0.5f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (20, ins);

      ins[iii] = -0.125f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (21, ins);
      ins[iii] = -0.125f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (22, ins);
      ins[iii] = 0.0f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (23, ins);
      ins[iii] = 0.125f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (24, ins);
      ins[iii] = 0.125f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (25, ins);
/*
      for (iii = 1; iii <= 10; ++iii)
      {
        weight_[iii] = 1.0f;
      }
      for (iii = 11; iii <= 25; ++iii)
      {
        weight_[iii] = 5.0f;
      }
*/
    }
    else
    {
      int iii = 1;
      int jjj = 2;
      //! 45
      ins[iii] = 0.875f;
      ins[jjj] = -0.875f;
      setAlphaBetaWeights (1, ins);
      ins[iii] = 0.875f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (2, ins);
      ins[iii] = 0.875f;
      ins[jjj] = 0.875f;
      setAlphaBetaWeights (3, ins);
      ins[iii] = 0.625f;
      ins[jjj] = -0.875f;
      setAlphaBetaWeights (4, ins);
      ins[iii] = 0.625f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (5, ins);
      ins[iii] = 0.625f;
      ins[jjj] = 0.875f;
      setAlphaBetaWeights (6, ins);
      ins[iii] = 0.375f;
      ins[jjj] = -0.875f;
      setAlphaBetaWeights (7, ins);
      ins[iii] = 0.375f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (8, ins);
      ins[iii] = 0.375f;
      ins[jjj] = 0.875f;
      setAlphaBetaWeights (9, ins);

      ins[iii] = -0.875f;
      ins[jjj] = -0.875f;
      setAlphaBetaWeights (10, ins);
      ins[iii] = -0.875f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (11, ins);
      ins[iii] = -0.875f;
      ins[jjj] = 0.875f;
      setAlphaBetaWeights (12, ins);
      ins[iii] = -0.625f;
      ins[jjj] = -0.875f;
      setAlphaBetaWeights (13, ins);
      ins[iii] = -0.625f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (14, ins);
      ins[iii] = -0.625f;
      ins[jjj] = 0.875f;
      setAlphaBetaWeights (15, ins);
      ins[iii] = -0.375f;
      ins[jjj] = -0.875f;
      setAlphaBetaWeights (16, ins);
      ins[iii] = -0.375f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (17, ins);
      ins[iii] = -0.375f;
      ins[jjj] = 0.875f;
      setAlphaBetaWeights (18, ins);
      
      
      ins[iii] = 0.25f;
      ins[jjj] = -1.0f;      
      setAlphaBetaWeights (19, ins);
      ins[iii] = 0.25f;
      ins[jjj] = -0.625f;
      setAlphaBetaWeights (20, ins);
      ins[iii] = 0.25f;
      ins[jjj] = -0.25f;
      setAlphaBetaWeights (21, ins);
      ins[iii] = 0.0f;
      ins[jjj] = -1.0f;
      setAlphaBetaWeights (22, ins);
      ins[iii] = 0.0f;
      ins[jjj] = -0.625f;
      setAlphaBetaWeights (23, ins);
      ins[iii] = 0.0f;
      ins[jjj] = -0.25f;
      setAlphaBetaWeights (24, ins);
      ins[iii] = -0.25f;
      ins[jjj] = -1.0f;
      setAlphaBetaWeights (25, ins);
      ins[iii] = -0.25f;
      ins[jjj] = -0.625f;
      setAlphaBetaWeights (26, ins);
      ins[iii] = -0.25f;
      ins[jjj] = -0.25f;
      setAlphaBetaWeights (27, ins);


      ins[iii] = 0.25f;
      ins[jjj] = 1.0f;      
      setAlphaBetaWeights (28, ins);
      ins[iii] = 0.25f;
      ins[jjj] = 0.625f;
      setAlphaBetaWeights (29, ins);
      ins[iii] = 0.25f;
      ins[jjj] = 0.25f;
      setAlphaBetaWeights (30, ins);
      ins[iii] = 0.0f;
      ins[jjj] = 1.0f;
      setAlphaBetaWeights (31, ins);
      ins[iii] = 0.0f;
      ins[jjj] = 0.625f;
      setAlphaBetaWeights (32, ins);
      ins[iii] = 0.0f;
      ins[jjj] = 0.25f;
      setAlphaBetaWeights (33, ins);
      ins[iii] = -0.25f;
      ins[jjj] = 1.0f;
      setAlphaBetaWeights (34, ins);
      ins[iii] = -0.25f;
      ins[jjj] = 0.625f;
      setAlphaBetaWeights (35, ins);
      ins[iii] = -0.25f;
      ins[jjj] = 0.25f;
      setAlphaBetaWeights (36, ins);
      
      ins[iii] = 0.125f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (37, ins);
      ins[iii] = 0.125f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (38, ins);
      ins[iii] = 0.125f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (39, ins);
      ins[iii] = 0.0f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (40, ins);
      ins[iii] = 0.0f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (41, ins);
      ins[iii] = 0.0f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (42, ins);
      ins[iii] = -0.125f;
      ins[jjj] = -0.125f;
      setAlphaBetaWeights (43, ins);
      ins[iii] = -0.125f;
      ins[jjj] = 0.0f;
      setAlphaBetaWeights (44, ins);
      ins[iii] = -0.125f;
      ins[jjj] = 0.125f;
      setAlphaBetaWeights (45, ins);

      for (iii = 1; iii <= 18; ++iii)
      {
        weight_[iii] = 5.0f;
      }
      for (iii = 19; iii <= 36; ++iii)
      {
        weight_[iii] = 5.0f;
      }
      for (iii = 37; iii <= 45; ++iii)
      {
        weight_[iii] = 5.0f;
      }
    } // if (option) ... else
  }


  template <class T1>
    void RBFClassification<T1>::writeErrorField ()
  {
/*
    ofstream result("errorfield.txt");
    double d, d2;
    int i = 0;
    result << "z2 = [";




    for (d = -1.0; d <= 1.0; d += 2.0/((double)samples-1), ++i)
    {
      j = 0;
      for (d2 = -1.0; d2 <= 1.0; d2 += 2.0/((double)samples-1), ++j)
      {
        invec[1] = d2;
        invec[2] = d;
        net.setInputs (invec);
        net.computeOutputs ();
        surf = net.getAlphaOutput (750);
        surf += net.getAlphaOutput (1500);
        result << surf << ", ";
      }
      result << ";" << endl;
    }
    result << "]" << endl;
*/
  }


  template <class T1>
    void RBFClassification<T1>::setAlphaBetaWeights (int index, double *in)
  {
    int j;

    if (index < 0 || index > interKFeatures_)
    {
      return;
    }

    setInputs (in);
    computeOutputs();

//    synapses1to2_[index][0] = (1.0f/(double)(-interJFeatures_)) + 0.5;
    synapses1to2_[index][0] = -1.0f;

    for (j = 1; j <= interJFeatures_; ++j)
    {
      synapses1to2_[index][j] = sgn(hiddenJOutputs_[j]) * (1.0f / (double)(interJFeatures_));
    } // for (j = 1; j <= interJFeatures_; ++j)
  }


  template <class T1>
    double RBFClassification<T1>::getMaxOutput (int output)
  {
    if (output > outputFeatures_ || output < 0)
    {
      return 0.0f;
    }
    return outputMaxVals_[output];
  }


  template <class T1>
    double RBFClassification<T1>::getMinOutput (int output)
  {
    if (output > outputFeatures_ || output < 0)
    {
      return 0.0f;
    }
    return outputMinVals_[output];
  }

} // NeuralNet




