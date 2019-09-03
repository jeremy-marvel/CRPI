///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        NeuralNet.cpp
//  Revision:        1.0 - 6 February, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Feed forward neural network class.
//
///////////////////////////////////////////////////////////////////////////////

#include "NeuralNet.h"
#include "../Neuron/Numerical/Numerical.h"
#include "../../Libraries/Math/VectorMath.h"
#include <fstream>
#include <cmath>

#define DEBUGOUT

using namespace std;
using namespace Math;

//! Explicit instantiations
template class LIBRARY_API NeuralNet::FeedForward<Neuron::TanhNeuron>;
template class LIBRARY_API NeuralNet::FeedForward<Neuron::SigmoidNeuron>;
template class LIBRARY_API NeuralNet::FeedForward<Neuron::LinearNeuron>;
template class LIBRARY_API NeuralNet::FeedForward<Neuron::RandBinNeuron>;
template class LIBRARY_API NeuralNet::FeedForward<Neuron::LogicNeuron>;

namespace NeuralNet
{
  template <class T> FeedForward<T>::FeedForward (int inFeat,
                                                  int interFeat,
                                                  int outFeat,
                                                  double initWeight,
                                                  double initEta) :
    inputFeatures_(inFeat),
    interFeatures_(interFeat),
    outputFeatures_(outFeat),
    initWeight_(initWeight),
    initEta_(initEta),
    pTrainingPatterns_(0),
    pTestingPatterns_(0),
    nIters_(0)
  {
    unsigned int i, j, k;

    //! Allocate memory for value passing & temporary storage
    inputs_ = new double [inputFeatures_ + 1];
    inputMaxVals_ = new double [inputFeatures_ + 1];
    inputMinVals_ = new double [inputFeatures_ + 1];

    hiddenInputs_ = new double [interFeatures_ + 1];
    hiddenOutputs_ = new double [interFeatures_ + 1];
    hiddenGPrimes_ = new double [interFeatures_ + 1];

    layer2Inputs_ = new double [outputFeatures_ + 1];
    outputs_ = new double [outputFeatures_ + 1];
    gPrimes_ = new double [outputFeatures_ + 1];
    outputMaxVals_ = new double [outputFeatures_ + 1];
    outputMinVals_ = new double [outputFeatures_ + 1];

    //! Allocate memory for the neurons
    interNeurons_ = new T [interFeatures_ + 1];
    outputNeurons_ = new T [outputFeatures_ + 1];

    //! Allocate the memory for synaptic connections from layer 0 to layer 1
    synapses0to1_ = new double * [interFeatures_ + 1];
    dEdw01_ = new double * [interFeatures_ + 1];
    dEdw01Old_ = new double * [interFeatures_ + 1];
    dw01Eta_ = new double * [interFeatures_ + 1];
    dw01_ = new double * [interFeatures_ + 1];
    for (j = 0; j <= interFeatures_; ++j)
    {
      synapses0to1_[j] = new double [inputFeatures_ + 1];
      dEdw01_[j] = new double [inputFeatures_ + 1];
      dEdw01Old_[j] = new double [inputFeatures_ + 1];
      dw01Eta_[j] = new double [inputFeatures_ + 1];
      dw01_[j] = new double [inputFeatures_ + 1];
    }


    //! Allocate the memory for synaptic connections from layer 1 to layer 2
    synapses1to2_ = new double * [outputFeatures_ + 1];
    dEdw12_ = new double * [outputFeatures_ + 1];
    dEdw12Old_ = new double * [outputFeatures_ + 1];
    dw02_ = new double * [outputFeatures_ + 1];
    dw12Eta_ = new double * [outputFeatures_ + 1];
    for (k = 0; k <= outputFeatures_; ++k)
    {
      synapses1to2_[k] = new double [interFeatures_+1];
      dEdw12_[k] = new double [interFeatures_+1];
      dEdw12Old_[k] = new double [interFeatures_+1];
      dw12Eta_[k] = new double [interFeatures_+1];
      dw02_[k] = new double [interFeatures_+1];
    }

    //! Initialize the synapses for the network by randomly assigning weights
    initializeSynapses ();

    //! Assign initial learning values
    for (j = 0; j <= interFeatures_; ++j)
    {
      dw12Eta_[0][j] = 0.0;        //! 0th term is always 1.0
      for (k = 1; k <= outputFeatures_; ++k)
      {
        dw12Eta_[k][j] = initEta_;
      }
    }
    for (i = 0; i <= inputFeatures_; ++i)
    {
      dw01Eta_[0][i] = 0.0;        //! bias term special case
      for (j = 1; j <= interFeatures_; ++j)
      {
        dw01Eta_[j][i] = initEta_;
      }
    }
  }


  template <class T> FeedForward<T>::~FeedForward ()
  {
    vector<double*>::iterator iter1, iter2;

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
  }


  template <class T> void FeedForward<T>::initialize ()
  {
    compute_dEdw01 ();
    compute_dEdw12 ();
    backupDerivs ();
    computeErrors ();
  }


  template <class T> double FeedForward<T>::trainNetwork (int iter,
                                                          bool prune)
  {
    double error = 0.0;
    for (int i = 0; i < iter; ++i)
    {
      backupDerivs ();   //! save all previously-computed dE/dw's
      compute_dEdw01 (); //! compute new derivatives
      compute_dEdw12 ();
      updateEtas ();     //! this is optional; default is do-nothing
      adjustWeights ();
      computeErrors ();  //! this computes all outputs and compares to targets
      error = RMSError_;
    }

    if (prune)
    {
      backupDerivs ();
      compute_dEdw01 ();
      compute_dEdw12 ();
      updateEtas ();
      adjustWeights ();
      computeErrorsWithDisable (); //! Remove the top 10% worse performers
      error = RMSError_;
    }

    return error;
  }


  //! JAM
  template <class T> void FeedForward<T>::runTests (int k)
  {
    char name[255];
    unsigned int p;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
    {
      return;
    }

//    cout << "running test..." << endl;
    sprintf (name, "outfile%i.m", k);
    ofstream out (name);
    ofstream out2 ("truth.csv");

    if (pTestingPatterns_ > 0)
    {
      out << "y = [1:" << pTestingPatterns_ << "];\n";
      out << "x = [";

      //! Run the validation data through the network
      for (p = 0; p < pTestingPatterns_; ++p)
      {
        setInputs (testingInputs_.at(p));
        computeOutputs ();
        out << outputs_[k] << ' ';
        out2 << outputs_[k] << ", ";
      }
      out << "];\n";
      out2 << endl;
    }

    out << "targety = [1:" << pTrainingPatterns_ << "];\n";
    out << "target = [";
    //! Run the original training data through the network
    for (p = 0; p < pTrainingPatterns_; ++p)
    {
      setInputs (trainingInputs_[p]);
      computeOutputs ();
      out << outputs_[k] << ' ';
    }
    out << "];\n";

    out << "figure(1);" << endl;
  
    if (pTestingPatterns_ > 0)
    {
      out << "plot(y, x, 'x');\n";
      out << "figure(2);" << endl;
    }
    out << "plot(targety, target, '*');\n";
    out.close();
  }


  template <class T> void FeedForward<T>::runTestsBest (int top, int k)
  {
    char name[255];
    unsigned int p, j;
    vector<double> outputs;
    vector<int> indexes;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
    {
      return;
    }

    sprintf (name, "bestPerformers%i.dat", k);
    ofstream out (name);

    for (p = 0; p < pTestingPatterns_; ++p)
    {
      setInputs (testingInputs_.at(p));
      computeOutputs ();
      outputs.push_back (outputs_[k]);
      indexes.push_back (p);
    }

    mergeSort (outputs, indexes);

    for (p = 0; p < indexes.size(); ++p)
    {
      out << p << ") ";
      for (j = 0; j < (inputFeatures_ + 1); ++j)
      {
        out << rawTestingInputs_.at(indexes.at(p))[j] << " ";
      }
      out << ": " << outputs.at(p);
      out << endl;
    }

    out.close();
  }


  template <class T> double FeedForward<T>::autoOptimize (vector<double>& minVals,
                                                          vector<double>& maxVals,
                                                          vector<double>& best,
                                                          int outWatch,
                                                          bool maximize)
  {
    vector<double> curr;
    curr.resize (minVals.size());
    int elem = 0;
    double bestout = (maximize ? -3.0f : 3.0f);

    optiHelper (minVals, maxVals, curr, best, elem, outWatch, bestout, maximize);
    return bestout;
  }


  template <class T> void FeedForward<T>::optiHelper (vector<double>& minVals,
                                                      vector<double>& maxVals,
                                                      vector<double>& current,
                                                      vector<double>& best,
                                                      unsigned int element,
                                                      int outWatch,
                                                      double& bestout,
                                                      bool maximize)
  {
    double inc = (maxVals.at(element) - minVals.at(element)) / 100.0;
    double val;
    double *testMe = NULL;
#ifdef DEBUGOUT
    static int count = 0;
    static int depth = 0;
    static char buff[8];
#endif

    if (element >= (minVals.size()-1))
    {
      testMe = new double[current.size()];
    }

    if ((maxVals.at (element) - minVals.at (element)) < 0.0001)
    {
      current.at(element) = minVals.at (element);
      if (element < (minVals.size()-1))
      {
#ifdef DEBUGOUT
        ++depth;
        optiHelper (minVals, maxVals, current, best, element+1, outWatch, bestout, maximize);
        --depth;
#else
        optiHelper (minVals, maxVals, current, best, element+1, outWatch, bestout, maximize);
#endif
      }
      else
      {
        //! Last element being tested
        scaleInput (current, testMe);

        //! Set the input to the test vector
        setInputs (testMe);
        //! Run the test
        computeOutputs ();

        if (maximize)
        {
          if (outputs_[outWatch] > bestout)
          {
            best.assign (current.begin (), current.end ());
            bestout = outputs_[outWatch];
          }
        }
        else
        {
          if (outputs_[outWatch] < bestout)
          {
            best.assign (current.begin (), current.end ());
            bestout = outputs_[outWatch];
          }
        } // if (maximize) ... else
      } // if (element < (minVals.size()-1)) ... else
    }
    else
    {
      for (val = minVals.at(element); val <= maxVals.at(element); val += inc)
      {
        current.at(element) = val;
        if (element < (minVals.size()-1))
        {
#ifdef DEBUGOUT
          if (depth == 0)
          {
            sprintf (buff, "%d\n", ++count);
            printf (buff);
          }
          ++depth;
          optiHelper (minVals, maxVals, current, best, element+1, outWatch, bestout, maximize);
          --depth;
#else
          optiHelper (minVals, maxVals, current, best, element+1, outWatch, bestout, maximize);
#endif
        }
        else
        {
          scaleInput (current, testMe);

          //! Set the input to the test vector
          setInputs (testMe);
          //! Run the test
          computeOutputs ();

          if (maximize)
          {
            if (outputs_[outWatch] > bestout)
            {
              best.assign (current.begin (), current.end ());
              bestout = outputs_[outWatch];
            }
          }
          else
          {
            if (outputs_[outWatch] < bestout)
            {
              best.assign (current.begin (), current.end ());
              bestout = outputs_[outWatch];
            }
          } // if (maximize) ... else
        } // if (element < (minVals.size()-1)) ... else
      } // for (val = minVals.at(element); val <= maxVals.at(element); val += inc)
    }
    delete [] testMe;
  }


  template <class T> void FeedForward<T>::addTrainingSet (double *in,
                                                          double *out)
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
      if (dist < 0.0000001)
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

    // TODO: Verify we haven't already added this point or one too close to it 


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


  template <class T> void FeedForward<T>::clearTrainingSets ()
  {
    vector<double*>::iterator iter;

    iter = rawTrainingInputs_.begin();
    for (; iter != rawTrainingInputs_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    rawTrainingInputs_.clear();

    iter = trainingInputs_.begin();
    for (; iter != trainingInputs_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    trainingInputs_.clear();

    iter = rawTrainingInputsBackup_.begin();
    for (; iter != rawTrainingInputsBackup_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    rawTrainingInputsBackup_.clear();

    iter = rawTrainingOutputs_.begin();
    for (; iter != rawTrainingOutputs_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    rawTrainingOutputs_.clear();

    iter = trainingOutputs_.begin();
    for (; iter != trainingOutputs_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    trainingOutputs_.clear();

    iter = rawTrainingOutputsBackup_.begin();
    for (; iter != rawTrainingOutputsBackup_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    rawTrainingOutputsBackup_.clear ();

    inputActive_.clear ();
    EVals_.clear ();

    pTrainingPatterns_ = pRawTrainingPatternsBackup_ = 0;
  }


  template <class T> void FeedForward<T>::adjustNeighborhood (int method,
                                                              unsigned int size)
  {
    vector<double> dists, ins, tests;
    vector<double*> storage1, storage2;
    vector<int> indexes;
    vector<double*>::iterator iter, iter2;
    double *tempVec1, *tempVec2, temp;
    unsigned int i, j;
    unsigned int subsize = (size < pRawTrainingPatternsBackup_) ? size : pRawTrainingPatternsBackup_;

    
    if (rawTrainingInputsBackup_.empty())
    {
      //! Cannot take a sub-population of an empty set;
      return;
    }

    //! Clear current input and output vectors
    iter = rawTrainingInputs_.begin();
    for (; iter != rawTrainingInputs_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    rawTrainingInputs_.clear();

    iter = trainingInputs_.begin();
    for (; iter != trainingInputs_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    trainingInputs_.clear();

    iter = rawTrainingOutputs_.begin();
    for (; iter != rawTrainingOutputs_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    rawTrainingOutputs_.clear();

    iter = trainingOutputs_.begin();
    for (; iter != trainingOutputs_.end(); ++iter)
    {
      delete [] *iter;
      *iter = NULL;
    }
    trainingOutputs_.clear();

    inputActive_.clear ();
    EVals_.clear ();

    //! Copy backups to the training input vectors
    iter = rawTrainingInputsBackup_.begin();
    for (; iter != rawTrainingInputsBackup_.end(); ++iter)
    {
      tempVec1 = new double[inputFeatures_ + 1];
      tempVec2 = new double[inputFeatures_ + 1];
      for (i = 0; i <= inputFeatures_; ++i)
      {
        tempVec1[i] = tempVec2[i] = (*iter)[i];
      }
      rawTrainingInputs_.push_back (tempVec1);
      trainingInputs_.push_back (tempVec2);
      inputActive_.push_back (true);
      EVals_.push_back (0.0);
      tempVec1 = tempVec2 = NULL;
    }
    
    iter = rawTrainingOutputsBackup_.begin ();
    for (; iter != rawTrainingOutputsBackup_.end(); ++iter)
    {
      tempVec1 = new double[outputFeatures_ + 1];
      tempVec2 = new double[outputFeatures_ + 1];
      for (i = 0; i <= outputFeatures_; ++i)
      {
        tempVec1[i] = tempVec2[i] = (*iter)[i];
      }
      rawTrainingOutputs_.push_back (tempVec1);
      trainingOutputs_.push_back (tempVec2);
      tempVec1 = tempVec2 = NULL;
    }

    pTrainingPatterns_ = trainingInputs_.size ();

    //! Find the minimum and maximum raw input values
    findInputMinMaxVals ();

    //! Find the minimum and maximum raw output values
    findOutputMinMaxVals ();

    //! Scale values
    scaleAllInputs ();
    scaleAllOutputs ();

    switch (method)
    {
    case NEAREST:
      /////////////////////////////////////////////////////////////////////////
      //! Use the currently-defined inputs
      for (i = 0; i <= inputFeatures_; ++i)
      {
        ins.push_back (inputs_[i]);
      }
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

      /////////////////////////////////////////////////////////////////////////
      //! Populate Inputs
      storage1.clear ();
      storage2.clear ();
      for (i = 0; i < subsize; ++i)
      {
        tempVec1 = new double[inputFeatures_ + 1];
        tempVec2 = new double[inputFeatures_ + 1];
        for (j = 0; j <= inputFeatures_; ++j)
        {
          tempVec1[j] = trainingInputs_.at(indexes.at(i))[j];
          tempVec2[j] = rawTrainingInputs_.at(indexes.at(i))[j];
        }
        storage1.push_back (tempVec1);
        storage2.push_back (tempVec2);
        tempVec1 = tempVec2 = NULL;
      }

      //! Clear current input vectors
      iter = rawTrainingInputs_.begin();
      for (; iter != rawTrainingInputs_.end(); ++iter)
      {
        delete [] *iter;
        *iter = NULL;
      }
      rawTrainingInputs_.clear();

      iter = trainingInputs_.begin();
      for (; iter != trainingInputs_.end(); ++iter)
      {
        delete [] *iter;
        *iter = NULL;
      }
      trainingInputs_.clear();

      inputActive_.clear ();
      EVals_.clear ();
      iter = storage1.begin();
      iter2 = storage2.begin();
      for (; iter != storage1.end(); ++iter, ++iter2)
      {
        trainingInputs_.push_back (*iter);
        rawTrainingInputs_.push_back (*iter2);
        *iter = *iter2 = NULL;
      }

      /////////////////////////////////////////////////////////////////////////
      //! Populate Outputs
      storage1.clear ();
      storage2.clear ();
      for (i = 0; i < subsize; ++i)
      {
        tempVec1 = new double[outputFeatures_ + 1];
        tempVec2 = new double[outputFeatures_ + 1];
        for (j = 0; j <= outputFeatures_; ++j)
        {
          tempVec1[j] = trainingOutputs_.at(indexes.at(i))[j];
          tempVec2[j] = rawTrainingOutputs_.at(indexes.at(i))[j];
        }
        storage1.push_back (tempVec1);
        storage2.push_back (tempVec2);
        tempVec1 = tempVec2 = NULL;
      }

      //! Clear current input and output vectors
      iter = rawTrainingOutputs_.begin();
      for (; iter != rawTrainingOutputs_.end(); ++iter)
      {
        delete [] *iter;
        *iter = NULL;
      }
      rawTrainingOutputs_.clear();

      iter = trainingOutputs_.begin();
      for (; iter != trainingOutputs_.end(); ++iter)
      {
        delete [] *iter;
        *iter = NULL;
      }
      trainingOutputs_.clear();

      iter = storage1.begin();
      iter2 = storage2.begin();
      for (; iter != storage1.end(); ++iter, ++iter2)
      {
        trainingOutputs_.push_back (*iter);
        rawTrainingOutputs_.push_back (*iter2);
        inputActive_.push_back (true);
        EVals_.push_back (0.0);
        *iter = *iter2 = NULL;
      }
      storage1.clear ();
      storage2.clear ();

      break;
    case RECENT:
      /////////////////////////////////////////////////////////////////////////
      //! Remove early training points until the size of the training sets
      while (pTrainingPatterns_ > subsize)
      {
        iter = trainingInputs_.begin ();
        iter2 = rawTrainingInputs_.begin ();
        trainingInputs_.erase (iter);
        rawTrainingInputs_.erase (iter2);

        inputActive_.erase (inputActive_.begin());
        EVals_.erase (EVals_.begin());

        iter = trainingOutputs_.begin();
        iter2 = rawTrainingOutputs_.begin();
        trainingOutputs_.erase (iter);
        rawTrainingOutputs_.erase (iter2);

        pTrainingPatterns_ = trainingInputs_.size ();
      }

      break;
    case FULL:
      //! Already populated, no need to do anything further
      return;
      break;
    default:
      return;
      break;
    }

    pTrainingPatterns_ = trainingInputs_.size ();

    //! Find the minimum and maximum raw input values
    findInputMinMaxVals ();

    //! Find the minimum and maximum raw output values
    findOutputMinMaxVals ();

    //! Scale values
    scaleAllInputs ();
    scaleAllOutputs ();
  }


  template <class T> void FeedForward<T>::addTestingSet (double *in)
  {
    unsigned int p;
    double *tempVec1, *tempVec2;
    
    tempVec1 = new double[inputFeatures_ + 1];
    tempVec2 = new double[inputFeatures_ + 1];
    for (p = 0; p < (inputFeatures_ + 1); ++p)
    {
      tempVec1[p] = tempVec2[p] = in[p];
    }
    rawTestingInputs_.push_back (tempVec1);
    testingInputs_.push_back (tempVec2);
    tempVec1 = tempVec2 = NULL;

    pTestingPatterns_ = rawTestingInputs_.size ();

    //! Scale values
    scaleAllInputs ();
  }


  template <class T> int FeedForward<T>::parseInputFile (const char *trainPath,
                                                         const char *testPath)
  {
    unsigned int nvals = 0, tvals=0;
    char message[256];
    double inval;
    double *tempVec1 = NULL, *tempVec2 = NULL;
    ifstream trainingFile, testingFile;
    unsigned int x, y, i;
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
          nvals = vals.size() / (inputFeatures_ + outputFeatures_);
          tempVec1 = new double[inputFeatures_+1];
          tempVec2 = new double[outputFeatures_+1];

          i = 0;
          for (y = 0; y < nvals; ++y)
          {
            tempVec1[0] = 1.0f;
            for (x = 0; x < inputFeatures_; ++x, ++i)
            {
              tempVec1[x+1] = vals.at(i);
            }

            tempVec2[0] = 1.0f;
            for (x = 0; x < outputFeatures_; ++x, ++i)
            {
              tempVec2[x+1] = vals.at(i);
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


  template <class T> bool FeedForward<T>::readNetworkFile (const char *netPath)
  {
    char message[256];
    ifstream netFile;
    int in, hidden, out;
    unsigned int x, y;

    if (netPath == NULL)
    {
      return false;
    }

    netFile.open (netPath);
    if (!netFile)
    {
      return false;
    }

    netFile >> in;
    netFile >> hidden;
    netFile >> out;

    if (in != inputFeatures_ || hidden != interFeatures_ || out != outputFeatures_)
    {
      return false;
    }

    for (x = 0; x <= inputFeatures_; ++x)
    {
      netFile >> inputMaxVals_[x];
      netFile >> inputMinVals_[x];
    }

    for (x = 0; x <= outputFeatures_; ++x)
    {
      netFile >> outputMaxVals_[x];
      netFile >> outputMinVals_[x];
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


  template <class T> bool FeedForward<T>::writeNetworkFile (const char *netPath)
  {
    char message[256];
    ofstream netFile;
    unsigned int x, y;

    if (netPath == NULL)
    {
      return false;
    }

    netFile.open (netPath);
    if (!netFile)
    {
      return false;
    }

    netFile << inputFeatures_ << " " << interFeatures_ << " " << outputFeatures_ << endl;

  for (x = 0; x <= inputFeatures_; ++x)
  {
    netFile << inputMaxVals_[x] << " " << inputMinVals_[x] << " ";
  }

  netFile << endl;

  for (x = 0; x <= outputFeatures_; ++x)
  {
    netFile << outputMaxVals_[x] << " " << outputMinVals_[x] << " ";
  }

  netFile << endl;

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


  template <class T> double FeedForward<T>::getRMSError ()
  {
    return RMSError_;
  }


  template <class T> double FeedForward<T>::getTotalError ()
  {
    return ETotal_;
  }


  template <class T> int FeedForward<T>::getNumEpochs ()
  {
    return nIters_;
  }


  template <class T> int FeedForward<T>::getNumTrainingPatterns()
  {
    return pTrainingPatterns_;
  }


  template <class T> double FeedForward<T>::getTrainingInput (unsigned int pattern,
                                                              unsigned int input)
  {
    if (pattern > pTrainingPatterns_ || input > inputFeatures_ ||
        !inputActive_.at (pattern))
    {
      return 0.0f;
    }
    return trainingInputs_.at(pattern)[input];
  }


  template <class T> double FeedForward<T>::getMaxOutput (unsigned int output)
  {
    if (output > outputFeatures_)
    {
      return 0.0f;
    }
    return outputMaxVals_[output];
  }

  template <class T> double FeedForward<T>::getMinOutput (unsigned int output)
  {
    if (output > outputFeatures_)
    {
      return 0.0f;
    }
    return outputMinVals_[output];
  }


  template <class T> double FeedForward<T>::getRawTrainingInput (unsigned int pattern,
                                                                 unsigned int input)
  {
    if (pattern > pTrainingPatterns_ || input > inputFeatures_ ||
        !inputActive_.at (pattern))
    {
      return 0.0f;
    }
    return rawTrainingInputs_.at(pattern)[input];
  }


  template <class T> double FeedForward<T>::getTrainingOutput (unsigned int pattern,
                                                               unsigned int output)
  {
    if (pattern > pTrainingPatterns_ || output > outputFeatures_ ||
        !inputActive_.at (pattern))
    {
      return 0.0f;
    }
    return trainingOutputs_.at(pattern)[output];
  }


  template <class T> double FeedForward<T>::getRawTrainingOutput (unsigned int pattern,
                                                                  unsigned int output)
  {
    if (pattern > pTrainingPatterns_ || output > outputFeatures_)
    {
      return -1.0f;
    }
    return rawTrainingOutputs_.at(pattern)[output];
  }


  template <class T> int FeedForward<T>::getNumInputs ()
  {
    return inputFeatures_;
  }


  template <class T> int FeedForward<T>::getNumOutputs ()
  {
    return outputFeatures_;
  }


  template <class T> void FeedForward<T>::setInputs (double *inputVec)
  {
    inputs_[0] = 1.0f; // bias term
    for (unsigned int i = 1; i <= inputFeatures_; ++i)
    {
      inputs_[i] = inputVec[i];
    }
  }


  template <class T> void FeedForward<T>::computeGPrimes ()
  {
    unsigned int j, k;
    //! Start with the interneuron layer:
    hiddenGPrimes_[0] = 1.0; // bias term for output layer
 
    for (j = 1; j <= interFeatures_; ++j)
    {
      // assume hidden_inputs[j] have already been computed;
      hiddenGPrimes_[j] = interNeurons_[j].derivative (hiddenInputs_[j]);
    }

    // repeat for the output layer, 
    gPrimes_[0] = 0.0;

    // assumes layer-k inputs have already been computed
    for (k = 1; k <= outputFeatures_; ++k) // this loop executes only one iteration for scalar output
    {
      gPrimes_[k] = outputNeurons_[k].derivative (layer2Inputs_[k]);
    }
  }


  template <class T> void FeedForward<T>::computeErrors ()
  {
    unsigned int p, k;
    ETotal_ = 0.0f;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
    {
      return;
    }

    for (p = 0; p < pTrainingPatterns_; ++p)
    {
      if (inputActive_.at (p))
      {
        setInputs (trainingInputs_.at(p));
        computeOutputs ();

        EVals_.at(p) = 0.0f;
        for (k = 1; k <= outputFeatures_; ++k)
        {
          EVals_.at(p) += (outputs_[k] - trainingOutputs_.at(p)[k]) *
                          (outputs_[k] - trainingOutputs_.at(p)[k]);
        }

        EVals_.at(p) *= 0.5f;
        ETotal_ += EVals_.at(p);
      }
    }

    RMSError_ = sqrt (ETotal_ / pTrainingPatterns_);
  }


  template <class T> void FeedForward<T>::computeErrorsWithDisable ()
  {
    unsigned int p, k;
    ETotal_ = 0.0;
    vector<double> outs;
    vector<double>::iterator outIter;
    vector<int> indexes;
    int active = 0, removed = 0;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
    {
      return;
    }

    for (p = 0; p < pTrainingPatterns_; ++p)
    {
      if (inputActive_.at (p))
      {
        setInputs (trainingInputs_.at(p));
        computeOutputs ();

        EVals_.at(p) = 0.0;
        for (k = 1; k <= outputFeatures_; ++k)
        {
          EVals_.at(p) += (outputs_[k] - trainingOutputs_.at(p)[k]) *
                          (outputs_[k] - trainingOutputs_.at(p)[k]);
        }

        EVals_.at(p) *= 0.5;
        ++active;
      }
      else
      {
        EVals_.at(p) = -1.0;
      }
    }

    outIter = EVals_.begin ();
    for (; outIter != EVals_.end (); ++outIter)
    {
      outs.push_back (*outIter);
    }

    indexes.resize (outs.size());
    mergeSort (outs, indexes);

    p = outs.size()-1;
    if (active > 0.0)
    {
      while (((double)removed/active) <= 0.10)
      {
        inputActive_.at(indexes.at(p)) = false;
        ++removed;
        --p;
      }
    }


    int z1 = outs.size();
    int z2 = EVals_.size();
    //for (p = pTrainingPatterns_-1; p >= 0; --p)
    for (; p >= 0; --p)
    {
      if (inputActive_.at(indexes.at(p)))
      {
        ETotal_ += outs.at (p);
      }
    }

    //RMSError_ = sqrt (ETotal_ / pTrainingPatterns_);
    RMSError_ = sqrt (ETotal_ / active);
  }


  template <class T> double FeedForward<T>::dEdwSumSqr ()
  {
    unsigned int i, j, k;
    double sum = 0.0f;

    for (k = 1; k <= outputFeatures_; ++k)
    {
      for (j = 0; j <= interFeatures_; ++j)
      {
        sum += (dEdw12_[k][j] * dEdw12_[k][j]);
      }
    }
    for (j = 1; j <= interFeatures_; ++j)
    {
      for (i = 0; i <= inputFeatures_; ++i)
      {
        sum += (dEdw01_[j][i] * dEdw01_[j][i]);
      }
    }
    return sum;
  }


  template <class T> void FeedForward<T>::findInputMinMaxVals ()
  {
    unsigned int i, p;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
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
        if (rawTrainingInputsBackup_.at(p)[i] > inputMaxVals_[i] && inputActive_.at (p))
        {
          inputMaxVals_[i] = rawTrainingInputsBackup_.at(p)[i];
        }
        if (rawTrainingInputsBackup_.at(p)[i] < inputMinVals_[i] && inputActive_.at (p))
        {
          inputMinVals_[i] = rawTrainingInputsBackup_.at(p)[i];
        }
      }
    } // for (i = 0; i <= inputFeatures_; ++i)
  }


  template <class T> void FeedForward<T>::findOutputMinMaxVals ()
  {
    unsigned int i, p;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
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
        if (rawTrainingOutputsBackup_.at(p)[i] > outputMaxVals_[i] && inputActive_.at (p))
        {
          outputMaxVals_[i] = rawTrainingOutputsBackup_.at(p)[i];
        }
        if (rawTrainingOutputsBackup_.at(p)[i] < outputMinVals_[i] && inputActive_.at (p))
        {
          outputMinVals_[i] = rawTrainingOutputsBackup_.at(p)[i];
        }
      }
    } // for (i = 0; i <= outputFeatures_; ++i)
  }


  template <class T> void FeedForward<T>::getRawRange (vector<double>& minInVals,
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


  template <class T> void FeedForward<T>::scaleInput (vector<double> &rawVec,
                                                      double *scaledVec)
  {
    unsigned int i;
    double range;

    scaledVec[0] = 1.0f;
    for (i = 1; i <= inputFeatures_; ++i)
    {
      range = inputMaxVals_[i] - inputMinVals_[i];
      if (range == 0.0)
      {
        scaledVec[i] = 1.0;
      }
      else
      {
        scaledVec[i] = -1.0 + 2.0 * (rawVec.at (i) - inputMinVals_[i]) / range;
      }
    }
  }


  template <class T> void FeedForward<T>::scaleAllInputs ()
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
      for (i = 1; i <= inputFeatures_; ++i)
      {
        range = inputMaxVals_[i] - inputMinVals_[i];
        if (range == 0.0)
        {
          trainingInputs_[p][i] = 1.0f;
        }
        else
        {
          trainingInputs_[p][i] = -1.0f + 2.0f * (rawTrainingInputs_[p][i] -
                                                  inputMinVals_[i]) / range;
        }
      }
    }

    //! Scale testing patterns
    if (pTestingPatterns_ > 0)
    {
      for (p = 0; p < pTestingPatterns_; ++p)
      {
        testingInputs_[p][0] = 1.0; // bias term is special case
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


  template <class T> void FeedForward<T>::scaleAllOutputs ()
  {
    unsigned int p, k;
    double range;

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
        range = outputMaxVals_[k] - outputMinVals_[k];
        if (range <= 0.00001f)
        {
          trainingOutputs_.at(p)[k] = 0.0f;
        }
        else
        {
          trainingOutputs_.at(p)[k] =
            (rawTrainingOutputs_.at(p)[k] - outputMinVals_[k]) / range;
        }
      }
    }
  }


  // copy elements of dEdw01_ to dEdw01Old_
  // and dEdw12_ to dEdw12Old_
  // may test changes in sign of derivatives to determine
  // how to vary learning params
  template <class T> void FeedForward<T>::backupDerivs ()
  {
    unsigned int i, j, k;
    for (i = 0; i <= inputFeatures_; ++i)
    {
      for (j = 0; j <= interFeatures_; ++j)
      {
        dEdw01Old_[j][i] = dEdw01_[j][i];
      }
    }
    for (j = 0; j <= interFeatures_; ++j)
    {
      for (k = 0; k <= outputFeatures_; ++k)
      {
        dEdw12Old_[k][j] = dEdw12_[k][j];
      }
    }
  }


  template <class T> void FeedForward<T>::initializeSynapses ()
  {
    unsigned int i, j, k;
    //! Initialize the weights to random values between -sqrt(3/|I|)
    //! and sqrt(3/|I|)
    double randomWeight = 0.0,
           baseWeight = -sqrt(3.0/(double)inputFeatures_),
           weightInc = (-2.0 * baseWeight) / 100.0;

    //! Weights to "0" node are all irrelevant, since 0 interneuron
    //! is a bias term that always outputs unity
    for(i = 0; i <= inputFeatures_; ++i)
    {
      synapses0to1_[0][i] = 0.0;
    }

    //! Give the weights random values
    for (i = 0; i <= inputFeatures_;++i)
    {
      for (j = 1; j <= interFeatures_; ++j)
      {
        randomWeight = ((double)(rand() % 100) * weightInc) + baseWeight;
        synapses0to1_[j][i] = randomWeight;
      }
    }

    //! Similarly for synapses btwn interneurons and output neurons weights 
    //! to "0" node are all irrelevant, since 0 output neuron is a bias term
    //! that always outputs unity
    for (j = 0; j <= interFeatures_; ++j)
    {
      synapses1to2_[0][j] = 0.0;
    }

    //! Like the weights between the i and j neurons, here we initialize the
    //! weights to random values between -sqrt(3/|J|) and sqrt(3/|J|)
    baseWeight = -sqrt(3.0/(double)interFeatures_);
    weightInc = (-2.0 * baseWeight) / 100.0;
    for (j = 0; j <= interFeatures_; ++j)
    {
      for (k = 1; k <= outputFeatures_; ++k)
      {
        randomWeight = ((double)(rand() % 100) * weightInc) + baseWeight;
        synapses1to2_[k][j] = randomWeight;
      }
    }
  }


  template <class T> void FeedForward<T>::computeOutputs ()
  {
    unsigned int i, j, k;

    //! Compute net input for each interneuron
    //! Store intermediate net values in hidden_inputs[]
    hiddenOutputs_[0] = 1.0f; // bias term for output layer
    hiddenInputs_[0] = 0.0f;
    for(j = 1; j <= interFeatures_; ++j)
    {
      //! Compute weighted sum of  pattern inputs:
      hiddenInputs_[j] = 0.0f;
      //! i = 0 : include inputs[0] bias contribution
      for(i = 0; i <= inputFeatures_; ++i)
      {
        hiddenInputs_[j] += inputs_[i] * synapses0to1_[j][i];
      }

      //! Use net input to each interneuron to compute respective outputs
      interNeurons_[j].activate (hiddenInputs_[j]);
      hiddenOutputs_[j] = interNeurons_[j].output ();
    }

    //! Compute net input for each output neuron using outputs of interneurons
    layer2Inputs_[0] = 0.0f;
    outputs_[0] = 0.0f;
    //! If outputFeatures_ == 1, scalar output
    for(k = 1; k <= outputFeatures_; ++k)
    {
      //! Compute weighted sum of pattern inputs:
      layer2Inputs_[k] = 0.0;
      //! j = 0 : include hidden_output[0] bias contribution
      for(j = 0; j <= interFeatures_; ++j)
      {
        layer2Inputs_[k] += hiddenOutputs_[j] * synapses1to2_[k][j];
      }
      //! Use "net" input to each output neuron to compute respective outputs
      outputNeurons_[k].activate (layer2Inputs_[k]);
      outputs_[k] = outputNeurons_[k].output ();
    }
  }


  template <class T> double FeedForward<T>::getOutput (unsigned int index)
  {
    double scaledout;
    if (index > outputFeatures_)
    {
      scaledout = -1.0f;
    }
    else
    {
      scaledout = (outputs_[index] * (outputMaxVals_[index] - outputMinVals_[index])) +
                  outputMinVals_[index];
//    scaledout = -1.0f * ((((outputs_[index] - 1.0f) / -2.0f) * (outputMaxVals_[index] - outputMinVals_[index])) - outputMaxVals_[index]);
    }
    return scaledout;
    
    //return (index > outputFeatures_) ? -1.0f : outputs_[index];
  }


  template <class T> void FeedForward<T>::compute_dEdw01 ()
  {
    unsigned int i, j, k, p;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
    {
      return;
    }

    //! clear the entire derivative matrix
    for(j = 0; j <= interFeatures_; ++j)
    {
      for(i = 0; i <= inputFeatures_; ++i)
      {
        dEdw01_[j][i] = 0.0f;
      }
    }

    //! dE/dw_j,i = SUM_k SUM_p (o_k(p) - t_k(p)) * 
    //!                          g'(u_k(p)) * 
    //!                          w_k,j * 
    //!                          g'(u_j(p)) * 
    //!                          o_i(p)
    //!
    for (p = 0; p < pTrainingPatterns_; ++p)
    {
      //! evaluate all nodal outputs due to stimulation w/ p'th input pattern
      setInputs (trainingInputs_.at(p));
      computeOutputs ();
      computeGPrimes ();

      for(k = 0; k <= outputFeatures_; ++k)
      {
        for(j = 1; j <= interFeatures_; ++j)
        {
          for(i = 0; i <= inputFeatures_; ++i)
          {
            dEdw01_[j][i] += (outputs_[k] - trainingOutputs_.at(p)[k]) * 
                              gPrimes_[k] *
                              synapses1to2_[k][j] *
                              hiddenGPrimes_[j] *
                              inputs_[i];
          } // for (i = 0; i <= inputFeatures_; ++i)
        } // for (j = 1; j <= interFeatures_; ++j)
      } // for (k = 0; k <= outputFeatures_; ++k)
    } // for (p = 0; p < P_training_patterns; ++p)
  }


  template <class T> void FeedForward<T>::compute_dEdw12 ()
  {
    //! dout_k/dwkj = dout/din * din/dwkj
    //!             = ok*(1-ok)*oj
    //! for pattern p, output k, dEsqd/dwkj = (ok-tk)*ok*(1-ok)*oj
    //! sum over output nodes "k" (simple if one output)
    //! sum over patterns "p"
    unsigned int k, j;

    //! Error: no training patterns have been added
    if (pTrainingPatterns_ <= 0)
    {
      return;
    }

    //! Clear the array of derivatives over all output neurons
    for (k = 0; k <= outputFeatures_; ++k)
    {
       for (j = 0; j <= interFeatures_; ++j)
       {
          dEdw12_[k][j] = 0;
       }
    }

    for (unsigned int p = 0; p < pTrainingPatterns_; ++p)
    {
      //! Evaluate all nodal outputs due to stimulation w/ p'th input pattern
      setInputs (trainingInputs_.at(p));
      computeOutputs ();
      computeGPrimes ();
      //! Compute change in sqd error for this pattern due to perturbation of
      //! weight k,j;
      //! Accumulate respective contributions over all patterns p
      for (k = 1; k <= outputFeatures_; ++k)
      {
        for (j = 0; j <= interFeatures_; ++j)
        {
          //! dEdw12 += (output error for k) * (output_k / du_k) * (du_k / dw_kj)
          dEdw12_[k][j] += (outputs_[k] - trainingOutputs_.at(p)[k]) *  //! output error
                           gPrimes_[k] *                                //! o_k/du_k
                           hiddenOutputs_[j];                           //! du_k/dw_kj
        }
      }
    } // for (int p = 0; p < pTrainingPatterns_; ++p)
  }


  //! @brief Adjust the input weights for the hidden and output layer neurons
  //!
  template <class T> void FeedForward<T>::adjustWeights ()
  {
    unsigned int i, j, k;

    ++nIters_;

    for(k = 1; k <= outputFeatures_; ++k)
    {
      for(j = 0; j <= interFeatures_; ++j)
      {
        synapses1to2_[k][j] -= (dw12Eta_[k][j] * dEdw12_[k][j]);
      }
    }

    for(j = 1; j <= interFeatures_; ++j)
    {
      for(i = 0; i <= inputFeatures_; ++i)
      {
        synapses0to1_[j][i] -= (dw01Eta_[j][i] * dEdw01_[j][i]);
      }
    }
  }


  //! @brief Adjust the learning factors for each of the weights
  //!
  template <class T> void FeedForward<T>::updateEtas ()
  {
    unsigned int i, j, k;
    double testSign = 0.0f;
    const double etaIncrease = 1.2f;

    for (i = 0; i <= inputFeatures_; ++i)
    {
      for (j = 1; j <= interFeatures_; ++j)
      {
        testSign = (dEdw01Old_[j][i] * dEdw01_[j][i]);
        if(testSign > 0)
        {
          //! Increase the learning rate if we're consistently moving in the
          //! same direction
          dw01Eta_[j][i] *= etaIncrease;
        }
        else
        {
          //! Revert to the inital value - back off the learning rate
          dw01Eta_[j][i] = initEta_;
        }
      } // for (j = 1; j <= interFeatures_; ++j)
    } // for (i = 0; i <= inputFeatures_; ++i)

    for (j = 0; j <= interFeatures_; ++j)
    {
      for (k = 1; k <= outputFeatures_; ++k)
      {
        testSign = (dEdw12Old_[k][j] * dEdw12_[k][j]);
        if(testSign > 0)
        {
          //! Increase the learning rate if we're consistently moving in the
          //! same direction
          dw12Eta_[k][j] *= etaIncrease;
        }
        else
        {
          //! Revert to the initial value - back off the learning rate
          dw12Eta_[k][j] = initEta_;
        }
      } // for (k = 1; k <= outputFeatures_; ++k)
    } // for (j = 0; j <= interFeatures_; ++j)
  }

} // NeuralNet
