///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        Models.h
//  Revision:        1.0 - 23 March, 2009   : Separated simulators from
//                                            FeedForward class header file
//                                            Added numerical and analytical
//                                            tangent simulators
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Neural network declarations for Feed-Forward and simulator networks.
//  Currently, can only use numerical neuron types
//
///////////////////////////////////////////////////////////////////////////////

#ifndef MODELS_H
#define MODELS_H

#include "NetCommon.h"
#include "../../portable.h"
#include <vector>

using namespace std;

namespace NeuralNet
{
  //! @ingroup NeuralNet
  //!
  //! @brief Neural network "black box" placeholder that simply returns the output of
  //!        an N-dimensional Gaussian function.  While it has the same functions and
  //!        interface as the Feedforward class (above), it cannot be used as a
  //!        function approximator.
  //!
  template <class T> class LIBRARY_API Blackbox
  {
  public:

    enum noise {NOISELESS = 0, GAUSSIAN = 1, NORMAL = 2};

    //! @brief Default constructor
    //!
    //! @param inFeat     The number of input features for the network
    //! @param interFeat  (ignored)
    //! @param outFeat    (ignored)
    //! @param initWeight (ignored)
    //! @param initEta    (ignored)
    //!
    Blackbox (int inFeat,
              int interFeat,
              int outFeat,
              double initWeight = 1.0,
              double initEta = 0.001);

    //! @brief Default destructor
    //!
    ~Blackbox ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    void initialize ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param iter  (ignored)
    //! @param noisy (ignored)
    //!
    //! @return The resulting RMS error (0 since this is a simulator)
    //!
    double trainNetwork (int iter, bool prune = false);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param k (ignored)
    //!  
    void runTests (int k);


    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param top (ignored)
    //! @param k   (ignored)
    //!
    void runTestsBest (int top, int k);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param minVals  (ignored)
    //! @param maxVals  (ignored)
    //! @param best     (ignored)
    //! @param outWatch (ignored)
    //! @param maximize (ignored)
    //!
    //! @return 0
    //!
    double autoOptimize (vector<double>& minVals,
                         vector<double>& maxVals,
                         vector<double>& best,
                         int outWatch,
                         bool maximize = true);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param trainPath (ignored)
    //! @param testPath (ignored)
    //!
    //! @return 1
    //!
    int parseInputFile (const char *trainPath = NULL,
                        const char *testPath = NULL);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param netPath (ignored)
    //!
    //! @return True
    //!
    bool readNetworkFile (const char *netPath);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param netPath (ignored)
    //!
    //! @return True
    //!
    bool writeNetworkFile (const char *netPath);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param in  (ignored)
    //! @param out (ignored)
    //!
    void addTrainingSet (double *in, double *out);

    //! @brief TODO
    //!
    void clearTrainingSets ();

    //! @brief Reduce the applicable training set population 
    //!
    //! @param method The method for generating the neighborhood subset, based
    //!               on the enumerated types found in NetCommon.h
    //! @param size   The size of the neighborhood subset to create
    //!
    void adjustNeighborhood (int method, int size = 0);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param in (ignored)
    //!
    void addTestingSet (double *in);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param rawVec    (ignored)
    //! @param scaledVec (ignored)
    //!
    void scaleInput (vector<double> &rawVec, double *scaledVec);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param minInVals  (ignored)
    //! @param maxInVals  (ignored)
    //!
    void getRawRange (vector<double>& minInVals,
                      vector<double>& maxInVals);

    //! @brief Stimulate the network with an input vector
    //!
    //! @param inputVec TODO
    //!
    //! @note: Must be called before computeOutputs() is called
    //!
    void setInputs (double *inputVec);

    //! @brief Compute the output value of the gaussian computed by the input vector
    //!
    void computeOutputs ();

    //! @brief Report the value of the gaussian for the specified input vector
    //!
    //! @param index (ignored)
    //!
    //! @return The value of output node 1
    //!
    double getOutput (int index);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    void updateEtas ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    double getTotalError ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    double getRMSError ();
    
    //! @brief TODO
    //!
    //! @return TODO
    //!
    int getNumInputs ();

    //! @brief TODO
    //!
    //! @return 1
    //!
    int getNumOutputs ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    int getNumEpochs ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param display (ignored)
    //!
//    void snapshot (NNDisplayAdapter &display);

    void useNoise (noise n);

  private:

    //! @brief The number of training patterns input into the system
    //!
    int pTrainingPatterns_;

    //! @brief The number of testing patterns input into the system
    //!
    int pTestingPatterns_;

    //! @brief The number of input features for the network
    //!
    int inputFeatures_;

    //! @brief The input vector for current computations
    //!
    double *inputs_;

    //! @brief The center vector for computing the gaussian
    //!
    double *center_;

    //! @brief The spread of the gaussian curve
    //!
    double *sigma_;

    //! @brief Output of the gaussian function
    //!
    double output_;

    //! @brief Output of the gaussian function minus any noise
    //!
    double actual_;

    //! @brief Compute the Gaussian curve centered at PI based on the inputs
    //!        set by the user using setInputs
    //!
    //! @return The value of the Gaussian equation
    //!
    double Gaussian ();

    //! @brief Compute the dual-Gaussian curves centered at PI and 6.0 based
    //!        on the inputs set by the user using setInputs
    //!
    //! @return The value of the double-Gaussian equation
    //!
    double DoubleGaussian ();

    //! @brief Compute the output for a mesa model (only a very narrow band
    //!        for each input actually results in an answer > 0)
    //!
    //! @return The value of the mesa equation
    //!
    double Mesa ();

    //! @brief Compute the output for a plain of noise
    //!
    //! @return A random value
    //!
    double PureNoise ();

    //! @brief Compute the output for a flat plain with very small features
    //!
    //! @return The value of the plain equation
    //!
    double BumpyPlain ();

    //! @brief Compute the output for a model for which only a small number of
    //!        inputs actually result in an output
    //!
    //! @return The value of the single (or few) variable equation
    //!
    double SingleVariable ();

    /*
    TODO:  INSERT ALTERNATIVE BLACKBOX MODELS HERE
    */

    //! @brief TODO
    //!
    noise noise_;

  }; // BlackBox


  //! @ingroup NeuralNet
  //!
  //! @brief TODO
  //!
  template <class T> class LIBRARY_API AnalyticTangent
  {
  public:

    //! @brief Default constructor
    //!
    //! @param inFeat     The number of input features for the network
    //! @param interFeat  (ignored)
    //! @param outFeat    (ignored)
    //! @param initWeight (ignored)
    //! @param initEta    (ignored)
    //!
    AnalyticTangent (int inFeat,
                     int interFeat,
                     int outFeat,
                     double initWeight = 1.0,
                     double initEta = 0.001);

    //! @brief Default destructor
    //!
    ~AnalyticTangent ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    void initialize ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param iter  (ignored)
    //! @param noisy (ignored)
    //!
    //! @return The resulting RMS error (0 since this is a simulator)
    //!
    double trainNetwork (int iter, bool prune = false);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param k (ignored)
    //!  
    void runTests (int k);


    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param top (ignored)
    //! @param k   (ignored)
    //!
    void runTestsBest (int top, int k);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param minVals  (ignored)
    //! @param maxVals  (ignored)
    //! @param best     (ignored)
    //! @param outWatch (ignored)
    //! @param maximize (ignored)
    //!
    //! @return 0
    //!
    double autoOptimize (vector<double>& minVals,
                         vector<double>& maxVals,
                         vector<double>& best,
                         int outWatch,
                         bool maximize = true);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param trainPath (ignored)
    //! @param testPath (ignored)
    //!
    //! @return 1
    //!
    int parseInputFile (const char *trainPath = NULL,
                        const char *testPath = NULL);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param netPath (ignored)
    //!
    //! @return True
    //!
    bool readNetworkFile (const char *netPath);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param netPath (ignored)
    //!
    //! @return True
    //!
    bool writeNetworkFile (const char *netPath);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param in  (ignored)
    //! @param out (ignored)
    //!
    void addTrainingSet (double *in, double *out);

    //! @brief TODO
    //!
    void clearTrainingSets ();

    //! @brief Reduce the applicable training set population 
    //!
    //! @param method The method for generating the neighborhood subset, based
    //!               on the enumerated types found in NetCommon.h
    //! @param size   The size of the neighborhood subset to create
    //!
    void adjustNeighborhood (int method, int size = 0);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param in (ignored)
    //!
    void addTestingSet (double *in);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param rawVec    (ignored)
    //! @param scaledVec (ignored)
    //!
    void scaleInput (vector<double> &rawVec, double *scaledVec);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param minInVals  (ignored)
    //! @param maxInVals  (ignored)
    //!
    void getRawRange (vector<double>& minInVals,
                      vector<double>& maxInVals);

    //! @brief Stimulate the network with an input vector
    //!
    //! @param inputVec TODO
    //!
    //! @note: Must be called before computeOutputs() is called
    //!
    void setInputs (double *inputVec);

    //! @brief Compute the output value of the gaussian computed by the input vector
    //!
    void computeOutputs ();

    //! @brief Report the value of the gaussian for the specified input vector
    //!
    //! @param index (ignored)
    //!
    //! @return The value of output node 1
    //!
    double getOutput (int index);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    double getTotalError ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    double getRMSError ();
    
    //! @brief TODO
    //!
    //! @return TODO
    //!
    int getNumInputs ();

    //! @brief TODO
    //!
    //! @return 1
    //!
    int getNumOutputs ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    int getNumEpochs ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param display (ignored)
    //!
//    void snapshot (NNDisplayAdapter &display);

  private:

    //! @brief The number of training patterns input into the system
    //!
    int pTrainingPatterns_;

    //! @brief The number of testing patterns input into the system
    //!
    int pTestingPatterns_;

    //! @brief The number of input features for the network
    //!
    int inputFeatures_;

    //! @brief The number of output features for the network
    //!
    int outputFeatures_;

    //! @brief The input vector for current computations
    //!
    double *inputs_;

    //! @brief The center vector for computing the gaussian
    //!
    double *center_;

    //! @brief The spread of the gaussian curve
    //!
    double *sigma_;

    //! @brief Input training data vectors
    //!
    vector<double *> trainingInputs_;
    
    //! @brief Input training data vectors
    //!
    vector<double *> trainingOutputs_;

    //! @brief Output of the gaussian function
    //!
    double output_;

  }; // Analytic Tangent


  //! @ingroup NeuralNet
  //!
  //! @brief TODO
  //!
  template <class T> class LIBRARY_API NumericTangent
  {
  public:

    //! @brief Default constructor
    //!
    //! @param inFeat     The number of input features for the network
    //! @param interFeat  (ignored)
    //! @param outFeat    (ignored)
    //! @param initWeight (ignored)
    //! @param initEta    (ignored)
    //!
    NumericTangent (int inFeat,
                    int interFeat,
                    int outFeat,
                    double initWeight = 1.0,
                    double initEta = 0.001);

    //! @brief Default destructor
    //!
    ~NumericTangent ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    void initialize ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param iter  (ignored)
    //! @param noisy (ignored)
    //!
    //! @return The resulting RMS error (0 since this is a simulator)
    //!
    double trainNetwork (int iter, bool prune = false);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param k (ignored)
    //!  
    void runTests (int k);


    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param top (ignored)
    //! @param k   (ignored)
    //!
    void runTestsBest (int top, int k);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param minVals  (ignored)
    //! @param maxVals  (ignored)
    //! @param best     (ignored)
    //! @param outWatch (ignored)
    //! @param maximize (ignored)
    //!
    //! @return 0
    //!
    double autoOptimize (vector<double>& minVals,
                         vector<double>& maxVals,
                         vector<double>& best,
                         int outWatch,
                         bool maximize = true);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param trainPath (ignored)
    //! @param testPath (ignored)
    //!
    //! @return 1
    //!
    int parseInputFile (const char *trainPath = NULL,
                        const char *testPath = NULL);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param netPath (ignored)
    //!
    //! @return True
    //!
    bool readNetworkFile (const char *netPath);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param netPath (ignored)
    //!
    //! @return True
    //!
    bool writeNetworkFile (const char *netPath);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param in  (ignored)
    //! @param out (ignored)
    //!
    void addTrainingSet (double *in, double *out);

    //! @brief TODO
    //!
    void clearTrainingSets ();

    //! @brief Reduce the applicable training set population 
    //!
    //! @param method The method for generating the neighborhood subset, based
    //!               on the enumerated types found in NetCommon.h
    //! @param size   The size of the neighborhood subset to create
    //!
    void adjustNeighborhood (int method, int size = 0);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param in (ignored)
    //!
    void addTestingSet (double *in);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param rawVec    (ignored)
    //! @param scaledVec (ignored)
    //!
    void scaleInput (vector<double> &rawVec, double *scaledVec);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param minInVals  (ignored)
    //! @param maxInVals  (ignored)
    //!
    void getRawRange (vector<double>& minInVals,
                      vector<double>& maxInVals);

    //! @brief Stimulate the network with an input vector
    //!
    //! @param inputVec TODO
    //!
    //! @note: Must be called before computeOutputs() is called
    //!
    void setInputs (double *inputVec);

    //! @brief Compute the output value of the gaussian computed by the input vector
    //!
    void computeOutputs ();

    //! @brief Report the value of the gaussian for the specified input vector
    //!
    //! @param index (ignored)
    //!
    //! @return The value of output node 1
    //!
    double getOutput (int index);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    double getTotalError ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    double getRMSError ();
    
    //! @brief TODO
    //!
    //! @return TODO
    //!
    int getNumInputs ();

    //! @brief TODO
    //!
    //! @return 1
    //!
    int getNumOutputs ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    int getNumEpochs ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param display (ignored)
    //!
//    void snapshot (NNDisplayAdapter &display);

  private:

    //! @brief The number of training patterns input into the system
    //!
    int pTrainingPatterns_;

    //! @brief The number of testing patterns input into the system
    //!
    int pTestingPatterns_;

    //! @brief The number of input features for the network
    //!
    int inputFeatures_;

    //! @brief The number of output features for the network
    //!
    int outputFeatures_;

    //! @brief The input vector for current computations
    //!
    double *inputs_;

    //! @brief The center vector for computing the gaussian
    //!
    double *center_;

    //! @brief Input training data vectors
    //!
    vector<double *> trainingInputs_;

    //! @brief Input training data vectors
    //!
    vector<double *> trainingOutputs_;

    //! @brief The spread of the gaussian curve
    //!
    double *sigma_;

    //! @brief Output of the gaussian function
    //!
    double output_;

  }; // Numeric Tangent


  //! @ingroup NeuralNet
  //!
  //! @brief TODO
  //!
  template <class T> class LIBRARY_API NaySayer
  {
  public:

    //! @brief Default constructor
    //!
    //! @param inFeat     The number of input features for the network
    //! @param interFeat  (ignored)
    //! @param outFeat    (ignored)
    //! @param initWeight (ignored)
    //! @param initEta    (ignored)
    //!
    NaySayer (int inFeat,
              int interFeat,
              int outFeat,
              double initWeight = 1.0,
              double initEta = 0.001);

    //! @brief Default destructor
    //!
    ~NaySayer ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    void initialize ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param iter  (ignored)
    //! @param noisy (ignored)
    //!
    //! @return The resulting RMS error (0 since this is a simulator)
    //!
    double trainNetwork (int iter, bool prune = false);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param k (ignored)
    //!  
    void runTests (int k);


    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param top (ignored)
    //! @param k   (ignored)
    //!
    void runTestsBest (int top, int k);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param minVals  (ignored)
    //! @param maxVals  (ignored)
    //! @param best     (ignored)
    //! @param outWatch (ignored)
    //! @param maximize (ignored)
    //!
    //! @return 0
    //!
    double autoOptimize (vector<double>& minVals,
                         vector<double>& maxVals,
                         vector<double>& best,
                         int outWatch,
                         bool maximize = true);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param trainPath (ignored)
    //! @param testPath (ignored)
    //!
    //! @return 1
    //!
    int parseInputFile (const char *trainPath = NULL,
                        const char *testPath = NULL);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param netPath (ignored)
    //!
    //! @return True
    //!
    bool readNetworkFile (const char *netPath);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param netPath (ignored)
    //!
    //! @return True
    //!
    bool writeNetworkFile (const char *netPath);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param in  (ignored)
    //! @param out (ignored)
    //!
    void addTrainingSet (double *in, double *out);

    //! @brief TODO
    //!
    void clearTrainingSets ();

    //! @brief Reduce the applicable training set population 
    //!
    //! @param method The method for generating the neighborhood subset, based
    //!               on the enumerated types found in NetCommon.h
    //! @param size   The size of the neighborhood subset to create
    //!
    void adjustNeighborhood (int method, int size = 0);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param in (ignored)
    //!
    void addTestingSet (double *in);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param rawVec    (ignored)
    //! @param scaledVec (ignored)
    //!
    void scaleInput (vector<double> &rawVec, double *scaledVec);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param minInVals  (ignored)
    //! @param maxInVals  (ignored)
    //!
    void getRawRange (vector<double>& minInVals,
                      vector<double>& maxInVals);

    //! @brief Stimulate the network with an input vector
    //!
    //! @param inputVec TODO
    //!
    //! @note: Must be called before computeOutputs() is called
    //!
    void setInputs (double *inputVec);

    //! @brief Compute the output value of the gaussian computed by the input vector
    //!
    void computeOutputs ();

    //! @brief Report the value of the gaussian for the specified input vector
    //!
    //! @param index (ignored)
    //!
    //! @return The value of output node 1
    //!
    double getOutput (int index);

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    double getTotalError ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    double getRMSError ();
    
    //! @brief TODO
    //!
    //! @return TODO
    //!
    int getNumInputs ();

    //! @brief TODO
    //!
    //! @return 1
    //!
    int getNumOutputs ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @return 0
    //!
    int getNumEpochs ();

    //! @brief This function doesn't do anything, but is here only for placeholder
    //!        purposes.
    //!
    //! @param display (ignored)
    //!
//    void snapshot (NNDisplayAdapter &display);

  private:

    //! @brief The number of training patterns input into the system
    //!
    int pTrainingPatterns_;

    //! @brief The number of testing patterns input into the system
    //!
    int pTestingPatterns_;

    //! @brief The number of input features for the network
    //!
    int inputFeatures_;

    //! @brief The number of output features for the network
    //!
    int outputFeatures_;

    //! @brief The input vector for current computations
    //!
    double *inputs_;

    //! @brief The center vector for computing the gaussian
    //!
    double *center_;

    //! @brief Input training data vectors
    //!
    vector<double *> trainingInputs_;

    //! @brief Input training data vectors
    //!
    vector<double *> trainingOutputs_;

    //! @brief The spread of the gaussian curve
    //!
    double *sigma_;

    //! @brief Output of the gaussian function
    //!
    double output_;

  }; // Naysayer

} // NeuralNet

#endif