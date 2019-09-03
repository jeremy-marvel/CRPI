///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        BioInspired.h
//  Revision:        1.0 - 21 July, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
//  Neural network declarations for Feed-Forward, Radial Basis Function, and
//  Black-Box simulator networks.
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
  //! TODO
  //! @brief Neural network "black box" placeholder that simply returns the output of
  //!        an N-dimensional Gaussian function.  While it has the same functions and
  //!        interface as the Feedforward class (above), it cannot be used as a
  //!        function approximator.
  //!
  template <class T> class LIBRARY_API BioInspired
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
    BioInspired (int inFeat,
              int interFeat,
              int outFeat,
              double initWeight = 1.0,
              double initEta = 0.001);

    //! @brief Default destructor
    //!
    ~BioInspired ();

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

    //! @brief Display an error message
    //!
    //! @param where The location where the error occurred
    //! @param what  The error type
    //!
    void exception (char *where, char *what);

  }; // BioInspired

} // NeuralNet

#endif