///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        NeuralNet.h
//  Revision:        1.0 - 6 February, 2008
//                   1.1 - 5 August, 2008   : Added accessor functions of the
//                                            network's state.
//                   2.0 - 27 January, 2009 : Added black-box NN simulator
//                   3.0 - 23 March, 2009   : Separated simulators from
//                                            FeedForward class header file
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Neural network declarations for Feed-Forward networks.
//  Currently, can only use numerical neuron types
//
///////////////////////////////////////////////////////////////////////////////

#ifndef NEURALNET_H
#define NEURALNET_H

#include "NetCommon.h"
#include "../../portable.h"
#include <vector>

using namespace std;

namespace NeuralNet
{
  //! @ingroup NeuralNet
  //!
  //! @brief Standard feed-forward neural network with back propagation with
  //!        one hidden layer.
  //!
  template <class T> class LIBRARY_API FeedForward
  {
  public:

    //! @brief Default constructor
    //!
    //! @param inFeat     The number of input features for the network
    //! @param interFeat  The number of hidden-layer neurons
    //! @param outFeat    The number of output-layer neurons
    //! @param initWeight TODO
    //! @param initEta    The initial learning rate during back propagation
    //!
    FeedForward (int inFeat,
                 int interFeat,
                 int outFeat,
                 double initWeight = 1.0,
                 double initEta = 0.001);

    //! @brief Default destructor
    //!
    ~FeedForward ();

    //! @brief Initialize the dE/dw values for learning
    //!
    //! @note:  Must be called following network creation and input insertion
    //!
    void initialize ();

    //! @brief Run the neural net for a specified number of iterations
    //!
    //! @param iter  The number of training iterations to run
    //! @param prune Whether or not to prune the top 10% worst performers from the
    //!              training set
    //!
    //! @return The resulting RMS error
    //!
    double trainNetwork (int iter, bool prune = false);

    //! @brief Run the test set through the neural network and write a .m file
    //!        to display the resulting outputs of the training and test sets
    //!        for comparison
    //!
    //! @param k The output neuron being queried
    //!
    void runTests (int k);


    //! @brief Run the test set through the neural network and write an output file
    //!        specifying the top performers
    //!
    //! @param top Specify how many of the top performers to return
    //! @param k   The output neuron being queried
    //!
    void runTestsBest (int top, int k);

    //! @brief Attempt to find the optimum input vector that results in the
    //!        best output for a specified output-layer node
    //!
    //! @param minVals  The minimum values for the input vector
    //! @param maxVals  The maximum values for the input vector
    //! @param best     The best input sequence according to the trained NN
    //! @param outWatch The output layer index over which the method optimizes
    //! @param maximize Whether or not the performance metric is maximization
    //!
    //! @return The optimized output node output
    //!
    double autoOptimize (vector<double>& minVals,
                         vector<double>& maxVals,
                         vector<double>& best,
                         int outWatch,
                         bool maximize = true);

    //! @brief TODO
    //!
    //! @param trainPath TODO
    //! @param testPath 
    //!
    //! @return TODO
    //!
    int parseInputFile (const char *trainPath = NULL,
                        const char *testPath = NULL);

    //! @brief Read a NN weight description file from disk
    //!
    //! @param netPath The location of the description file to be read
    //!
    //! @return True if the file was successfully read, False otherwise
    //!
    bool readNetworkFile (const char *netPath);

    //! @brief Write a NN weight description file from disk
    //!
    //! @param netPath The location of the description file to be written
    //!
    //! @return True if the file was successfully written, False otherwise
    //!
    bool writeNetworkFile (const char *netPath);

    //! @brief Add a training sample to the network's collection
    //!
    //! @param in  The raw input feature values for the training sample
    //! @param out The raw output feature values for the training sample
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
    void adjustNeighborhood (int method, unsigned int size = 0);

    //! @brief Add a testing sample to the network's collection
    //!
    //! @param in  The raw input feature values for the testing sample
    //!
    void addTestingSet (double *in);

    //! @brief Scale an input vector to the range of values observed in the
    //!        training set, and convert to a usable format
    //!
    //! @param rawVec    The raw (unscaled) input vector
    //! @param scaledVec The input vector scaled to the range observed in the
    //!                  training set
    //!
    void scaleInput (vector<double> &rawVec, double *scaledVec);

    //! @brief Get the minimum and maximum value vectors for both the intputs and
    //!        outputs that were fed into the training system
    //!
    //! @param minInVals  The per-element minimum values given as inputs
    //! @param maxInVals  The per-element maximum values given as inputs
    //!
    void getRawRange (vector<double>& minInVals,
                      vector<double>& maxInVals);

    //! @brief TODO
    //!
    //! @param inputVec TODO
    //!
    void setInputs (double *inputVec); // stimulate network with an input vec
                                        // must do this before compute_outputs()

    //! @brief Compute the outputs for all neurons in the network
    //!
    //! Note:  No error detection:  assumes inputs have been allocated &
    //!        populated properly
    //!
    void computeOutputs ();

    //! @brief Report the value for the specified output node
    //!
    //! @param index The output node index being queried
    //!
    //! @return The value of the output node, or -1 if node does not exist
    //!
    double getOutput (unsigned int index);

    //! @brief TODO
    //!
    //! @return The value of ETotal_
    //!
    double getTotalError ();

    //! @brief TODO
    //!
    //! @return TODO
    //!
    double getRMSError ();
    
    //! @brief TODO
    //!
    //! @return TODO
    //!
    int getNumInputs ();

    //! @brief TODO
    //!
    //! @return TODO
    //!
    int getNumOutputs ();

    //! @brief Get the number of training epochs the network has undergone
    //!
    //! @return The number of times adjustWeights() has been called
    //!
    int getNumEpochs ();

    //! @brief Get the number of training patterns added to the collection
    //!
    //! @return The number of training patterns added
    //!
    int getNumTrainingPatterns();

    //! @brief Access the scaled training input patterns
    //!
    //! @param pattern The training pattern vector to be inquiried
    //! @param input   The training pattern element to be retrieved
    //!
    //! @return The scaled input parameter
    //!
    double getTrainingInput (unsigned int pattern, unsigned int input);

    //! @brief TODO
    //!
    //! @param pattern TODO
    //! @param input   TODO
    //!
    //! @return TODO
    //!
    double getRawTrainingInput (unsigned int pattern, unsigned int input);

    //! @brief Access the scaled training ouput patterns
    //!
    //! @param pattern The training pattern vector to be inquiried
    //! @param output  The training pattern element to be retrieved
    //!
    //! @return The scaled output parameter
    //!
    double getTrainingOutput (unsigned int pattern, unsigned int output);

    //! @brief TODO
    //!
    //! @param pattern TODO
    //! @param output  TODO
    //!
    //! @return TODO
    //!
    double getRawTrainingOutput (unsigned int pattern, unsigned int output);

    //! @brief TODO
    //!
    //! @param output TODO
    //!
    //! @return TODO
    //!
    double getMaxOutput (unsigned int output);

    //! @brief TODO
    //!
    //! @param TODO
    //!
    //! @return TODO
    //!
    double getMinOutput (unsigned int output);

  private:

    //! @brief Recursive helper function for the auto-optimization method
    //!
    //! @param minVals  The minimum values for the input vector
    //! @param maxVals  The maximum values for the input vector
    //! @param current  The current input vector being tested
    //! @Param best     The best input sequence according to the trained NN
    //! @param element  The current input vector index being sequenced
    //! @param outwatch The output layer index over which the method optimizes
    //! @param bestout  The current best value for the specified output node
    //! @param maximize Whether or not the performance metric is maximization
    //!
    void optiHelper (vector<double>& minVals,
                     vector<double>& maxVals,
                     vector<double>& current,
                     vector<double>& best,
                     unsigned int element,
                     int outWatch,
                     double& bestout,
                     bool maximize);

    //! @brief TODO
    //!
    void adjustWeights (); // here's the main deal--learn by adjusting wts

    //! @brief Scale the currently loaded training and testing values
    //!
    void scaleAllInputs ();

    //! @brief Scale the outputs to be in the range of min(outputs) to
    //!        max(outputs)
    //!
    void scaleAllOutputs ();

    //! @brief Compute do/du for each neuron (calls the derivative(u) function
    //!        of the respective neuron type)
    //!
    void computeGPrimes ();

    // may use these to adjust learning factors...
    // copy elements of dEdw01_ to dEdw01Old_
    // and dEdw12_ to dEdw12Old_
    // may test changes in sign of derivatives to determine
    // how to vary learning params
    //! @brief TODO
    //!
    void backupDerivs (); // makes a copy of all derivs to old_dEsqd_dw

    //! @brief TODO
    //!
    //! @note May adjust learning factors up or down based on the convergence
    //!       behavior (ex. sign reversal of dE/dw)
    //!
    void updateEtas ();

    //! @brief Get the vector of minimum and maximum values for the
    //!        collection of training input values (populates
    //!        inputMinVals_ and inputMaxVals_, use prior to scaling
    //!        inputs)
    //!
    void findInputMinMaxVals ();

    //! @brief Get the vector of minimum and maximum values for the
    //!        collection of training output values (populates
    //!        outputMinVals_ and outputMaxVals_, use prior to scaling
    //!        outputs)
    //!
    void findOutputMinMaxVals ();

    //! @brief Compute the value of the RMS error for convergence testing
    //!
    void computeErrors ();

    //! @brief Compute the value of the RMS error for convergence testing,
    //!        modified to allow for selective pruning of inputs
    //!
    void computeErrorsWithDisable ();

    //! @brief Compute the sum of squared errors for dE/dw
    //!
    //! @return The summed value of the squares of all dEdw##_ values
    //!
    double dEdwSumSqr ();

    //! @brief Compute the gradient descent sensitivity of the layer-I to 
    //!        layer-J synapses:  dE/dw term
    //!
    void compute_dEdw01 ();

    //! @brief TODO
    //!
    void compute_dEdw12 (); // same, for all interneuron-to-output neuron wts

    //! @brief Range (-initWeight_, +initWeight_) to which the random
    //!        synaptic weights are set
    //!
    double initWeight_;

    //! @brief Initial learning factor to which all other etas are reset
    //!        once the training begins to show signs of diverging
    //!
    double initEta_;

    //! @brief The number of training patterns input into the system
    //!
    unsigned int pTrainingPatterns_;

    //! @brief The number of training patterns entered into the backup system,
    //!        for use when we want to select subpopulations for training
    //!
    unsigned int pRawTrainingPatternsBackup_;

    //! @brief The number of testing patterns input into the system
    //!
    unsigned int pTestingPatterns_;

    //! @brief The number of input features for the network
    //!
    unsigned int inputFeatures_;

    //! @brief The number of hidden-layer neurons for the J layer
    //!
    unsigned int interFeatures_;

    //! @brief The number of output-layer neurons for the K layer
    //!
    unsigned int outputFeatures_;

    //! @brief Matrix to hold synapse values from inputs to hidden layer
    //!
    //! @note Indexing is [hidden][input]
    //!
    double **synapses0to1_;

    //! @brief Matrix to hold synapse values from hidden to output layers
    //!
    //! @note Indexing is [output][hidden]
    //!
    double **synapses1to2_;

    //! @brief TODO
    //!
    double *hiddenInputs_;

    //! @brief TODO
    //!
    double *hiddenOutputs_;

    //! @brief TODO
    //!
    double *hiddenGPrimes_;

    //! @brief TODO
    //!
    double *layer2Inputs_;

    //! @brief TODO
    //!
    double *gPrimes_;

    //! @brief Variable used to keep track of the number of training iterations
    //!        the network undergoes
    //!
    int nIters_;

    //! @brief TODO
    //!
    T *interNeurons_;

    //! @brief TODO
    //!
    T *outputNeurons_;

    //! @brief Scaled input training data vectors
    //!
    vector<double *> trainingInputs_;

    //! @brief Scaled output training data vectors
    //!
    vector<double *> trainingOutputs_;

    //! @brief Scaled input testing data vectors
    //!
    vector<double *> testingInputs_;

    //! @brief Raw (unscaled) input training data vectors
    //!
    vector<double *> rawTrainingInputs_;

    //! @brief Raw (unscaled) output training data vectors
    //!
    vector<double *> rawTrainingOutputs_;

    //! @brief A backup copy of all training input vectors, for use when we
    //!        want to select subpopulations for training
    //!
    vector<double *> rawTrainingInputsBackup_;

    //! @brief A backup copy of all training input vectors, for use when we
    //!        want to select subpopulations for training
    //!
    vector<double *> rawTrainingOutputsBackup_;

    //! @brief Raw (unscaled) input testing data vectors
    //!
    vector<double *> rawTestingInputs_;

    //! @brief TODO
    //!
    vector<bool> inputActive_;

    //! @brief TODO
    //!
    double *inputs_;

    //! @brief TODO
    //!
    double *inputMaxVals_;

    //! @brief TODO
    //!
    double *inputMinVals_;

    //! @brief TODO
    //!
    double *outputs_; // same as *layer2_outputs;

    //! @brief TODO
    //!
    double *outputMaxVals_;

    //! @brief TODO
    //!
    double *outputMinVals_;

    //! @brief TODO
    //!
    vector<double> EVals_;

    //! @brief TODO
    //!
    double ETotal_;

    //! @brief Computed root mean squared error value
    //!
    double RMSError_;

    //! @brief Matrix to hold weight sensitivities, dE/dw_j,i
    //!
    double **dEdw01_;

    //! @brief TODO
    //!
    double **dEdw01Old_;

    //! @brief Matrix to hold weight sensitivities, dEdw_j,k
    //!
    double **dEdw12_;

    //! @brief TODO
    //!
    double **dEdw12Old_;

    //! @brief Matrix of learning factors for each synapse from layer
    //!        1 to Layer 2
    //!
    double **dw01Eta_;

    //! @brief Matrix of learning factors for each synapse from layer
    //!        2 to layer 3
    //!
    double **dw12Eta_;

    //! @brief Matrix of computed perturbations of input synapses
    //!
    double **dw01_;

    //! @brief Matrix of computed perturbations of each weight
    //!
    double **dw02_;

    //! @brief Give an initial random value to the network's synapses
    //!
    void initializeSynapses ();

  }; // FeedForward
} // NeuralNet

#endif