///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       NeuralNet
//  Workfile:        RBFClassification.h
//  Revision:        1.0 - 21 December, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
//
///////////////////////////////////////////////////////////////////////////////

#ifndef RBFCLASSIFICATION_H
#define RBFCLASSIFICATION_H

//#include "../../portable.h"
#include "../Neuron/Numerical/Numerical.h"
#include <vector>

using namespace std;

namespace NeuralNet
{
  //! @ingroup NeuralNet
  //!
  //! @brief Radial Basis Function implementation
  //!
  template <class T1> class LIBRARY_API RBFClassification
  {
  public:

    //! @brief Default constructor
    //!
    //! @param inFeat     The number of input features for the network
    //! @param interFeat  The number of beta-layer neurons
    //! @param outFeat    The number of output-layer neurons
    //! @param initWeight TODO
    //! @param initEta    The initial learning rate during back propagation
    //!
    RBFClassification (  int inFeat,
                         int interFeat,
                         int outFeat,
                         double initWeight = 1.0,
                         double initEta = 0.001);

    //! @brief Default destructor
    //!
    ~RBFClassification ();

    //! @brief Initialize the dE/dw values for learning
    //!
    //! @note:  Must be called following network creation and input insertion.
    //!
    void initialize ();

    //! @brief Run the neural net for a specified number of iterations
    //!
    //! @param iter  The number of training iterations to run
    //! @param noisy Whether or not to prune the top 10% worst performers from the
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
    void adjustNeighborhood (int method, int size = 0);

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

    //! @brief Get the minimum and maximum value vectors for the inputs that
    //!        were fed into the training system
    //!
    //! @param minInVals  The per-element minimum values given as inputs
    //! @param maxInVals  The per-element maximum values given as inputs
    //!
    void getRawRange (vector<double>& minInVals,
                      vector<double>& maxInVals);

    //! @brief Set the minimum and maximum value vectors for both the inputs
    //!        and the outputs for scaling purposes
    //!
    //! @param minInVals  The per-element minimum values given as inputs
    //! @param maxInVals  The per-element maximum values given as inputs
    //! @param minOutVals The per-element minimum values given as outputs
    //! @param maxOutVals The per-element maximum values given as outputs
    //!
    void setRawRange (vector<double>& minInVals,
                      vector<double>& maxInVals,
                      vector<double>& minOutVals,
                      vector<double>& maxOutVals);

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
    double getOutput (int index);

    double getAlphaOutput (int index);
    double getBetaOutput (int index);

    void setBetaActive (int index, bool active);

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
    double getTrainingInput (int pattern, int input);

    //! @brief Access the scaled training ouput patterns
    //!
    //! @param pattern The training pattern vector to be inquiried
    //! @param output  The training pattern element to be retrieved
    //!
    //! @return The scaled output parameter
    //!
    double getTrainingOutput (int pattern, int output);

    double getRawTrainingOutput (int pattern, int output);

    double getMaxOutput (int output);
    double getMinOutput (int output);

    //! @brief TODO
    //!
    //! @param display TODO
    //!
//    void snapshot (NNDisplayAdapter &display);

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
                     int element,
                     int outWatch,
                     double& bestout,
                     bool maximize);

    //! @brief TODO
    // here is the random-search learning function:
    // create a set of weight perturbations; try it out.  If the
    // perturbations result in an improvement, accept all changes; if not,
    // reject all changes
    //!
    void adjustWeights (); // here's the main deal--learn by adjusting wts

    //! @brief Scale the currently loaded training and testing values
    //!
    void scaleAllInputs ();

    //! @brief Scale the outputs to be in the range of min(outputs) to
    //!        max(outputs)
    //!
    void scaleAllOutputs ();

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

    void addDWBetaGammaWeights();
    void subDWBetaGammaWeights();
    void fillRandDWeights (double val);

    //! @brief The number of alpha-layer decision planes per axis in the input
    //!        parameter space.  This is a preset value that must be changed
    //!        at compile-time.
    //!
    int alphaAxis_;

    //! @brief Flag used if preset values are used for scaling purposes in
    //!        lieu of automatically extracting the information from the
    //!        training set
    //!
    bool scalingOverride_;

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
    int pTrainingPatterns_;

    //! @brief The number of training patterns entered into the backup system,
    //!        for use when we want to select subpopulations for training
    //!
    int pRawTrainingPatternsBackup_;

    //! @brief The number of testing patterns input into the system
    //!
    int pTestingPatterns_;

    //! @brief The number of input features for the network
    //!
    int inputFeatures_;

    //! @brief The number of hidden-layer neurons for the alpha layer
    //!
    int interJFeatures_;

    //! @brief The number of hidden-layer neurons for the beta layer
    //!
    int interKFeatures_;

    //! @brief The number of output-layer neurons for the gamma layer
    //!
    int outputFeatures_;

    //! @brief Matrix to hold synapse values from inputs to hidden1 layer
    //!
    //! @note Indexing is [hidden1][input]
    //!
    double **synapses0to1_;

    //! @brief Matrix to hold synapse values from hidden1 to hidden2 layers
    //!
    //! @note Indexing is [hidden2][hidden1]
    //!
    double **synapses1to2_;

    //! @brief Matrix to hold synapse values from hidden2 to output layers
    //!
    //! @note Indexing is [output][hidden2]
    //!
    double **synapses2to3_;

    //! @brief TODO
    //!
    double *hiddenJInputs_;

    //! @brief TODO
    //!
    double *hiddenJOutputs_;

    //! @brief TODO
    //!
    double *hiddenKInputs_;

    double *weight_;

    //! @brief TODO
    //!
    double *hiddenKOutputs_;

    //! @brief TODO
    //!
    double *outputLInputs_;

    //! @brief Variable used to keep track of the number of training iterations
    //!        the network undergoes
    //!
    int nIters_;

    //! @brief TODO
    //!
    Neuron::LogicNeuron *interJNeurons_;

    //! @brief TODO
    //!
    T1 *interKNeurons_;

    //! @brief TODO
    //!
    bool *interKNeuronsActive_;

    //! @brief TODO
    //!
    Neuron::LinearNeuron *outputNeurons_;

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

    //! @brief Perturbation values for weights from beta layer to gamma layer.
    //!        Typically will have only one column of interest, but carry
    //!        along dummy 0'th column for bias
    //!
    double **dwBetaGamma_;

    //! @brief TODO
    //!
    double randRange_;

    //! @brief Set the optimum Beta-Gamma synaptic weights
    //!
    void setBetaGammaWeights ();

    //! @brief Give an initial random value to the network's synapses
    //!
    //! @note There must be training points defined before RBF as NN can be
    //!       used.
    //!
    void initializeSynapses ();

    //! @brief Train the alpha-beta synapses using a specific pattern
    //!
    //! @param index The beta-layer neuron whose input weights we are adjusting
    //! @param in    The training pattern for which we are creating the synaptic
    //!              weights
    //!
    void setAlphaBetaWeights (int index, double *in);

    //! @brief TODO
    //!
    void initializeBetas ();
    void initializeBetasSpecial ();

    void writeErrorField ();

  }; // RBFClassification
} // NeuralNet

#endif
