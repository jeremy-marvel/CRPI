///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Neuron
//  Workfile:        numerical.h
//  Revision:        1.0 - 21 January, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Numerical (artificial) neuron declarations.  These neural models have
//  built-in derivative functions for ANN implementation capacity.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef NUMERICAL_NEURON_H
#define NUMERICAL_NEURON_H

#include "../../../portable.h"

namespace Neuron
{
  //! @ingroup Neuron
  //!
  //! @brief Hyperbolic tangent activation numerical neuron model
  //!
  class LIBRARY_API TanhNeuron
  {
  public:

    //! @brief Default constructor
    //!
    TanhNeuron ();

    //! @brief Default destructor
    //!
    ~TanhNeuron ();

    //! @brief Compute the neuron's output given combined signal input
    //!
    //! @param netInput TODO
    //!
    void activate (double netInput, double aux = 0.66666666666f);

    //! @brief Get the neuron's output
    //!
    //! @return TODO
    //!
    double output ();

    //! @brief Compute the derivative of the activation function
    //!
    //! @param u TODO
    //!
    //! @return TODO
    //!
    double derivative (double u, double aux = 0.666666666666f);

  private:
    
    //! @brief TODO
    //!
    double output_;  //private data; use "get_output()" to access value
 
    //! @brief TODO
    //!
    //! @param u TODO
    //!
    //! @return TODO
    //!
    double actTanh (double u, double aux);
    
  }; // class TanhNeuron


  //! @ingroup Neuron
  //!
  //! @brief Sigmoidal activation numerical neuron model
  //!
  class LIBRARY_API SigmoidNeuron
  {
  public:

    //! @brief Default constructor
    //!
    SigmoidNeuron ();

    //! @brief Default destructor
    //!
    ~SigmoidNeuron ();

    //! @brief Compute the neuron's output given combined signal input
    //!
    //! @param netInput Sum of weighted inputs into this neuron
    //! @param aux      TODO
    //!
    void activate (double netInput, double aux = 1.5f);

    //! @brief Get the neuron's output
    //!
    //! @return The output value of this neuron
    //!
    double output ();

    //! @brief Compute the derivative of the activation function
    //!
    //! @param u   Input into the activation function, from which the derivative output is computed
    //! @param aux TODO
    //!
    //! @return The output of the der
    //!
    double derivative (double u, double aux = 1.5f);

  private:

    //! @brief Current output of this neuron
    //!
    double output_;  //private data; use "get_output()" to access value
 
    //! @brief TODO
    //!
    //! @param u TODO
    //!
    //! @return TODO
    //!
    double actSigmoid (double u, double aux);

  }; // class SigmoidNeuron


  //! @ingroup Neuron
  //!
  //! @brief Linear activation numerical neuron model
  //!
  class LIBRARY_API LinearNeuron
  {
  public:

    //! @brief Default constructor
    //!
    LinearNeuron ();

    //! @brief Default destructor
    //!
    ~LinearNeuron ();

    //! @brief Compute the neuron's output given combined signal input
    //!
    //! @param netInput TODO
    //!
    void activate (double netInput, double aux = 1.0f);

    //! @brief Get the neuron's output
    //!
    //! @return TODO
    //!
    inline double output ();

    //! @brief Compute the derivative of the activation function
    //!
    //! @param u TODO
    //!
    //! @return TODO
    //!
    double derivative (double u, double aux = 1.0f);

  private:

    //! @brief Current output of this neuron
    //!
    double output_;  //private data; use "get_output()" to access value

    //! @brief TODO
    //!
    //! @param u TODO
    //!
    //! @return TODO
    //!
    double actLinear (double u, double aux);

  }; // class LinearNeuron


  //! @ingroup Neuron
  //!
  //! @brief Thresholded binary output neuron model with random firing
  //!
  class LIBRARY_API RandBinNeuron
  {
  public:

    //! @brief Default constructor
    //!
    RandBinNeuron ();

    //! @brief Default destructor
    //!
    ~RandBinNeuron ();

    //! @brief Compute the neuron's output given combined signal input
    //!
    //! @param netInput TODO
    //!
    void activate (double netInput, double aux = 1.0f);

    //! @brief Get the neuron's output
    //!
    //! @return TODO
    //!
    inline double output ();

    //! @brief Compute the derivative of the activation function
    //!
    //! @param u TODO
    //!
    //! @return TODO
    //!
    double derivative (double u, double aux = 1.0f);

  private:

    //! @brief Action potential
    //!
    double threshold_;

    //! @brief Current output of this neuron
    //!
    double output_;  //private data; use "get_output()" to access value

    //! @brief Binary neuron activation function
    //!
    //! @param u The summed, weighted input signals coming into the neuron
    //!
    //! @return 1 if the input signals are greater than the threshold, 0 otherwise
    //!
    double actBinary (double u, double aux);
  }; // class RandBinNeuron


  //! @ingroup Neuron
  //!
  //! @brief Basic neuron type that returns -1 if the summed input is negative,
  //!        +1 otherwise
  //!
  class LIBRARY_API LogicNeuron
  {
  public:

    //! @brief Default constructor
    //!
    LogicNeuron ();

    //! @brief Default destructor
    //!
    ~LogicNeuron ();

    //! @brief Compute the neuron's output given combined signal input
    //!
    //! @param netInput TODO
    //!
    void activate (double netInput, double aux = 1.0f);

    //! @brief Get the neuron's output
    //!
    //! @return TODO
    //!
    inline double output ();

    //! @brief Compute the derivative of the activation function
    //!
    //! @param u   TODO
    //! @param aux TODO
    //!
    //! @return TODO
    //!
    double derivative (double u, double aux = 1.0f);

  private:

    //! @brief Current output of this neuron
    //!
    double output_;  //private data; use "get_output()" to access value

    //! @brief Binary neuron activation function
    //!
    //! @param u   The summed, weighted input signals coming into the neuron
    //! @param aux 
    //!
    //! @return 1 if the input signals are greater than 0, -1 otherwise
    //!
    double actLogic (double u, double aux);

  }; // class LogicNeuron

} // namespace Neuron

#endif