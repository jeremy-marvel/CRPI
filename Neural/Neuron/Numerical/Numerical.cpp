///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Neuron
//  Workfile:        numerical.cpp
//  Revision:        1.0 - 22 January, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Numerical neuron definitions.
//
///////////////////////////////////////////////////////////////////////////////

#include "numerical.h"
#include <cmath>

namespace Neuron
{
  /////////////////////////////////////////////////////////////////////////////
  //                         Hyperbolic Tangent Neuron                       //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API TanhNeuron::TanhNeuron ()
  {
    //! Initialize output to zero; 
    output_ = 0.0;
  }


  LIBRARY_API TanhNeuron::~TanhNeuron ()
  {
  }


  LIBRARY_API void TanhNeuron::activate (double netInput, double aux)
  { 
    output_ = actTanh (netInput, aux);
  }


  LIBRARY_API double TanhNeuron::output ()
  {
    return output_;
  }


  LIBRARY_API double TanhNeuron::derivative (double u, double aux)
  {
    static const double a = 1.7159;
    static const double b = 2.0/3.0;
    //static const double a = 1.0f;
    //static const double b = 1.0f;

    static double gprime, sigma;

    //! g'(u) = ab - (b/a)(tanh(u)) = (b/a) * (a - tanh(u)) * (a + tanh(u))
    sigma = actTanh(u, aux);

    gprime = (b/a) * ((a - sigma) * (a + sigma));
    return gprime;
  }


  LIBRARY_API double TanhNeuron::actTanh (double u, double aux)
  {
    static const double a = 1.7159;
    static const double b = 2.0/3.0;
    //static const double a = 1.0f;
    //static const double b = 1.0f;
    
    static double exp_min, exp_pl;
    static double tanh;

    double arg = aux * u;

    if (arg > 10.0f)
    {
      return a;
    }
    else if (arg < -10.0f)
    {
      return -a;
    }

    exp_min = exp (-arg);
    exp_pl = exp (arg);
    tanh = a * (exp_pl - exp_min) / (exp_pl + exp_min);

    return tanh;
  }


  /////////////////////////////////////////////////////////////////////////////
  //                             Sigmoidal Neuron                            //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API SigmoidNeuron::SigmoidNeuron ()
  {
    //! Initialize output to zero
    output_ = 0.0;
  }


  LIBRARY_API SigmoidNeuron::~SigmoidNeuron ()
  {
  }


  LIBRARY_API void SigmoidNeuron::activate (double netInput, double aux)
  { 
    output_ = actSigmoid (netInput, aux);
  }


  LIBRARY_API double SigmoidNeuron::output ()
  {
    return output_;
  }


  LIBRARY_API double SigmoidNeuron::derivative (double u, double aux)
  {
    static double sigma, gprime;
    sigma = actSigmoid (u, aux);
    gprime = sigma * (1 - sigma);
    return gprime;
  }


  LIBRARY_API double SigmoidNeuron::actSigmoid (double u, double aux)
  {
    double a = 2.0f,
           b = 1.5f, //! Larger value for b == "spikier" RBF response
           c = 0.0f;

    c = a / (1.0f + exp (-u * aux));
    //! Sigmoid curve
    return c;
  }


  /////////////////////////////////////////////////////////////////////////////
  //                               Linear Neuron                             //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API LinearNeuron::LinearNeuron ()
  {
    //! Initialize output to zero
    output_ = 0.0;
  }


  LIBRARY_API LinearNeuron::~LinearNeuron ()
  {
  }


  LIBRARY_API void LinearNeuron::activate (double netInput, double aux)
  { 
    output_ = actLinear (netInput, aux);
  }


  LIBRARY_API double LinearNeuron::output ()
  {
    return output_;
  }

  
  LIBRARY_API double LinearNeuron::derivative (double u, double aux)
  {
    //! Derivative of a variable is a constant
    return 1.0;
  }


  LIBRARY_API double LinearNeuron::actLinear (double u, double aux)
  {
    return u;
  }


  /////////////////////////////////////////////////////////////////////////////
  //                           Random Binary Neuron                          //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API RandBinNeuron::RandBinNeuron ()
  {
    //! Initialize output to zero
    output_ = 0.0;
    //! Threshold
    threshold_ = 1.0;
  }


  LIBRARY_API RandBinNeuron::~RandBinNeuron ()
  {
  }


  LIBRARY_API void RandBinNeuron::activate (double netInput, double aux)
  { 
    output_ = actBinary (netInput, aux);
  }


  LIBRARY_API double RandBinNeuron::output ()
  {
    return output_;
  }

  
  LIBRARY_API double RandBinNeuron::derivative (double u, double aux)
  {
    //! Derivative of a variable is a constant
    return 1.0;
  }


  LIBRARY_API double RandBinNeuron::actBinary (double u, double aux)
  {
    int r = (rand() % 1000000); 
    if (u > threshold_ || r < 20)
    {
      return 1.0f;
    }
    return 0.0f;
  }



  /////////////////////////////////////////////////////////////////////////////
  //                          Bimodal Logic Neuron                           //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API LogicNeuron::LogicNeuron ()
  {
    //! Initialize output to zero
    output_ = 0.0;
  }


  LIBRARY_API LogicNeuron::~LogicNeuron ()
  {
  }


  LIBRARY_API void LogicNeuron::activate (double netInput, double aux)
  { 
    output_ = actLogic (netInput, aux);
  }


  LIBRARY_API double LogicNeuron::output ()
  {
    return output_;
  }

  
  LIBRARY_API double LogicNeuron::derivative (double u, double aux)
  {
    //! Not used here
    return 0.0f;
  }


  LIBRARY_API double LogicNeuron::actLogic (double u, double aux)
  {
    if (u >= 0.0)
      return 1.0f;
    return -1.0f;
  }


} // namespace Neuron