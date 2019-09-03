///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Neuron
//  Workfile:        HHNeuron.h
//  Revision:        1.0 - 21 August, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Hodgkin-Huxley mathematical neural model class description.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef NUMERICAL_H
#define NUMERICAL_H

#include "../../../portable.h"

namespace Neuron
{
  //! @ingroup Neuron
  //!
  //! @brief Hodgkin-Huxley neuron model
  //!
  class LIBRARY_API HHNeuron
  {
  public:

    //! @brief Default constructor
    //!
    HHNeuron ();

    //! @brief Default destructor
    //!
    ~HHNeuron ();

  private:
    
    //! @brief TODO
    //!
    double output_;  //private data; use "get_output()" to access value
    
  }; // class HHNeuron

} // namespace Neuron

#endif