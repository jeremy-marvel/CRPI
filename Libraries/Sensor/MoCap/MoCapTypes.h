///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Motion Capture Sensor
//  Workfile:        MoCapTypes.h
//  Revision:        1.0 - 21 December, 2016
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Collection of common data types used by the motion capture system
//  interfaces.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef MOCAPTYPES_H
#define MOCAPTYPES_H

#include <vector>

#include <string.h>

#if defined(_MSC_VER)
#include "MatrixMath.h"
#elif defined(__GNUC__)
#include "../../Math/MatrixMath.h"
#endif

#pragma warning( disable : 4996 )

using namespace std;
using namespace Math;

namespace Sensor
{
  //! @brief Storage container for rigid bodies detected by the Vicon motion capture system
  //!
  typedef LIBRARY_API struct MoCapSubject_
  {
    //! @brief The pre-defined name of the rigid-body subject identified
    //!
    std::string name;

    //! @brief The pose of the rigid body subject
    //!
    Math::pose pose;

    //! @brief The rotation of the rigid body (in R^(3x3) format)
    //!
    Math::matrix rotation;

    //! @brief Collection of individual markers that compose the rigid body
    //!
    //vector<point> labeledMarkers;

    //! @brief Whether this subject instance is considered valid
    //!
    bool valid;

    //! @brief Default constructor
    //!
    MoCapSubject_()
    {
      rotation.resize(3, 3);
      //labeledMarkers.clear();
      valid = false;
    }

    //! @brief Default destructor
    //!
    ~MoCapSubject_()
    {
      if (this != NULL)
      {
        //labeledMarkers.clear();
      }
      valid = false;
    }
  } MoCapSubject;

}

#endif // MOCAPTYPES
