///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
//  Subsystem:       Motion Primitives
//  Workfile:        AssemblyPrims.h
//  Revision:        15 December, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Motion primitives for assembly search functions.
///////////////////////////////////////////////////////////////////////////////

#ifndef ASSEMBLY_PRIMS
#define ASSEMBLY_PRIMS

//#include "ulapi.h"
#include "nist_core.h"
#include "crpi.h"

#include <vector>

#include "crpi_robot.h"

//! Robots that currently work with assembly
#include "crpi_demo_hack.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"

//! Robots that do not work with assembly
#include "crpi_schunk_sdh.h"
#include "crpi_robotiq.h"

namespace MotionPrims
{
  //! @brief Assembly search type identifiers
  typedef enum
  {
    ASSEMBLY_RANDOM = 0,
    ASSEMBLY_SPIRAL,
	ASSEMBLY_SQ_SPIRAL,
    ASSEMBLY_RASTER,
    ASSEMBLY_TILT,
    ASSEMBLY_ROTATION,
    ASSEMBLY_CIRCLE,
    ASSEMBLY_HOP,
    ASSEMBLY_LINEAR,
    ASSEMBLY_CONST_OFFSET
  } SearchType;

  //! @brief Termination condition identifiers
  //!
  typedef enum
  {
    TERMINATOR_EXTSIGNAL = 0,
    TERMINATOR_CONTACT,
    TERMINATOR_TIMER,
    TERMINATOR_DISTANCE,
    TERMINATOR_REPETITION,
    TERMINATOR_NONE
  } TermType;

  //! @brief Basic timer functionality structure
  //!
  struct AssemblyTimer
  {
    //! @brief The time (in seconds) at which the timer was started
    //!
    double inittime;

    //! @brief Whether the timer has been started
    //!
    bool started;

    //! @brief Default constructor
    //!
    AssemblyTimer ()
    {
      inittime = -1.0f;
      started = false;
    }

    //! @brief Default destructor
    //!
    ~AssemblyTimer ()
    {
      started = false;
    }

    //! @brief Start the timer functionality by recording the time
    //!
    //! @return True if the timer was started, false if the timer is already running
    //!
    bool startTimer()
    {
      if (started)
      {
        return false;
      }
      started = true;
      inittime = ulapi_time();
      return true;
    }

    //! @brief Stop the timer functionality
    //!
    //! @return True if the timer was stopped, false if the timer was not running
    //!
    bool stopTimer()
    {
      if (!started)
      {
        return false;
      }
      started = false;
      return true;
    }

    //! @brief Get the amount of time (in seconds) since the timer was started
    //!
    //! @return The elapsed time if the timer is started, -1 otherwise
    //!
    double timeElapsed()
    {
      if (started)
      {
        return ulapi_time() - inittime;
      }
      else
      {
        return -1.0f;
      }
    }

  };


  //! @brief A collection of various parameters that define the assembly search primitives
  //!
  struct LIBRARY_API assemblyParams
  {
    SearchType sType;
    bool randWalk;
    double speed;
    double length;
    double width;
    int turns;
    double x;
    double y;
    double z;
    double magnitude;
    double radius;
    double radiusOffset;
    double totalLength;
    double lengthDelta;
    double lengthStep;
    double xOffset;
    double yOffset;
    double zOffset;
    double degOffset;
    double degOffsetDelta;
    double thetaMax;
    double rasterRatio;
	int totalPoints;
  };

  //! @brief A collection of termination conditions that define when to stop the assembly search
  //!
  struct LIBRARY_API terminatorParams
  {
    TermType tType;
    CanonReturn rType;
    bool result;
    double threshold;
    int timer;
    double endTime;
    int signal;
    double xDelta;
    double yDelta;
    double zDelta;
  };

  //! @ingroup MotionPrims
  //!
  //! @brief CRPI interface for the Kuka lightweight robot
  //!
  class LIBRARY_API Assembly
  {
  public:
    //! @brief Default constructor
    //!
    Assembly ();

    //! @brief Default destructor
    //!
    ~Assembly ();

    //! @brief TODO
    //!
    //! @param radius TODO
    //! @param walk   Allow the search to explore beyond the radius (true)
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchRandom (double radius, bool walk = false);

    //! @brief TODO
    //!
    //! @param turns  TODO
    //! @param radius TODO
    //! @param speed  The speed of the search in degrees per second
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchSpiral (int turns, double radius, double speed);

    //! @brief TODO
    //!
    //! @param step Search step size in mm.
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchSqSpiral (double step, double radius);

    //! @brief TODO
    //!
    //! @param rasters TODO
    //! @param width   TODO
    //! @param length  TODO
    //! @param speed   TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchRaster (int rasters, double width, double length, double speed);

    //! @brief TODO
    //!
    //! @param TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchTilt ();

    //! @brief TODO
    //!
    //! @param range TODO
    //! @param speed TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchRotation (double range, double speed);

    //! @brief TODO
    //!
    //! @param radius TODO
    //! @param speed  TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchCircle (double radius, double speed);

    //! @brief TODO
    //!
    //! @param magnitude TODO
    //! @param frequency TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchHop (double magnitude, double frequency);

    //! @brief TODO
    //!
    //! @param xoff  TODO
    //! @param yoff  TODO
    //! @param zoff  TODO
    //! @param speed TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchLinear (double xoff, double yoff, double zoff, double speed);

    //! @brief TODO
    //!
    //! @param xoff The X axis distance threshold
    //! @param yoff The Y axis distance threshold
    //! @param zoff The Z axis distance threshold
    //!
    //! @return TODO
    //!
    CanonReturn AddSearchConstOffset (double xoff, double yoff, double zoff);

    //! @brief TODO
    //!
    //! @param rType   TODO
    //! @param timeout The value of the timer in seconds
    //!
    //! @return TODO
    //!
    CanonReturn AddTerminatorTimer (CanonReturn rType, double timeout);

    //! @brief TODO
    //!
    //! @param rType     TODO
    //! @param threshold TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddTerminatorContact (CanonReturn rType, double threshold);

    //! @brief TODO
    //!
    //! @param rType  TODO
    //! @param signal The DI/O input number to monitor.  When the prescribed input goes high, the search must stop
    //!
    //! @return TODO
    //!
    CanonReturn AddTerminatorSignal (CanonReturn rType, int signal);

    //! @brief TODO
    //!
    //! @param rType TODO
    //! @param x     TODO
    //! @param y     TODO
    //! @param z     TODO
    //! @param total TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddTerminatorDistance (CanonReturn rType, double x, double y, double z, double total);

    //! @brief TODO
    //!
    //! @param rType TODO
    //! @param reps  TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddTerminatorRepetition (CanonReturn rType, int reps);

    //! @brief Clear the search and terminator parameters
    //!
    //! @return CANON_SUCCESS if paramters successfully cleared, CANON_FAILURE otherwise
    //!
    CanonReturn ClearSearch ();

    //! @brief TODO
    //!
    //! @param counter TODO
    //!
    //! @return TODO
    //!
    CanonReturn RunAssemblyStep (int counter, robotPose &robPose, robotPose &newPose, robotIO &ios);

    //! @brief TODO
    //!
    //! @return TODO
    //!
    CanonReturn RunAssembly ();

  private:
    //! @brief TODO
    //!
    bool newSearch;

    //! @brief TODO
    //!
    robotPose initPose_;

    //! @brief TODO
    //!
    robotPose newPose_;

    //! @brief TODO
    //!
    robotPose curPose_;

    //! @brief TODO
    //!
    robotIO curIO_;

    //! @brief TODO
    //!
    AssemblyTimer timer_;

    //! @brief TODO
    //!
    vector<terminatorParams> termParams_;

    //! @brief TODO
    //!
    vector<assemblyParams> assemblyParams_;

    //! @brief TODO
    //!
    double curFreq_;

    //! @brief TODO
    //!
    int *sqs_x;
	int *sqs_y;

    //! @brief TODO
    //!
    //! @param aP TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddSearch (assemblyParams &aP);

    //! @brief TODO
    //!
    //! @param tP TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddTerminator (terminatorParams &tP);

    //! @brief TODO
    //! 
    //! @param aP TODO
    //!
    //! @return TODO
    //!
    bool motionCfg (assemblyParams &aP);

    //! @brief TODO
    //!
    //! @param counter
    //! @param ap
    //!
    //! @return TODO
    //!
    bool applyOffset (int counter, assemblyParams &ap);

    //! @brief TODO
    //!
    //! @return TODO
    //!
    bool testTerm (int &index);

	void init_sqs_table( int table_size );
    
  }; // Assembly

} // namespace MotionPrims

#endif