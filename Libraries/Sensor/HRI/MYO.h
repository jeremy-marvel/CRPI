///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Human-Robot Interaction
//  Workfile:        MYO.h
//  Revision:        1.0 - 13 July, 2016
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Interface wrapper for the Thalmic Labs Myo  
//
///////////////////////////////////////////////////////////////////////////////


#ifndef MYO_H
#define MYO_H

#include <string.h>
#include <ulapi.h>
#ifdef WIN32
#include <../../portable.h>
#include <myo/myo.hpp>
#include <ulapi.h>
#include <array>
#include <vector>
#else
#include "portable.h"
#include <array>
#include <vector>
#include <myo.hpp>
#endif

#include <string.h>
#include <ulapi.h>
#include <string.h>

using namespace std;

namespace Sensor
{
  //! @brief Armband data container for reporting
  //!
  struct MyoSubject
  {
    //! @brief Current pose of the operator (rest, fist, waveIn, waveOut, fingerSpread, doubleTap)
    //!
    string pose;

    //! @brief EMG data
    //!
    vector<int8_t> emgSamples;

    //! @brief Accelerometer data
    //!
    vector<double> accelSamples;

    //! @brief Orientation data (in quaternion format)
    //!
    vector<double> orientSamples;

    vector<double> orientEuler;

    //! @brief Gyroscope data
    //!
    vector<double> gyroSamples;

    //! @brief Default constructor
    //!
    MyoSubject() : emgSamples()
    {
      orientSamples.resize(4);
      accelSamples.resize(3);
      gyroSamples.resize(3);
      emgSamples.resize(8);
    }

    //! @brief Default destructor
    //!
    ~MyoSubject()
    {
      accelSamples.clear();
      orientSamples.clear();
      gyroSamples.clear();
      emgSamples.clear();
    }

    //! @brief Write the data to the screen
    //!
    void print()
    {
      printf("Pose:         %s\n", pose.c_str());
      printf("Orientation:  %f %f %f %f\n", orientSamples.at(0),
                                            orientSamples.at(1),
                                            orientSamples.at(2),
                                            orientSamples.at(3));
      printf("Euler: %f %f %f,\n", orientEuler.at(0),
                                   orientEuler.at(1),
                                   orientEuler.at(2));
      printf("Acceleration: %f %f %f\n", accelSamples.at(0),
                                         accelSamples.at(1),
                                         accelSamples.at(2));
      printf("Gyroscope:    %f %f %f\n", gyroSamples.at(0),
                                         gyroSamples.at(1),
                                         gyroSamples.at(2));
      printf("EMG           %d %d %d %d %d %d %d %d\n", emgSamples.at(0),
                                                        emgSamples.at(1),
                                                        emgSamples.at(2),
                                                        emgSamples.at(3),
                                                        emgSamples.at(4),
                                                        emgSamples.at(5),
                                                        emgSamples.at(6),
                                                        emgSamples.at(7));
    }
  };



  class LIBRARY_API MyoObj : public myo::DeviceListener {
  public:

    //! @brief Default constructor
    //!
    MyoObj();

    //! @brief Callback function called when Myo Connect successfuly pairs with armband
    //!
    //! @param myo             Calling Myo armband object
    //! @Param timestamp       Event timestamp
    //! @param firmwareVersion Myo hardware firmware version
    //!
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);

    //! @brief Process pose change events from a paired Myo armband
    //!
    //! @param myo       Calling Myo armband object
    //! @Param timestamp Event timestamp
    //! @param pose      The hand pose information
    //!
    //! @note Poses are infrequently identified, and it's not clear how useful this function really is
    //!
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);

    //! @brief Process EMG data from a paired Myo armband
    //!
    //! @param myo       Calling Myo armband object
    //! @Param timestamp Event timestamp
    //! @param emg       EMG data
    //!
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);

    //! @brief Process accelerometer data
    //!
    //! @param myo       Calling Myo armband object
    //! @Param timestamp Event timestamp
    //! @param data      Accelerometer data (x, y, z)
    //!
    void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float> &accel);

    //! @brief Process gyroscope data
    //!
    //! @param myo       Calling Myo armband object
    //! @Param timestamp Event timestamp
    //! @param data      Gyroscope data (x, y, z)
    //!
    void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float> &gyro);

    //! @brief Process armband orientation data
    //!
    //! @param myo       Calling Myo armband object
    //! @Param timestamp Event timestamp
    //! @param data      Orientation data (x, y, z, w)
    //!
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float> &rotation);

    //! @brief Process armband connection event
    //!
    //! @param myo             Calling Myo armband object
    //! @Param timestamp       Event timestamp
    //! @param firmwareVersion Myo hardware firmware version
    //!
    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion);

    //! @brief Process armband disconnect event
    //!
    //! @param myo       Calling Myo armband object
    //! @Param timestamp Event timestamp
    //!
    void onDisconnect(myo::Myo* myo, uint64_t timestamp);

    //! @brief Handy little function yoinked from the Myo SDK examples that identifies a specified Myo armband
    //!
    //! @param myo The Myo armband to be identified
    //!
    //! @return The index number of the armband as it is currently known
    //!
    size_t identifyMyo(myo::Myo* myo);

    //! @brief Grab the most up-to-date pushed data from all connected armbands
    //!
    //! @param data Data structure to be populated, contains accelerometer, gyroscope, orientation,
    //!             pose, and EMG data
    //!
    void getData(vector<MyoSubject> &data);

    //! @brief Grab the most up-to-date pushed data from all connected armbands
    //!
    //! @return The most recent accelerometer, gyroscope, orientation, pose, and EMG data
    //!    
    vector<MyoSubject>& getData();

    //! @brief Collection of previously established Myo armbands connected to the computer
    //!
    vector<myo::Myo*> Myos_;

    //! @brief Collection of data stuctures to be returned to the owning program
    //!
    vector<MyoSubject> subjects_;

    //! @brief Mutex for protecting shared data
    //!
    ulapi_mutex_struct *handle_;

    //! @brief Thread object handler
    //!
    void *task_;
  };
}

#endif
