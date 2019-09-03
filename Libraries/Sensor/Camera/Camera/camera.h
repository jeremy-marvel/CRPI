///////////////////////////////////////////////////////////////////////////////
//
//  Original System: NIST Projects
//  Subsystem:       
//  Workfile:        kinect.h
//  Revision:        1.0 - 6 January, 2011
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CAMERA_H
#define CAMERA_H

#include "..\..\..\..\portable.h"
#include "..\..\..\..\types.h"
#include "..\..\..\Reporter\reporter.h"
#include "..\..\..\ThirdParty\OpenCV\include\cxcore.h"
//#include "..\..\..\ThirdParty\OpenCV\include\cvaux.h"
#include "..\..\..\ThirdParty\OpenCV\include\cv.h"
#include "..\..\..\ThirdParty\OpenCV\include\highgui.h"
/*
#include <cxcore.h>
#include <cvcam.h>
#include <cv.h>
#include <highgui.h>
*/

#define HIRES

#ifndef HIRES
#define CAMERA_IMAGE_WIDTH 640
#define CAMERA_IMAGE_HEIGHT 480
#define CAMERA_IMAGE_FPS 30
#else
#define CAMERA_IMAGE_WIDTH 1280
#define CAMERA_IMAGE_HEIGHT 1024
#define CAMERA_IMAGE_FPS 15
#endif


/*
  using namespace xn;
  typedef DepthGenerator depthCapType;
  typedef ImageGenerator imageCapType;
  typedef unsigned short kinShort;
  typedef unsigned int  kinByte;
  typedef ImageMetaData  imageType;
  typedef DepthMetaData  greyType;
*/

using namespace Reporter;

namespace Sensor
{
  //! @ingroup Sensor
  //!
  //! @brief   TODO
  //!
  class LIBRARY_API Camera
  {
  public:

    //! @brief Default constructor
    //!
    Camera ();

    //! @brief Default destructor
    //!
    ~Camera ();

    //! @brief TODO
    //!
    void linkLogger (Logger *ptr);

    //! @brief Initialize the Camera (turn on the imager)
    //!
    //! @return Whether or not initialization was successful
    //!
    bool start ();

    //! @brief Halt the Camera object
    //!
    //! @return Whether or not the stop command was successful
    //!
    //! @note Currently there is a problem with the CL NUI library that
    //!       causes a program crash when calling StopNUICamera.  The error
    //!       seems to stem from calling any of the GetNUICamera______
    //!       functions.
    //!
    bool stop ();

    //! @brief Grab an image from the color camera
    //!
    bool grabImageFrame (RGBimage *img);

  private:

    //! @brief Camera address of the found Kinect
    //!
    CvCapture* camera_;

    //! @brief TODO
    //!
    IplImage* cameraImage_;

    //! @brief TODO
    //!
    //! @param img TODO
    //!
    //! @return Whether or not the conversion attempt was successful
    //!
    bool convertImage (RGBimage *img);

    //! @brief TODO
    //!
    bool started_;

    //! @brief TODO
    //!
    Logger *logger_;
  }; // Camera
} // Sensor namespace

#endif