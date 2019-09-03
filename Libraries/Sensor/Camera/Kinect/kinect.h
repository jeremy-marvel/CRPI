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

#ifndef KINECT_H
#define KINECT_H

#include "..\..\..\..\portable.h"
#include "..\..\..\..\types.h"
#include "..\..\..\Reporter\reporter.h"

#define HIRES

#define KINECT_DEPTH_IMAGE_WIDTH 640
#define KINECT_DEPTH_IMAGE_HEIGHT 480
#define KINECT_DEPTH_IMAGE_FPS 30

#ifndef HIRES
#define KINECT_CAMERA_IMAGE_WIDTH 640
#define KINECT_CAMERA_IMAGE_HEIGHT 480
#define KINECT_CAMERA_IMAGE_FPS 30
#else
#define KINECT_CAMERA_IMAGE_WIDTH 1280
#define KINECT_CAMERA_IMAGE_HEIGHT 1024
#define KINECT_CAMERA_IMAGE_FPS 15
#endif


#ifdef WIN32
//! Decide which library you are using (can only have one driver installed)
//#define KINECT_NUI
#define KINECT_OPENNI
#endif

#ifdef KINECT_NUI
#include "..\..\..\ThirdParty\CL_NUI\include\CLNUIDevice.h"
  typedef PBYTE       kinByte;
  typedef PUSHORT     kinShort;
  typedef PDWORD      kinWord;
  typedef PCHAR       kinChar;
  typedef CLNUICamera camType;
  typedef CLNUIMotor  motorType;
  typedef kinByte     imageType;
  typedef kinShort    greyType;
#elif defined KINECT_OPENNI
#include "..\..\..\ThirdParty\OpenNI\include\XnCppWrapper.h"
  using namespace xn;
  typedef DepthGenerator depthCapType;
  typedef ImageGenerator imageCapType;
  typedef unsigned short kinShort;
  typedef unsigned int  kinByte;
  typedef ImageMetaData  imageType;
  typedef DepthMetaData  greyType;
#else
  //! TODO
#endif

typedef unsigned short gryVal;
typedef unsigned char imgVal;

struct GREYimage
{
  //! @brief TODO
  //!
  gryVal *imgData;

  //! @brief TODO
  //!
  unsigned int width;

  //! @brief TODO
  //!
  unsigned int height;

  //! @brief TODO
  //!
  unsigned int bytes;

  //! @brief TODO
  //!
  unsigned int step;

  //! @brief TODO
  //!
  unsigned int depth;

  //! @brife Default constructor
  //!
  GREYimage () :
    width (0),
    height (0),
    bytes (0),
    step (1),
    imgData (NULL)
  {
    depth = sizeof (gryVal) * 8;
  }

  //! @brief Assignment constructor
  //!
  //! @param w TODO
  //! @param h TODO
  //!
  GREYimage (unsigned int w, unsigned int h) :
    step (1)
  {
    width = w;
    height = h;
    depth = sizeof (gryVal) * 8;
    bytes = w * h * step;
    if (imgData != NULL)
    {
      delete [] imgData;
      imgData = NULL;
    }
    imgData = new gryVal[bytes];
  }

  //! @brief Default destructor
  //!
  ~GREYimage ()
  {
    if (imgData != NULL)
    {
      delete [] imgData;
      imgData = NULL;
    }
  }

  //! @brief Adjust the size of the image to a specified height and width
  //!
  //! @param w The new width of the image
  //! @param h The new height of the image
  //!
  void resize (unsigned int w, unsigned int h)
  {
    width = w;
    height = h;
    bytes = w * h * step;
    if (imgData != NULL)
    {
      delete [] imgData;
      imgData = NULL;
    }
    imgData = new gryVal[bytes];
  }

  //! @brief TODO
  //!
  void clear ()
  {
    static unsigned int x;
    if (imgData != NULL)
    {
      for (x = 0; x < bytes; ++x)
      {
        imgData[x] = 0;
      }
    }
  }

};


struct RGBimage
{
  //! @brief TODO
  //!
  imgVal *imgData;

  //! @brief TODO
  //!
  unsigned int width;

  //! @brief TODO
  //!
  unsigned int height;

  //! @brief TODO
  //!
  unsigned int step;

  //! @brief TODO
  //!
  unsigned int bytes;

  //! @brief TODO
  //!
  unsigned int depth;

  //! @brief Default constructor
  //!
  RGBimage () :
    width (0),
    height (0),
    bytes (0),
    step (3),
    imgData (NULL)
  {
    depth = sizeof(imgVal) * 8;
    width = height = bytes = 0;
    imgData = NULL;
  }

  //! @brief Assignment constructor
  //!
  //! @param w TODO
  //! @param h TODO
  //!
  RGBimage (unsigned int w, unsigned int h) :
    step (3)
  {
    width = w;
    height = h;
    depth = sizeof(imgVal) * 8;
    bytes = w * h * step;
    if (imgData != NULL)
    {
      delete [] imgData;
      imgData = NULL;
    }
    imgData = new imgVal[bytes];
  }

  //! @brief Default destructor
  //!
  ~RGBimage ()
  {
    if (imgData != NULL)
    {
      delete [] imgData;
      imgData = NULL;
    }
  }

  //! @brief Adjust the size of the image to a specified height and width
  //!
  //! @param w The new width of the image
  //! @param h The new height of the image
  //!
  void resize (unsigned int w, unsigned int h)
  {
    width = w;
    height = h;
    bytes = w * h * step;
    if (imgData != NULL)
    {
      delete [] imgData;
      imgData = NULL;
    }
    imgData = new imgVal[bytes];
  }

  //! @brief TODO
  //!
  void clear ()
  {
    static unsigned int x;
    if (imgData != NULL)
    {
      for (x = 0; x < bytes; ++x)
      {
        imgData[x] = 0;
      }
    }
  }

  void convert (GREYimage *grey)
  {
    static unsigned int x, y;
    static unsigned char val;
    clear ();
    resize (grey->width, grey->height);
    if (grey->imgData != NULL)
    {
      for (x = 0; x < bytes; x += 3)
      {
        y = x / 3;
        val = (grey->imgData[y] < 1000) ? (255 - (grey->imgData[y] - 500)) : 0;
        imgData[x] = imgData[x+1] = imgData[x+2] = grey->imgData[y] - 500;
      }
    }
  }
};



namespace Sensor
{
  //! @ingroup Sensor
  //!
  //! @brief   TODO
  //!
  class LIBRARY_API Kinect
  {
  public:

    //! @brief Default constructor
    //!
    Kinect ();

    //! @brief Default destructor
    //!
    ~Kinect ();

    //! @brief Initialize the Kinect (turn on the cameras & motor)
    //!
    //! @return Whether or not initialization was successful
    //!
    bool start ();

    //! @brief Halt the Kinect object (turn off the cameras and motor)
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
    bool grabImageFrame (RGBimage *img, bool overRide = false);

    //! @brief Grab a depth frame image from the laser-camera configuration
    //!
    bool grabDepthFrame (GREYimage *img);

    //! @brief 
    //!
    //! @note TODO angle between -31 & 31
    //!
    bool move (short int angle);

    //! @brief Get an estimation of the current pose of the camera based on
    //!        the accelerometer
    //!
    //! @return TODO
    //!
    short int pose ();

    //! @brief TODO
    //!
    //! @param pt  TODO
    //! @param img TODO
    //! @param x   TODO
    //! @param y   TODO
    //!
    //! @return TODO
    //!
    bool pix2world (point& pt, GREYimage &img, unsigned int x, unsigned int y);

    void adjustActiveWindow (unsigned int min, unsigned int max);

  private:

#ifdef KINECT_NUI
    //! @brief TODO
    //!
    double *depthConversion_;

    //! @brief Camera address of the found Kinect
    //!
    camType camera_;

    //! @brief Motor address of the found Kinect
    //!
    motorType motor_;

    //! @brief Self-proclaimed serial number of the first Kinect found
    //!
    kinChar deviceSerial_;

#endif

#ifdef KINECT_OPENNI
    //! @brief TODO
    //!
    Context context_;

    depthCapType depthCap_;

    imageCapType imageCap_;

    XnStatus rc_;
#endif

    //! @brief TODO
    //!
    greyType depthImage_;

    //! @brief TODO
    //!
    imageType cameraImage_;

    //! @brief TODO
    //!
    //! @param img TODO
    //!
    //! @return Whether or not the conversion attempt was successful
    //!
    bool convertImage (RGBimage *img, bool overRide);

    //! @brief TODO
    //!
    //! @param img TODO
    //!
    //! @return Whether or not the conversion attempt was successful
    //!
    bool convertDepth (GREYimage *img);

    //! @brief TODO
    //!
    bool started_;

    unsigned int winMin_;
    unsigned int winMax_;
  }; // Kinect
} // Sensor namespace

#endif