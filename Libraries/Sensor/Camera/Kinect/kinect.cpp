///////////////////////////////////////////////////////////////////////////////
//
//  Original System: 
//  Subsystem:       
//  Workfile:        Kinect.cpp
//  Revision:        1.0 - 7 August, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
//
///////////////////////////////////////////////////////////////////////////////

#include "kinect.h"

//#define PROTOTYPE

namespace Sensor
{
  LIBRARY_API Kinect::Kinect () :
    started_(false),
    winMin_(500),
    winMax_(1500)
  {
#ifdef KINECT_NUI
    unsigned int x;
    depthImage_ = (greyType) new greyType[KINECT_DEPTH_IMAGE_WIDTH * KINECT_DEPTH_IMAGE_HEIGHT];
    cameraImage_ = (imageType) new imageType[KINECT_CAMERA_IMAGE_WIDTH * KINECT_CAMERA_IMAGE_HEIGHT * 3];

    deviceSerial_ = '\0';
    camera_ = NULL;
    motor_ = NULL;

    //! Values taken from the MRPT reference (http://reference.mrpt.org) to transform
    //! raw values to meters
    double k1 = 1.1863f, k2 = 2842.5f, k3 = 0.1236f;
    double scale = 1000.0f; // mm

    depthConversion_ = new double[1024];
    for (x = 1; x < 1023; ++x)
    {
      depthConversion_[x] = k3 * tan(x/k2 + k1) * scale;
    }
    depthConversion_[0] = depthConversion_[1023] = 0.0f;
#elif defined KINECT_OPENNI
    rc_ = XN_STATUS_OK;

#else
    //! TODO
#endif
  }


  LIBRARY_API Kinect::~Kinect ()
  {
#ifdef KINECT_NUI
    if (camera_ != NULL)
    {
      DestroyNUICamera (camera_);
    }
    if (motor_ != NULL)
    {
      DestroyNUIMotor (motor_);
    }

    delete [] depthImage_;
    delete [] cameraImage_;
    delete [] depthConversion_;
#elif defined KINECT_OPENNI
    //! TODO
#else
    //! TODO
#endif
  }


 LIBRARY_API bool Kinect::start ()
  {
#ifdef KINECT_NUI
    if (GetNUIDeviceCount() < 1)
    {
      started_ = false;
      return false;
    }

    //! Create the serial connection to the Kinect
    if (deviceSerial_ == '\0')
    {
      deviceSerial_ = GetNUIDeviceSerial (0);
    }

    if (motor_ == NULL && deviceSerial_ != '\0')
    {
      motor_ = CreateNUIMotor (deviceSerial_);
    }
    
    if (camera_ == NULL && deviceSerial_ != '\0')
    {
      //! Create the camera device if not done so already
      camera_ = CreateNUICamera (deviceSerial_);
    }
    if (camera_ != NULL && motor_ != NULL)
    {
      started_ = StartNUICamera (camera_);
    }
    SetNUIMotorLED (motor_, 1);
#elif defined KINECT_OPENNI
    EnumerationErrors errors;

    rc_ = context_.Init();
    if (rc_ == XN_STATUS_NO_NODE_PRESENT)
    {
      XnChar strError[1024];
      errors.ToString (strError, 1024);
      printf("%s\n", strError);
      return false;
    }
    else if (rc_ != XN_STATUS_OK)
    {
      printf ("Start failed: %s\n", xnGetStatusString(rc_));
      return false;
    }

    rc_ = depthCap_.Create (context_);
    if (rc_ != XN_STATUS_OK)
    {
      printf ("bad depth create\n");
      return false;
    }
    rc_ = imageCap_.Create (context_);
    if (rc_ != XN_STATUS_OK)
    {
      printf ("bad image create\n");
      return false;
    }

    rc_ = imageCap_.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
    if (rc_ != XN_STATUS_OK)
    {
      printf ("bad set image pixel format\n");
      return false;
    }

    XnMapOutputMode outputMode;
    outputMode.nXRes = KINECT_CAMERA_IMAGE_WIDTH;
    outputMode.nYRes = KINECT_CAMERA_IMAGE_HEIGHT;
    outputMode.nFPS = KINECT_CAMERA_IMAGE_FPS;
    rc_ = imageCap_.SetMapOutputMode(outputMode);
    if (rc_ != XN_STATUS_OK)
    {
      printf ("bad set image output mode\n");
      return false;
    }

    outputMode.nXRes = KINECT_DEPTH_IMAGE_WIDTH;
    outputMode.nYRes = KINECT_DEPTH_IMAGE_HEIGHT;
    outputMode.nFPS = KINECT_DEPTH_IMAGE_FPS;
    rc_ = depthCap_.SetMapOutputMode(outputMode);
    if (rc_ != XN_STATUS_OK)
    {
      printf ("bad output set mode\n");
    }


#ifdef PROTOTYPE
    rc_ = depthCap_.GetAlternativeViewPointCap().SetViewPoint(imageCap_);
    if (rc_ != XN_STATUS_OK)
    {
      printf ("bad viewpoint conversion\n");
    }
#endif

    //rc_ = depthCap_.StartGenerating();
    //rc_ = imageCap_.StartGenerating();

    rc_ = context_.StartGeneratingAll ();
    if (rc_ != XN_STATUS_OK)
    {
      printf ("bad generation\n");
      return false;
    }
    started_ = true;
    //! TODO
#else
    //! TODO
#endif

    return started_;
  }


  LIBRARY_API bool Kinect::stop ()
  {
    static bool returnMe;
#ifdef KINECT_NUI
    if (camera_ != NULL || !started_)
    {
      SetNUIMotorLED (motor_, 0);
      started_ = (StopNUICamera (camera_) ? false : started_);
    }
    else
    {
      //! Error: Camera not started
      started_ = false;
      return false;
    }
#elif defined KINECT_OPENNI
    rc_ = context_.StopGeneratingAll ();
    if (rc_ != XN_STATUS_OK)
    {
      printf("could not stop generating\n");
    }
    started_ = false;

    context_.Shutdown ();
#else
    //! TODO
#endif
    
    return !started_;
  }

  //! @note angle between -31 & 31
  //!
  LIBRARY_API bool Kinect::move (short int angle)
  {
    static short int setAngle;
    static bool returnMe;

    setAngle = ((angle < -31) ? -31 : (angle > 31) ? 31 : angle) * (0x4000)/31;

#ifdef KINECT_NUI
    if (motor_ == NULL)
    {
      return false;
    }

    return SetNUIMotorPosition (motor_, setAngle);
#elif defined KINECT_OPENNI
    //! TODO
    return false;
#else
    //! TODO
#endif
  }


  LIBRARY_API short int Kinect::pose ()
  {
    static short int getX, getY, getZ;
    static bool state;
#ifdef KINECT_NUI
    state = GetNUIMotorAccelerometer(motor_, getX, getY, getZ);
    return (short)(-32 + ((getY+420) * (62.0f / 790.0f)));
#elif defined KINECT_OPENNI
    //! TODO
    return false;
#else
    //! TODO
#endif
  }


  LIBRARY_API bool Kinect::grabImageFrame (RGBimage *in, bool overRide)
  {
    static bool state;
#ifdef KINECT_NUI
    state = GetNUICameraColorFrameRGB24 (camera_, cameraImage_, 500);
#elif defined KINECT_OPENNI
    state = (context_.WaitOneUpdateAll(imageCap_) == XN_STATUS_OK);
    if (!state)
    {
      printf ("no camera update\n");
      return false;
    }

    state &= (imageCap_.IsValid () == TRUE);
    if (state)
    {
      imageCap_.GetMetaData(cameraImage_);
#ifdef PROTOTYPE
      context_.WaitOneUpdateAll(depthCap_);
      depthCap_.GetMetaData(depthImage_);
#endif
    }
    else
    {
      printf ("not valid\n");
    }

#else
    //! TODO
#endif

    if (state)
    {
      state &= convertImage (in, overRide);
    }
    if (!state)
    {
      printf ("bad conversion\n");
    }
    return state;
  }


  LIBRARY_API bool Kinect::grabDepthFrame (GREYimage *in)
  {
    //! Wait half a second at most
    static bool state;
#ifdef KINECT_NUI
    state = GetNUICameraDepthFrameRAW (camera_, depthImage_, 500);
#elif defined KINECT_OPENNI
    state = (context_.WaitOneUpdateAll(depthCap_) == XN_STATUS_OK);
    if (!state)
    {
      printf ("no depth update\n");
      return false;
    }

    state &= (depthCap_.IsValid () == TRUE);
    if (state)
    {
      depthCap_.GetMetaData(depthImage_);
    }
    else
    {
      printf ("not valid\n");
    }
#else
    //! TODO
#endif

    if (state)
    {
      state &= convertDepth (in);
    }
    return state;
  }


  bool Kinect::convertImage (RGBimage *img, bool overRide)
  {
#ifdef KINECT_OPENNI
    static const XnRGB24Pixel *imgPtr;
#ifdef PROTOTYPE
    static const XnDepthPixel *dpthPtr;
#endif
#endif
    static unsigned int x, size;
    if (img == NULL)
    {
      return false;
    }

    if (img->imgData == NULL || img->width != KINECT_CAMERA_IMAGE_WIDTH ||
        img->height != KINECT_CAMERA_IMAGE_HEIGHT)
    {
      img->resize (KINECT_CAMERA_IMAGE_WIDTH, KINECT_CAMERA_IMAGE_HEIGHT);
    }

#ifdef KINECT_NUI
    size = img->bytes;
#elif defined KINECT_OPENNI
    imgPtr = cameraImage_.RGB24Data();
    size = ((cameraImage_.FullXRes() < img->width) ? cameraImage_.FullXRes() : img->width) *
           ((cameraImage_.FullYRes() < img->height) ? cameraImage_.FullYRes() : img->height);
#ifdef PROTOTYPE
    dpthPtr = depthImage_.Data();
#endif
#endif

    unsigned int bytes = img->height * img->width;
    for (x = 0; x < size; ++x)
    {
#ifdef KINECT_NUI
      img->imgData[x] = (imgVal)(cameraImage_[x]);
#elif defined KINECT_OPENNI
      //! Convert to BGR format
#ifdef PROTOTYPE
      if (overRide || (dpthPtr[x] > winMin_ && dpthPtr[x] < winMax_))
#endif
      {
        img->imgData[(x*img->step)+0] = imgPtr[x].nBlue;
        img->imgData[(x*img->step)+1] = imgPtr[x].nGreen;
        img->imgData[(x*img->step)+2] = imgPtr[x].nRed;
      }
#ifdef PROTOTYPE
      else
      {
        img->imgData[(x*img->step)+0] = img->imgData[(x*img->step)+1] = img->imgData[(x*img->step)+2] = 0;
      }
#endif


#else
      //! TODO
#endif
    }
    return true;
  }


  bool Kinect::convertDepth (GREYimage *img)
  {
#ifdef KINECT_OPENNI
    static const XnDepthPixel *imgPtr;
#endif
    static unsigned int size;
    static unsigned int x;

    if (img == NULL)
    {
      //! Error unallocated image data
      return false;
    }

    if (img->imgData == NULL || img->width != KINECT_DEPTH_IMAGE_WIDTH ||
        img->height != KINECT_DEPTH_IMAGE_HEIGHT)
    {
      img->resize (KINECT_DEPTH_IMAGE_WIDTH, KINECT_DEPTH_IMAGE_HEIGHT);
    }
    static unsigned int z = ((img->height / 4) + 18) * (img->width * img->step);

#ifdef KINECT_NUI
    size = img->bytes;
#elif defined KINECT_OPENNI
    imgPtr = depthImage_.Data();
    size = ((depthImage_.FullXRes() < img->width) ? depthImage_.FullXRes() : img->width) *
           ((depthImage_.FullYRes() < img->height) ? depthImage_.FullYRes() : img->height);
#endif

    unsigned int bytes = img->height * img->width;
    for (x = 0; x < size; ++x)
    {
#ifdef KINECT_NUI
      img->imgData[x] = (imgVal)(cameraImage_[x]);
#elif defined KINECT_OPENNI
      img->imgData[x] = imgPtr[x];
#else
      //! TODO
#endif
    }
    return true;
  }

  LIBRARY_API bool Kinect::pix2world (point& pt,
                                      GREYimage &img,
                                      unsigned int x,
                                      unsigned int y)
  {
    static unsigned int index;
    index = (y * img.width) + x;
#ifdef KINECT_NUI
    //! TODO
    pt.x = x;
    pt.y = y;
    pt.z = img.imgData[index];
#elif defined KINECT_OPENNI
    static double mult, val;
    static XnUInt64 zpd;  // Zero plane distance:  focal length in mm
    static XnDouble pixSize; // Pixel size at zero plane

    depthCap_.GetIntProperty("ZPD", zpd);
    depthCap_.GetRealProperty("ZPPS", pixSize);
    val = (double)(img.imgData[index]);
    mult = val * pixSize * 1.0/(double)zpd;

    pt.x = ((double)x-(img.width / 2)) * mult;
    pt.y = ((double)y-(img.height / 2)) * mult;
    pt.z = val;
#else
    //! TODO
#endif
    return false;
  }


  LIBRARY_API void Kinect::adjustActiveWindow (unsigned int min, unsigned int max)
  {
    winMin_ = (min < max) ? min : max;
    winMax_ = (max > min) ? max : min;
  }

}