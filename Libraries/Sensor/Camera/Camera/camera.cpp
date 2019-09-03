///////////////////////////////////////////////////////////////////////////////
//
//  Original System: 
//  Subsystem:       
//  Workfile:        camera.cpp
//  Revision:        1.0 - 7 August, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
//
///////////////////////////////////////////////////////////////////////////////

#include "camera.h"

namespace Sensor
{
  LIBRARY_API Camera::Camera () :
    started_(false)
  {
    //! TODO
  }


  LIBRARY_API Camera::~Camera ()
  {
    //! TODO
  }


  LIBRARY_API void Camera::linkLogger (Logger *ptr)
  {
    logger_ = ptr;
  }


  LIBRARY_API bool Camera::start ()
  {
    camera_ = cvCreateCameraCapture (0);
    cameraImage_ = cvQueryFrame (camera_);

    started_ = true;
    if (!cameraImage_)
    {
      started_ = false;
    }

    return started_;
  }


  LIBRARY_API bool Camera::stop ()
  {
    cvReleaseCapture(&camera_);
    started_ = false;
    return !started_;
  }


  LIBRARY_API bool Camera::grabImageFrame (RGBimage *in)
  {
    static bool state = true;

    cameraImage_ = cvQueryFrame (camera_);
    if (!cameraImage_)
    {
      return false;
    }
    return convertImage (in);
  }


  bool Camera::convertImage (RGBimage *img)
  {
    static unsigned int x, size;
    if (img == NULL)
    {
      return false;
    }

    if (img->imgData == NULL || img->width != cameraImage_->width ||
        img->height != cameraImage_->height)
    {
      img->resize (cameraImage_->width, cameraImage_->height);
    }

    for (x = 0; x < img->bytes; ++x)
    {
      img->imgData[x] = cameraImage_->imageData[x];
    }
    return true;
  }

} // Sensor


