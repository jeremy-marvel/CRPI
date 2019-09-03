///////////////////////////////////////////////////////////////////////////////
//
//  Original System: 
//  Subsystem:       
//  Workfile:        KinectTest.cpp
//  Revision:        1.0 - 7 January, 2011
//  Author:          J. Marvel
//
//  Description
//  ===========
//  TODO
///////////////////////////////////////////////////////////////////////////////

#include "../../portable.h"
#include "../../types.h"
#include "../../Libraries/Sensor/Camera/Kinect/kinect.h"
#include <iostream>
#include <fstream>

#ifndef _EiC
#include "../../Libraries/ThirdParty/OpenCV/include/cxcore.h"
#include "../../Libraries/ThirdParty/OpenCV/include/highgui.h"
#endif

#define COLOR
#define DEPTH
#define DISPLAY
//#define MOVE
//#define POINTCLOUD
//#define IMAGEOUT
//#define POSETEST

using namespace std;
using namespace Sensor;

void excp (const char *origin, const char *what);

void main()
{
  unsigned int winMax = 600, winMin = 500;
  unsigned int y;
  char name[32];
  double ms;
  int ci = 0;
  bool state = true;
  Kinect knct;
  RGBimage RGBimg;
  GREYimage GREYimg;
  //! JAM: Disabled until integrated with ulapi timer
  //timer timmy;
#ifdef DISPLAY
  char c;
#ifdef COLOR
  IplImage *cImage;
#endif
#ifdef DEPTH
  IplImage *gImage;
#ifdef IMAGEOUT
  IplImage *gcImage;
#endif
#endif
#endif

  state = knct.start ();
  cout << (state ? "Kinect camera started" : "Could not start device") << endl;

  if (!state)
  {
    return;
  }

#ifdef MOVE
  state = true;
  short x;
  cout << "Move Test:" << endl;
  cout << "  Move to -31 degrees: ";
  for (x = 0; x >= -31; --x)
  {
    state &= knct.move (x);
    Sleep (50);
  }
  cout << (state ? "Okay" : "Fail") << endl;
  state = true;
  cout << "  Move to +31 degrees: ";
  for (x = -31; x <= 31; ++x)
  {
    state &= knct.move (x);
    Sleep (50);
  }
  cout << (state ? "Okay" : "Fail") << endl;
  state = true;
  cout << "  Center camera: ";
  for (x = -31; x <= 0; ++x)
  {
    state &= knct.move (x);
    Sleep (50);
  }
  cout << (state ? "Okay" : "Fail") << endl;
#endif

#ifdef COLOR
  RGBimg.resize(KINECT_CAMERA_IMAGE_WIDTH,
                KINECT_CAMERA_IMAGE_HEIGHT);
  state &= knct.grabImageFrame (&RGBimg);
  if (!state)
  {
    cout << "oops!" << endl;
  }
#ifdef DISPLAY
  cout << "Displaying color frame" << endl;
  cImage = cvCreateImage (cvSize(RGBimg.width, RGBimg.height), RGBimg.depth, RGBimg.step);
  cvSetData (cImage, RGBimg.imgData, cImage->widthStep);
  cvNamedWindow ("Image", CV_WINDOW_AUTOSIZE);
#endif
#endif

#ifdef DEPTH
  GREYimg.resize(KINECT_DEPTH_IMAGE_WIDTH,
                 KINECT_DEPTH_IMAGE_HEIGHT);
  state &= knct.grabDepthFrame (&GREYimg);
#ifdef DISPLAY
  cout << "Displaying depth frame" << endl;
  gImage = cvCreateImage (cvSize(GREYimg.width, GREYimg.height), GREYimg.depth, GREYimg.step);
  cvSetData (gImage, GREYimg.imgData, gImage->widthStep);
  cvNamedWindow ("Depth", CV_WINDOW_AUTOSIZE);
#endif
#endif

  y = 0;
  //timmy.start();
  state = true;
  cout << "Streaming images..." << "Press ESC key to quit" << endl;
  while (true)
  {
#ifdef POSETEST
    ms = knct.pose ();
#endif

#ifdef COLOR
    state &= knct.grabImageFrame (&RGBimg);
#ifdef DISPLAY
    cvSetData (cImage, RGBimg.imgData, cImage->widthStep);
    cvShowImage("Image", cImage);
#endif
#endif

#ifdef DEPTH
    state &= knct.grabDepthFrame (&GREYimg);
#ifdef DISPLAY
    cvSetData (gImage, GREYimg.imgData, gImage->widthStep);
    cvShowImage("Depth", gImage);
#endif
#endif
    ++y;
#ifdef DISPLAY
    c = cvWaitKey(5);
    if (c == 27) // Escape key
    {
      break;
    }
    else if (c == ' ')
    {
      sprintf (name, "image_%d.jpg", ci++);
      cvSaveImage(name, cImage);
      cout << "wrote " << name << endl;
    }
    else if (c > 0)
    {
      switch (c)
      {
      case 45:   //! '-' or '_'
      case 95:
        --winMin;
        break;
      case 61:   //! '=' or '+'
      case 43:
        ++winMin;
        break;
      case 91:   //! '[' or '{'
      case 123:
        --winMax;
        break;
      case 93:   //! ']' or '}'
      case 125:
        ++winMax;
        break;
      default:
        break;
      }
      cout << "<" << winMin << ", " << winMax << ">" << endl;
      knct.adjustActiveWindow(winMin, winMax);
    }
#else
#ifdef WIN32
    if (keyPressed (KEY_ESC))
    {
      break;
    }
#endif
#endif
  }
  /*
  //! JAM: Disabled until integrated with ulapi timer
  ms = timmy.stop ();
  cout << "Stopped streaming images after " << ms << "ms" << endl;
  cout << "Performance average: " << y << " frames" << endl;
  cout << "                     " << ms/y << "ms per frame" << endl;
  cout << "                     " << y/(ms/1000.0) << "Hz" << endl;
  */
#ifdef DISPLAY
#ifdef IMAGEOUT
#ifdef COLOR
  cvSaveImage("colorImage.jpg", cImage);
  state &= knct.grabImageFrame (&RGBimg, true);
  cvSetData (cImage, RGBimg.imgData, cImage->widthStep);
  cvSaveImage("colorImageOverride.jpg", cImage);
#endif
#ifdef DEPTH
  RGBimg.convert(&GREYimg);
  gcImage = cvCreateImage (cvSize(RGBimg.width, RGBimg.height), RGBimg.depth, RGBimg.step);
  cvSetData (gcImage, RGBimg.imgData, gcImage->widthStep);
  cvSaveImage("greyImage.jpg", gcImage);
#endif
#endif
#endif

#ifdef DISPLAY
#ifdef COLOR
  cvDestroyWindow("Image");
  cvReleaseImage (&cImage);
#endif
#ifdef DEPTH
  cvDestroyWindow("Depth");
  cvReleaseImage (&gImage);
#endif
#endif

#ifdef DEPTH
#ifdef POINTCLOUD
  unsigned int z;
  cout << "Writing logplay.m" << endl;
  ofstream fout ("logplay.m");
  unsigned int inc = 1;
  point pt;
  vector<double> xvec, yvec, zvec;
  vector<double>::iterator iter;

  for (y = 0; y < GREYimg.height; y += inc)
  {
    for (z = 0; z < GREYimg.width; z += inc)
    {
      if (GREYimg.imgData[(y * GREYimg.width) + z] > winMin &&
          GREYimg.imgData[(y * GREYimg.width) + z] < winMax)
      {
        knct.pix2world(pt, GREYimg, z, y);
        zvec.push_back(pt.z);
        xvec.push_back(pt.x);
        yvec.push_back(pt.y);
      }
    }
  }

  fout << "x = [";
  //! Save x vector
  for (iter = xvec.begin(); iter != xvec.end(); ++iter)
  {
    fout << *iter << "; ";
  }
  fout << "];" << endl;

  //! Save y vector
  fout << "y = [";
  for (iter = yvec.begin(); iter != yvec.end(); ++iter)
  {
    fout << *iter << "; ";
  }
  fout << "];" << endl;

  // save the matrix z(y(i),x(j))
  fout << "z = [";
  for (iter = zvec.begin(); iter != zvec.end(); ++iter)
  {
    fout << *iter << "; ";
  }
  fout << "];" << endl;
  fout << "plot3(x,z,-y,'.');" << endl;
#endif
#endif

  cout << (knct.stop () ? "Finished demo" : "Could not stop device") << endl;
}


void excp (const char *origin, const char *what)
{
  cout << origin << " " << what << endl;
}
