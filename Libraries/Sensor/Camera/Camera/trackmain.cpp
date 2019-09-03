/* ============================================================
   |           Camera Motion Control Main Program             |
   |----------------------------------------------------------|
   | Author:   Jeremy A. Marvel                               |
   | Date:     12 April, 2003                                 |
   ============================================================ 
   Modified 15 May, 2006 (vision code removed) for generic video playback
   Modified 26 June, 2007 for customizable video playback using the camera drivers
   Modified 5 July, 2007 (vision code reinserted) for cell phone assembly task
*/

#ifdef _CH_
#define WIN32
#pragma package <opencv>
#error "The file needs cvaux, which is not wrapped yet."
#endif

#include "socknet.h"
#include "types.h"
#include "xml.h"

#ifndef _EiC
#include <fstream>
#include <cxcore.h>
#include <cvcam.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#endif

#ifdef _EiC
#define WIN32
#endif

//#define TEST        //! disable camera input
//#define ECHOSERVER  //! disable image processing
#define NOISY       //! enable loquacious display
//#define GREY        //! enable greyscale imaging
//#define EDGEDET     //! enable edge detection
#define ALWAYSTRUTH //! override the localization request requirement

using namespace std;

//! =======================================================================
//!                         OpenCV event handlers
//! =======================================================================

//! @brief Generic function for image handling.  Image is rendered automatically.
//!        Any changes made to image are reflected in the rendered picture.
//!
//! @param image The image object passed automatically to the callback function
//!
void callback(IplImage* image);

//! @brief Handle mouse events on the video image
//!
//! @param event  Event code of the mouse action that caused this method call
//! @param x      X image coordinate of the mouse pointer at the time of the event
//! @param y      Y image coordinate of the mouse pointer at the time of the event
//! @param eflags Combination of OpenCV event flags
//! @param param  (Not used in this function implementation)
//!
//! NOTE: OpenCV mouse events are as follows:
//!       CV_EVENT_MOUSEMOVE, CV_EVENT_LBUTTONDOWN, CV_EVENT_RBUTTONDOWN,
//!       CV_EVENT_MBUTTONDOWN, CV_EVENT_LBUTTONUP, CV_EVENT_RBUTTONUP,
//!       CV_EVENT_MBUTTONUP, CV_EVENT_LBUTTONDBLCLK, CV_EVENT_RBUTTONDBLCLK,
//!       CV_EVENT_MBUTTONDBLCLK
//!
//!       OpenCV event flags used are as follows:
//!       CV_EVENT_FLAG_LBUTTON, CV_EVENT_FLAG_RBUTTON, CV_EVENT_FLAG_MBUTTON,
//!       CV_EVENT_FLAG_CTRLKEY, CV_EVENT_FLAG_SHIFTKEY, CV_EVENT_FLAG_ALTKEY
//!
void on_mouse(int event, int x, int y, int eflags, void* param);

//! =======================================================================
//!                     Custom image processing methods
//! =======================================================================

//! @brief Perform erosion on the binary images used in this application
//!
//! @param in     2D input array of size (width, height)
//! @param out    2D output array of size (width, height)
//! @param width  The width of the two images
//! @param height The height of the two images
//!
//! Note:  Background pixels (black) should be set to -2, while foreground
//!        pixels (white) should be set to -1
//!
void erode(int **in, int **out, int width, int height);

//! @brief Perform dilution on the binary images used in this application
//!
//! @param in     2D input array of size (width, height)
//! @param out    2D output array of size (width, height)
//! @param width  The width of the two images
//! @param height The height of the two images
//!
//! Note:  Background pixels (black) should be set to -2, while foreground
//!        pixels (white) should be set to -1
//!
void dilute(int **in, int **out, int width, int height);

//! @brief TODO
//!
//! @param in  TODO
//! @param out TODO
//!
void edge(IplImage *in, IplImage *out);

//! =======================================================================
//!                      Miscellaneous helper methods
//! =======================================================================

//! @brief Load the component origin data from an external file 
//!
void loadSettings();

//! @brief GSI communication server thread - receives and sends data
//!        independently of the main program thread
//!
//! @param param Arguments passed to the thread to avoid needing the
//!              use of global variables (oops)
//!
//! @return Exit status (not utilized in this program)
//!
DWORD __stdcall MainServerThread( LPVOID param);


//! =======================================================================
//!                           Data structures
//! =======================================================================

struct object
{
  double centx;  //! X coordinate of the centroid of the object
  double centy;  //! Y coordinate of the centroid of the object
  double area;   //! Area of the object
  double orient; //! Orientation of the object
  int minx;      //! Minimum X coordinate value of the object
  int miny;      //! Minimum Y coordinate value of the object
  int maxx;      //! Maximum X coordinate value of the object
  int maxy;      //! Maximum Y coordinate value of the object

  object()
  {
    centx = 0;
    centy = 0;
    area = 0;
    orient = 0;
    minx = 100000;
    maxx = 0;
    miny = 100000;
    maxy = 0;
  };
};

struct image
{
  IplImage *frame[3];
  HANDLE mutex[3];

  IplImage *grey;
};

struct flag
{
  bool scrcap;       //! A request for a current screen capture has been made
  bool capped;       //! A screen capture has been taken
  bool grabback;     //! A background screen capture has been requested
  bool firstback;    //! A first-time background screen capture is required
  bool backdefined;  //! The background surface image has been defined
  bool computing;    //! The main program loop is performing computations on the image
  bool grabflag;     //! Semi-semaphore used to ensure that a segmented image is displayed
  int cross;         //! Display the crossbar?

  flag ()
  {
    scrcap = false;
    capped = false;
    grabback = false;
    firstback = true;
    backdefined = false;
    computing = false;
    grabflag = false;
    cross = 0;
  }
};

//! Woohoo!  Global variables!
int counter = 0;
int ce = 0;
int xpos, ypos;
const int xp = 0, yp = 1;
int xvals, yvals;

HANDLE grabmutex;

image images;
flag flags;
SettingsAggregate *settings;
robotStruct *robot;
int bk_b2[2], bk_g2[2], bk_r2[2];
double ***img, ***world;

//! =======================================================================
//!                           Method definitions
//! =======================================================================

int main (int argc, char** argv)
{
  int width, height;
  CvCapture* capture = 0;
#ifndef ECHOSERVER
  LARGE_INTEGER start, end, freaque;
  double result;
  int n, offset, row, col, pt;
#endif
  int threshold = 2000;
  int ncams = cvcamGetCamerasCount(), mouseParam = 0;
  int thresh = 50;
  int option = 0;
  int track = 0;
  int overflow = 0;
  bool override = false; //! Override the requirement that a robot must
                         //! request a truthed value
  int bk_b[2], bk_g[2], bk_r[2];

  //! Segmentation variables
  int **id, **od;

#ifndef ECHOSERVER
  int avgb, avgg, avgr;
  int r1, g1, b1, r2, g2, b2;
  double a, b, c, temp;
  double dtheta, theta, omega, delta, phi, beta, lambda, gamma, tau;
  int y, z, obj;
#endif
  int x;

  double p1x, p1y,
         p2x, p2y,
         p3x, p3y,
         p4x, p4y;
  double x0x, x0y,
         x1x, x1y,
         y0x, y0y,
         y1x, y1y;
  double wx1, wy1;
  int ind1x, ind2x, ind3x, ind4x,
      ind1y, ind2y, ind3y, ind4y;
  double x1, y1, xprime, yprime, ux;

  p1x = p1y = p2x = p2y = p3x = p3y = p4x = p4y = -1;
  x0x = x0y = x1x = x1y = y0x = y0y = y1x = y1y = -1;


#ifdef ALWAYSTRUTH
  override = true;
#endif

  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* contour = 0;

  vector<object> regions;

  cout << "USB Camera Server (V" << VERSION << ")" << endl;
  cout << "Press 'F12' to close." << endl << endl;

  id = NULL;
  od = NULL;

  settings = new SettingsAggregate;
  robot = new robotStruct;
  settings->globalRunServer = true;

  loadSettings();

  //! Multithreading for network communication handling
  DWORD dwThreadId;
  CreateThread (NULL,             //! default security attributes 
                0,                //! use default stack size  
                MainServerThread, //! thread function 
                NULL,             //! argument to thread function 
                0,                //! use default creation flags 
                &dwThreadId);     //! returns the thread identifier 

  images.mutex[0] = CreateMutex (NULL, false, NULL);
  images.mutex[1] = CreateMutex (NULL, false, NULL);
  images.mutex[2] = CreateMutex (NULL, false, NULL);
  grabmutex = CreateMutex (NULL, false, NULL);

  //! Create the main display window
  cvNamedWindow ("Video", 1);
  cvCreateTrackbar ("Threshold", "Video", &thresh, 255, NULL);
  cvCreateTrackbar ("Option", "Video", &option, 3, NULL);
  cvCreateTrackbar ("Track", "Video", &track, 1, NULL);
  cvCreateTrackbar ("Crossbar", "Video", &flags.cross, 1, NULL);

  mouseParam=5;
  cvSetMouseCallback ("Video", on_mouse, &mouseParam);

#ifdef TEST
  //! Load a local image & perform segmentation processes on it
  images.frame[1] = cvLoadImage ("temp1.bmp");
  images.frame[0] = cvLoadImage ("temp2.bmp");
  images.frame[2] = (IplImage*)cvClone(images.frame[1]);
  images.grey = cvCreateImage(cvSize(images.frame[0]->width,
                                     images.frame[0]->height),
                              IPL_DEPTH_8U, 1);

  flags.backdefined = true;
  flags.grabflag = true;
  flags.capped = true;
  track = 1;
  width = images.frame[0]->width;
  height = images.frame[0]->height;

#else
  //! Have the "streaming" video sent to a closed window, while the actual image
  //! will be displayed in the primary window (above) in order to get full mouse
  //! support.
  cvNamedWindow ("temp", 1);
  HWND ex = (HWND)cvGetWindowHandle("temp");
  CloseWindow(GetParent(ex));
  ShowWindow(GetParent(ex), SW_HIDE);

  HWND MyWin = (HWND)cvGetWindowHandle("Video");
  printf ("using camera #%i\n", ncams);
  cvcamSetProperty (0, CVCAM_PROP_ENABLE, &capture);
  cvcamSetProperty (0, CVCAM_PROP_RENDER, CVCAMTRUE);
  cvcamSetProperty (0, CVCAM_PROP_WINDOW,&ex);
  cvcamSetProperty (0, CVCAM_PROP_CALLBACK,callback);
  cvcamGetProperty (0, CVCAM_CAMERAPROPS, NULL);
  cvcamGetProperty (0, CVCAM_VIDEOFORMAT, NULL);

  cvcamGetProperty (0, CVCAM_SRCWIDTH, &width);
  cvcamGetProperty (0, CVCAM_SRCHEIGHT, &height);

  cvcamInit();
  cvcamStart();

  printf("image resolution: %i x %i\n", width, height);

#endif

  bk_b[0] = bk_g[0] = bk_r[0] = bk_b[0] = bk_g[0] = bk_r[0] = 255;
  bk_b[1] = bk_g[1] = bk_r[1] = bk_b2[1] = bk_g2[1] = bk_r2[1] = 0;

  while (true)
  {
    //! Press the "end" key to quit the video application
    if (GetAsyncKeyState (VK_F12) != 0)
    {
      break;
    }
    cvWaitKey (1);

#ifndef ECHOSERVER
    if (flags.scrcap && flags.capped)
    {
      WaitForSingleObject (images.mutex[0], INFINITE);
      n = images.frame[0]->nChannels;
      offset = width * images.frame[0]->nChannels;
      row = ypos * offset;
      col = xpos * n;
      pt = row + col;
      b1 = (unsigned char)images.frame[0]->imageData[pt + 0];
      g1 = (unsigned char)images.frame[0]->imageData[pt + 1];
      r1 = (unsigned char)images.frame[0]->imageData[pt + 2];
      ReleaseMutex(images.mutex[0]);
#ifdef NOISY
      cout << "(" << xpos << ", " << ypos << ") BGR = (" << b1 << ", " 
           << g1 << ", " << r1 << ")" << endl;
#endif

      bk_b[0] = (b1 < bk_b[0] ? b1 : bk_b[0]);
      bk_g[0] = (g1 < bk_g[0] ? g1 : bk_g[0]);
      bk_r[0] = (r1 < bk_r[0] ? r1 : bk_r[0]);
      bk_b[1] = (b1 > bk_b[1] ? b1 : bk_b[1]);
      bk_g[1] = (g1 > bk_g[1] ? g1 : bk_g[1]);
      bk_r[1] = (r1 > bk_r[1] ? r1 : bk_r[1]);
#ifdef NOISY
      cout << "B[" << bk_b[0] << ", " << bk_b[1] << "]  G[" << bk_g[0] << ", "
           << bk_g[1] << "]  R[" << bk_r[0] << ", " << bk_r[1] << "]" << endl;
#endif

      flags.scrcap = false;
#ifndef TEST
      flags.capped = false;
#endif
    } // if (flags.scrcap && flags.capped)

    //! Segmentation
    if (flags.backdefined && flags.grabflag)
    {
      QueryPerformanceCounter(&end);
      if (flags.firstback)
      {
        QueryPerformanceCounter(&start);
        //! Initialize the storage variables if this is the first call to the
        //! segmentation loop
        id = new int*[width];
        od = new int*[width];
        for (x = 0; x < width; ++x)
        {
          id[x] = new int[height];
          od[x] = new int[height];
        }
#ifndef TEST
//        cvNamedWindow( "Components", 1 );
#endif
        flags.firstback = false;
      }

      flags.computing = true;

      WaitForSingleObject (images.mutex[0], INFINITE);
      WaitForSingleObject (images.mutex[1], INFINITE);
      WaitForSingleObject (images.mutex[2], INFINITE);
      overflow = 0;

      avgb = avgg = avgr = 0;
      offset = width * images.frame[1]->nChannels;
      for (y = 0; y < height; ++y)
      {
        row = y * offset;
        for (x = 0; x < width; ++x)
        {
          col = x * images.frame[1]->nChannels;
          pt = row + col;

          avgb += (unsigned char)images.frame[0]->imageData[pt+0];
          avgg += (unsigned char)images.frame[0]->imageData[pt+1];
          avgr += (unsigned char)images.frame[0]->imageData[pt+2];
        }
      }
      avgb /= (width * height);
      avgg /= (width * height);
      avgr /= (width * height);


      //offset = width * images.frame[1]->nChannels;
      for (y = 0; y < height; ++y)
      {
        row = y * offset;
        for (x = 0; x < width; ++x)
        {
          col = x * images.frame[1]->nChannels;
          pt = row + col;

          b1 = (unsigned char)images.frame[1]->imageData[pt+0];
          g1 = (unsigned char)images.frame[1]->imageData[pt+1];
          r1 = (unsigned char)images.frame[1]->imageData[pt+2];
          b2 = (unsigned char)images.frame[0]->imageData[pt+0];
          g2 = (unsigned char)images.frame[0]->imageData[pt+1];
          r2 = (unsigned char)images.frame[0]->imageData[pt+2];

          switch (option)
          {
          case 0:
            //! Background image separation
            if (abs(r1 - r2) <= thresh && abs(g1 - g2) <= thresh && abs(b1 - b2) <= thresh)
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = 0;
              id[x][y] = -2;
            }
            else
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = (unsigned char)255;
              id[x][y] = -1;
              overflow++;
            }
            break;
          case 1:
            //! Color-based background segmentation
            if ((r2 >= bk_r[0] && r2 <= bk_r[1]) && 
                (g2 >= bk_g[0] && g2 <= bk_g[1]) &&
                (b2 >= bk_b[0] && b2 <= bk_b[1]))
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = 0;
              id[x][y] = -2;
            }
            else
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = (unsigned char)255;
              id[x][y] = -1;
              overflow++;
            }
            break;
          case 2:
            //! Predefinition-based color segmentation
            if (abs(avgr - r2) <= thresh && abs(avgg - g2) <= thresh && abs(avgb - b2) <= thresh)
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = 0;
              id[x][y] = -2;
            }
            else
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = (unsigned char)255;
              id[x][y] = -1;
              overflow++;
            }
            break;
          case 3:
            //! Color-based background segmentation
            if ((r2 >= bk_r2[0] && r2 <= bk_r2[1]) && 
                (g2 >= bk_g2[0] && g2 <= bk_g2[1]) &&
                (b2 >= bk_b2[0] && b2 <= bk_b2[1]))
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = 0;
              id[x][y] = -2;
            }
            else
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = (unsigned char)255;
              id[x][y] = -1;
              overflow++;
            }
            break;
          default:
            break;
          }
        } // for (x = 0; x < width; ++x)
      } // for (y = 0; y < height; ++y)
#ifdef NOISY
      cout << overflow << " pixels marked as foreground" << endl;
#endif

      if (track == 0)
      {
        ReleaseMutex(images.mutex[0]);
        ReleaseMutex(images.mutex[1]);
        ReleaseMutex(images.mutex[2]);
#ifndef TEST
        flags.computing = false;
        flags.grabflag = false;
#endif
        continue;
      }


//      if (overflow <= some_threshold)
      if (robot->grabImg > 0 || override)
      {
        //! Segment out the components by subtracting the background from the image
        erode(id, od, width, height);
        dilute(od, id, width, height);
        dilute(id, od, width, height);
        dilute(od, id, width, height);

        offset = width * images.grey->nChannels;
        for (y = 0; y < height; ++y)
        {
          row = y * offset;
          for (x = 0; x < width; ++x)
          {
            col = x * images.grey->nChannels;
            pt = row + col;

            images.grey->imageData[pt] = (id[x][y] > -2 ? 255 : 0);
          }
        }
      
        cvFindContours( images.grey, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

        obj = 1;

        for (; contour != 0; contour = contour->h_next, obj++)
        {
          CvScalar color = CV_RGB( obj, obj, obj );

          //! Replace CV_FILLED with 1 to see the outlines
          cvDrawContours( images.grey, contour, color, color, -1, CV_FILLED, 8 );
        }

        offset = width * images.grey->nChannels;
        for (y = 0; y < height; ++y)
        {
          row = y * offset;
          for (x = 0; x < width; ++x)
          {
            col = x * images.grey->nChannels;
            pt = row + col;

            id[x][y] = (images.grey->imageData[pt] > 0 ? images.grey->imageData[pt]-1 : -2);
          }
        }

        obj--;
#ifndef TEST
        if (flags.cross == 1)
        {
          cvLine (images.grey, cvPoint (width / 2, 0),
                  cvPoint (width/2, height), CV_RGB(0,255,0),
                  1, 0 );
          cvLine (images.grey, cvPoint (0, height / 2),
                  cvPoint (width, height / 2), CV_RGB(0,255,0),
                  1, 0 );
        }
//        cvShowImage( "Components", images.grey );
#endif

#ifdef TEST
        offset = width * images.frame[1]->nChannels;
        for (y = 0; y < height; ++y)
        {
          row = y * offset;
          for (x = 0; x < width; ++x)
          {
            col = x * images.frame[1]->nChannels;
            pt = row + col;

            if (id[x][y] < 0)
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = 0;
            }
            else
            {
              images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] 
                = images.frame[2]->imageData[pt+2] = (unsigned int)((id[x][y]+1) * 50);
            }
          }
        } // for (y = 0; y < height; ++y)
#endif

#ifdef NOISY
        cout << "Found " << obj << " objects in the image." << endl;
#endif

        regions.resize(obj);

        //! Reset/clear object data structure
        for (z = 0; z < obj; ++z)
        {
          regions.at(z).centx = regions.at(z).centy = regions.at(z).area 
            = regions.at(z).orient = 0.0;
          regions.at(z).maxx = regions.at(z).maxy = 0;
          regions.at(z).minx = width;
          regions.at(z).miny = height;
        }

        //! Compute the first moments (i.e. the centroid)
        for (z = 0; z < obj; ++z)
        {
          for (y = 0; y < height; ++y)
          {
            for (x = 0; x < width; ++x)
            {
              if (id[x][y] == z)
              {
                regions.at(z).centx += x;
                regions.at(z).centy += y;

                regions.at(z).minx = (x < regions.at(z).minx ? x : regions.at(z).minx);
                regions.at(z).maxx = (x > regions.at(z).maxx ? x : regions.at(z).maxx);
                regions.at(z).miny = (y < regions.at(z).miny ? y : regions.at(z).miny);
                regions.at(z).maxy = (y > regions.at(z).maxy ? y : regions.at(z).maxy);

                regions.at(z).area += 1;
              }
            }
          }
          regions.at(z).centx /= regions.at(z).area;
          regions.at(z).centy /= regions.at(z).area;
        }

        offset = width * images.frame[1]->nChannels;

        for (z = 0; z < obj; ++z)
        {
          a = b = c = 0;
          //! Compute the second moments
          for (y = 0; y < height; ++y)
          {
            for (x = 0; x < width; ++x)
            {
              if (id[x][y] == z)
              {
                a += (x - regions.at(z).centx) * (x - regions.at(z).centx); //! Uxx
                b += 2 * (x - regions.at(z).centx) * (y - regions.at(z).centy); //! Uxy
                c += (y - regions.at(z).centy) * (y - regions.at(z).centy); //! Uyy
              }
            }
          }

          //! Draw the centroid
          {
            row = (int)regions.at(z).centy * offset;
            col = (int)regions.at(z).centx * images.frame[1]->nChannels;
            pt = row + col;
            images.frame[2]->imageData[pt+2] = (unsigned char)255;
            images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] = 0;

            col = (int)(regions.at(z).centx+1) * images.frame[1]->nChannels;
            pt = row + col;
            images.frame[2]->imageData[pt+2] = (unsigned char)255;
            images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] = 0;

            col = (int)(regions.at(z).centx-1) * images.frame[1]->nChannels;
            pt = row + col;
            images.frame[2]->imageData[pt+2] = (unsigned char)255;
            images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] = 0;

            row = (int)(regions.at(z).centy+1) * offset;
            col = (int)regions.at(z).centx * images.frame[1]->nChannels;
            pt = row + col;
            images.frame[2]->imageData[pt+2] = (unsigned char)255;
            images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] = 0;

            row = (int)(regions.at(z).centy-1) * offset;
            pt = row + col;
            images.frame[2]->imageData[pt+2] = (unsigned char)255;
            images.frame[2]->imageData[pt+0] = images.frame[2]->imageData[pt+1] = 0;
          } //! Centroid drawing

          temp = (b * b) + ((a - c) * (a - c));
          temp = sqrt(temp);

          regions.at(z).orient = atan2(b, (a - c + temp));
          regions.at(z).orient = (-180.0 * regions.at(z).orient) /3.14159265;

          //! Orientation correction for the image orientation
          //! (0, 0) is in the top left corner
          if (regions.at(z).orient < 0)
          {
            regions.at(z).orient += 180.0;
          }

          if (regions.at(z).orient < 10)
          {
            if ((regions.at(z).maxx - regions.at(z).minx) < 
                (regions.at(z).maxy - regions.at(z).miny))
            {
              regions.at(z).orient += 90.0;
            }
          }
          else if (regions.at(z).orient > 175)
          {
            if ((regions.at(z).maxx - regions.at(z).minx) < 
                (regions.at(z).maxy - regions.at(z).miny))
            {
              regions.at(z).orient -= 90.0;
            }
          }
          else if (regions.at(z).orient > 85 && regions.at(z).orient < 95)
          {
            if ((regions.at(z).maxx - regions.at(z).minx) >
                (regions.at(z).maxy - regions.at(z).miny))
            {
              regions.at(z).orient += 90.0;
            }
          }
        } // for (z = 0; z < obj; ++z)

        for (z = 0; z < obj; ++z)
        {
          //! TODO: Component position reporting

          //! Component or noise?
          if (regions.at(z).area > threshold)
          {
            //! Larger than threshold, object is a component

            if (regions.at(z).centx < width/2)
            {
              //! Left half of the image space:  back component
              WaitForSingleObject (grabmutex, INFINITE);
              robot->lOutput.theta = robot->lOrigin.theta - regions.at(z).orient; //! dtheta

              dtheta = fabs(robot->lOutput.theta) * (3.14159265/180.0);
              omega = 1.57079633 - (dtheta / 2.0);

              if (robot->lOutput.theta < 0)
              {
                theta = robot->lOrigin.theta * (3.14159265/180.0);
                if ((180.0 - regions.at(z).orient) > robot->lOrigin.theta)
                {
                  phi = 3.14159265 - omega;
                  lambda = 3.14159265 - (phi + theta);
                  tau = -2;
                }
                else
                {
                  phi = 3.14159265 - theta;
                  lambda = 3.14159265 - (phi + omega);
                  tau = -1;
                }
              }
              else
              {
                theta = regions.at(z).orient * (3.14159265/180.0);
                if (robot->lOrigin.theta > 90)
                {
                  phi = 3.14159265 - omega;
                  lambda = 3.14159265 - (phi + theta);
                  tau = 2;
                }
                else
                {
                  phi = 3.14159266 - theta;
                  lambda = 3.14159265 - (phi + omega);
                  tau = 1;
                }
              }

              delta = sin (dtheta) * (robot->lOrigin.alpha / sin (omega));
//              phi = 3.14159265 - theta;
//              lambda = 3.14159265 - (phi + omega);
              beta = delta * cos(lambda);
              gamma = delta * sin(lambda);

              x1 = regions.at(z).centx;
              y1 = regions.at(z).centy;

              for (int wy = 1; wy < yvals; ++wy)
              {
                for (int wx = 1; wx < xvals; ++wx)
                {
                  if (x1 >= img[wx-1][wy][xp] && x1 <= img[wx][wy][xp] &&
                      x1 >= img[wx-1][wy-1][xp] && x1 <= img[wx][wy-1][xp] &&
                      y1 <= img[wx-1][wy-1][yp] && y1 <= img[wx][wy-1][yp] &&
                      y1 >= img[wx-1][wy][yp] && y1 >= img[wx][wy][yp])
                  {
                    ind1x = ind4x = wx-1;
                    ind2x = ind3x = wx;
                    ind1y = ind2y = wy;
                    ind3y = ind4y = wy-1;

                    p4x = img[ind4x][ind4y][xp];
                    p4y = img[ind4x][ind4y][yp];
                    p3x = img[ind3x][ind3y][xp];
                    p3y = img[ind3x][ind3y][yp];
                    p2x = img[ind2x][ind2y][xp];
                    p2y = img[ind2x][ind2y][yp];
                    p1x = img[ind1x][ind1y][xp];
                    p1y = img[ind1x][ind1y][yp];
                  }
                }
              }

              x0x = (x1 - p1x)/(p2x - p1x);
              x0y = x0x * ((p2y - p1y)/(p2x - p1x));
              x1x = (x1 - p4x)/(p3x - p4x);
              x1y = (x1x * ((p3y - p4y)/(p3x - p4x))) + 1;
              y0y = (y1 - p1y)/(p4y - p1y);
              y0x = y0y * ((p4x - p1x)/(p4y - p1y));
              y1y = (y1 - p2y)/(p3y - p2y);
              y1x = (y1y * ((p3x - p2x)/(p3y - p2y))) + 1;

              ux = (((x0x - x1x) * (y0y - x1y)) - ((x0y - x1y) * (y0x - x1x))) /
                   (((x0y - x1y) * (y1x - y0x)) - ((x0x - x1x) * (y1y - y0y)));

              xprime = y0x + (ux * (y1x - y0x));
              yprime = y0y + (ux * (y1y - y0y));

              wx1 = world[ind1x][ind1y][xp] +
                   ((world[ind2x][ind2y][xp] - world[ind1x][ind1y][xp]) * xprime);
              wy1 = world[ind1x][ind1y][yp] +
                   ((world[ind4x][ind4y][yp] - world[ind1x][ind1y][yp]) * yprime);

//              cout << "computed position: (" << wx1 << " " << wy1 << ")" << endl;

              //robot->lOutput.x = robot->lOrigin.x - regions.at(z).centx; //! dx
              //robot->lOutput.y = robot->lOrigin.y - regions.at(z).centy; //! dy
              robot->lOutput.x = robot->lOrigin.x - wx1; //! dx
              robot->lOutput.y = robot->lOrigin.y - wy1; //! dy

              cout << "theta: " << robot->lOrigin.theta << " " << regions.at(z).orient << " " << dtheta << endl;
              cout << "x: " << robot->lOrigin.x << " " << x1 << " " << wx1 << " " << robot->lOutput.x << endl;
              cout << "y: " << robot->lOrigin.y << " " << y1 << " " << wy1 << " " << robot->lOutput.y << endl;


              robot->lOutput.x -= (tau > 0 ? beta : -beta);
              if (fabs(tau) < 1)
              {
                robot->lOutput.y += (tau > 0 ? -gamma : gamma);
              }
              else
              {
                robot->lOutput.y -= (tau > 0 ? -gamma : gamma);
              }
              

//              robot->lOutput.x /= 3.0315;
//              robot->lOutput.y /= 3.0315;

              ReleaseMutex(grabmutex);
            }
            else
            {
              //! Right half of the image space:  cell phone assembly
              WaitForSingleObject (grabmutex, INFINITE);
              robot->rOutput.theta = robot->rOrigin.theta - regions.at(z).orient; //! dtheta

              dtheta = robot->rOutput.theta * (3.14159265/180.0);
              theta = regions.at(z).orient * (3.14159265/180.0);
              omega = 1.57079633 - (dtheta / 2.0);
              delta = sin (dtheta) * (robot->rOrigin.alpha / sin (omega));
              phi = 3.14159265 - theta;
              beta = sin (omega) * (delta / sin (phi));
              lambda = 3.14159265 - (phi + omega);
              gamma = sin (lambda) * (delta / sin (phi));
              tau = gamma * cos (theta);

              robot->rOutput.x = robot->rOrigin.x - regions.at(z).centx; //! dx
              robot->rOutput.x -= (tau + beta); //! dx' - rotation compensation
              robot->rOutput.x /= 3.0315;

              robot->rOutput.y = robot->rOrigin.y - regions.at(z).centy; //! dy
              robot->rOutput.y -= (gamma * sin (theta)); //! dy'
              robot->rOutput.y /= 3.0315;
/*
              dtheta = fabs(robot->lOutput.theta) * (3.14159265/180.0);
              if (robot->lOutput.theta < 0)
              {
                theta = robot->lOrigin.theta * (3.14159265/180.0);
                tau = -1;
              }
              else
              {
                theta = regions.at(z).orient * (3.14159265/180.0);
                tau = 1;
              }
              omega = 1.57079633 - (dtheta / 2.0);
              delta = sin (dtheta) * (robot->lOrigin.alpha / sin (omega));
              phi = 3.14159265 - theta;
              lambda = 3.14159265 - (phi + omega);
              beta = delta * cos(lambda);
              gamma = delta * sin(lambda);

              robot->lOutput.x = robot->lOrigin.x - regions.at(z).centx; //! dx
              robot->lOutput.x += (tau < 1 ? beta : -beta);
              robot->lOutput.x /= 3.0315;

              robot->lOutput.y = robot->lOrigin.y - regions.at(z).centy; //! dy
              robot->lOutput.y += (tau < 1 ? -gamma : gamma);
              robot->lOutput.y /= 3.0315;
*/


              ReleaseMutex(grabmutex);
            }
          }

#ifdef NOISY
          if (regions.at(z).area > threshold)
          {
            cout << "Object " << z << ": centroid : (" << regions.at(z).centx << ", " 
                 << regions.at(z).centy << ")" << endl;
            cout << "                    orientation: " << regions.at(z).orient 
                 << " degrees" << endl;
            cout << "                    area: " << regions.at(z).area << endl;
          }
#endif
        }
        robot->moveFlag = 2;
      } // if (overflow <= some_threshold)
      else
      {
        robot->moveFlag = 0;
      }

      ReleaseMutex(images.mutex[0]);
      ReleaseMutex(images.mutex[1]);
      ReleaseMutex(images.mutex[2]);

      //! Force a redraw of the image
#ifdef TEST
      if (flags.cross == 1)
      {
        cvLine (images.frame[2], cvPoint (width / 2, 0),
                cvPoint (width/2, height), CV_RGB(0,255,0),
                1, 0 );
        cvLine (images.frame[2], cvPoint (0, height / 2),
                cvPoint (width, height / 2), CV_RGB(0,255,0),
                1, 0 );
      }
      cvShowImage("Video", images.frame[2]);
      counter++;
#else
      //! Force a redraw of the image
      flags.computing = false;
      flags.grabflag = false;
#endif
    } // if (backdefined && grabflag)

#endif // ifndef ECHOSERVER
  } // while (true)

#ifndef ECHOSERVER
  QueryPerformanceFrequency(&freaque);
  result = (double)(end.QuadPart-start.QuadPart)/(double)(freaque.QuadPart);
  result /= (double)counter;
  printf ("frequency: %f FPS: %f\n", result, 1.0/result);
#endif

  if (id != NULL)
  {
    //! Garbage collection
    for (x = 0; x < width; ++x)
    {
      delete [] id[x];
      delete [] od[x];
    }
    delete [] id;
    delete [] od;
  }
  delete robot;

  cvcamStop();
  cvcamExit();

  while (settings->connected)
  {
    Sleep(100);
  }

  delete settings;

  return 0;
}


void callback(IplImage* image)
{
  static double r, g, b, grey;
  static int n = image->nChannels, offset = image->width * image->nChannels,
             row, col, pt;
  static bool firstrun = true;
  static IplImage *img2;

#ifdef GREY
  //! Turn image greyscale
  for (int y = 0; y < image->height; ++y)
  {
    row = y * offset;
    for (int x = 0; x < image->width; ++x)
    {
      //! perform repetitive equations only once to improve performance
      col = x * n;
      pt = row + col;
      b = (unsigned char)image->imageData[pt + 0];
      g = (unsigned char)image->imageData[pt + 1];
      r = (unsigned char)image->imageData[pt + 2];
      grey = (0.3 * r) + (0.59 * g) + (0.11 * b);
      image->imageData[pt + 0] = image->imageData[pt + 1] =
        image->imageData[pt + 2] = (char)grey;
    }
  }
#endif

  if (firstrun)
  {
    WaitForSingleObject (images.mutex[0], INFINITE);
    images.frame[0] = (IplImage*)cvClone(image);
    ReleaseMutex(images.mutex[0]);
    WaitForSingleObject (images.mutex[1], INFINITE);
    images.frame[1] = (IplImage*)cvClone(image);
    ReleaseMutex(images.mutex[1]);
    images.frame[2] = (IplImage*)cvClone(image);
    images.grey = cvCreateImage(cvSize(images.frame[0]->width,
                                       images.frame[0]->height),
                                IPL_DEPTH_8U, 1);

    img2 = (IplImage*)cvClone(image);
    firstrun = false;
  }

  if (!flags.computing)
  {
    WaitForSingleObject (images.mutex[0], INFINITE);
    cvCopy(image, images.frame[0]);
    ReleaseMutex(images.mutex[0]);
    flags.capped = true;
  }

  if (flags.grabback)
  {
    WaitForSingleObject (images.mutex[1], INFINITE);
    cvCopy (image, images.frame[1]);
    bk_b2[0] = bk_g2[0] = bk_r2[0] = 255;
    bk_b2[1] = bk_g2[1] = bk_r2[1] = 0;

    for (int y = 0; y < image->height; ++y)
    {
      row = y * offset;
      for (int x = 0; x < image->width; ++x)
      {
        //! perform repetitive equations only once to improve performance
        col = x * n;
        pt = row + col;
        b = (unsigned char)images.frame[1]->imageData[pt + 0];
        g = (unsigned char)images.frame[1]->imageData[pt + 1];
        r = (unsigned char)images.frame[1]->imageData[pt + 2];

        bk_b2[0] = (b < bk_b2[0] ? b : bk_b2[0]);
        bk_g2[0] = (g < bk_g2[0] ? g : bk_g2[0]);
        bk_r2[0] = (r < bk_r2[0] ? r : bk_r2[0]);
        bk_b2[1] = (b > bk_b2[1] ? b : bk_b2[1]);
        bk_g2[1] = (g > bk_g2[1] ? g : bk_g2[1]);
        bk_r2[1] = (r > bk_r2[1] ? r : bk_r2[1]);
      }
    }


    ReleaseMutex (images.mutex[1]);
    flags.backdefined = true;
    flags.grabback = false;
  }

  if (!flags.backdefined)
  {
#ifdef EDGEDET
    edge(image, img2);
#endif
    if (flags.cross == 1)
    {
      cvLine (image, cvPoint (image->width / 2, 0),
              cvPoint (image->width/2, image->height), CV_RGB(0,255,0),
              1, 0 );
      cvLine (image, cvPoint (0, image->height / 2),
              cvPoint (image->width, image->height / 2), CV_RGB(0,255,0),
              1, 0 );
      cvLine (image, cvPoint (0, 0),
              cvPoint (image->width, 0), CV_RGB(255,0,0),
              1, 0 );
    }
#ifndef EDGEDET
    cvShowImage("Video", image);
#else
    cvShowImage("Video", img2);
#endif
  }
  else
  {
    if (!flags.computing && !flags.grabflag)
    {
      WaitForSingleObject (images.mutex[2], INFINITE);
      if (flags.cross == 1)
      {
        cvLine (images.frame[2], cvPoint (image->width / 2, 0),
                cvPoint (image->width/2, image->height), CV_RGB(0,255,0),
                1, 0 );
        cvLine (images.frame[2], cvPoint (0, image->height / 2),
                cvPoint (image->width, image->height / 2), CV_RGB(0,255,0),
                1, 0 );
      }
      cvShowImage("Video", images.frame[2]);
      ReleaseMutex(images.mutex[2]);
      flags.grabflag = true;
    }
  }

  counter++;
}


void on_mouse(int event, int x, int y, int eflags, void* param)
{
  if (event != CV_EVENT_MOUSEMOVE)
  {
#ifndef ECHOSERVER
    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
      xpos = x;
      ypos = images.frame[0]->height - y;
      flags.scrcap = true;
      break;
    case CV_EVENT_RBUTTONDOWN:
      flags.grabback = true;
      break;
    default:
      break;
    }
#endif
  }
}



void erode(int **in, int **out, int width, int height)
{
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      if (x == 0 || x == (width-1) || y == 0 || y == (height-1))
      {
        out[x][y] = -2;
      }
      else
      {
        if (in[x][y] == -1 && in[x-1][y] == -1 && in[x+1][y] == -1 &&
            in[x][y-1] == -1 && in[x][y+1] == -1)
        {
          out[x][y] = -1;
        }
        else
        {
          out[x][y] = -2;
        }
      }
    } // for (int x = 0; x < width; ++x)
  } // for (int y = 0; y < height; ++y)
}



void dilute(int **in, int **out, int width, int height)
{
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      if (in[x][y] == -1)
      {
        out[x][y] = -1;
        if (x > 0)
        {
          out[x-1][y] = -1;
        }
        if (x < (width-1))
        {
          out[x+1][y] = -1;
        }
        if (y > 0)
        {
          out[x][y-1] = -1;
        }
        if (y < (height-1))
        {
          out[x][y+1] = -1;
        }
      }
      else
      {
        out[x][y] = -2;
      }
    } // for (int x = 0; x < width; ++x)
  } // for (int y = 0; y < height; ++y)
}


void edge(IplImage *in, IplImage *out)
{
  static int width = in->width,
             height = in->height;
  static int n = in->nChannels, offset = in->width * in->nChannels,
             row, row2, col, col2, pt1, pt2, pt3, pt4, b, b2, y, x;

  for (y = 0; y < height-1; ++y)
  {
    row = y * offset;
    row2 = (y+1) * offset;
    for (x = 0; x < width-1; ++x)
    {
      col = x * n;
      col2 = (x+1) * n;
      pt1 = row + col;
      pt2 = row2 + col;
      pt3 = row + col2;
      pt4 = row2 + col2;

      b = (unsigned char)in->imageData[pt1] +
          (unsigned char)in->imageData[pt3] -
          (unsigned char)in->imageData[pt2] -
          (unsigned char)in->imageData[pt4];
      b2 = (unsigned char)in->imageData[pt1] -
           (unsigned char)in->imageData[pt3] +
           (unsigned char)in->imageData[pt2] -
           (unsigned char)in->imageData[pt4];

      b = (b > b2 ? b : b2);

      b2 = -(unsigned char)in->imageData[pt1] -
           (unsigned char)in->imageData[pt3] -
           (unsigned char)in->imageData[pt2] +
           (unsigned char)in->imageData[pt4];

      b = (b > b2 ? b : b2);

      b = (b > 20 ? 255 : 0);

      out->imageData[pt1] = out->imageData[pt1+1] 
        = out->imageData[pt1+2] = (unsigned char)abs(b);
    }
  }
  for (y = 0; y < height; ++y)
  {
    row = y * offset;
    col = x * n;
    pt1 = row + col;
    out->imageData[pt1] = out->imageData[pt1+1] 
        = out->imageData[pt1+2] = (unsigned char)0;
  }
  y = height-1;
  row = y * offset;
  for (x = 0; x < width; ++x)
  {
    col = x * n;
    pt1 = row + col;
    out->imageData[pt1] = out->imageData[pt1+1] 
        = out->imageData[pt1+2] = (unsigned char)0;
  }

}


void loadSettings()
{
  char buffer[256];
  int width, height, x, y;
  Xml::XmlParse parser(robot);
  ifstream inc("config.xml");
  ifstream in("calib.dat");

  if (!inc)
  {
    cout << "Configuration file (config.xml) not found." << endl;
    //! Configuration file not found
  }
  inc.getline(buffer, 256);
  if (!parser.parse(buffer))
  {
    cout << "Error parsing configuration file." << endl;
  }

#ifdef NOISY
  cout << "Origins: " << endl;
  cout << "left: (" << robot->lOrigin.x << ", " << robot->lOrigin.y << ") " << robot->lOrigin.theta << endl;
  cout << "right: (" << robot->rOrigin.x << ", " << robot->rOrigin.y << ") " << robot->rOrigin.theta << endl;
#endif

  if (!in)
  {
    cout << "error!" << endl;
  }

  in >> width >> height >> xvals >> yvals;
  img = new double**[xvals];
  world = new double**[xvals];


  for (x = 0; x < xvals; ++x)
  {
    img[x] = new double*[yvals];
    world[x] = new double*[yvals];
    for (y = 0; y < yvals; ++y)
    {
      img[x][y] = new double[2];
      world[x][y] = new double[2];
    }
  }

  //! Image x values
  for (y = 0; y < yvals; ++y)
  {
    for (x = 0; x < xvals; ++x)
    {
      in >> img[x][y][xp];
    }
  }
  
  //! Image y values
  for (y = 0; y < yvals; ++y)
  {
    for (x = 0; x < xvals; ++x)
    {
      in >> img[x][y][yp];
    }
  }  
  
  //! World x values
  for (y = 0; y < yvals; ++y)
  {
    for (x = 0; x < xvals; ++x)
    {
      in >> world[x][y][xp];
    }
  }
  
  //! World y values
  for (y = 0; y < yvals; ++y)
  {
    for (x = 0; x < xvals; ++x)
    {
      in >> world[x][y][yp];
    }
  }

}



DWORD __stdcall MainServerThread( LPVOID param)
{
  networkStruct nS;
  nS.sP = 3002;
  Network::socketNet sN(settings);
  char inbuffer[REQUEST_MSG_SIZE], outbuffer[1024];
  Xml::XmlParse parser(robot);
  double ts1, ts2;

  robot->rOutput.y = 55;
  robot->seq = 0;

  while (settings->globalRunServer)
  {
    cout << "Creating sensor server..." << endl;
    sN.create (nS);
    cout << nS.address << " connected to server" << endl;

    ts1 = getCurrentTime();
    while (settings->connected)
    {
      if (sN.getData(inbuffer, nS))
      {
        ts1 = getCurrentTime();
#ifdef NOISY
        cout << "  --  " << inbuffer << "\n";
#endif
        try
        {
          parser.parse(inbuffer);
        }
        catch(...)
        {
          cout << "error parsing." << endl;
        }

        WaitForSingleObject (grabmutex, INFINITE);
        try
        {
          parser.encode(outbuffer);
        }
        catch (...)
        {
          cout << "error encoding." << endl;
        }
        ReleaseMutex(grabmutex);

#ifdef NOISY
        cout << " SENT " << outbuffer << "\n";
#endif
        //! Send feedback string to client.
        if(!sN.sendData(outbuffer, nS))
        {
          //! Error in sending
        }

        if (!settings->globalRunServer)
        {
          //! Shutdown command from outside this thread
          break;
        }
      } // if (sN.getData(inbuffer, nS))
      else
      {
        ts2 = getCurrentTime();
        if (ts2 - ts1 > 0.010)
        {
          cout << endl << "Connection timed out.  Closing." << endl;
          sN.closeConnection(nS);
        }
      }

    } // while (settings->connected)
  } // if (!settings->globalRunServer)

#ifdef NOISY
  cout << "Server destroyed" << endl;
#endif
  sN.closeConnection (nS);

  //! TODO: re-create server unless program is shutting down

  return 1;
}

#ifdef _EiC
main (1,"trackmain.cpp");
#endif