///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Genetic Algorithms
//  Subsystem:       Generic Multithreaded Demonstration
//  Workfile:        multi.cpp
//  Revision:        1.0 - 17 March, 2010
//  Author:          J. Marvel
//
//  Description
//  ===========
//  A stupid little program that doesn't really do anything except demo how to
//  use multithreading in a Windows application using VC6.0.  The 'main'
//  function creates two threads and then listens for keyboard input.  The two
//  threads then act on whatever key was pressed.
///////////////////////////////////////////////////////////////////////////////

#include <windows.h>
#include <iostream>

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//                           DATA TYPE DEFINITIONS                           //
///////////////////////////////////////////////////////////////////////////////

//! @brief Bundle of variables used for this demonstration.  Access to shared
//!        values is needed for communicating between threads.
//!
struct passMe
{
  HANDLE grabmutex; //! Semaphore used for data protection
  char c;           //! The shared variable we're most interested in
  bool t1, t2;      //! Whether threads 1 and 2 are active

  //! @brief Default constructor
  //!
  passMe ()
  {
    grabmutex = CreateMutex (NULL, false, NULL);
    t1 = t2 = false;
    c = '0';
  }
};


///////////////////////////////////////////////////////////////////////////////
//                           FUNCTION DECLARATIONS                           //
///////////////////////////////////////////////////////////////////////////////

//! @brief Display an error message window alerting the user to a program problem
//!
//! @param where Which function generated the error (for debugging purposes)
//! @param what  The nature of the error
//!
void excp (char *where, const char *what);

//! @brief Primary thread that acts on the key 'g'
//!
//! @param param Arguments passed to the thread to avoid needing the use of
//!              global variables
//!
//! @return Exit status (not utilized)
//!
DWORD __stdcall MainDemoThread (LPVOID param);

//! @brief Secondary thread that acts on the key 'h'
//!
//! @param param Arguments passed to the thread to avoid needing the use of
//!              global variables
//!
//! @return Exit status (not utilized)
//!
DWORD __stdcall SecondaryDemoThread (LPVOID param);


///////////////////////////////////////////////////////////////////////////////
//                            FUNCTION DEFINITIONS                           //
///////////////////////////////////////////////////////////////////////////////

int main ()
{
  passMe pm; //! State variable used to communicate with the two threads
  char c;    //! Input character
  bool flag; //! Exit status flag

  DWORD dwD1Id, dwD2Id; //! IDs for the two threads

  //! Create thread #1
  CreateThread (NULL,           //! Default security attributes
                0,              //! Use default stack size?
                MainDemoThread, //! Thread function
                &pm,            //! Parameter to thread function
                0,              //! Use default creation flags?
                &dwD1Id);       //! Thread identifier

  //! Create thread #2
  CreateThread (NULL, 0, SecondaryDemoThread, &pm, 0, &dwD2Id);

  cout << "Program running.  Type characters and press the Enter key." << endl;

  //! Wait for user input from keyboard & keep checking until 'q' is pressed
  while (true)
  {
    cin >> c;

    //! Use a semaphor to protect your shared variables!  If two threads try
    //! to access the same variable at the same time, nasty, horrible things
    //! can happen.  Also, don't hold on to the semaphore any longer than is
    //! absolutely necessary.  Other threads are waiting for you to finish.
    WaitForSingleObject (pm.grabmutex, INFINITE);
    pm.c = c;
    ReleaseMutex (pm.grabmutex);

    if (c == 'q')
    {
      break;
    }
  } // while (true)

  //! Wait for both threads to exit
  WaitForSingleObject (pm.grabmutex, INFINITE);
  flag = pm.t1 || pm.t2;
  ReleaseMutex (pm.grabmutex);

  while (flag)
  {
    WaitForSingleObject (pm.grabmutex, INFINITE);
    flag = pm.t1 || pm.t2;
    ReleaseMutex (pm.grabmutex);
  }

  //! Program finished.  Yay, you're done.  Get a job.
	return 0;
} // int main ()


DWORD __stdcall MainDemoThread (LPVOID param)
{
  //! Explicitly cast your shared data back from a void* to the proper structure
  passMe *pm = (passMe*)param;

  char buffer[256];
  int count = 0;
  char c;

  //! Inform the main function that this thread is running.
  WaitForSingleObject (pm->grabmutex, INFINITE);
  pm->t1 = true;
  ReleaseMutex (pm->grabmutex);

  while (true)
  {
    WaitForSingleObject (pm->grabmutex, INFINITE);
    c = pm->c;
    ReleaseMutex (pm->grabmutex);

    if (c == 'g')
    {
      sprintf (buffer, "I waited %d cycles before you pressed g!", count);
      excp ("Secondary Thread", buffer);
      WaitForSingleObject (pm->grabmutex, INFINITE);
      pm->c = '0';
      ReleaseMutex (pm->grabmutex);
      count = 0;
    } // if (c == 'g')
    else if (c == 'q')
    {
      break;
    }
    else
    {
      ++count;
    }

    //! Don't slam your processor!  You don't need to poll at full speed.
    Sleep (100);
  } // while (true)

  WaitForSingleObject (pm->grabmutex, INFINITE);
  pm->t1 = false;
  ReleaseMutex (pm->grabmutex);

  return 1;
} // DWORD __stdcall MainDemoThread (LPVOID param)


DWORD __stdcall SecondaryDemoThread (LPVOID param)
{
  //! Explicitly cast your shared data back from a void* to the proper structure
  passMe *pm = (passMe*)param;

  char buffer[256];
  char c;
  int count = 0;

  //! Inform the main function that this thread is running.
  WaitForSingleObject (pm->grabmutex, INFINITE);
  pm->t2 = true;
  ReleaseMutex (pm->grabmutex);

  while (true)
  {
    WaitForSingleObject (pm->grabmutex, INFINITE);
    c = pm->c;
    ReleaseMutex (pm->grabmutex);

    if (c == 'h')
    {
      sprintf (buffer, "I waited %d cycles before you pressed h!", count);
      excp ("Secondary Thread", buffer);
      WaitForSingleObject (pm->grabmutex, INFINITE);
      pm->c = '0';
      ReleaseMutex (pm->grabmutex);
      count = 0;
    } // if (c == 'h')
    else if (c == 'q')
    {
      break;
    }
    else
    {
      ++count;
    }

    //! Don't slam your processor!  You don't need to poll at full speed.
    Sleep (100);
  } // while (true)

  WaitForSingleObject (pm->grabmutex, INFINITE);
  pm->t2 = false;
  ReleaseMutex (pm->grabmutex);

  return 1;
} // DWORD __stdcall SecondaryDemoThread (LPVOID param)


void excp (char *where, const char *what)
{
  cout << where << " " << what << endl;
}