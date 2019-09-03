///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Demo:  PingPong
//  Workfile:        PingPong.cpp
//  Revision:        1.0 - 26 January, 2010
//                   2.0 - 19 August, 2014 Updated to use ULAPI interface
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Socket communications test.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ulapi.h"
using namespace std;


#define REQUEST_MSG_SIZE 8192
#define DUMBCLIENT



void main ()
{
  ulapi_integer server, client;

  int option, get, sent, x;
  char inbuffer[REQUEST_MSG_SIZE], outbuffer[REQUEST_MSG_SIZE];

  option = 0;
  while (option != 1 && option != 2)
  {
    cout << "Running as server (1) or client (2)? ";
    cin >> option;
  }

  if (option == 1)
  {
    //! Run as server
    cout << "Waiting for connection on port 6008..." << endl;
    server = ulapi_socket_get_server_id(6008);
    client = ulapi_socket_get_connection_id(server);
    ulapi_socket_set_blocking(client);

    cout << "Client connected!" << endl;
    while (true)
    {
      for (x = 0; x < REQUEST_MSG_SIZE; ++x)
      {
        inbuffer[x] = '\0';
      }

      get = ulapi_socket_read(client, inbuffer, REQUEST_MSG_SIZE);
      //! Read data from client
      if (get > 0)
      {
        cout << "PING! " << inbuffer << endl;
        sprintf (outbuffer, "Pong: %s\0", inbuffer);

        sent = ulapi_socket_write(client, outbuffer, strlen(outbuffer));
      }
    } // while (true)

  } // if (option == 1)
  else
  {
    //! Run as client
    cout << "Connect to which address (xxx.xxx.xxx.xxx)? : ";
    cin >> inbuffer;

    server = ulapi_socket_get_client_id (1025, inbuffer);
    if (server < 0)
    {
      cout << "could not connect" << endl;
      return;
    }
    else
    {
      cout << "connected..." << endl;
      cout << ulapi_socket_set_blocking(server) << endl;
      cout << server << endl;
    }

    int counter = 0;

#ifndef DUMBCLIENT
    while (true)
    {
      cout << "text to send: ";
      cin >> outbuffer;
      cout << "Sending \"" << outbuffer << "\"" << endl;
      sent = ulapi_socket_write(server, outbuffer, strlen(outbuffer));

      get = ulapi_socket_read(server, inbuffer, REQUEST_MSG_SIZE);
      for (x = 0; x < get; ++x)
      {
        cout << inbuffer[x];
      }
      cout << endl;
    } // while (true)
#else
    counter = 0;
    while (true)
    {
      get = ulapi_socket_read(server, inbuffer, 64);
      if (get > 0)
      {
        cout << counter++ << " ";
        for (x = 0; x < get; ++x)
        {
          cout << inbuffer[x];
        }
        cout << endl;
      }
    }
#endif
  } // if (option == 1) ... else
  
}

