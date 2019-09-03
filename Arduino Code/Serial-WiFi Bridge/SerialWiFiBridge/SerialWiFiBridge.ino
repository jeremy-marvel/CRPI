#include <SPI.h>
#include <WiFi101.h>


WiFiServer server(1025);
int status = WL_IDLE_STATUS;   //!  WiFi radio's status

//#define VERBOSE

char ssid[] = "CollabLab_G";   //!  Network SSID 
char pass[] = "nrh3nknkv3w6v"; //!  Network password
char stng[] = "P,-000.000,-000.000,-000.000,-000.000,-000.000,-000.000,-000.000";

bool ready = false;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(19200, SERIAL_8E1);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#ifdef VERBOSE
  Serial.println("Serial up and running");  
#endif

  if (WiFi.status() == WL_NO_SHIELD)
  {
#ifdef VERBOSE
    Serial.println("WiFi shield not present.");
#endif
    while (true);
  }

#ifdef VERBOSE
  //! Scan for existing networks:
  Serial.println("Scanning available networks...");
  listNetworks();
#endif

  while (status != WL_CONNECTED)
  {
#ifdef VERBOSE
    Serial.print("Attempting to connect to ");
    Serial.println(ssid);
#endif    
    status = WiFi.begin(ssid, pass);

    if (status != WL_CONNECTED)
    {
#ifdef VERBOSE
      Serial.println("Can't connect.");
#endif
      delay(2000);
    }
  }
  server.begin();

#ifdef VERBOSE
  Serial.println("Connected.");
  printWiFiStatus();
#endif
}


void loop()
{
  WiFiClient client = server.available();
  byte bte;
  char car;

  {
    int test = 2;

    if (test == 0)
    {
      //! Basic streaming test:  get a character write a character
      Serial.write(bte);
      if (Serial.available() > 1)
      {
        stng[0] = (char)Serial.read();
        client.write(stng[0]);
      }
    } // if test == 0
    else if (test == 1)
    {
      //! Line stream test:  grab 62 characters, and write it out to wifi
      Serial.write(bte);
      while (Serial.available() < 62)
      {
        ;
      }
      for (int i = 0; i < 62; ++i)
      {
        stng[i] = (char)Serial.read();
        if ((i%5) == 0)
        {
          delay(1);
        }
      }
      client.println(stng);
    } // if test == 1
    else if (test == 2)
    {
      //! Line stream test:  force a line to have an indicator marker be the first character
      Serial.write(bte);
      while (Serial.available() < 50)
      {
        ;
      }
      
      stng[0] = (char)Serial.read();
      while (stng[0] != 'A' && stng[0] != 'P')
      {
        stng[0] = (char)Serial.read();
      }
      
      for (int i = 1; i < 64; ++i)
      {
        delay(1);
        stng[i] = (char)Serial.read();
      }
      client.println(stng);      
    } // if test == 2
  } // if (client)
}




void listNetworks() 
{
  //! Scan for nearby networks:
  Serial.println("** Scan Networks **");
  byte numSsid = WiFi.scanNetworks();

  //! Print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  //! Print the network number and name for each network found:
  for (int thisNet = 0; thisNet<numSsid; thisNet++)
  {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    Serial.println(WiFi.encryptionType(thisNet));
  }
}

void printWiFiStatus()
{
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("Signal Strength: ");
  Serial.print(rssi);
  Serial.println(" dBm");
}
