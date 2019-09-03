# Collaborative Robotics Programming Interface TCP client middle-ware framework for MACI integration
This is the man-in-the-middle framework that is utilized for integrating the incoming data from the Vicon Android interpreter and the Android Unity App. This framework takes in commands sent thorough the app and translates them into CRPI-compatible commands. This is dependent on the type of command-joint values that are being sent from a user to the target robot. No two joint-value lists can be the same for different robots as their structure parameters are different and CRPI may not support that type of robot as well. 

![alt-text](https://github.com/OvercodedStack/CRPI-UI-DOCUMENTATION-Summer-of-2019/blob/master/Images/CRPI%20Preview.PNG?raw=true)

### Installation 
Download the repository and drag and drop into a current installation of [CRPI](https://github.com/usnistgov/CRPI) in a folder called C:/CRPI/Applications/ (Differs from where you may have installed it.) Upon opening the visual studio solution, press “Start Without Debugging”.

### Useage 
Upon initialization, the program will attempt to connect to the TCP socket for the Vicom and the Android Apps, once connection is achieved, the script will continue running until one of the servers closes, upon which the client may close at the same time. If desired, the application's settings can be tweaked to debug or change some functionality.

### Parameters

- **SHUTOFF_CRPI**: This disables the CRPI translation functionality and only enables the interpretation of recieved messages from the Android app.

- **DISABLE_VICOM**: This disables the functionality of using the secondary system, [the motion capture system add-on](https://github.com/OvercodedStack/MOTION_CAPTURE_UNITY-Summer-of-2019-NIST/tree/master), with this system. 

1 = Enables the flag

0 = Disables the flag 

### Bugs
There are two bugs related with this script at this time: 
-	Servers that connect to the clients may stay stuck and require restart.
-	CRPI may fail to latch to the robot TCP client and may eternally fail to latch, requiring restart as well. Happens more frequently when trying to switch between robots using the Vicon. 

### Conclusion

Special thanks to the following people during SURF 2019:

- Shelly Bagchi
- Dr. Jeremy Marvel
- Megan Zimmerman
- Holiday Inn SURF Fellows

Thank you all for being the great people you guys are!
