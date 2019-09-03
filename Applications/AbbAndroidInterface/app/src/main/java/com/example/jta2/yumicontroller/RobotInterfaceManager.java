package com.example.jta2.yumicontroller;

import android.util.Log;

import java.io.DataOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.net.Socket;

/**
 * Created by jta2 on 9/12/2016.
 */

/**
 * Class for connecting to and managing data from the robot control server.
 */
public class RobotInterfaceManager extends Thread  {

    private static final int PORT = 30010; //Port for left arm
    private static final String IP_ADDRESS = "129.6.35.26"; //fake emulator localhost ip
    //private static final String IP_ADDRESS = "169.254.152.80"; //actual robot server ip address

    public RobotInterfaceManager(){

    }

    @Override
    public void run(){
        try {
            Socket socket = new Socket(IP_ADDRESS,PORT);
            Log.i("Network","Connected to server.");

            DataOutputStream outputStream = new DataOutputStream(socket.getOutputStream());
            PrintWriter out = new PrintWriter(outputStream);
            Log.i("Print","Tried to print some data");
            out.write("Some char buffer");

            while(!Thread.currentThread().isInterrupted()){

                //output.writeUTF("Test");
            }


        } catch (IOException e){
            Log.e("Error","Could not connect to server.");
            e.printStackTrace();

        }
    }

    //abb_irb_140000_right.xml
   /*
    <ROBOT>
    <TCP_IP Address="169.254.152.80" Port="1026" Client="false"/>
    <ComType Val="TCP_IP"/>
    <Mounting X="0" Y="0" Z="0" XR="0" YR="0" ZR="0"/>
    <ToWorld X="890" Y="890" Z="0" XR="0" YR="0" ZR="0" M00="1" M01="0" M02="0" M03="890" M10="0" M11="1" M12="0" M13="890" M20="-0" M21="0" M22="1" M23="0" M30="0" M31="0" M32="0" M33="1"/>
    <Tool ID="1" Name="Yumi_Parallel" X="0" Y="0" Z="136.0" XR="0" YR="0" ZR="0" Mass="0.0.262" MX="7.8" MY="11.9" MZ="50.7"/>
    <Tool ID="2" Name="Yumi_Vacuum" X="63.5" Y="18.5" Z="37.5" XR="0" YR="90" ZR="0" Mass="0.0.262" MX="7.8" MY="11.9" MZ="50.7"/>
    </ROBOT>
    */

    //abb_irb_140000_left.xml
    /*
    <ROBOT>
    <TCP_IP Address="169.254.152.80" Port="1025" Client="false"/>
    <ComType Val="TCP_IP"/>
    <Mounting X="0" Y="0" Z="0" XR="0" YR="0" ZR="0"/>
    <ToWorld X="890" Y="890" Z="0" XR="0" YR="0" ZR="0" M00="1" M01="0" M02="0" M03="890" M10="0" M11="1" M12="0" M13="890" M20="-0" M21="0" M22="1" M23="0" M30="0" M31="0" M32="0" M33="1"/>
    <Tool ID="1" Name="Yumi_Parallel" X="0" Y="0" Z="136.0" XR="0" YR="0" ZR="0" Mass="0.0.262" MX="7.8" MY="11.9" MZ="50.7"/>
    <Tool ID="2" Name="Yumi_Vacuum" X="63.5" Y="18.5" Z="37.5" XR="0" YR="90" ZR="0" Mass="0.0.262" MX="7.8" MY="11.9" MZ="50.7"/>
    </ROBOT>
    */

    //Example command for robot motion (does not work apparently):
  /*//You're sending the XML data to a computer which then relays those commands to the robot.
    Example command for robot motion:

    <?xml version="1.0" encoding="UTF-8"?>
    <CRCLCommandInstance
     xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
     xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
      <CRCLCommand xsi:type="MoveToType">
        <CommandID>2</CommandID>
        <MoveStraight>false</MoveStraight>
        <Pose>
          <Point>
            <X>2.5</X> <Y>1</Y> <Z>1</Z>
          </Point>
          <XAxis>
            <I>1</I> <J>0</J> <K>0</K>
          </XAxis>
          <ZAxis>
            <I>0</I> <J>0</J> <K>-1</K>
          </ZAxis>
        </Pose>
        <NumPositions>1</NumPositions>
      </CRCLCommand>
    </CRCLCommandInstance>

    Example command for gripper:
    <?xml version="1.0" encoding="UTF-8"?>
    <CRCLCommandInstance xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
      <CRCLCommand xsi:type="SetEndEffectorType">
        <CommandID>123</CommandID>
        <NumPositions>0.5</NumPositions>
      </CRCLCommand>
    </CRCLCommandInstance>
  */
}
