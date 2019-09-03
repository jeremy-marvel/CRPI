package smartsystem;

import processing.core.*;
import SimpleOpenNI.*;

import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import javax.imageio.ImageIO;

public class DepthSensor extends PApplet {

	SimpleOpenNI context;
	PrintWriter output = createWriter("depthdata.csv");

	//get initial row depth information to compare new depths to 
	private int[] baseRow = new int[640];
	private int bottomRowStart = 640*300; //change row if desired 
	private int bottomRowEnd = 640*301; 

	private int minDepth; 
	private int minIndex; 

	private int prevZone = 0; 

	//NETWORKING 
	private Socket tabletClient;
	private PrintWriter tabletWriter;
	private ServerSocket serverSocket; 
	private static final int SERVER_PORT = 8080;
	private boolean tabletConnected= false; 

	private static final int STREAM_PORT = 8181; 
	private Socket streamClient; 
	private ServerSocket streamSocket; 
	private boolean streamConnected = false; 

	private boolean socketsClosed = false; 


	//********* PROCESSING METHODS (setup/draw) *****************************//

	public void setup() {
		context = new SimpleOpenNI(this);  

		frameRate(10);              // Change the framerate, has no effect on the program

		context.enableDepth();      // Turns on the grayscale depth camera
		context.enableRGB();
		size(context.depthWidth(), context.depthHeight());    // Configures the size of the output screen to match the width and height resolution of the camera (640x480)

		//to get first frame to set initial depth
		baseRow[0] = -5; 

		try {
			serverSocket = new ServerSocket(SERVER_PORT);
			streamSocket = new ServerSocket(STREAM_PORT);
		} catch(IOException ie) {
			ie.printStackTrace();
		}

		startNetworking(); 
	}


	public void draw() {
		background(0);              // Sets the basic screen color to black
		context.update();           // Receives updated values from the camera

		image(context.depthImage(), 0, 0);  // Displays the camera vision on the output screen

		int[] depthMap = context.depthMap();  

		//Get initial depth to compare later for changes. 
		if(baseRow[0] < 0) {
			getBaseRow(depthMap);
		}

		//to compare min depth of subsequent frames to minDepth
		int frameMinDepth = minDepth; 
		int frameMinIndex = minIndex; 

		//find biggest depth change and mark disturbance
		for(int i = bottomRowStart; i < bottomRowEnd; i++) {

			if(baseRow[i-bottomRowStart] != depthMap[i] && depthMap[i] < minDepth && depthMap[i]!=0) {
				frameMinDepth = depthMap[i];
				frameMinIndex = i; 
			}
		}

		//check zone disturbance and send zone info if tablet is connected
		if(tabletConnected) {
			findZone(frameMinDepth, frameMinIndex);
		} 


		//send image over stream, or try to reconnect if not connected 
		if(streamConnected) {
			BufferedImage b = (BufferedImage) (context.rgbImage()).getNative();
			sendStreamImage(b);
		} 

		if(socketsClosed) { //start accepting connections again if app disconnects
			startNetworking(); 
		}
	}


	//************ HELPER METHODS ***************************//



	//get background image info
	public void getBaseRow(int[] depthMap) {
		boolean foundMinDepth = false; 

		output.println("BASE ROW: ");
		//Record base row and find minimum depth
		for(int i = bottomRowStart; i < bottomRowEnd; i++) {

			int index = i-bottomRowStart; 

			baseRow[index] = depthMap[i];

			//get initial value that is not zero (because sometimes that happens) 
			if(baseRow[index] > 0 && !foundMinDepth) {
				minDepth = baseRow[index];
				foundMinDepth = true; 
				minIndex = index; 
			}

			if(foundMinDepth) {
				if(baseRow[index] < minDepth && baseRow[index] != 0) {
					minDepth = baseRow[index];
					minIndex = index; 
				}
			}
			output.println(depthMap[i]);
		}
		output.println("END BASE ROW");
		output.println("minDepth: " + minDepth + "minIndex: " + minIndex);
		System.out.println("minDepth: " + minDepth);
	}


	//get zone from min depth information and send zone to tablet 
	public void findZone(int frameMinDepth, int frameMinIndex) {
		try {
			boolean significantChange = (minDepth-frameMinDepth) > 130; //accounting for noise, 5 inch min change

			if(frameMinDepth < 1200 && prevZone != 1 && significantChange) { //4ft
				output.println("USER ENTERED ZONE 1");
				System.out.println("USER ENTERED ZONE 1");
				prevZone = 1; 
				updateZone(1);
			}

			if(frameMinDepth < 2100 && frameMinDepth >= 1200 && prevZone != 2 && significantChange) { //~7 ft
				output.println("USER ENTERED ZONE 2");
				System.out.println("USER ENTERED ZONE 2");
				prevZone = 2; 
				updateZone(2); 
			}

			if(frameMinDepth >= 2100 && frameMinDepth < 3000 && prevZone != 3 && significantChange) { //~10 ft
				output.println("USER ENTERED ZONE 3 - SAFE.");
				System.out.println("USER ENTERED ZONE 3, SAFE.");
				prevZone = 3; 
				updateZone(3);
			}

			if((frameMinDepth >=3000 && prevZone!=4 && prevZone!=0)) { //user clears zones
				System.out.println("Clear");
				output.println("clear");
				prevZone = 4; 
				updateZone(4);
			}

		} catch(SocketException se) { //in case app shuts down while kinect is running
			closeConnections(); 
			socketsClosed = true; 
		}
		
		output.println(frameMinDepth);
		//System.out.println("minDepth: " + minDepth + "frameMinDepth: " + frameMinDepth + "franeMinIndex: " + minIndex);

	}

	//start new threads to accept connections 
	public void startNetworking() {
		Thread server = new Thread(new setupServer());
		server.start(); 

		Thread stream = new Thread(new setupStreamServer());
		stream.start(); 
	}

	//send stream image
	public void sendStreamImage(BufferedImage b) {
		try{
			ByteArrayOutputStream baos = new ByteArrayOutputStream();
			ImageIO.write(b, "jpg", baos );
			baos.flush();
			byte[] imageInByte = baos.toByteArray();
			baos.close();

			DataOutputStream dOut = new DataOutputStream(streamClient.getOutputStream());

			dOut.writeInt(imageInByte.length); // write length of the message
			dOut.write(imageInByte); 

		} catch (SocketException se) {
			closeConnections(); 
			socketsClosed = true; 

		}catch (IOException e) {
			e.printStackTrace();
		}
	}

	//close all client connections
	public void closeConnections() {
		try {
			if(tabletConnected) {
				tabletWriter.println("stop");
				tabletClient.close(); 
				tabletConnected = false; 
			}
			if(streamConnected) {
				System.out.println("in streamConnected");
				streamClient.close();
				streamConnected = false; 
			}
		} catch(IOException ie) {
			ie.printStackTrace();
		}
	}

	//This section finishes writing to the file 
	//and closes the program upon any key press.
	public void keyPressed() {
		try {
			closeConnections(); 
			output.flush();
			output.close(); 
			exit();
		} catch(Exception e) {
			e.printStackTrace();
		}
	}

	//notifies tablet of zone intrusion
	public void updateZone(int zone) throws SocketException {
		DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
		Calendar cal = Calendar.getInstance();
		System.out.println(dateFormat.format(cal.getTime())); //2014/08/06 16:00:22
		tabletWriter.println(zone);
	}


	//****************** NETWORKING THREADS *************************************/

	//tablet network thread
	private class setupServer extends Thread {

		public void run() {
			try {
				tabletClient = serverSocket.accept();
				tabletWriter = new PrintWriter(tabletClient.getOutputStream(), true);
				tabletConnected = true; 
				System.out.println("tablet connected");
			}
			catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	//tablet live stream thread 
	private class setupStreamServer extends Thread {

		public void run() {
			try { 
				streamClient = streamSocket.accept();
				streamConnected = true; 
				System.out.println("stream connected");
			}
			catch (IOException e) {
				e.printStackTrace();
			}
		}
	}


}








