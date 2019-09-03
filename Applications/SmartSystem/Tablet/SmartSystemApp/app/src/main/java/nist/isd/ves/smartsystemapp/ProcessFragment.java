package nist.isd.ves.smartsystemapp;

import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Vibrator;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import com.getpebble.android.kit.PebbleKit;
import com.getpebble.android.kit.util.PebbleDictionary;

import org.json.JSONArray;
import org.json.JSONObject;
import org.xml.sax.SAXException;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.Serializable;
import java.net.ConnectException;
import java.net.ServerSocket;
import java.net.Socket;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;

import smartsystem.RobotProcess;
import smartsystem.RobotSAXHandler;

/**
 * Created by ves on 7/16/2015.
 */
public class ProcessFragment extends android.support.v4.app.Fragment implements Serializable{

    //choose user group pebble app
    //private final static UUID PEBBLE_APP_UUID = UUID.fromString("5b5b73be-983f-4f0e-b408-eaa5b66fbcea");

    //custom menu pebble app
    private final static UUID PEBBLE_APP_UUID = UUID.fromString("8c4590ef-504c-496d-8260-441d7b02c68e");

    private PebbleKit.PebbleDataReceiver mReceiver;
    private final int KEY_BUTTON_EVENT = 0;

    // layout variables
    private Button bKinect;
    private TextView tvAlert;
    private Button bRobot;
    private TextView tvTimeElapsed;
    private TextView tvCurrTask;
    private TextView tvNextTask;
    private TextView tvPrevTask;
    private TextView tvSpeedX;
    private TextView tvSpeedY;
    private TextView tvSpeedZ;
    private TextView tvForceX;
    private TextView tvForceY;
    private TextView tvForceZ;
    private TextView tvToolX;
    private TextView tvToolY;
    private TextView tvToolZ;
    private TextView tvTimeCurrent;
    private Button bPebble;
    private Spinner sUser;
    private String[] users;
    private String user;

    static final BlockingQueue queue = new ArrayBlockingQueue(1024);
    private String curr = "curr"; //for animation

    //device variables
    private boolean pebbleConnected = false;
    private boolean robotConnected = false;

    private ArrayList<PrintWriter> pebbleWriters;

    Thread robotListenerThread;
    Thread kinectServerThread;


    // Inflate the fragment layout we defined above for this fragment
    // Set the associated text for the title
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.activity_main, container, false);

        users = getResources().getStringArray(R.array.user_groups);
        user = users[0];

        pebbleWriters = new ArrayList<>();

        tvAlert = (TextView) view.findViewById(R.id.user_alert_box);
        tvTimeElapsed = (TextView) view.findViewById(R.id.tvTimeElapsed);
        tvCurrTask = (TextView) view.findViewById(R.id.tvCurrTask);
        tvNextTask = (TextView) view.findViewById(R.id.tvNextTask);
        tvPrevTask = (TextView) view.findViewById(R.id.tvPrevTask);
        tvToolX = (TextView) view.findViewById(R.id.tvToolX);
        tvToolY = (TextView) view.findViewById(R.id.tvToolY);
        tvToolZ = (TextView) view.findViewById(R.id.tvToolZ);
        tvForceX = (TextView) view.findViewById(R.id.tvForceX);
        tvForceY = (TextView) view.findViewById(R.id.tvForceY);
        tvForceZ = (TextView) view.findViewById(R.id.tvForceZ);
        tvSpeedX = (TextView) view.findViewById(R.id.tvSpeedX);
        tvSpeedY = (TextView) view.findViewById(R.id.tvSpeedY);
        tvSpeedZ = (TextView) view.findViewById(R.id.tvSpeedZ);
        tvTimeCurrent = (TextView) view.findViewById(R.id.tvTimeCurrent);

        bKinect = (Button) view.findViewById(R.id.kinect_button);
        bRobot = (Button) view.findViewById(R.id.robot_button);
        bPebble = (Button) view.findViewById(R.id.pebble_button);

        sUser = (Spinner) view.findViewById(R.id.user_group_spinner);

        /*************** BUTTON LISTENERS **********************/

        bKinect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                kinectServerThread = new kinectServerThread(queue);
                kinectServerThread.start();
            }
        });

        bRobot.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (robotConnected) {
                    robotListenerThread.interrupt();
                    bRobot.setText("UR10: OFFLINE");
                    bRobot.setBackgroundResource(R.drawable.button_border);
                    robotConnected = false;
                } else {
                    robotListenerThread = new robotListenerThread(queue);
                    robotListenerThread.start();
                }
            }
        });


        bPebble.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (pebbleConnected) {
                    bPebble.setText("Pebble: OFFLINE");
                    bPebble.setBackgroundResource(R.drawable.button_border);
                    PebbleKit.closeAppOnPebble(getActivity(), PEBBLE_APP_UUID);
                    pebbleConnected = false;
                } else {
                    if (PebbleKit.isWatchConnected(getActivity())) {
                        pebbleConnected = true;
                        Log.i(getActivity().getLocalClassName(), "Pebble is " + (pebbleConnected ? "connected" : "not connected"));
                        Toast.makeText(getActivity(), "Connected!", Toast.LENGTH_SHORT).show();
                        bPebble.setText("Pebble: ONLINE");
                        bPebble.setBackgroundResource(R.drawable.connected_button);
                        PebbleKit.startAppOnPebble(getActivity(), PEBBLE_APP_UUID);

                        sendPebble("Connection successful", "SmartSystem app is online.");

                    } else {
                        Toast.makeText(getActivity(), "Pebble not detected.", Toast.LENGTH_SHORT).show();
                    }
                }

            }
        });

        sUser.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                user = users[position];
                Log.d("settings", user);
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });

        //initialize receiver for pebble messages
        mReceiver = new PebbleKit.PebbleDataReceiver(PEBBLE_APP_UUID) {
            @Override
            public void receiveData(Context context, int transactionId, PebbleDictionary data) {
                //ACK the message
                PebbleKit.sendAckToPebble(context, transactionId);

                //Check the key exists
                if(data.getUnsignedIntegerAsLong(KEY_BUTTON_EVENT) != null) {
                    int button = data.getUnsignedIntegerAsLong(KEY_BUTTON_EVENT).intValue();

                    //code for custom operator menu
                    if(button==0) {
                        user = users[1];
                        sUser.setSelection(1);
                    }
                    if(button==1) {
                        user = users[0];
                        sUser.setSelection(0);
                    }

                    //code for user group menu
              /*      user = users[button];
                    sUser.setSelection(button);*/
                }
            }

        };

        PebbleKit.registerReceivedDataHandler(getActivity(), mReceiver);

        Thread rp = new receiveOtherPebble();
        rp.start();

        return view;
    }

    //task sequence animation
    public void move(String next, String curr, String prev) {
        Animation nextToCurr = AnimationUtils.loadAnimation(getActivity(), R.anim.next_to_current);
        Animation currToPrev = AnimationUtils.loadAnimation(getActivity(), R.anim.current_to_previous);
        Animation prevToNext = AnimationUtils.loadAnimation(getActivity(), R.anim.prev_to_next);

        tvPrevTask.startAnimation(prevToNext);
        tvCurrTask.startAnimation(currToPrev);
        tvNextTask.startAnimation(nextToCurr);

        tvPrevTask.setBackgroundResource(R.drawable.next_task);
        tvPrevTask.setText(next);

        tvNextTask.setBackgroundResource(R.drawable.current_task);
        tvNextTask.setText(curr);

        tvCurrTask.setBackgroundResource(R.drawable.previous_task);
        tvCurrTask.setText(prev);

    }

    //send notification to pebble when disturbance detected
    public void sendPebble(String title, String body) {
        if (pebbleConnected) {
            final Intent i = new Intent("com.getpebble.action.SEND_NOTIFICATION");

            final Map<String, String> data = new HashMap<String, String>();
            data.put("title", title);
            data.put("body", body);

            final JSONObject jsonData = new JSONObject(data);
            final String notificationData = new JSONArray().put(jsonData).toString();
            i.putExtra("messageType", "PEBBLE_ALERT");
            i.putExtra("sender", "Test");
            i.putExtra("notificationData", notificationData);

            Log.d("device", "Sending to Pebble: " + notificationData);
            getActivity().sendBroadcast(i);
        }
    }

    //generate and send xml message to robot
    public String sendXML(int zone) {
        String message = "<Command ";
        if (user.equals("Visitor")) {
            if (zone != 4)
                message += "State=\"Stopped\" Speed=\"Zero\"";
            else {
                message += "State=\"Normal\" ";
                message += "Speed=\"Full\"";
            }
        } else if (user.equals("Operator")) {
            if (zone == 1) {
                message += "State=\"Pose\" ";
                message += "Speed=\"Full\"";
            } else if (zone == 2) {
                message += "State=\"Normal\" ";
                message += "Speed=\"Quarter\"";
            } else if (zone == 3) {
                message += "State=\"Normal\" ";
                message += "Speed=\"Half\"";
            } else {
                message += "State=\"Normal\" ";
                message += "Speed=\"Full\"";
            }
        } else if (user.equals("Maintenance")){
            if (zone == 1) {
                message += "State=\"Stopped\" ";
                message += "Speed=\"Zero\"";
            } else if (zone == 2) {
                message += "State=\"Normal\" ";
                message += "Speed=\"Quarter\"";
            } else if (zone == 3) {
                message += "State=\"Normal\" ";
                message += "Speed=\"Half\"";
            } else {
                message += "State=\"Normal\" ";
                message += "Speed=\"Full\"";
            }
        } else {
            if (zone == 1) {
                message += "State=\"Stopped\" ";
                message += "Speed=\"Zero\"";
            } else if (zone == 2) {
                message += "State=\"Stopped\" ";
                message += "Speed=\"Zero\"";
            } else if (zone == 3) {
                message += "State=\"Normal\" ";
                message += "Speed=\"Quarter\"";
            } else {
                message += "State=\"Normal\" ";
                message += "Speed=\"Full\"";
            }
        }
        message += "/>";
        return message;
    }

    //parse xml info from robot and display on UI
    private class robotListenerThread extends Thread {

        static final int ROBOT_PORT = 6009;
        static final String ROBOT_IP_ADDRESS = "169.254.152.24";

        protected BlockingQueue queue;

        private String toolX;
        private String toolY;
        private String toolZ;
        private String speedX;
        private String speedY;
        private String speedZ;
        private String forceX;
        private String forceY;
        private String forceZ;
        private String elapsedTime;
        private String timeCurrent;
        private String iteration;
        private String currStep;
        private String prevStep;
        private String nextStep;

        public robotListenerThread(BlockingQueue queue) {
            this.queue = queue;
        }

        @Override
        public void run() {
            boolean scanning = true;  //automatically reconnect to robot
            while(scanning) {
                try {
                    Socket robotClient = new Socket(ROBOT_IP_ADDRESS, ROBOT_PORT);
                    BufferedReader r = new BufferedReader(new InputStreamReader(robotClient.getInputStream()));
                    PrintWriter w = new PrintWriter(robotClient.getOutputStream(), true);
                    robotConnected = true;
                    scanning = false;

                    getActivity().runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Log.d("network", "connected");
                            Toast.makeText(getActivity(), "Connected!", Toast.LENGTH_LONG).show();
                            bRobot.setText("UR10: ONLINE");
                            bRobot.setBackgroundResource(R.drawable.connected_button);
                        }
                    });

                    String filePath = getActivity().getFilesDir().getPath().toString() + "/robotProcess.xml";
                    File f = new File(filePath);

                    FileOutputStream fos = new FileOutputStream(f);

                    while (!Thread.currentThread().isInterrupted()) {

                        RobotProcess rp;
                        boolean instance = true;
                        String line;
                        String end = "</Process>";
                        String begin = "<Process>";

                        //gets depth sensor alert super important
                        if (!queue.isEmpty()) {
                            String s = sendXML((int) queue.take());
                            Log.d("xml", s);
                            w.println(s);
                        }

                        do { //write xml file
                            line = r.readLine();

                            if (line.contains("</Process>")) {
                                fos.write(end.getBytes());
                                instance = false;
                            } else
                                fos.write(line.getBytes());

                        } while (instance);

                        fos.close();

                        // parse xml file
                        SAXParserFactory factory = SAXParserFactory.newInstance();
                        SAXParser saxParser = factory.newSAXParser();
                        RobotSAXHandler handler = new RobotSAXHandler();
                        saxParser.parse(f, handler);

                        rp = handler.getRobotProcess();

                        //update UI
                        if (rp != null) {
                            toolX = rp.getToolX();
                            toolY = rp.getToolY();
                            toolZ = rp.getToolZ();
                            speedX = rp.getSpeedX();
                            speedY = rp.getSpeedY();
                            speedZ = rp.getSpeedZ();
                            forceX = rp.getForceX();
                            forceY = rp.getForceY();
                            forceZ = rp.getForceZ();
                            iteration = rp.getIteration();
                            elapsedTime = rp.getElapsedTime();
                            timeCurrent = rp.getCurrElapsedTime();
                            currStep = rp.getCurrStep();
                            nextStep = rp.getNextStep();
                            prevStep = rp.getPrevStep();

                            getActivity().runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    tvSpeedX.setText(speedX);
                                    tvSpeedY.setText(speedY);
                                    tvSpeedZ.setText(speedZ);

                                    tvForceX.setText(forceX);
                                    tvForceY.setText(forceY);
                                    tvForceZ.setText(forceZ);

                                    tvToolX.setText(toolX);
                                    tvToolY.setText(toolY);
                                    tvToolZ.setText(toolZ);

                                    tvTimeCurrent.setText(timeCurrent);
                                    tvTimeElapsed.setText(elapsedTime);

                                    //run animation on new task
                                    if (!curr.equals(currStep)) {
                                        Log.d("robot", "move: curr " + curr + " currStep " + currStep);
                                        move(nextStep, currStep, prevStep);
                                        curr = currStep;
                                    }
                                }
                            });
                        }

                        fos = getActivity().openFileOutput("robotProcess.xml", Context.MODE_PRIVATE);
                        fos.write(begin.getBytes());

                    } //end reading stream

                    robotClient.close();
                    fos.close();
                    robotConnected = false;
                    Log.d("network", "closed");

                } catch (ConnectException ce) {
                    ce.printStackTrace();
                    Log.d("network", "Connect failed, waiting and trying again");
                    try {
                        Thread.sleep(2000);//2 seconds
                    } catch (InterruptedException ie) {
                        ie.printStackTrace();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (ParserConfigurationException pce) {
                    pce.printStackTrace();
                } catch (SAXException se) {
                    se.printStackTrace();
                } catch (InterruptedException ie) {
                    ie.printStackTrace();
                    Log.d("network", "interrupted");
                }
            }
        }
    }// end robotListenerThread


    //get depth sensor info from server
    private class kinectServerThread extends Thread {

        static final int PORT = 8080;
        static final String IP_ADDRESS = "169.254.152.3";
        protected BlockingQueue queue;

        public kinectServerThread(BlockingQueue queue) {
            this.queue = queue;
        }

        @Override
        public void run() {
            try {
                Socket socket = new Socket(IP_ADDRESS, PORT);

                getActivity().runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Log.d("network", "connected");
                        Toast.makeText(getActivity(), "Connected!", Toast.LENGTH_LONG).show();
                        bKinect.setText("Kinect: ONLINE");
                        bKinect.setBackgroundResource(R.drawable.connected_button);
                        bKinect.setEnabled(false);
                    }
                });
                BufferedReader r = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                String line;
                do {
                    line = r.readLine();
                    if (line != null) {
                        Log.d("network", line);

                        if (line.equals("1")) {
                            getActivity().runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    tvAlert.setText("Danger: User detected in Zone 1");
                                    tvAlert.setBackgroundColor(Color.parseColor("#f01e13"));
                                }
                            });

                            sendPebble("Danger", user + " detected in Zone 1");

                            Log.d("pebble", "size: " + pebbleWriters.size());
                            for (PrintWriter temp : pebbleWriters) {
                                temp.println(user + " detected in Zone 1");
                            }
                            queue.put(1);
                        }

                        if (line.equals("2")) {
                            getActivity().runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    tvAlert.setText("User detected in Zone 2");
                                    tvAlert.setBackgroundColor(Color.parseColor("#fbc138"));
                                }
                            });
                            sendPebble("Disturbance", user + " detected in Zone 2");
                            Log.d("pebble", "size: " + pebbleWriters.size());
                            for (PrintWriter temp : pebbleWriters) {
                                temp.println(user + " detected in Zone 2");
                            }
                            queue.put(2);
                        }

                        if (line.equals("3")) {
                            getActivity().runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    tvAlert.setText("Disturbance, User detected in Zone 3");
                                    tvAlert.setBackgroundColor(Color.parseColor("#80b052"));
                                }
                            });
                            queue.put(3);
                            sendPebble("Disturbance", user + " detected in Zone 3");
                            Log.d("pebble", "size: " + pebbleWriters.size());
                            for (PrintWriter temp : pebbleWriters) {
                                temp.println(user + " detected in Zone 3");
                            }
                        }

                        if (line.equals("4")) {
                            getActivity().runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    tvAlert.setText("No disturbance detected.");
                                    tvAlert.setBackgroundColor(Color.parseColor("#0686d4"));
                                }
                            });
                            queue.put(4);
                        }
                        if(line.equals("stop")) { //if kinect program closes
                            socket.close();
                            IOException e = new IOException();
                            throw e;
                        }
                    }
                }
                while (!Thread.currentThread().isInterrupted());

            } catch (IOException e) {
                getActivity().runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getActivity(), "Disconnected", Toast.LENGTH_LONG).show();
                        bKinect.setEnabled(true);
                        bKinect.setText("Kinect: OFFLINE");
                        bKinect.setBackgroundResource(R.drawable.button_border);
                    }
                });
                e.printStackTrace();
            } catch (InterruptedException ie) {
                ie.printStackTrace();
            }
        }
    }// end kinectServerThread


    //receive connections from other pebbles
    private class receiveOtherPebble extends Thread {

        static final int PEBBLE_PORT = 8383;

        @Override
        public void run() {
            try {
                ServerSocket socket = new ServerSocket(PEBBLE_PORT);

                while(true) {
                    try {
                        Socket client= socket.accept();
                        Log.d("network", "additional pebble connected");
                        PrintWriter w = new PrintWriter(client.getOutputStream(), true);
                        PebbleConnection pc =new PebbleConnection(client, w);
                        pc.start();
                    } catch (IOException ie) {
                        ie.printStackTrace();
                    }

                }

            } catch(IOException ie) {
                ie.printStackTrace();
            }
        }
    }

    //Accepts connections and messages from other pebbles
    private class PebbleConnection extends Thread{

        private Socket socket;
        private int pos;
        private PrintWriter w;

        public PebbleConnection(Socket socket, PrintWriter w) {
            this.socket = socket;
            this.w = w;
            pebbleWriters.add(w);
            Log.d("pebble", "added one");
        }

        public void run(){
            try {
                BufferedReader r = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                String line;
                do {
                    line = r.readLine();
                    if(line!=null) {
                        pos = Integer.parseInt(line);
                        user = users[pos];
                        getActivity().runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                sUser.setSelection(pos);
                            }
                        });
                    }

                } while(!Thread.currentThread().isInterrupted());

            } catch (IOException ie) {
                ie.printStackTrace();
            }
        }

    }


}
