package nist.isd.ves.pebbledummy;

import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.getpebble.android.kit.PebbleKit;
import com.getpebble.android.kit.util.PebbleDictionary;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ConnectException;
import java.net.Socket;
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;


public class MainActivity extends ActionBarActivity {

    // all user groups pebble app
   // private final static UUID PEBBLE_APP_UUID = UUID.fromString("5b5b73be-983f-4f0e-b408-eaa5b66fbcea");

    //maintanence pebble app
    private final static UUID PEBBLE_APP_UUID = UUID.fromString("8c4590ef-504c-496d-8260-441d7b02c68e");

    private String[] users;
    private String user;
    private PebbleKit.PebbleDataReceiver mReceiver;
    private final int KEY_BUTTON_EVENT = 0;

    private TextView tvConnected;
    private PrintWriter w;
    private BufferedReader r;
    boolean connected = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        users = getResources().getStringArray(R.array.user_groups);
        user = users[0];

        tvConnected = (TextView) findViewById(R.id.tv_connection);

        if (PebbleKit.isWatchConnected(getApplicationContext())) {
            PebbleKit.startAppOnPebble(getApplicationContext(), PEBBLE_APP_UUID);
        }


        mReceiver = new PebbleKit.PebbleDataReceiver(PEBBLE_APP_UUID) {
            @Override
            public void receiveData(Context context, int transactionId, PebbleDictionary data) {
                //ACK the message
                PebbleKit.sendAckToPebble(context, transactionId);

                //Check the key exists
                if(data.getUnsignedIntegerAsLong(KEY_BUTTON_EVENT) != null) {
                    int button = data.getUnsignedIntegerAsLong(KEY_BUTTON_EVENT).intValue();

                    //maintanence pebble app code
                    if(button==0) {
                        button = 3;
                    }
                    if(button==1) {
                        button = 0;
                    }

                    user = users[button];

                    if(connected) {
                        w.println(button);
                    }
                }
            }

        };

        PebbleKit.registerReceivedDataHandler(getApplicationContext(), mReceiver);

        Thread sendMessage = new sendPebbleMessage();
        sendMessage.start();

    }

    //send notification to pebble when disturbance detected
    public void sendPebble(String title, String body) {
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
            getApplicationContext().sendBroadcast(i);

    }


    private class sendPebbleMessage extends Thread {

        static final int TABLET_PORT = 8383;
        static final String TABLET_IP_ADDRESS = "169.254.152.62";

        @Override
        public void run() {
            boolean scanning = true;
            while(scanning) {
                try {
                    Socket robotClient = new Socket(TABLET_IP_ADDRESS, TABLET_PORT);

                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            tvConnected.setText("Connected");
                            tvConnected.setTextColor(Color.parseColor("#95c568"));
                        }
                    });

                    connected = true;
                    w = new PrintWriter(robotClient.getOutputStream(), true);
                    r = new BufferedReader(new InputStreamReader(robotClient.getInputStream()));

                    String line;
                    do{
                        line = r.readLine();
                        Log.d("network", line);
                        if(line!=null) {
                            Log.d("network", line);
                            sendPebble("Disturbance", line);
                        }
                    }
                    while (!Thread.currentThread().isInterrupted());

                } catch (ConnectException ce) {
                    ce.printStackTrace();
                    Log.d("network", "Connect failed, waiting and trying again");
                    try {
                        Thread.sleep(2000);//2 seconds
                    } catch (InterruptedException ie) {
                        ie.printStackTrace();
                    }
                } catch (IOException ie) {
                    ie.printStackTrace();
                }
            }
        }
    }



    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
