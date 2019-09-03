package com.example.jta2.yumicontroller;

import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.view.View.OnClickListener;
import android.content.Intent;

import java.util.ResourceBundle;

/**
 * Activity class for the main menu of the Android interface
 */

public class MainActivity extends AppCompatActivity implements OnClickListener {

    private Button startButton;
    private Button aboutButton;
    private Button networkTestButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        startButton = (Button)findViewById(R.id.start_button);
        startButton.setOnClickListener(this);

        aboutButton = (Button)findViewById(R.id.about_button);
        aboutButton.setOnClickListener(this);

        networkTestButton = (Button)findViewById(R.id.network_test_button);
        networkTestButton.setOnClickListener(this);

        startButton.setOnTouchListener(new OnButtonTouchListener());
        aboutButton.setOnTouchListener(new OnButtonTouchListener());
        networkTestButton.setOnTouchListener(new OnButtonTouchListener());

    }

    //Onclick listeners for main menu
    public void onClick(View v){
        switch(v.getId()){
            //"START" button.
            case R.id.start_button:
                Intent intent = new Intent(this,ControlActivity.class);
                this.startActivity(intent);
                Log.i("Info","Started Control Activity");
                break;

            //"ABOUT" button
            case R.id.about_button:
                AlertDialog.Builder builder = new AlertDialog.Builder(this);
                builder.setTitle("About");
                builder.setMessage("This app was developed by Tyler Arcano.");
                builder.setPositiveButton(
                        "OK",
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                dialog.cancel();
                            }
                        });
                builder.show();
                break;

            //"TEST NETWORK" button
            case R.id.network_test_button:
                Log.i("Test","About to test network connection");
                RobotInterfaceManager manager = new RobotInterfaceManager();
                manager.start();
                break;
        }
    }
}
