package com.example.jta2.yumicontroller;

import android.graphics.Color;
import android.graphics.PorterDuff;
import android.graphics.PorterDuffColorFilter;
import android.graphics.drawable.Drawable;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.DragEvent;
import android.view.GestureDetector;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.ToggleButton;

/**
 * Activity class for the controller interface
 */
public class ControlActivity extends AppCompatActivity implements View.OnClickListener{

    private ImageView liveVideoFeed;
    private RelativeLayout controllerActivityLayout;
    private JoystickView leftStick;
    private JoystickView rightStick;

    //Control Buttons
    private Button leftGrabberOn;
    private Button leftGrabberOff;
    private Button rightGrabberOn;
    private Button rightGrabberOff;

    //"Edit" button for customizable interface

    private ToggleButton customizeInterfaceButton;

    //Vertical seek bars to control Z axis.
    VerticalSeekBar leftVerticalControl;
    VerticalSeekBar rightVerticalControl;
    private GestureDetector gestureDetector;

    //Global UI color/theme. Can be changed here.
    public static final int UI_COLOR = Color.parseColor("#0099cc");

    public static boolean isInEditMode = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control);
        //Hide the navigation bar when in the interface
        View decorView = getWindow().getDecorView();

        //UI Option flags
        int uiOptions = View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_IMMERSIVE;
        decorView.setSystemUiVisibility(uiOptions);
        //Create object for video feed
        liveVideoFeed = (ImageView)findViewById(R.id.main_video_feed);
        Drawable liveVideoFeedInitialState = liveVideoFeed.getDrawable();
        controllerActivityLayout = (RelativeLayout)findViewById(R.id.controller_activity_layout);

        //Objects for vertical controls
        leftVerticalControl = (VerticalSeekBar)findViewById(R.id.vertical_left);
        rightVerticalControl = (VerticalSeekBar)findViewById(R.id.vertical_right);

        //Set the seek bar color.
        leftVerticalControl.getProgressDrawable().setColorFilter(UI_COLOR, PorterDuff.Mode.MULTIPLY);
        rightVerticalControl.getProgressDrawable().setColorFilter(UI_COLOR, PorterDuff.Mode.MULTIPLY);

        //Identify gripper buttons
        leftGrabberOff = (Button)findViewById(R.id.l_grabber_off);
        leftGrabberOn = (Button)findViewById(R.id.l_grabber_on);
        rightGrabberOff = (Button)findViewById(R.id.r_grabber_off);
        rightGrabberOn = (Button)findViewById(R.id.r_grabber_on);

        leftStick = (JoystickView)findViewById(R.id.left_stick);
        rightStick = (JoystickView)findViewById(R.id.right_stick);

        leftGrabberOff.setOnTouchListener(new OnButtonTouchListener());
        leftGrabberOn.setOnTouchListener(new OnButtonTouchListener());
        rightGrabberOn.setOnTouchListener(new OnButtonTouchListener());
        rightGrabberOff.setOnTouchListener(new OnButtonTouchListener());

        leftGrabberOff.setOnClickListener(this);
        leftGrabberOn.setOnClickListener(this);
        rightGrabberOn.setOnClickListener(this);
        rightGrabberOff.setOnClickListener(this);

        leftStick.setOnTouchListener(new OnButtonTouchListener());
        rightStick.setOnTouchListener(new OnButtonTouchListener());
        leftVerticalControl.setOnTouchListener(new OnButtonTouchListener());
        rightVerticalControl.setOnTouchListener(new OnButtonTouchListener());

        customizeInterfaceButton = (ToggleButton)findViewById(R.id.customize_interface_button);
        customizeInterfaceButton.setOnTouchListener(new OnButtonTouchListener());

        leftVerticalControl.setProgress(50);
        rightVerticalControl.setProgress(50);

        liveVideoFeed.setOnDragListener(new View.OnDragListener() {
            @Override
            public boolean onDrag(View v, DragEvent event) {
                int action = event.getAction();
                View view = (View) event.getLocalState();

                switch (action) {

                    case DragEvent.ACTION_DRAG_STARTED:

                        System.out.println("Drag started");
                        break;

                    case DragEvent.ACTION_DRAG_ENTERED:

                        break;

                    case DragEvent.ACTION_DRAG_EXITED:

                        break;
                    case DragEvent.ACTION_DRAG_ENDED:
                        Drawable viewDrawable = view.getBackground();

                        viewDrawable.setColorFilter(new PorterDuffColorFilter(Color.WHITE, PorterDuff.Mode.MULTIPLY));

                        break;
                    case DragEvent.ACTION_DROP:
                        //Get the location where the user dropped the button

                        float dropPointX = event.getX();
                        float dropPointY = event.getY();

                        view.setX(dropPointX - view.getWidth()/2); //Calculate and set the exact X and Y position of the dragged button.
                        view.setY(dropPointY - view.getHeight()/2);

                        ViewGroup owner = (ViewGroup)view.getParent();
                        owner.removeView(view);
                        RelativeLayout container = (RelativeLayout) findViewById(R.id.controller_activity_layout);
                        container.addView(view);
                        view.setVisibility(View.VISIBLE);

                    default:
                        break;
                }

                return true;
            }
        });

        customizeInterfaceButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                Drawable defaultDrawable = buttonView.getBackground();
                if (isChecked) {
                    // The toggle is enabled
                    defaultDrawable.setColorFilter(new PorterDuffColorFilter(ControlActivity.UI_COLOR, PorterDuff.Mode.MULTIPLY));
                    liveVideoFeed.setBackgroundColor(Color.BLACK);
                    isInEditMode = true;
                } else {
                    // The toggle is disabled
                    Drawable videoFeedInitialState = liveVideoFeed.getDrawable();
                    liveVideoFeed.setBackground(videoFeedInitialState);
                    isInEditMode = false;

                }
            }
        });

        //Create left joystick object and set move listeners on it.

        leftStick.setOnJoystickMoveListener(new JoystickView.OnJoystickMoveListener() {

            @Override
            public void onValueChanged(int angle, int power, int direction) {
                leftStick.reset();
                switch (direction) {
                    case JoystickView.FRONT:

                        Log.i("Tag","Moved front??");

                        break;
                    case JoystickView.FRONT_RIGHT:

                        break;
                    case JoystickView.RIGHT:

                        break;
                    case JoystickView.RIGHT_BOTTOM:

                        break;
                    case JoystickView.BOTTOM:

                        break;
                    case JoystickView.BOTTOM_LEFT:

                        break;
                    case JoystickView.LEFT:

                        break;
                    case JoystickView.LEFT_FRONT:

                        break;
                    default:
                }
            }
        }, JoystickView.DEFAULT_LOOP_INTERVAL);

        //Create right joystick object and set move listeners on it.

        rightStick.setOnJoystickMoveListener(new JoystickView.OnJoystickMoveListener() {

            @Override
            public void onValueChanged(int angle, int power, int direction) {
                rightStick.reset(); //Fixes joystick wheel alignment bug
                switch (direction) {
                    case JoystickView.FRONT:

                        break;
                    case JoystickView.FRONT_RIGHT:

                        break;
                    case JoystickView.RIGHT:

                        break;
                    case JoystickView.RIGHT_BOTTOM:

                        break;
                    case JoystickView.BOTTOM:

                        break;
                    case JoystickView.BOTTOM_LEFT:

                        break;
                    case JoystickView.LEFT:

                        break;
                    case JoystickView.LEFT_FRONT:

                        break;
                    default:
                }
            }
        }, JoystickView.DEFAULT_LOOP_INTERVAL);

    }

    @Override
    public void onClick(View v){

        switch (v.getId()){

            case R.id.l_grabber_off:
                Log.i("Click","Did you click L GRABBER OFF?");
                break;
            case R.id.l_grabber_on:
                Log.i("Click","Did you click L GRABBER ON?");
                break;
            case R.id.r_grabber_off:
                Log.i("Click","Did you click R GRABBER OFF?");
                break;
            case R.id.r_grabber_on:
                Log.i("Click","Did you click R GRABBER ON?");
                break;
        }
    }

}
