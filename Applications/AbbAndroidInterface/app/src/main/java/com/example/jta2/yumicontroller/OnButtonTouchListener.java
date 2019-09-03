package com.example.jta2.yumicontroller;

import android.graphics.Color;
import android.graphics.PorterDuff;
import android.graphics.PorterDuffColorFilter;
import android.graphics.drawable.Drawable;
import android.view.MotionEvent;
import android.view.View;

/**
 * Created by jta2 on 9/19/2016.
 */

/**
 * UI interface coloring class
 */
public class OnButtonTouchListener implements View.OnTouchListener {
    @Override
    public boolean onTouch(View v, MotionEvent event){
        Drawable viewDrawable = v.getBackground();
        if (event.getAction() == MotionEvent.ACTION_DOWN){
            viewDrawable.setColorFilter(new PorterDuffColorFilter(ControlActivity.UI_COLOR, PorterDuff.Mode.MULTIPLY));

            if(ControlActivity.isInEditMode == true && v.getId() != R.id.customize_interface_button && v.getId() != R.id.start_button && v.getId() != R.id.about_button  && v.getId() != R.id.network_test_button){
                View.DragShadowBuilder shadowBuilder = new View.DragShadowBuilder(v);
                v.startDrag(null, shadowBuilder, v, 0);
                v.setVisibility(View.INVISIBLE);
            }
        }
        if (event.getAction() == MotionEvent.ACTION_UP){
            viewDrawable.setColorFilter(new PorterDuffColorFilter(Color.WHITE, PorterDuff.Mode.MULTIPLY));
            return false;
        }
        if (event.getAction() != MotionEvent.ACTION_MOVE){

        }
        return false;
    }
}
