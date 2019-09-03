package nist.isd.ves.smartsystemapp;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Parcel;
import android.os.Parcelable;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.Toast;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ConnectException;
import java.net.Socket;

/**
 * Created by ves on 7/16/2015.
 */
public class VideoFragment extends Fragment {

    private ImageView ivVideo;
    private Button bVideo;
    private boolean streamConnected = false;

    // Inflate the fragment layout we defined above for this fragment
    // Set the associated text for the title
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.video_fragment, container, false);

        ivVideo = (ImageView) view.findViewById(R.id.iv_video);
        bVideo = (Button) view.findViewById(R.id.video_button);

        bVideo.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                Thread kinectStream = new kinectStreamThread();
                kinectStream.start();
                bVideo.setEnabled(false);
            }
        });

        return view;
    }

    private class kinectStreamThread extends Thread {
        private static final int PORT = 8181;
        private static final String IP_ADDRESS = "169.254.152.3";

        private Bitmap bitmap;

        @Override
        public void run() {
            try {
                Socket socket = new Socket(IP_ADDRESS, PORT);
                Log.d("network", "stream connected");



                DataInputStream dIn = new DataInputStream(socket.getInputStream());
                int length = dIn.readInt();                    // read length of incoming message


                while(!Thread.currentThread().isInterrupted() && length!=0) {
                    byte[] message = new byte[length];
                    dIn.readFully(message, 0, message.length); // read the message

                    BitmapFactory.Options options = new BitmapFactory.Options();
                    bitmap = BitmapFactory.decodeByteArray(message, 0, message.length, options);

                    getActivity().runOnUiThread(new Runnable() {
                        @Override
                        public void run() {

                            if (bitmap != null) {
                                ivVideo.setImageBitmap(bitmap);
                            } else
                                Log.d("bitmap", "bitmap null");

                        }
                    });
                    length = dIn.readInt();
                }
            } catch (IOException ie){
                ie.printStackTrace();
                getActivity().runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        bVideo.setEnabled(true);
                    }
                });

            }
        }

    }
}


