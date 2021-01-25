package org.firstinspires.ftc.teamcode.ringdetect;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.teamcode.util.CaptureCamera;
import org.firstinspires.ftc.teamcode.util.ICaptureCameraListener;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

@TeleOp(name="ImageCaptureOpMode", group="Robot")
@Disabled
public class ImageCaptureOpMode extends OpMode implements ICaptureCameraListener {

    private File mCaptureDirectory = null;

    private CaptureCamera mCaptureCamera = null;

    private boolean mCaptureImage = false;

    private boolean mLastCaptureButtonState = false;
    private boolean mLastTagButtonState = false;
    private String mCurrentTag = RingDetectorNeuralNetwork.LABEL_STRING_NO_RING;
    private int mRecordNumber = 0;

    @Override
    public void init() {
        try{
            mCaptureCamera = new CaptureCamera();
            mCaptureCamera.init(this,this);
        }
        catch (Exception e){
            telemetry.addLine("Error initializing camera:"+e.getMessage());
        }
        File root = Environment.getRootDirectory();
        String path = root.getPath()+"/sdcard/capture";
        mCaptureDirectory = new File(path);
        if (!mCaptureDirectory.exists()){
            mCaptureDirectory.mkdir();
        }
        else{
            // Directory exists. delete all the existing files
            File files[] = mCaptureDirectory.listFiles();
            for(int i=0;i < files.length;i++){
                files[i].delete();
            }
        }
    }

    @Override
    public void loop() {
        // detect the tag button edge and change tag state if rising edge
        boolean edgeDetect = false;
        boolean buttonState = gamepad1.y;
        if (buttonState) {
            // button pressed. check if last false
            if (mLastTagButtonState == false) {
                edgeDetect = true;
            }
        }
        // if we had a rising edge, cycle to the next tag
        if (edgeDetect == true) {
            if (mCurrentTag == RingDetectorNeuralNetwork.LABEL_STRING_NO_RING) {
                mCurrentTag = RingDetectorNeuralNetwork.LABEL_STRING_ONE_RING;
            } else if (mCurrentTag == RingDetectorNeuralNetwork.LABEL_STRING_ONE_RING) {
                mCurrentTag = RingDetectorNeuralNetwork.LABEL_STRING_FOUR_RINGS;
            } else {
                mCurrentTag = RingDetectorNeuralNetwork.LABEL_STRING_NO_RING;
            }
        }
         // Save tag button for next time
        mLastTagButtonState = buttonState;

        // check for capture button edge
        edgeDetect = false;
        buttonState = gamepad1.a;
        if (buttonState) {
            // button pressed. check if last false
            if (mLastCaptureButtonState == false) {
                edgeDetect = true;
            }
        }

        if (edgeDetect){
            // Capture the next image
            mCaptureImage = true;
        }
        // service the camera
        mCaptureCamera.serviceCaptureCamera();

//        telemetry.addLine("Current Label="+mCurrentTag);
//        telemetry.addLine("Capture image="+mCaptureImage);
//        telemetry.update();
     }

    @Override
    public void onNewFrame(Bitmap bitmap) {
        if (mCaptureImage){
            telemetry.addLine("Got to onNewFrame");
            telemetry.update();
            Bitmap rgbBitmap = bitmap.copy(Bitmap.Config.ARGB_8888,true);
            saveBitmap(rgbBitmap);
            mCaptureImage = false;
        }
    }

    private void saveBitmap(Bitmap bitmap) {
        if (mCaptureDirectory == null)
            return;
        String filename = mCurrentTag+"_"+mRecordNumber+".png";
        File file = new File(mCaptureDirectory, filename);
        mRecordNumber++;  // for next time
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.PNG,100, outputStream);
                telemetry.addLine("captured: "+file.getName());
                telemetry.update();
            }
        } catch (IOException e) {
            RobotLog.ee(getClass().getName(), e, "exception in saveBitmap()");
            telemetry.addLine("exception saving "+file.getName());
            telemetry.update();
        }
    }
}
