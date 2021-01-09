package org.firstinspires.ftc.teamcode.ringdetect;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CaptureCamera;
import org.firstinspires.ftc.teamcode.util.ICaptureCameraListener;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name="ImageCaptureOpMode", group="Robot")
public class ImageCaptureOpMode extends OpMode implements ICaptureCameraListener {

    private File mCaptureDirectory = null;

    private CaptureCamera mCaptureCamera = null;

    private boolean mCaptureImage = false;
    private int mCaptureCounter = 1;
    private boolean mLastCaptureButtonState = false;
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
    }

    @Override
    public void loop() {
        // detect the tag button edge and change tag state if rising edge
        boolean edgeDetect = false;
        boolean buttonState = gamepad1.y;
        if (buttonState) {
            // button pressed. check if last false
            if (mLastCaptureButtonState == false) {
                edgeDetect = true;
            }
        }
        // Save tag button for next time
        mLastCaptureButtonState = buttonState;

        if (edgeDetect){
            // Capture the next image
            mCaptureImage = true;
        }
        // service the camera
        mCaptureCamera.serviceCaptureCamera();
    }

    @Override
    public void onNewFrame(Bitmap bitmap) {
        if (mCaptureImage){
            Bitmap rgbBitmap = bitmap.copy(Bitmap.Config.ARGB_8888,true);
            saveBitmap(rgbBitmap);
            mCaptureImage = false;
        }
    }

    private void saveBitmap(Bitmap bitmap) {
        if (mCaptureDirectory == null)
            return;
        if (!mCaptureDirectory.exists()){
            mCaptureDirectory.mkdir();
        }
        File file = new File(mCaptureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", mCaptureCounter++));
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
