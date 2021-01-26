package org.firstinspires.ftc.teamcode.ringdetect;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.CaptureCamera;
import org.firstinspires.ftc.teamcode.util.ICaptureCameraListener;
import org.firstinspires.ftc.teamcode.util.ImageUtils;

import java.io.File;

@TeleOp(name="ImageCaptureOpMode", group="Robot")
//@Disabled
public class ImageCaptureOpMode extends OpMode implements ICaptureCameraListener {


    private CaptureCamera mCaptureCamera = null;

    private boolean mCaptureImage = false;

    private File mCaptureDirectory = new File("/sdcard/nndata");

    private boolean mLastCaptureButtonState = false;
    private boolean mLastTagButtonState = false;
    private String mCurrentTag = RingDetectorNeuralNetwork.LABEL_STRING_NO_RING;
    private int mRecordNumber = 100;
    private Bitmap mCapturedBitmap = null;
    private String mLastSaveFile = "";

    @Override
    public void init() {
        try{
            mCaptureCamera = new CaptureCamera();
            mCaptureCamera.init(this,this);
        }
        catch (Exception e){
            telemetry.addLine("Error initializing camera:"+e.getMessage());
        }
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

        // Start the camera (if it wasn't already)
        mCaptureCamera.startCapture();

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
        mLastCaptureButtonState = buttonState;

        if (edgeDetect){
            // Capture the next image
            mCaptureImage = true;
        }

        // service the camera
        mCaptureCamera.serviceCaptureCamera();

        if (mCapturedBitmap != null){
            // Save this one, recycle and clear for next time
            if (mCaptureDirectory != null) {
                mLastSaveFile = mCurrentTag + "_" + mRecordNumber + ".png";
                File file = new File(mCaptureDirectory, mLastSaveFile);
                mRecordNumber++;  // for next time

                ImageUtils.savePNG(mCapturedBitmap, file);
            }
            mCapturedBitmap.recycle();
            mCapturedBitmap = null;
        }

        telemetry.addLine("Current Label="+mCurrentTag);
        telemetry.addLine("Last Saved File="+mLastSaveFile);
        telemetry.update();
     }

    @Override
    public void stop() {
        super.stop();
        mCaptureCamera.stop();
    }

    @Override
    public void onNewFrame(Bitmap bitmap) {
        if (mCaptureImage){
            mCaptureImage = false;
            mCapturedBitmap = bitmap.copy(Bitmap.Config.ARGB_8888,true);
        }
        bitmap.recycle();
    }


}
