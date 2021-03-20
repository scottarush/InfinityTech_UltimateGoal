package org.firstinspires.ftc.teamcode.ringdetect;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.CaptureCamera;
import org.firstinspires.ftc.teamcode.util.ICaptureCameraListener;

import java.io.File;
import java.util.ArrayList;

/**
 * This class implements the ring detector using a neural network.
 */
public class RingDetector implements ICaptureCameraListener {

    private RingDetectorNeuralNetwork mNetwork = null;
    private OpMode mOpMode = null;

    private CaptureCamera mCaptureCamera = null;

    private int mLastResult = RingDetectorNeuralNetwork.UNKNOWN;

    private Bitmap mCameraFrame = null;

    private int[] mAveragingInferenceArray = null;
    private int mAveragingInferenceIndex = 0;
    private boolean mAveragingInferenceActive = false;
    private IRingDetectorResultListener mRingDetectionResultListener = null;
    /**
     *
     * @param opMode
     */
    public RingDetector(OpMode opMode) {
        mOpMode = opMode;
    }

    public void init() throws Exception {
        String initErrString = "";
        try{
            mCaptureCamera = new CaptureCamera();
            mCaptureCamera.init(mOpMode,this);
        }
        catch (Exception e){
            initErrString += "camera init error";
        }

        // Load the neural network
        File nnFilePath = new File("/sdcard/nnfiles");
        File nnLogFile = new File("/sdcard/logs/ringnnlog.csv");
        try {
            mNetwork = new RingDetectorNeuralNetwork(nnFilePath, nnLogFile,true);
        }
        catch(Exception e){
            initErrString += "NeuralNet init error:"+e.getMessage();
        }

        if (initErrString.length() > 0) {
            throw new Exception(initErrString);
        }

    }

    /**
     * Must be called from OpMode loop to service the capture camera
     */
    public void serviceRingDetector(){
        mCaptureCamera.serviceCaptureCamera();
        processAverageInference();
     }

    private void processAverageInference(){
        // Check if average inference active
        if (mAveragingInferenceActive){
            int result = doSingleInference();
            // Skip it if we had a -1 for no camera frame, otherwise, record
            // the result and check if we have filled the window
            if (result >= 0) {
                mAveragingInferenceArray[mAveragingInferenceIndex++] = result;
                if (mAveragingInferenceIndex >= mAveragingInferenceArray.length) {
                    // filled the window, now pick the highest one.
                    mAveragingInferenceActive = false;
                    int countArray[] = new int[3];
                    for (int i = 0; i < mAveragingInferenceArray.length; i++) {
                        countArray[mAveragingInferenceArray[i]]++;
                    }
                    // Now return the index of the max count
                    int maxIndex = 0;
                    for (int i = 1; i < countArray.length; i++) {
                        if (countArray[i] > countArray[i - 1]) {
                            maxIndex = i;
                        }
                    }
                    mRingDetectionResultListener.averageDetectionResult(maxIndex);
                }
            }
        }
    }
    /**
     * Implementation of ICameraCaptureListener to receive a frame from the CaptureCamera
     * @param bitmap the new bitmap
     */
    @Override
    public void onNewFrame(Bitmap bitmap) {
        if (mCameraFrame != null){
            // Didn't process it so recycle first
            mCameraFrame.recycle();
        }
        mCameraFrame = bitmap.copy(Bitmap.Config.ARGB_8888,true);
    }

    /**
     * called to start ring detection with the camera
     */
    public void startDetection(){
        mCaptureCamera.startCapture();
    }
    /**
     * called to pause ring detection with the camera
     */
    public void pauseDetection(){
        mCaptureCamera.stopCapture();
    }


    public void stop() {
        mCaptureCamera.stop();
        mNetwork.closeLogFile();
    }

    /**
     * Performs a single inference
     * @return the inference result enumeration
     */
    public int doSingleInference(){
        if (!mCaptureCamera.isCaptureActive()){
            mCaptureCamera.startCapture();
        }
        // use the last frame unless null
        if (mCameraFrame == null){
            // Return last result as we have no new frame
            return mLastResult;
        }

        mLastResult = mNetwork.doInference(mCameraFrame);
        mCameraFrame.recycle();
        mCameraFrame = null;
        return mLastResult;
    }

    /**
     * Initiates an averaging inference.  Inferences will be done in the serviceRingDetector loop
     * after started.  Result will be returned via an IRingDetectorResultListener
     */
    public void doAveragedInference(int windowSize, IRingDetectorResultListener callbackListener){
        if (callbackListener == null)
            return;
        mRingDetectionResultListener = callbackListener;
        // Allocate the inference array
        mAveragingInferenceArray = new int[windowSize];
        mAveragingInferenceIndex = 0;
        mAveragingInferenceActive = true;
    }

    private void error(String message){
        mOpMode.telemetry.addLine(message);
        mOpMode.telemetry.update();
    }


}
