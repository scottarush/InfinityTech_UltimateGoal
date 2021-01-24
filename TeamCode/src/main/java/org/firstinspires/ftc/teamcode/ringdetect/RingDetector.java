package org.firstinspires.ftc.teamcode.ringdetect;

import android.graphics.Bitmap;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.teamcode.util.CaptureCamera;
import org.firstinspires.ftc.teamcode.util.ICaptureCameraListener;

import java.io.File;

/**
 * This class implements the ring detector using a neural network.
 */
public class RingDetector implements ICaptureCameraListener {

    private int mRingDetectorConfiguration = -1;

    private boolean mDistanceSensorEnabled = false;
    private boolean mMidColorSensorEnabled = false;
    private boolean mCameraEnabled = false;

    private RingDetectorNeuralNetwork mNetwork = null;
    private OpMode mOpMode = null;
    private RevColorSensorV3 mBottomColorSensor;
    private RevColorSensorV3 mTopColorSensor;
    private RevColorSensorV3 mMidColorSensor;
    private DistanceSensor mDistanceSensor;

    private CaptureCamera mCaptureCamera = null;
    private boolean mCameraProcessingActive = false;
    private Bitmap mCameraInputBitmap = null;



    /**
     *
     * @param ringDetectorConfiguration enumeration of color and
     * @param opMode
     */
    public RingDetector(int ringDetectorConfiguration,OpMode opMode) {
        mOpMode = opMode;
        mRingDetectorConfiguration = ringDetectorConfiguration;
    }

    public void init() throws Exception {
        String initErrString = "";
        // Enabled/disable sensors
        switch (mRingDetectorConfiguration) {
            case RingDetectorNeuralNetwork.CONFIGURATION_ALL_SENSORS:
                mMidColorSensorEnabled = true;
                mDistanceSensorEnabled = true;
                mCameraEnabled = false;
                break;
            case RingDetectorNeuralNetwork.CONFIGURATION_TOP_BOTTOM_COLOR_SENSORS_ONLY:
                mMidColorSensorEnabled = false;
                mDistanceSensorEnabled = false;
                mCameraEnabled = false;
                break;
            case RingDetectorNeuralNetwork.CONFIGURATION_NO_DISTANCE_SENSOR:
                mMidColorSensorEnabled = true;
                mDistanceSensorEnabled = false;
                mCameraEnabled = false;
                break;
            case RingDetectorNeuralNetwork.CONFIGURATION_NO_MID_COLOR_SENSOR:
                mMidColorSensorEnabled = false;
                mDistanceSensorEnabled = true;
                mCameraEnabled = false;
                break;
            case RingDetectorNeuralNetwork.CONFIGURATION_CAMERA_ONLY:
                mCaptureCamera = new CaptureCamera();
                mCameraEnabled = true;
                break;
        }
        try {
            File nnFilePath = new File("/sdcard/nnfiles");
            File nnLogFile = new File("/sdcard/logs/ringnnlog.csv");
            mNetwork = new RingDetectorNeuralNetwork(nnFilePath, mRingDetectorConfiguration, nnLogFile);
        } catch (Exception e) {
            initErrString += e.getMessage();
        }
        try {
            mTopColorSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "topClr");
        } catch (Exception e) {
            initErrString += "top color sensor error";
        }
        if (mMidColorSensorEnabled) {
            try {
                mMidColorSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "midClr");
            } catch (Exception e) {
                initErrString += "middle color sensor error";
            }
        }
        try {
            mBottomColorSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "bottomClr");
        } catch (Exception e) {
            initErrString += "bottom color sensor error";
        }
        if (mDistanceSensorEnabled) {
            try {
                mDistanceSensor = mOpMode.hardwareMap.get(DistanceSensor.class, "range");
            } catch (Exception e) {
                initErrString += "distance sensor error";
            }
        }
        if (mCameraEnabled){
            try{
                mCaptureCamera = new CaptureCamera();
                mCaptureCamera.init(mOpMode,this);
            }
            catch (Exception e){
                initErrString += "camera init error";
            }
        }
        // Configuration the color sensor unless the camera is enabled
        if (!mCameraEnabled) {
            configureColorSensor(mTopColorSensor);
            configureColorSensor(mBottomColorSensor);
            if (mMidColorSensorEnabled) {
                configureColorSensor(mMidColorSensor);
            }
        }

        if (initErrString.length() > 0) {
            throw new Exception(initErrString);
        }

    }

    /**
     * Implementation of ICameraCaptureListener to receive a frame from the CaptureCamera
     * @param bitmap the new bitmap
     */
    @Override
    public void onNewFrame(Bitmap bitmap) {

    }

    /**
     * called to start ring detection with the camera
     */
    public void startCameraDetection(){
        mCameraProcessingActive = true;
    }
    /**
     * called to pause ring detection with the camera
     */
    public void pauseCameraDetection(){
        mCameraProcessingActive = false;
    }


    public int getRingDetectorConfiguration(){
        return mRingDetectorConfiguration;
    }

    public void stop() {
            mNetwork.closeLogFile();
    }

    private void configureColorSensor(RevColorSensorV3 sensor) {
        sensor.setGain(15);
    }

    /**
     * Called for logging to telemetry.
     *
     * @return formatted string of current color sensor readings
     */
    public String getTelemetryString() {
        String s = "Top: {" + formatSensorValue(mTopColorSensor.getNormalizedColors().red) +
                ":" + formatSensorValue(mTopColorSensor.getNormalizedColors().green) +
                ":" + formatSensorValue(mTopColorSensor.getNormalizedColors().blue) +
                ":" + formatColorDistance(mTopColorSensor) +
                "}";
        s += "\nBottom: {" + formatSensorValue(mBottomColorSensor.getNormalizedColors().red) +
                ":" + formatSensorValue(mBottomColorSensor.getNormalizedColors().green) +
                ":" + formatSensorValue(mBottomColorSensor.getNormalizedColors().blue) +
                "," + formatColorDistance(mBottomColorSensor) +
                "}";
        if (mMidColorSensorEnabled) {
            s += "\nMiddle: {" + formatSensorValue(mMidColorSensor.getNormalizedColors().red) +
                    ":" + formatSensorValue(mMidColorSensor.getNormalizedColors().green) +
                    ":" + formatSensorValue(mMidColorSensor.getNormalizedColors().blue) +
                    ":" + formatColorDistance(mMidColorSensor) +
                    "}";
        }
        if (mDistanceSensorEnabled) {
            s += "\nDistance=" + String.format("%.01f mm", mDistanceSensor.getDistance(DistanceUnit.MM));
        }
        return s;
    }


    private String formatColorDistance(RevColorSensorV3 sensor) {
        return formatSensorValue((float) sensor.getDistance(DistanceUnit.MM));
    }

    // formats the numbers into a string to be able to print in telemetry
    private String formatSensorValue(float value) {
        return String.format("%.5f", value);
    }

    /**
     * Called to read the sensors and do an inference.
     *
     * @return the result according to the codes in {@link RingDetectorNeuralNetwork}
     */
    public int readDetector() {
        // Read all the sensors into a measurement object for the network
        switch (mRingDetectorConfiguration) {
            case RingDetectorNeuralNetwork.CONFIGURATION_ALL_SENSORS:
                return doAllSensorReadDetector();
            case RingDetectorNeuralNetwork.CONFIGURATION_NO_DISTANCE_SENSOR:
                return doNoDistanceSensorReadDetector();
            case RingDetectorNeuralNetwork.CONFIGURATION_NO_MID_COLOR_SENSOR:
                return doNoMidSensorReadDetector();
            case RingDetectorNeuralNetwork.CONFIGURATION_TOP_BOTTOM_COLOR_SENSORS_ONLY:
                return doTopBottomOnlyReadDetector();
        }
        // Invalid.  Shouldn't happen
        return RingDetectorNeuralNetwork.UNKNOWN;
    }

    private int doAllSensorReadDetector() {
        RingDetectorNeuralNetwork.AllSensorInputData data = new RingDetectorNeuralNetwork.AllSensorInputData();
        data.topColorRed = mTopColorSensor.getNormalizedColors().red;
        data.topColorBlue = mTopColorSensor.getNormalizedColors().blue;
        data.topColorGreen = mTopColorSensor.getNormalizedColors().green;
        data.topDistanceMM = mTopColorSensor.getDistance(DistanceUnit.MM);
        data.midColorRed = mMidColorSensor.getNormalizedColors().red;
        data.midColorBlue = mMidColorSensor.getNormalizedColors().blue;
        data.midColorGreen = mMidColorSensor.getNormalizedColors().green;
        data.midDistanceMM = mMidColorSensor.getDistance(DistanceUnit.MM);
        data.bottomColorRed = mBottomColorSensor.getNormalizedColors().red;
        data.bottomColorBlue = mBottomColorSensor.getNormalizedColors().blue;
        data.bottomColorGreen = mBottomColorSensor.getNormalizedColors().green;
        data.bottomDistanceMM = mBottomColorSensor.getDistance(DistanceUnit.MM);
        data.distanceSensorMM = mDistanceSensor.getDistance(DistanceUnit.MM);
        return mNetwork.doInference(data);
    }

    private int doNoDistanceSensorReadDetector() {
        RingDetectorNeuralNetwork.NoDistanceSensorInputData data = new RingDetectorNeuralNetwork.NoDistanceSensorInputData();
        data.topColorRed = mTopColorSensor.getNormalizedColors().red;
        data.topColorBlue = mTopColorSensor.getNormalizedColors().blue;
        data.topColorGreen = mTopColorSensor.getNormalizedColors().green;
        data.topDistanceMM = mTopColorSensor.getDistance(DistanceUnit.MM);
        data.midColorRed = mMidColorSensor.getNormalizedColors().red;
        data.midColorBlue = mMidColorSensor.getNormalizedColors().blue;
        data.midColorGreen = mMidColorSensor.getNormalizedColors().green;
        data.midDistanceMM = mMidColorSensor.getDistance(DistanceUnit.MM);
        data.bottomColorRed = mBottomColorSensor.getNormalizedColors().red;
        data.bottomColorBlue = mBottomColorSensor.getNormalizedColors().blue;
        data.bottomColorGreen = mBottomColorSensor.getNormalizedColors().green;
        data.bottomDistanceMM = mBottomColorSensor.getDistance(DistanceUnit.MM);
        return mNetwork.doInference(data);
    }

    private int doNoMidSensorReadDetector() {
        RingDetectorNeuralNetwork.NoMidInputData data = new RingDetectorNeuralNetwork.NoMidInputData();
        data.topColorRed = mTopColorSensor.getNormalizedColors().red;
        data.topColorBlue = mTopColorSensor.getNormalizedColors().blue;
        data.topColorGreen = mTopColorSensor.getNormalizedColors().green;
        data.topDistanceMM = mTopColorSensor.getDistance(DistanceUnit.MM);
        data.bottomColorRed = mBottomColorSensor.getNormalizedColors().red;
        data.bottomColorBlue = mBottomColorSensor.getNormalizedColors().blue;
        data.bottomColorGreen = mBottomColorSensor.getNormalizedColors().green;
        data.bottomDistanceMM = mBottomColorSensor.getDistance(DistanceUnit.MM);
        data.distanceSensorMM = mDistanceSensor.getDistance(DistanceUnit.MM);
        return mNetwork.doInference(data);
    }

    private int doTopBottomOnlyReadDetector() {
        RingDetectorNeuralNetwork.TopBottomOnlyInputData data = new RingDetectorNeuralNetwork.TopBottomOnlyInputData();
        data.topColorRed = mTopColorSensor.getNormalizedColors().red;
        data.topColorBlue = mTopColorSensor.getNormalizedColors().blue;
        data.topColorGreen = mTopColorSensor.getNormalizedColors().green;
        data.topDistanceMM = mTopColorSensor.getDistance(DistanceUnit.MM);
        data.bottomColorRed = mBottomColorSensor.getNormalizedColors().red;
        data.bottomColorBlue = mBottomColorSensor.getNormalizedColors().blue;
        data.bottomColorGreen = mBottomColorSensor.getNormalizedColors().green;
        data.bottomDistanceMM = mBottomColorSensor.getDistance(DistanceUnit.MM);
        return mNetwork.doInference(data);
    }


}
