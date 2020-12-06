package org.firstinspires.ftc.teamcode.ringdetect;

import android.util.Log;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.NormalSsaInsn;
import org.firstinspires.ftc.teamcode.util.LogFile;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

@TeleOp(name="NeuralNetworkDataCollection", group="Robot")
public class RingNeuralNetworkDataCollectionOpMode extends OpMode {
    public static final String LOG_PATHNAME = "/sdcard";

    public static final String LOG_FILENAME = "nndata.csv";
    public static final String[] LOG_COLUMNS = {"Record#","tag", "light status","distance", "top_red", "top_green", "top_blue", "top_distance", "mid_red", "mid_green", "mid_blue", "mid_distance", "bottom_red", "bottom_green", "bottom_blue", "bottom_distance"};
    private LogFile mLogFile;
    private RevColorSensorV3 mBottomColorSensor;
    private RevColorSensorV3 mTopColorSensor;
    private RevColorSensorV3 mMiddleSensor;
    private DistanceSensor mProximitySensor;
    private String mCurrentTag = "No Rings";
    private int mRecordNumber = 0;
    private OneShotTimer mCaptureMessageTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
        }
    });
    private boolean mLightOn = true;
    private boolean mLastTagButtonState = false;
    private boolean mLastTriggerButtonState = false;
    private boolean mLastLightButtonState = false;
    public static final String NO_RING_TAG = "NoRing";
    public static final String ONE_RING_TAG = "OneRing";
    public static final String FOUR_RING_TAG = "FourRings";

    @Override
    public void init() {
        mLogFile = new LogFile(LOG_PATHNAME, LOG_FILENAME, LOG_COLUMNS);
        mLogFile.openFile();

        mTopColorSensor = hardwareMap.get(RevColorSensorV3.class, "topClr");
        mBottomColorSensor = hardwareMap.get(RevColorSensorV3.class, "bottomClr");
        mMiddleSensor = hardwareMap.get(RevColorSensorV3.class, "midClr");
        mProximitySensor = hardwareMap.get(DistanceSensor.class, "range");

        configureColorSensor(mTopColorSensor);
        configureColorSensor(mBottomColorSensor);
        configureColorSensor(mMiddleSensor);
    }

    private void configureColorSensor(RevColorSensorV3 sensor) {
        sensor.setGain(15);
    }

    public void stop() {
        mLogFile.closeFile();
        super.stop();
    }

    @Override
    public void start() {
        super.start();
    }

    public void loop() {
        // Service timer
        mCaptureMessageTimer.checkTimer();

        // detect the tag button edge and change tag state if rising edge
        boolean edgeDetect = false;
        boolean buttonState = gamepad1.y;
        if (buttonState) {
            // button pressed. check if last false
            if (mLastTagButtonState == false) {
                edgeDetect = true;
            }
        }
        // Save tag button for next time
        mLastTagButtonState = buttonState;

        // if we had a rising edge, cycle to the next tag
        if (edgeDetect == true) {
            if (mCurrentTag == "No Rings") {
                mCurrentTag = "One Ring";
            } else if (mCurrentTag == "One Ring") {
                mCurrentTag = "Four Rings";
            } else {
                mCurrentTag = "No Rings";
            }
        }
        // check for x button triggered to toggle the light flag
        edgeDetect = false;
        buttonState = gamepad1.x;
        if (buttonState) {
            if (mLastLightButtonState == false) {
                edgeDetect = true;
            }
        }
        if (edgeDetect == true) {
            mLightOn = !mLightOn;
        }
        // save last state for next time
        mLastLightButtonState = buttonState;

        // check for trigger button edge
        edgeDetect = false;
        buttonState = gamepad1.a;
        if (buttonState) {
            // button pressed. check if last false
            if (mLastTriggerButtonState == false) {
                edgeDetect = true;
            }
        }
        if (edgeDetect) {
            // capture data
            logData();
            // Start capture message timer
            mCaptureMessageTimer.start();
        }
        // Save trigger button for next time
        mLastTriggerButtonState = buttonState;

        String message = "";
        if (mCaptureMessageTimer.isRunning()){
            message = "Captured record "+mRecordNumber;
        }
        telemetry.addData("Status",  mCurrentTag+" "+message);
        // Now build the logging data output
        String s = "{"+ formatSensorValue(mTopColorSensor.getNormalizedColors().red)+
                ", " + formatSensorValue(mTopColorSensor.getNormalizedColors().green)+
                ", " + formatSensorValue(mTopColorSensor.getNormalizedColors().blue)+
                "}\nBottom: {" + formatSensorValue(mBottomColorSensor.getNormalizedColors().red)+
                ", " + formatSensorValue(mBottomColorSensor.getNormalizedColors().green)+
                ", " + formatSensorValue(mBottomColorSensor.getNormalizedColors().blue)+
                "}\nMiddle: {"+ formatSensorValue(mMiddleSensor.getNormalizedColors().red)+
                ", " + formatSensorValue(mMiddleSensor.getNormalizedColors().green)+
                ", " + formatSensorValue(mMiddleSensor.getNormalizedColors().blue) + "}";

        String distance = "\nTop Color Sensor : " + formatColorDistance(mTopColorSensor)+
                "mm\nMiddle Color Sensor : " + formatColorDistance(mMiddleSensor)+
                "mm\nBottom Color Sensor : " + formatColorDistance(mBottomColorSensor)+"mm";
        String lightstatus = "No";
        if (mLightOn) {
            lightstatus = "Yes";
        }
        telemetry.addData("Light Status", lightstatus);
        telemetry.addData("Top", s);
        telemetry.addData("Proximity Sensor Distance", String.format("%.01f mm", mProximitySensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("Color sensor Distances", distance);
        telemetry.update();

     }
    // formats the numbers into a string to be able to print in telemetry
     private String formatSensorValue(float value){
        return String.format("%.4f",value);
     }
     private String formatColorDistance(RevColorSensorV3 sensor){
        return formatSensorValue((float) ((DistanceSensor) sensor).getDistance(DistanceUnit.MM));
     }
    /**
     * Saves the capture data
     */
    private void logData(){
        // increment the record number
        mRecordNumber++;
        // Now form the record for the log
        String[] logRecord = new String[LOG_COLUMNS.length];
        int logIndex = 0;
        logRecord[logIndex++] = Integer.toString(mRecordNumber);
        logRecord[logIndex++] = mCurrentTag;
        String light = "Off";
        if (mLightOn)
            light = "On";
        logRecord[logIndex++] = light;
        logRecord[logIndex++] = String.format("%.01f", mProximitySensor.getDistance(DistanceUnit.MM));
        logRecord[logIndex++] = formatSensorValue(mTopColorSensor.getNormalizedColors().red);
        logRecord[logIndex++] = formatSensorValue(mTopColorSensor.getNormalizedColors().green);
        logRecord[logIndex++] = formatSensorValue(mTopColorSensor.getNormalizedColors().blue);
        logRecord[logIndex++] = formatColorDistance(mTopColorSensor);
        logRecord[logIndex++] = formatSensorValue(mMiddleSensor.getNormalizedColors().red);
        logRecord[logIndex++] = formatSensorValue(mMiddleSensor.getNormalizedColors().green);
        logRecord[logIndex++] = formatSensorValue(mMiddleSensor.getNormalizedColors().blue);
        logRecord[logIndex++] = formatColorDistance(mMiddleSensor);
        logRecord[logIndex++] = formatSensorValue(mBottomColorSensor.getNormalizedColors().red);
        logRecord[logIndex++] = formatSensorValue(mBottomColorSensor.getNormalizedColors().green);
        logRecord[logIndex++] = formatSensorValue(mBottomColorSensor.getNormalizedColors().blue);
        logRecord[logIndex] = formatColorDistance(mBottomColorSensor);

                mLogFile.writeLogRow(logRecord);

    }

}
