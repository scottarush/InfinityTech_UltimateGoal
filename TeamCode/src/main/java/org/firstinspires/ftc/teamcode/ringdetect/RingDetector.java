package org.firstinspires.ftc.teamcode.ringdetect;

import android.os.Environment;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;

/**
 * This class implements the ring detector using a neural network.
 */
public class RingDetector {

    private RingDetectorNeuralNetwork mNetwork = null;
    private OpMode mOpMode = null;
    private RevColorSensorV3 mBottomColorSensor;
    private RevColorSensorV3 mTopColorSensor;
    private RevColorSensorV3 mMidColorSensor;
    private DistanceSensor mDistanceSensor;

    private FileWriter mLogWriter = null;

    public RingDetector(OpMode opMode) {
        mOpMode = opMode;
    }

    public void init() throws Exception{
        String initErrString = "";
        File nnPath = new File("/sdcard/neural_networks");
        File nnFile = new File(nnPath,"all_sensors_ring_neuralnetwork.bin");

        File logPath = new File("/sdcard/logs");
        File logFile = new File(logPath,"nnlog.csv");
        try{
            InputStream is = new FileInputStream(nnFile);
            mNetwork = new RingDetectorNeuralNetwork(is);

            if (logFile.exists()){
                logFile.delete();
            }
            mLogWriter = new FileWriter(logFile);
            mNetwork.setLogStream(mLogWriter);
        }
        catch(Exception e){
            initErrString += "Error reading neural network file:"+e.getMessage();
        }
        try {
            mTopColorSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "topClr");
        }
        catch(Exception e){
            initErrString += "top color sensor error";
        }
        try {
            mMidColorSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "midClr");
        }
        catch(Exception e){
            initErrString += "middle color sensor error";
        }
        try {
            mBottomColorSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "bottomClr");
        }
        catch(Exception e){
            initErrString += "bottom color sensor error";
        }
        try {
            mDistanceSensor = mOpMode.hardwareMap.get(DistanceSensor.class, "range");
        }
        catch(Exception e){
            initErrString += "distance sensor error";
        }

        configureColorSensor(mTopColorSensor);
        configureColorSensor(mBottomColorSensor);
        configureColorSensor(mMidColorSensor);

        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }

    }

    public void stop(){
        try {
            mLogWriter.close();
        }
        catch(IOException e){

        }
    }

    private void configureColorSensor(RevColorSensorV3 sensor) {
        sensor.setGain(15);
    }

    /**
     * Called to read the sensors and do an inference.
     * @return the result according to the codes in {@link RingDetectorNeuralNetwork}
     */
    public int readDetector(){
        // Read all the sensors into a measurement object for the network
        RingDetectorNeuralNetwork.RingSensorData data = new RingDetectorNeuralNetwork.RingSensorData();
        data.topColorRed = mTopColorSensor.getNormalizedColors().red;
        data.topColorBlue= mTopColorSensor.getNormalizedColors().blue;
        data.topColorGreen = mTopColorSensor.getNormalizedColors().green;
        data.topDistanceMM = mTopColorSensor.getDistance(DistanceUnit.MM);
        data.midColorRed = mMidColorSensor.getNormalizedColors().red;
        data.midColorBlue= mMidColorSensor.getNormalizedColors().blue;
        data.midColorGreen = mMidColorSensor.getNormalizedColors().green;
        data.midDistanceMM = mMidColorSensor.getDistance(DistanceUnit.MM);
        data.bottomColorRed = mBottomColorSensor.getNormalizedColors().red;
        data.bottomColorBlue= mBottomColorSensor.getNormalizedColors().blue;
        data.bottomColorGreen = mBottomColorSensor.getNormalizedColors().green;
        data.bottomDistanceMM = mBottomColorSensor.getDistance(DistanceUnit.MM);
        data.distanceSensorMM = mDistanceSensor.getDistance(DistanceUnit.MM);

        return  mNetwork.doInference(data);
    }
}
