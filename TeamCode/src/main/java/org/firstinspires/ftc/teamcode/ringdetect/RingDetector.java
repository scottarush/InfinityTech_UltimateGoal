package org.firstinspires.ftc.teamcode.ringdetect;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.ejml.data.DMatrixRMaj;
import org.firstinspires.ftc.teamcode.util.AppContextUtility;

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
    private RevColorSensorV3 mMiddleSensor;
    private DistanceSensor mProximitySensor;

    public RingDetector(OpMode opMode) {
        mOpMode = opMode;
    }

    public void init() throws Exception{
        String initErrString = "";

        try{
            InputStream is = AppContextUtility.getAssetManager().open("neural_network.bin");
            mNetwork = new RingDetectorNeuralNetwork(is);
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
            mMiddleSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "midClr");
        }
        catch(Exception e){
            initErrString += "middle color sensor error";
        }
        try {
            mTopColorSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "bottomClr");
        }
        catch(Exception e){
            initErrString += "bottom color sensor error";
        }
        try {
            mProximitySensor = mOpMode.hardwareMap.get(DistanceSensor.class, "range");
        }
        catch(Exception e){
            initErrString += "bottom color sensor error";
        }

        configureColorSensor(mTopColorSensor);
        configureColorSensor(mBottomColorSensor);
        configureColorSensor(mMiddleSensor);

        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }

    }
    private void configureColorSensor(RevColorSensorV3 sensor) {
        sensor.setGain(15);
    }

    /**
     * Called to read the sensors and do an inference.
     */
    public int readDetector(){
        // Read all the sensors into a measurement object for the network
        RingDetectorNeuralNetwork.MeasurementData data = new RingDetectorNeuralNetwork.MeasurementData();
        return  mNetwork.doInference(data);
    }
}
