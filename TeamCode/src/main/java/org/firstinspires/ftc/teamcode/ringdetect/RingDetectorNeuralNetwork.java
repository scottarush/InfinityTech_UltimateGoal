package org.firstinspires.ftc.teamcode.ringdetect;

import org.ejml.simple.SimpleMatrix;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;

/**
 * This class extends JavaNeuralNetwork for the RingDetector specific network.
 */
public class RingDetectorNeuralNetwork extends NeuralNetwork {
    // Detection value for single ring
    public static final int NO_RING = 0;
    // Detection value for one ring
    public static final int ONE_RING = 1;
    // Detection value for four rings
    public static final int FOUR_RINGS = 2;
    // Detection value for unknown
    public static final int UNKNOWN = -1;

    // Sensor configuration with all sensors
    public static final int ALL_SENSORS = 0;
    // Sensor configuration with no mid color sensor
    public static final int NO_MID_COLOR_SENSOR = 1;
    // Sensor with no distance sensor
    public static final int NO_DISTANCE_SENSOR = 2;
    // Sensor with only top and bottom color sensors
    public static final int TOP_BOTTOM_COLOR_SENSORS_ONLY = 3;

    // Selected configuration
    private int mSensorConfiguration = ALL_SENSORS;

    //------------------------------------------------------
    // Indexes for All Sensors configuration
    //------------------------------------------------------
    public static final String ALL_SENSORS_NEURAL_NETWORK_FILE = "all_sensors_ringnn.bin";
    public static final int ALL_SENSORS_INPUT_ROWS = 13;
    // Columns in the X INPUT DATA for the all sensor configuration
    public static final int ALL_SENSORS_DISTANCE_ROW_INDEX = 0;
    public static final int ALL_SENSORS_TOP_RED_ROW_INDEX = 1;
    public static final int ALL_SENSORS_TOP_GREEN_ROW_INDEX = 2;
    public static final int ALL_SENSORS_TOP_BLUE_ROW_INDEX = 3;
    public static final int ALL_SENSORS_TOP_DISTANCE_ROW_INDEX = 4;
    public static final int ALL_SENSORS_MID_RED_ROW_INDEX = 5;
    public static final int ALL_SENSORS_MID_GREEN_ROW_INDEX = 6;
    public static final int ALL_SENSORS_MID_BLUE_ROW_INDEX = 7;
    public static final int ALL_SENSORS_MID_DISTANCE_ROW_INDEX = 8;
    public static final int ALL_SENSORS_BOTTOM_RED_ROW_INDEX = 9;
    public static final int ALL_SENSORS_BOTTOM_GREEN_ROW_INDEX = 10;
    public static final int ALL_SENSORS_BOTTOM_BLUE_ROW_INDEX = 11;
    public static final int ALL_SENSORS_BOTTOM_DISTANCE_ROW_INDEX = 12;

    private static final String ALL_SENSORS_LOGGING_HEADER = "Result,y0,y1,y2,Dista,TopR,TopG,TopB,TopDist,MidR,MidG,MidB,MidDist,BottomR,BottomG,BottomB,BottomDist";

    //------------------------------------------------------
    // Indexes for No Middle Color Sensor configuration
    //------------------------------------------------------
    public static final String NO_MID_COLOR_SENSOR_NEURAL_NETWORK_FILE = "no_mid_ringnn.bin";

    public static final int NO_MID_COLOR_SENSOR_INPUT_ROWS = 9;
    // Columns in the X INPUT DATA for the all sensor configuration
    public static final int NO_MID_COLOR_SENSOR_DISTANCE_ROW_INDEX = 0;
    public static final int NO_MID_COLOR_SENSOR_TOP_RED_ROW_INDEX = 1;
    public static final int NO_MID_COLOR_SENSOR_TOP_GREEN_ROW_INDEX = 2;
    public static final int NO_MID_COLOR_SENSOR_TOP_BLUE_ROW_INDEX = 3;
    public static final int NO_MID_COLOR_SENSOR_TOP_DISTANCE_ROW_INDEX = 4;
    public static final int NO_MID_COLOR_SENSOR_BOTTOM_RED_ROW_INDEX = 5;
    public static final int NO_MID_COLOR_SENSOR_BOTTOM_GREEN_ROW_INDEX = 6;
    public static final int NO_MID_COLOR_SENSOR_BOTTOM_BLUE_ROW_INDEX = 7;
    public static final int NO_MID_COLOR_SENSOR_BOTTOM_DISTANCE_ROW_INDEX = 8;

    private static final String NO_MID_COLOR_SENSOR_LOGGING_HEADER = "Result,y0,y1,y2,Dista,TopR,TopG,TopB,TopDist,BottomR,BottomG,BottomB,BottomDist";
    //------------------------------------------------------
    // Indexes for No Distance Sensor configuration
    //------------------------------------------------------
    public static final String NO_DISTANCE_SENSOR_NEURAL_NETWORK_FILE = "no_distance_ringnn.bin";
    public static final int NO_DISTANCE_SENSOR_INPUT_ROWS = 12;
    // Columns in the X INPUT DATA for the all sensor configuration
    public static final int NO_DISTANCE_SENSOR_TOP_RED_ROW_INDEX = 0;
    public static final int NO_DISTANCE_SENSOR_TOP_GREEN_ROW_INDEX = 1;
    public static final int NO_DISTANCE_SENSOR_TOP_BLUE_ROW_INDEX = 2;
    public static final int NO_DISTANCE_SENSOR_TOP_DISTANCE_ROW_INDEX = 3;
    public static final int NO_DISTANCE_SENSOR_MID_RED_ROW_INDEX = 4;
    public static final int NO_DISTANCE_SENSOR_MID_GREEN_ROW_INDEX = 5;
    public static final int NO_DISTANCE_SENSOR_MID_BLUE_ROW_INDEX = 6;
    public static final int NO_DISTANCE_SENSOR_MID_DISTANCE_ROW_INDEX = 7;
    public static final int NO_DISTANCE_SENSOR_BOTTOM_RED_ROW_INDEX = 8;
    public static final int NO_DISTANCE_SENSOR_BOTTOM_GREEN_ROW_INDEX = 9;
    public static final int NO_DISTANCE_SENSOR_BOTTOM_BLUE_ROW_INDEX = 10;
    public static final int NO_DISTANCE_SENSOR_BOTTOM_DISTANCE_ROW_INDEX = 11;

    private static final String NO_DISTANCE_SENSOR_LOGGING_HEADER = "Result,y0,y1,y2,TopR,TopG,TopB,TopDist,MidR,MidG,MidB,MidDist,BottomR,BottomG,BottomB,BottomDist";
    //------------------------------------------------------
    // Indexes for Top and Bottom color sensor only configuration
    //------------------------------------------------------
    public static final String TOP_BOTTOM_ONLY_NEURAL_NETWORK_FILE = "top_bottom_ringnn.bin";

    public static final int TOP_BOTTOM_ONLY_INPUT_ROWS = 9;
    // Columns in the X INPUT DATA for the all sensor configuration
    public static final int TOP_BOTTOM_ONLY_TOP_RED_ROW_INDEX = 0;
    public static final int TOP_BOTTOM_ONLY_TOP_GREEN_ROW_INDEX = 1;
    public static final int TOP_BOTTOM_ONLY_TOP_BLUE_ROW_INDEX = 2;
    public static final int TOP_BOTTOM_ONLY_TOP_DISTANCE_ROW_INDEX = 3;
    public static final int TOP_BOTTOM_ONLY_BOTTOM_RED_ROW_INDEX = 5;
    public static final int TOP_BOTTOM_ONLY_BOTTOM_GREEN_ROW_INDEX = 6;
    public static final int TOP_BOTTOM_ONLY_BOTTOM_BLUE_ROW_INDEX = 7;
    public static final int TOP_BOTTOM_ONLY_BOTTOM_DISTANCE_ROW_INDEX = 8;

    private static final String TOP_BOTTOM_ONLY_LOGGING_HEADER = "Result,y0,y1,y2,TopR,TopG,TopB,TopDist,BottomR,BottomG,BottomB,BottomDist";

    private int mLastInference = RingDetectorNeuralNetwork.UNKNOWN;

    private FileWriter mLogWriter = null;

    /**
     * @param sensorConfig enum of current sensor config.
     * @throws Exception if neural network cannot be read from the file
     */
    public RingDetectorNeuralNetwork(int sensorConfig) throws Exception {
        mSensorConfiguration = sensorConfig;

         File nnFile = getNeuralNetworkFile(mSensorConfiguration);
        try {
            // Open the file and deserialize the network
            InputStream is = new FileInputStream(nnFile);
            deserializeNetwork(is);

            // And initialize the logging files
            File logPath = new File("/sdcard/logs");
            File logFile = new File(logPath, "nnlog.csv");
            initLogFile(logFile);
        }
        catch (Exception e) {
            throw new Exception("Error reading neural network file:" + e.getMessage());
        }

    }

    /**
     * Utility to retrieve the neural network File for a specific network configuration
     * @param networkConfiguration the neural network configuration
     * @throws IOException
     */
    public static File getNeuralNetworkFile(int networkConfiguration){
        File nnPath = new File("/sdcard/neural_networks");
        File nnFile = null;
        switch(networkConfiguration){
            case ALL_SENSORS:
                nnFile = new File(nnPath,ALL_SENSORS_NEURAL_NETWORK_FILE);
                break;
            case NO_MID_COLOR_SENSOR:
                nnFile = new File(nnPath,NO_MID_COLOR_SENSOR_NEURAL_NETWORK_FILE);
                break;
            case NO_DISTANCE_SENSOR:
                nnFile = new File(nnPath,NO_DISTANCE_SENSOR_NEURAL_NETWORK_FILE);
                break;
            case TOP_BOTTOM_COLOR_SENSORS_ONLY:
                nnFile = new File(nnPath,TOP_BOTTOM_ONLY_NEURAL_NETWORK_FILE);
                break;
        }
        return nnFile;
    }
    private void initLogFile(File logFile) throws IOException{
        if (logFile.exists()) {
            logFile.delete();
        }
        mLogWriter = new FileWriter(logFile);
        String header = "";
        switch(mSensorConfiguration){
            case ALL_SENSORS:
                header = ALL_SENSORS_LOGGING_HEADER;
                break;
            case NO_MID_COLOR_SENSOR:
                header = NO_MID_COLOR_SENSOR_LOGGING_HEADER;
                break;
            case NO_DISTANCE_SENSOR:
                header = NO_DISTANCE_SENSOR_LOGGING_HEADER;
                break;
            case TOP_BOTTOM_COLOR_SENSORS_ONLY:
                header = TOP_BOTTOM_ONLY_LOGGING_HEADER;
                break;
        }
        mLogWriter.write(header);
        mLogWriter.write("\n");
    }

    /**
     * Input data  for all sensors configuration
     */
    static class AllSensorInputData {
        public double topColorRed;
        public double topColorBlue;
        public double topColorGreen;
        public double topDistanceMM;
        public double midColorRed;
        public double midColorBlue;
        public double midColorGreen;
        public double midDistanceMM;
        public double bottomColorRed;
        public double bottomColorBlue;
        public double bottomColorGreen;
        public double bottomDistanceMM;
        public double distanceSensorMM;
    }

    /**
     * Input data for no mid sensor configuration
     */
    static class NoMidInputData {
        public double topColorRed;
        public double topColorBlue;
        public double topColorGreen;
        public double topDistanceMM;
        public double bottomColorRed;
        public double bottomColorBlue;
        public double bottomColorGreen;
        public double bottomDistanceMM;
        public double distanceSensorMM;
    }

    /**
     * Input data for top and bottom only sensor configuration
     */
    static class TopBottomOnlyInputData {
        public double topColorRed;
        public double topColorBlue;
        public double topColorGreen;
        public double topDistanceMM;
        public double bottomColorRed;
        public double bottomColorBlue;
        public double bottomColorGreen;
        public double bottomDistanceMM;
    }
    /**
     * Input data  for no distance sensor configuration
     */
    static class NoDistanceSensorInputData {
        public double topColorRed;
        public double topColorBlue;
        public double topColorGreen;
        public double topDistanceMM;
        public double midColorRed;
        public double midColorBlue;
        public double midColorGreen;
        public double midDistanceMM;
        public double bottomColorRed;
        public double bottomColorBlue;
        public double bottomColorGreen;
        public double bottomDistanceMM;
    }

    /**
     * Performs a measurement for the ALL_SENSORS configuration.
     * @param measurementData vector of new measurement data
     * @returns NO_RING, ONE_RING, or FOUR_RINGS, or UNKNOWN
     */
    public int doInference(AllSensorInputData measurementData) {
        SimpleMatrix x = new SimpleMatrix(ALL_SENSORS_INPUT_ROWS, 1);
        x.set(ALL_SENSORS_DISTANCE_ROW_INDEX, measurementData.distanceSensorMM);
        x.set(ALL_SENSORS_TOP_RED_ROW_INDEX, measurementData.topColorRed);
        x.set(ALL_SENSORS_TOP_BLUE_ROW_INDEX, measurementData.topColorBlue);
        x.set(ALL_SENSORS_TOP_GREEN_ROW_INDEX, measurementData.topColorGreen);
        x.set(ALL_SENSORS_TOP_DISTANCE_ROW_INDEX, measurementData.topDistanceMM);
        x.set(ALL_SENSORS_MID_RED_ROW_INDEX, measurementData.midColorRed);
        x.set(ALL_SENSORS_MID_BLUE_ROW_INDEX, measurementData.midColorBlue);
        x.set(ALL_SENSORS_MID_GREEN_ROW_INDEX, measurementData.midColorGreen);
        x.set(ALL_SENSORS_MID_DISTANCE_ROW_INDEX, measurementData.midDistanceMM);
        x.set(ALL_SENSORS_BOTTOM_RED_ROW_INDEX, measurementData.bottomColorRed);
        x.set(ALL_SENSORS_BOTTOM_BLUE_ROW_INDEX, measurementData.bottomColorBlue);
        x.set(ALL_SENSORS_BOTTOM_GREEN_ROW_INDEX, measurementData.bottomColorGreen);
        x.set(ALL_SENSORS_BOTTOM_DISTANCE_ROW_INDEX, measurementData.bottomDistanceMM);
        return doInference(x);
    }
    /**
     * Performs a measurement for the NO_MID_COLOR_SENSOR configuration.
     * @param measurementData vector of new measurement data
     * @returns NO_RING, ONE_RING, or FOUR_RINGS, or UNKNOWN
     */
    public int doInference(NoMidInputData measurementData) {
        SimpleMatrix x = new SimpleMatrix(NO_MID_COLOR_SENSOR_INPUT_ROWS, 1);
        x.set(NO_MID_COLOR_SENSOR_DISTANCE_ROW_INDEX, measurementData.distanceSensorMM);
        x.set(NO_MID_COLOR_SENSOR_TOP_RED_ROW_INDEX, measurementData.topColorRed);
        x.set(NO_MID_COLOR_SENSOR_TOP_BLUE_ROW_INDEX, measurementData.topColorBlue);
        x.set(NO_MID_COLOR_SENSOR_TOP_GREEN_ROW_INDEX, measurementData.topColorGreen);
        x.set(NO_MID_COLOR_SENSOR_TOP_DISTANCE_ROW_INDEX, measurementData.topDistanceMM);
        x.set(NO_MID_COLOR_SENSOR_BOTTOM_RED_ROW_INDEX, measurementData.bottomColorRed);
        x.set(NO_MID_COLOR_SENSOR_BOTTOM_BLUE_ROW_INDEX, measurementData.bottomColorBlue);
        x.set(NO_MID_COLOR_SENSOR_BOTTOM_GREEN_ROW_INDEX, measurementData.bottomColorGreen);
        x.set(NO_MID_COLOR_SENSOR_BOTTOM_DISTANCE_ROW_INDEX, measurementData.bottomDistanceMM);
        return doInference(x);
    }
    /**
     * Performs a measurement for the NO_DISTANCE_SENSOR configuration.
     * @param measurementData vector of new measurement data
     * @returns NO_RING, ONE_RING, or FOUR_RINGS, or UNKNOWN
     */
    public int doInference(NoDistanceSensorInputData measurementData) {
        SimpleMatrix x = new SimpleMatrix(ALL_SENSORS_INPUT_ROWS, 1);
        x.set(NO_DISTANCE_SENSOR_TOP_RED_ROW_INDEX, measurementData.topColorRed);
        x.set(NO_DISTANCE_SENSOR_TOP_BLUE_ROW_INDEX, measurementData.topColorBlue);
        x.set(NO_DISTANCE_SENSOR_TOP_GREEN_ROW_INDEX, measurementData.topColorGreen);
        x.set(NO_DISTANCE_SENSOR_TOP_DISTANCE_ROW_INDEX, measurementData.topDistanceMM);
        x.set(NO_DISTANCE_SENSOR_MID_RED_ROW_INDEX, measurementData.midColorRed);
        x.set(NO_DISTANCE_SENSOR_MID_BLUE_ROW_INDEX, measurementData.midColorBlue);
        x.set(NO_DISTANCE_SENSOR_MID_GREEN_ROW_INDEX, measurementData.midColorGreen);
        x.set(NO_DISTANCE_SENSOR_MID_DISTANCE_ROW_INDEX, measurementData.midDistanceMM);
        x.set(NO_DISTANCE_SENSOR_BOTTOM_RED_ROW_INDEX, measurementData.bottomColorRed);
        x.set(NO_DISTANCE_SENSOR_BOTTOM_BLUE_ROW_INDEX, measurementData.bottomColorBlue);
        x.set(NO_DISTANCE_SENSOR_BOTTOM_GREEN_ROW_INDEX, measurementData.bottomColorGreen);
        x.set(NO_DISTANCE_SENSOR_BOTTOM_DISTANCE_ROW_INDEX, measurementData.bottomDistanceMM);
        return doInference(x);
    }
    /**
     * Performs a measurement for the TOP_BOTTOM_COLOR_SENSORS_ONLY configuration.
     * @param measurementData vector of new measurement data
     * @returns NO_RING, ONE_RING, or FOUR_RINGS, or UNKNOWN
     */
    public int doInference(TopBottomOnlyInputData measurementData) {
        SimpleMatrix x = new SimpleMatrix(ALL_SENSORS_INPUT_ROWS, 1);
        x.set(TOP_BOTTOM_ONLY_TOP_RED_ROW_INDEX, measurementData.topColorRed);
        x.set(TOP_BOTTOM_ONLY_TOP_BLUE_ROW_INDEX, measurementData.topColorBlue);
        x.set(TOP_BOTTOM_ONLY_TOP_GREEN_ROW_INDEX, measurementData.topColorGreen);
        x.set(TOP_BOTTOM_ONLY_TOP_DISTANCE_ROW_INDEX, measurementData.topDistanceMM);
         x.set(TOP_BOTTOM_ONLY_BOTTOM_RED_ROW_INDEX, measurementData.bottomColorRed);
        x.set(TOP_BOTTOM_ONLY_BOTTOM_BLUE_ROW_INDEX, measurementData.bottomColorBlue);
        x.set(TOP_BOTTOM_ONLY_BOTTOM_GREEN_ROW_INDEX, measurementData.bottomColorGreen);
        x.set(TOP_BOTTOM_ONLY_BOTTOM_DISTANCE_ROW_INDEX, measurementData.bottomDistanceMM);
        return doInference(x);
    }

    /**
     * Internal method for inference
     * @param x input vector that must be in the format for the current configuration
     * @returns NO_RING, ONE_RING, or FOUR_RINGS, or UNKNOWN
     */
    private int doInference(SimpleMatrix x){
        SimpleMatrix y = feedForward(x);
        int inference = decodeOutput(y);

        if (mLogWriter != null){
            if (mLastInference != inference) {
                logInference(x, y, inference);
                mLastInference = inference;
            }
        }

        return inference;
    }

    private void logInference(SimpleMatrix x, SimpleMatrix y,int inference){
        try{
            mLogWriter.write(RingDetectorNeuralNetwork.convertToString(inference));
            mLogWriter.write(",");
            for(int i=0;i < y.numRows();i++){
                mLogWriter.write(String.format("%1.5f",y.get(i,0)));
                mLogWriter.write(",");
            }
            int numRows = 0;
            switch(mSensorConfiguration){
                case ALL_SENSORS:
                    numRows = ALL_SENSORS_INPUT_ROWS;
                    break;
                case NO_DISTANCE_SENSOR:
                    numRows = NO_DISTANCE_SENSOR_INPUT_ROWS;
                    break;
                case NO_MID_COLOR_SENSOR:
                    numRows = NO_MID_COLOR_SENSOR_INPUT_ROWS;
                    break;
                case TOP_BOTTOM_COLOR_SENSORS_ONLY:
                    numRows = TOP_BOTTOM_ONLY_INPUT_ROWS;
                    break;
            }
            for(int i = 0; i < numRows; i++){
                mLogWriter.write(String.format("%1.5f",x.get(i,0)));
                if (i < numRows -1){
                    mLogWriter.write(",");
                }
            }
            mLogWriter.write("\n");
        }
        catch(IOException e){

        }

    }

    /**
     * Performs a test measurement
     *
     * @returns output vector
     */
    public SimpleMatrix doTestInference(SimpleMatrix rawInput) {
        SimpleMatrix result = feedForward(rawInput);
        return result;
    }

    public static int decodeOutput(SimpleMatrix result) {
        double max = 0d;
        int maxIndex = UNKNOWN;
        for(int i=0;i < result.numRows();i++){
            if (result.get(i) > max){
                max = result.get(i);
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    public static String convertToString(int result) {
        switch (result) {
            case NO_RING:
                return "No Ring";
            case ONE_RING:
                return "One Ring";
            case FOUR_RINGS:
                return "Four Rings";
            default:
                return "Unknown";
        }
    }

}
