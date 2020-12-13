package org.firstinspires.ftc.teamcode.ringdetect;

import org.ejml.simple.SimpleMatrix;

import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;

/**
 * This class extends JavaNeuralNetwork for the RingDetector specific network.
 */
public class RingDetectorNeuralNetwork extends JavaNeuralNetwork {
    // Detection value for single ring
    public static final int NO_RING = 0;
    // Detection value for one ring
    public static final int ONE_RING = 1;
    // Detection value for four rings
    public static final int FOUR_RINGS = 2;
    // Detection value for unknown
    public static final int UNKNOWN = -1;

    private static final int INPUT_ROWS = 13;
    // Columns in the X INPUT DATA
    private static final int DISTANCE_ROW_INDEX = 0;
    private static final int TOP_RED_ROW_INDEX = 1;
    private static final int TOP_GREEN_ROW_INDEX = 2;
    private static final int TOP_BLUE_ROW_INDEX = 3;
    private static final int TOP_DISTANCE_ROW_INDEX = 4;
    private static final int MID_RED_ROW_INDEX = 5;
    private static final int MID_GREEN_ROW_INDEX = 6;
    private static final int MID_BLUE_ROW_INDEX = 7;
    private static final int MID_DISTANCE_ROW_INDEX = 8;
    private static final int BOTTOM_RED_ROW_INDEX = 9;
    private static final int BOTTOM_GREEN_ROW_INDEX = 10;
    private static final int BOTTOM_BLUE_ROW_INDEX = 11;
    private static final int BOTTOM_DISTANCE_ROW_INDEX = 12;

    private static final String LOGGING_HEADER = "Result,y0,y1,y2,Dista,TopR,TopG,TopB,TopDist,MidR,MidG,MidB,MidDist,BottomR,BottomG,BottomB,BottomDist";

    private int mLastInference = RingDetectorNeuralNetwork.UNKNOWN;

    private FileWriter mLogWriter = null;

    public RingDetectorNeuralNetwork(InputStream is) throws Exception {
        super(is);
    }

    public void setLogStream(FileWriter fw){
        mLogWriter = fw;
        // Initialize stream with row header
        try{
            mLogWriter.write(LOGGING_HEADER);
            mLogWriter.write("\n");
        }
        catch(IOException e){

        }
    }

    static class RingSensorData {
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
     * Performs a measurement with another sample of data.
     * @param measurementData vector of new measurement data
     * @returns NO_RING, ONE_RING, or FOUR_RINGS, or UNKNOWN
     */
    public int doInference(RingSensorData measurementData) {
        SimpleMatrix x = new SimpleMatrix(INPUT_ROWS,1);
        x.set(DISTANCE_ROW_INDEX,measurementData.distanceSensorMM);
        x.set(TOP_RED_ROW_INDEX,measurementData.topColorRed);
        x.set(TOP_BLUE_ROW_INDEX,measurementData.topColorBlue);
        x.set(TOP_GREEN_ROW_INDEX,measurementData.topColorGreen);
        x.set(TOP_DISTANCE_ROW_INDEX,measurementData.topDistanceMM);
        x.set(MID_RED_ROW_INDEX,measurementData.midColorRed);
        x.set(MID_BLUE_ROW_INDEX,measurementData.midColorBlue);
        x.set(MID_GREEN_ROW_INDEX,measurementData.midColorGreen);
        x.set(MID_DISTANCE_ROW_INDEX,measurementData.midDistanceMM);
        x.set(BOTTOM_RED_ROW_INDEX,measurementData.bottomColorRed);
        x.set(BOTTOM_BLUE_ROW_INDEX,measurementData.bottomColorBlue);
        x.set(BOTTOM_GREEN_ROW_INDEX,measurementData.bottomColorGreen);
        x.set(BOTTOM_DISTANCE_ROW_INDEX,measurementData.bottomDistanceMM);

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
            for(int i=0;i < INPUT_ROWS;i++){
                mLogWriter.write(String.format("%1.5f",x.get(i,0)));
                if (i < INPUT_ROWS-1){
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
