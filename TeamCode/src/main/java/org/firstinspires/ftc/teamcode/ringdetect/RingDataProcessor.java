package org.firstinspires.ftc.teamcode.ringdetect;

import org.ejml.simple.SimpleMatrix;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.StringTokenizer;

/**
 * Utility class imports ring neural network training data and prepares the data
 * in a format suitable for training.
 */
public class RingDataProcessor {

    // values for y output node column indexes
    public static final int NO_RING_INDEX = 0;
    public static final int ONE_RING_INDEX = 1;
    public static final int FOUR_RINGS_INDEX = 2;

    // number of states to recognize
    public static final int NUM_OUTPUT_STATES = 3;

    private int mNetworkConfiguration = -1;
    /**
     * Constructor
     * @param networkConfiguration configs defined in {@link RingDetectorNeuralNetwork}
     */
    public RingDataProcessor(int networkConfiguration) {
        mNetworkConfiguration = networkConfiguration;
    }

    private SimpleMatrix mXTrainingData = null;
    private SimpleMatrix mYTrainingData = null;

    private SimpleMatrix mXTestData = null;
    private SimpleMatrix mYTestData= null;

    private SimpleMatrix mScaleFactors = null;

    public SimpleMatrix getXTrainingData(){
        return mXTrainingData;
    }
    public SimpleMatrix getYTrainingData(){
        return mYTrainingData;
    }
    public SimpleMatrix getXTestData(){
        return mXTestData;
    }
    public SimpleMatrix getYTestData(){
        return mYTestData;
    }
    public SimpleMatrix getScaleFactors(){return mScaleFactors; }

   private File mInputFile = null;
    /**
     * Reads the test data from the input file
     * @param testFraction
     * @param normalize
     */
    public void processData(File inputFile,double testFraction,boolean shuffle,boolean normalize) {
        mInputFile = inputFile;
        if (!mInputFile.exists()) {
            System.out.println("Cannot find input file.");
            return;
        }
        BufferedReader reader = null;
        try {
            FileReader fr = new FileReader(mInputFile);
            reader = new BufferedReader(fr);

            // First count the number of training samples
            int count = 1;
             String line = reader.readLine();  // throw away header line
            line = reader.readLine();
            while(line != null){
                count++;
                line=reader.readLine();
            }
            reader.close();
            fr.close();

            // Now create X and Y arrays and re-read data into the arrays
            switch(mNetworkConfiguration){
                case RingDetectorNeuralNetwork.ALL_SENSORS:
                    mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.ALL_SENSORS_INPUT_ROWS,count);
                    break;
                case RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR:
                    mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_INPUT_ROWS,count);
                    break;
                case RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR:
                    mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_INPUT_ROWS,count);
                    break;
                case RingDetectorNeuralNetwork.TOP_BOTTOM_COLOR_SENSORS_ONLY:
                    mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_INPUT_ROWS,count);
                    break;
            }
            // Y array is always the same
            mYTrainingData = new SimpleMatrix(NUM_OUTPUT_STATES,count);

            fr = new FileReader(mInputFile);
            reader = new BufferedReader(fr);
            int colIndex = 0;
            line = reader.readLine();  // throw away header line
            line = reader.readLine();
            while (line != null) {
                parseLine(line,mXTrainingData,mYTrainingData,colIndex++);
                line = reader.readLine();
            }
            reader.close();
            fr.close();
         } catch (IOException e) {
            e.printStackTrace();
            return;
        }

        if (normalize) {
            // Normalize the data.  Must save the scale Factors to be used for runtime operation
            mScaleFactors = NeuralNetworkMatrixUtils.normalizeRows(mXTrainingData);
            NeuralNetworkMatrixUtils.normalizeRows(mYTrainingData);
        }
        else{
            mScaleFactors = new SimpleMatrix(mXTrainingData.numRows(),1);
            mScaleFactors.fill(1.0d);
        }

        // Shuffle the data if requested to randomize the split between training and test
        if (shuffle) {
            int columns[] = NeuralNetworkMatrixUtils.genShuffleColumnIndexVector(mXTrainingData.numCols());
            mXTrainingData = NeuralNetworkMatrixUtils.shuffleMatrix(mXTrainingData, columns);
            mYTrainingData = NeuralNetworkMatrixUtils.shuffleMatrix(mYTrainingData, columns);
        }
        // Compute the split between training and test data
        int testCount = (int)Math.round((double)mXTrainingData.numCols()*testFraction);
        int testColumnStart = mXTrainingData.numCols()-testCount;
        //  split off test data starting testColumnStart
        mXTestData = mXTrainingData.cols(testColumnStart,mXTrainingData.numCols());
        mXTrainingData = mXTrainingData.cols(0,testColumnStart);

        // And split off the test data starting at the testColumnStart
        mYTestData = mYTrainingData.cols(testColumnStart,mYTrainingData.numCols());
        mYTrainingData = mYTrainingData.cols(0,testColumnStart);

 //       System.out.println(MatrixUtils.printMatrix(mXTrainingData.getDDRM(),"xtrain"));
 //       System.out.println(MatrixUtils.printMatrix(mYTrainingData.getDDRM(),"ytrain"));


    }

    private void parseLine(String line,SimpleMatrix x,SimpleMatrix y,int columnIndex) {
        StringTokenizer tokenizer = new StringTokenizer(line, ",");
        tokenizer.nextToken();  // Skip record number
        String tag = tokenizer.nextToken();

        // Set the y data by first clearing the columm
        for(int i=0;i < NUM_OUTPUT_STATES;i++){
            y.set(i,columnIndex,0d);
        }
        // And then set the ground truth one
        switch (tag) {
            case RingNeuralNetworkDataCollectionOpMode.NO_RING_TAG:
                y.set(NO_RING_INDEX,columnIndex, 1d);
                break;
            case RingNeuralNetworkDataCollectionOpMode.ONE_RING_TAG:
                y.set(ONE_RING_INDEX,columnIndex,1d);
                break;
            case RingNeuralNetworkDataCollectionOpMode.FOUR_RING_TAG:
                y.set(FOUR_RINGS_INDEX,columnIndex, 1d);
                break;
            default:
                throw new RuntimeException("Invalid tag" + tag + " found in input file=" + mInputFile.getName());
        }
        tokenizer.nextToken();  // Skip the light status
        // Now create the x array according to the configuration
        switch(mNetworkConfiguration){
            case RingDetectorNeuralNetwork.ALL_SENSORS:
                parseAllSensorConfigLine(x,columnIndex,tokenizer);
                break;
            case RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR:
                parseNoMidSensorConfigLine(x,columnIndex,tokenizer);
                break;
            case RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR:
                parseNoDistanceSensorConfigLine(x,columnIndex,tokenizer);
                break;
            case RingDetectorNeuralNetwork.TOP_BOTTOM_COLOR_SENSORS_ONLY:
                parseTopBottomOnlyConfigLine(x,columnIndex,tokenizer);
                break;
        }
     }

    private void parseAllSensorConfigLine(SimpleMatrix x,int columnIndex,StringTokenizer tokenizer){
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_TOP_RED_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_TOP_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_TOP_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_TOP_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_MID_RED_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_MID_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_MID_GREEN_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_MID_DISTANCE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_BOTTOM_RED_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_BOTTOM_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_BOTTOM_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.ALL_SENSORS_BOTTOM_DISTANCE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
    }
    private void parseNoMidSensorConfigLine(SimpleMatrix x,int columnIndex,StringTokenizer tokenizer){
        x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_TOP_RED_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_TOP_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_TOP_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_TOP_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        // Skip Mid values
        tokenizer.nextToken();
        tokenizer.nextToken();
        tokenizer.nextToken();
        tokenizer.nextToken();
         x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_BOTTOM_RED_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_BOTTOM_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_BOTTOM_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_BOTTOM_DISTANCE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
    }
    private void parseNoDistanceSensorConfigLine(SimpleMatrix x,int columnIndex,StringTokenizer tokenizer){
        tokenizer.nextToken();  // skip distance
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_TOP_RED_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_TOP_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_TOP_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_TOP_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_MID_RED_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_MID_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_MID_GREEN_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_MID_DISTANCE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_BOTTOM_RED_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_BOTTOM_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_BOTTOM_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_BOTTOM_DISTANCE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
    }
    private void parseTopBottomOnlyConfigLine(SimpleMatrix x,int columnIndex,StringTokenizer tokenizer){
        tokenizer.nextToken();  // skip distance
        x.set(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_TOP_RED_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_TOP_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_TOP_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_TOP_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        // Skip Mid values
        tokenizer.nextToken();
        tokenizer.nextToken();
        tokenizer.nextToken();
        tokenizer.nextToken();
        x.set(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_BOTTOM_RED_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_BOTTOM_BLUE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_BOTTOM_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_BOTTOM_DISTANCE_ROW_INDEX, columnIndex,Float.parseFloat(tokenizer.nextToken()));
    }


}
