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
public class RingTrainingDataPreProcessor {

    // number of states to recognize
    public static final int NUM_OUTPUT_STATES = 3;
    // values for y output node column indexes
    public static final int NO_RING_INDEX = 0;
    public static final int ONE_RING_INDEX = 1;
    public static final int FOUR_RINGS_INDEX = 2;

    // row indexes into the training array
    public static final int TRAINING_DATA_NUM_ROWS = 13;

    public static final int DISTANCE_ROW_INDEX = 0;
    public static final int TOP_RED_ROW_INDEX = 1;
    public static final int TOP_GREEN_ROW_INDEX = 2;
    public static final int TOP_BLUE_ROW_INDEX = 3;
    public static final int TOP_DISTANCE_ROW_INDEX = 4;
    public static final int MID_RED_ROW_INDEX = 5;
    public static final int MID_GREEN_ROW_INDEX = 6;
    public static final int MID_BLUE_ROW_INDEX = 7;
    public static final int MID_DISTANCE_ROW_INDEX = 8;
    public static final int BOTTOM_RED_ROW_INDEX = 9;
    public static final int BOTTOM_GREEN_ROW_INDEX = 10;
    public static final int BOTTOM_BLUE_ROW_INDEX = 11;
    public static final int BOTTOM_DISTANCE_ROW_INDEX = 12;

    private String mInputFilename = null;

    public RingTrainingDataPreProcessor(String inputFilename) {
        mInputFilename = inputFilename;
    }

    private SimpleMatrix mXTrainingData = null;
    private SimpleMatrix mYTrainingData = null;

    public SimpleMatrix getXTrainingData(){
        return mXTrainingData;
    }
    public SimpleMatrix getYTrainingData(){
        return mYTrainingData;
    }

    public void processData() {
        File inputFile = new File(mInputFilename);
        if (!inputFile.exists()) {
            System.out.println("Cannot find input file.");
            return;
        }
        BufferedReader reader = null;
        try {
            FileReader fr = new FileReader(inputFile);
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
            mXTrainingData = new SimpleMatrix(TRAINING_DATA_NUM_ROWS,count);
            mYTrainingData = new SimpleMatrix(NUM_OUTPUT_STATES,count);
            fr = new FileReader(inputFile);
            reader = new BufferedReader(fr);
            int colIndex = 0;
            line = reader.readLine();  // throw away header line
            line = reader.readLine();
            while (line != null) {
                parseLine(line,colIndex++);
                line = reader.readLine();
            }
            reader.close();
            fr.close();
         } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        // Now normalize the data
        normalize();
    }

    private void parseLine(String line,int columnIndex) {
        StringTokenizer tokenizer = new StringTokenizer(line, ",");
        tokenizer.nextToken();  // Skip record number
        String tag = tokenizer.nextToken();

        // Set the y data by first clearing the columm
        for(int i=0;i < NUM_OUTPUT_STATES;i++){
            mYTrainingData.set(i,columnIndex,0d);
        }
        // And then set the ground truth one
        switch (tag) {
            case RingNeuralNetworkDataCollectionOpMode.NO_RING_TAG:
                mYTrainingData.set(NO_RING_INDEX,columnIndex, 1d);
                break;
            case RingNeuralNetworkDataCollectionOpMode.ONE_RING_TAG:
                mYTrainingData.set(ONE_RING_INDEX,columnIndex,1d);
                break;
            case RingNeuralNetworkDataCollectionOpMode.FOUR_RING_TAG:
                mYTrainingData.set(FOUR_RINGS_INDEX,columnIndex, 1d);
                break;
            default:
                throw new RuntimeException("Invalid tag" + tag + " found in input file=" + mInputFilename);
        }
        tokenizer.nextToken();  // Skip the light status
        mXTrainingData.set(DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(TOP_RED_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(TOP_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(TOP_BLUE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(TOP_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(MID_RED_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(MID_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(MID_BLUE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(MID_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(BOTTOM_RED_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(BOTTOM_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(BOTTOM_BLUE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        mXTrainingData.set(BOTTOM_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
    }

    private void normalize() {
        double[] maxArray = new double[TRAINING_DATA_NUM_ROWS];
        for(int i=0;i < maxArray.length;i++){
            maxArray[i] = 0f;
        }
        // Compute the maxes
        for(int column = 0; column < mXTrainingData.numCols(); column++){
            for(int row = 0; row < mXTrainingData.numRows(); row++){
                if (mXTrainingData.get(row,column) > maxArray[row]){
                    maxArray[row] = mXTrainingData.get(row,column);
                }
            }
        }
        for(int column = 0; column < mXTrainingData.numCols(); column++){
            for(int row = 0; row < mXTrainingData.numRows(); row++){
                if (mXTrainingData.get(row,column) > maxArray[row]){
                    maxArray[row] = mXTrainingData.get(row,column);
                }
            }
        }
        // Compute the scale factors to normalize to 1.000
        double[] scaleFactors = new double[maxArray.length];
        double max = 1.0d;
        for(int i = 0; i < scaleFactors.length; i++){
            scaleFactors[i] = max / maxArray[i];
        }
        // Now normalize to using the scaleFactors
        for(int column = 0; column < mXTrainingData.numCols(); column++){
            for(int row = 0; row < mXTrainingData.numRows(); row++){
                double value = mXTrainingData.get(row,column) * scaleFactors[row];
                mXTrainingData.set(row,column,value);
            }
        }
    }

}
