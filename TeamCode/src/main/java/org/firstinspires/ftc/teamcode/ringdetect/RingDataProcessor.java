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

    public static final int TRAINING_DATA_ROWS = 13;
    // Columns in the X array training data which are also the columns in the input vector
    // at runtime
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

    // number of states to recognize
    public static final int NUM_OUTPUT_STATES = 3;

    public RingDataProcessor() {
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
    public void processData(File inputFile,double testFraction,boolean normalize) {
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
            mXTrainingData = new SimpleMatrix(TRAINING_DATA_ROWS,count);
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
            mScaleFactors = MatrixUtils.normalizeRows(mXTrainingData);
            MatrixUtils.normalizeRows(mYTrainingData);
        }
        else{
            mScaleFactors = new SimpleMatrix(mXTrainingData.numRows(),1);
            mScaleFactors.fill(1.0d);
        }

        // Shuffle the data to randomize the split between training and test
        int columns[] = MatrixUtils.genShuffleColumnIndexVector(mXTrainingData.numCols());
        mXTrainingData = MatrixUtils.shuffleMatrix(mXTrainingData,columns);
        mYTrainingData = MatrixUtils.shuffleMatrix(mYTrainingData,columns);

        // Compute the split between training and test data
        int trainingCount = (int)Math.round((double)mXTrainingData.numCols()*testFraction);
        int testColumnStart = mXTrainingData.numCols()-trainingCount;
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
        x.set(DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(TOP_RED_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(TOP_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(TOP_BLUE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(TOP_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(MID_RED_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(MID_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(MID_BLUE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(MID_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(BOTTOM_RED_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(BOTTOM_GREEN_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(BOTTOM_BLUE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
        x.set(BOTTOM_DISTANCE_ROW_INDEX,columnIndex,Float.parseFloat(tokenizer.nextToken()));
    }

}
