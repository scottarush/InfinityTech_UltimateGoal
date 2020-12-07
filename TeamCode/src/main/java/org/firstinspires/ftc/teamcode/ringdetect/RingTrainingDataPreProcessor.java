package org.firstinspires.ftc.teamcode.ringdetect;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.StringTokenizer;

/**
 * Utility class imports ring neural network training data and prepares the data
 * in a format suitable for training.
 */
public class RingTrainingDataPreProcessor {

    private String mInputFilename = null;
    public static final int INT_CONVERSION_SCALE_FACTOR = 10000;

    public RingTrainingDataPreProcessor(String inputFilename) {
        mInputFilename = inputFilename;
    }

    ArrayList<int[]> mData = new ArrayList<>();

    /**
     * Returns the preprocessed data
     */
    public ArrayList<int[]> getProcessedData(){
        return mData;
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

             String line = reader.readLine();  // throw away header line
            line = reader.readLine();
            while (line != null) {
                int[] retArray = parseLine(line);
                line = reader.readLine();
                mData.add(retArray);
            }
            reader.close();
         } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        // Now normalize the data
        normalize();
    }

    private int[] parseLine(String line) {
        int[] retArray = new int[RingDetectorNeuralNetwork.DATA_DIMENSION];
        StringTokenizer tokenizer = new StringTokenizer(line, ",");
        tokenizer.nextToken();  // Skip record number
        String tag = tokenizer.nextToken();
        switch (tag) {
            case RingNeuralNetworkDataCollectionOpMode.NO_RING_TAG:
                retArray[RingDetectorNeuralNetwork.TAG_INDEX] = RingDetectorNeuralNetwork.NO_RING;
                break;
            case RingNeuralNetworkDataCollectionOpMode.ONE_RING_TAG:
                retArray[RingDetectorNeuralNetwork.TAG_INDEX] = RingDetectorNeuralNetwork.ONE_RING;
                break;
            case RingNeuralNetworkDataCollectionOpMode.FOUR_RING_TAG:
                retArray[RingDetectorNeuralNetwork.TAG_INDEX] = RingDetectorNeuralNetwork.FOUR_RINGS;
                break;
            default:
                throw new RuntimeException("Invalid tag" + tag + " found in input file=" + mInputFilename);
        }
        tokenizer.nextToken();  // Skip the light status
        retArray[RingDetectorNeuralNetwork.DISTANCE_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.TOP_RED_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.TOP_GREEN_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.TOP_BLUE_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.TOP_DISTANCE_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.MID_RED_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.MID_GREEN_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.MID_BLUE_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.MID_DISTANCE_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.BOTTOM_RED_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.BOTTOM_GREEN_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.BOTTOM_BLUE_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);
        retArray[RingDetectorNeuralNetwork.BOTTOM_DISTANCE_INDEX] = Math.round(Float.parseFloat(tokenizer.nextToken()) * INT_CONVERSION_SCALE_FACTOR);

        return retArray;
    }

    private void normalize() {
        int[] maxArray = new int[RingDetectorNeuralNetwork.DATA_DIMENSION];
        for(int i=0;i < maxArray.length;i++){
            maxArray[i] = 0;
        }
        // Compute the maxes
        for (Iterator<int[]> iter = mData.iterator(); iter.hasNext(); ) {
            int[] sample = iter.next();
            for(int i = 1; i < RingDetectorNeuralNetwork.DATA_DIMENSION; i++){
                if (sample[i] > maxArray[i]){
                    maxArray[i] = sample[i];
                }
            }
        }
        // Compute the scale factors to scale to 16 bit data = 65535 max
        float[] scaleFactors = new float[RingDetectorNeuralNetwork.DATA_DIMENSION];
        float max = 65535f;
        for(int i = 1; i < RingDetectorNeuralNetwork.DATA_DIMENSION; i++){
            scaleFactors[i] = max / (float)maxArray[i];
        }
        // Now normalize to using the scaleFactors
        for (Iterator<int[]> iter = mData.iterator(); iter.hasNext(); ) {
            int[] sample = iter.next();

            for(int i = 1; i < RingDetectorNeuralNetwork.DATA_DIMENSION; i++){
                sample[i] = Math.round((float)sample[i] * scaleFactors[i]);
            }
        }
    }

}
