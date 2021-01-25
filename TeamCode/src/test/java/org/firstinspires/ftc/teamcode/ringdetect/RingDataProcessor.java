package org.firstinspires.ftc.teamcode.ringdetect;

import org.ejml.simple.SimpleMatrix;

import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileFilter;
import java.io.FileReader;
import java.io.IOException;
import java.util.StringTokenizer;

import javax.imageio.ImageIO;

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

   private File mInputLocation = null;

    /**
     * Reads the test data from the input file for the color and distance sensor configurations
     * @param input either a reference to a File for color and distance or the direction where the
     *              color sensor data is.
     * @param testFraction
     * @param normalize
     */
    public void processData(File input,double testFraction,boolean shuffle,boolean normalize) {
        if (mNetworkConfiguration == RingDetectorNeuralNetwork.CONFIGURATION_CAMERA_ONLY){
            processCameraData(input, testFraction, shuffle, normalize);
        }
        else{
            processColorDistanceData(input,testFraction,shuffle,normalize);
        }
    }

        /**
         * processing function for color and distance sensor configurations
         * @param testFraction
         * @param normalize
         */
    private void processColorDistanceData(File input,double testFraction,boolean shuffle,boolean normalize) {
        mInputLocation = input;
        if (!mInputLocation.exists()) {
            System.out.println("Cannot find input file.");
            return;
        }
        BufferedReader reader = null;
        try {
            FileReader fr = new FileReader(mInputLocation);
            reader = new BufferedReader(fr);

            // First count the number of training samples
            int count = 0;
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
                case RingDetectorNeuralNetwork.CONFIGURATION_ALL_SENSORS:
                    mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.ALL_SENSORS_INPUT_ROWS,count);
                    break;
                case RingDetectorNeuralNetwork.CONFIGURATION_NO_MID_COLOR_SENSOR:
                    mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR_INPUT_ROWS,count);
                    break;
                case RingDetectorNeuralNetwork.CONFIGURATION_NO_DISTANCE_SENSOR:
                    mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR_INPUT_ROWS,count);
                    break;
                case RingDetectorNeuralNetwork.CONFIGURATION_TOP_BOTTOM_COLOR_SENSORS_ONLY:
                    mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.TOP_BOTTOM_ONLY_INPUT_ROWS,count);
                case RingDetectorNeuralNetwork.CONFIGURATION_CAMERA_ONLY:
                    throw new RuntimeException("Must call processCamerData function for CONFIGURATION_CAMERA_ONLY");
            }
            // Y array is always the same
            mYTrainingData = new SimpleMatrix(NUM_OUTPUT_STATES,count);

            fr = new FileReader(mInputLocation);
            reader = new BufferedReader(fr);
            int colIndex = 0;
            line = reader.readLine();  // throw away header line
            line = reader.readLine();
            while (line != null) {
                parseColorDistanceFileLine(line,mXTrainingData,mYTrainingData,colIndex++);
                line = reader.readLine();
            }
            reader.close();
            fr.close();
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        // Now finish shuffling, normalizing ,and split the testFraction
        finishDataProcessing(shuffle, normalize, testFraction);

    }
    /**
     * processing function for camera data configurations
     * @param testFraction
     * @param normalize
     */
    private void processCameraData(File input,double testFraction,boolean shuffle,boolean normalize) {
        mInputLocation = input;
        if (!mInputLocation.exists()) {
            System.out.println("Cannot find input directory.");
            return;
        }

        // Get a list of files in the input directory.  Must only be PNGs
        File[] inputFiles = mInputLocation.listFiles(new FileFilter() {
            @Override
            public boolean accept(File pathname) {
                String filename = pathname.getName();
                int index = filename.lastIndexOf('.');
                String extension = filename.substring(index+1,filename.length());
                if (extension.compareTo("png") == 0){
                    return true;
                }
                else{
                    return false;
                }
            }
        });

        // Create the TrainingData matrices
        mXTrainingData = new SimpleMatrix(RingDetectorNeuralNetwork.CAMERA_ONLY_INPUT_ROWS,inputFiles.length);
        mYTrainingData = new SimpleMatrix(NUM_OUTPUT_STATES,inputFiles.length);

        for(int column = 0; column < inputFiles.length; column++){
            // parse each PNG file into a column of the training data matrices
            parsePNGFile(inputFiles[column],mXTrainingData,mYTrainingData,column);
        }

        // Now finish shuffling, normalizing ,and split the testFraction
        finishDataProcessing(shuffle, normalize, testFraction);
    }

    /**
     * Parses data from a PNG file into the matrices
     * @param pngFile
     * @param x
     * @param y
     * @param columnIndex
     */
    private void parsePNGFile(File pngFile, SimpleMatrix x, SimpleMatrix y, int columnIndex) {
        // Open the pngFile and parse it byte by byte using the Swing ImageIO library.
        // Note that this library is not supported on Android, but this operation doesn't run
        // on the target so we can use it in this test class
        try {
            BufferedImage image = ImageIO.read(pngFile);
            // Now resize the image to the dimensions needed
            int height = RingDetectorNeuralNetwork.CAMERA_ONLY_IMAGE_HEIGHT;
            int width = RingDetectorNeuralNetwork.CAMERA_ONLY_IMAGE_WIDTH;
            Image tmp = image.getScaledInstance(width,height, Image.SCALE_AREA_AVERAGING);

            BufferedImage resized = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
            Graphics2D g2d = resized.createGraphics();
            g2d.drawImage(tmp, 0, 0, null);
            g2d.dispose();

            // Now parse each line of resized pixels into the matrix scanning left->right and top->bottom
            int[] pixel = new int[3];
            for(int py=0;py < resized.getHeight();py++){
                for(int px=0;px < resized.getWidth();px++){
                    int startRow = 3*(py*resized.getWidth()+px);
                    resized.getData().getPixel(px,py,pixel);
                    for(int channel=0;channel < 3;channel++){
                        x.set(startRow+channel,columnIndex,pixel[channel]);
                    }
                }
            }
        }
        catch(IOException e){
            e.printStackTrace();
        }
        // Now set the Y output row value using the prefix of the filename to determine the
        // label
        String name = pngFile.getName();
        int index = name.lastIndexOf('_');
        String prefix = name.substring(0,index);
        int yRowIndex = 0;
        switch(prefix){
            case RingDetectorNeuralNetwork.LABEL_STRING_NO_RING:
                yRowIndex = RingDetectorNeuralNetwork.LABEL_NO_RING;
                break;
            case RingDetectorNeuralNetwork.LABEL_STRING_ONE_RING:
                yRowIndex = RingDetectorNeuralNetwork.LABEL_ONE_RING;
                break;
            case RingDetectorNeuralNetwork.LABEL_STRING_FOUR_RINGS:
                yRowIndex = RingDetectorNeuralNetwork.LABEL_FOUR_RINGS;
                break;
        }
        for(int i=0;i < 3;i++){
            if (i == yRowIndex) {
                y.set(i,columnIndex,1d);
            }
            else{
                y.set(i,columnIndex,0d);
            }
        }
    }

        /**
         * Parses a line from the color and distance sensor CSV input files into the matrices
         * @param line
         * @param x
         * @param y
         * @param columnIndex
         */
    private void parseColorDistanceFileLine(String line, SimpleMatrix x, SimpleMatrix y, int columnIndex) {
        StringTokenizer tokenizer = new StringTokenizer(line, ",");
        tokenizer.nextToken();  // Skip record number
        String tag = tokenizer.nextToken();

        // Set the y data by first clearing the columm
        for(int i=0;i < NUM_OUTPUT_STATES;i++){
            y.set(i,columnIndex,0d);
        }
        // And then set the ground truth one
        switch (tag) {
            case RingDetectorNeuralNetwork.LABEL_STRING_NO_RING:
                y.set(NO_RING_INDEX,columnIndex, 1d);
                break;
            case RingDetectorNeuralNetwork.LABEL_STRING_ONE_RING:
                y.set(ONE_RING_INDEX,columnIndex,1d);
                break;
            case RingDetectorNeuralNetwork.LABEL_STRING_FOUR_RINGS:
                y.set(FOUR_RINGS_INDEX,columnIndex, 1d);
                break;
            default:
                throw new RuntimeException("Invalid tag" + tag + " found in input file=" + mInputLocation.getName());
        }
        tokenizer.nextToken();  // Skip the light status
        // Now create the x array according to the configuration
        switch(mNetworkConfiguration){
            case RingDetectorNeuralNetwork.CONFIGURATION_ALL_SENSORS:
                parseAllSensorConfigLine(x,columnIndex,tokenizer);
                break;
            case RingDetectorNeuralNetwork.CONFIGURATION_NO_MID_COLOR_SENSOR:
                parseNoMidSensorConfigLine(x,columnIndex,tokenizer);
                break;
            case RingDetectorNeuralNetwork.CONFIGURATION_NO_DISTANCE_SENSOR:
                parseNoDistanceSensorConfigLine(x,columnIndex,tokenizer);
                break;
            case RingDetectorNeuralNetwork.CONFIGURATION_TOP_BOTTOM_COLOR_SENSORS_ONLY:
                parseTopBottomOnlyConfigLine(x,columnIndex,tokenizer);
                break;
        }
     }

    /**
     * normalizes and shuffles data in the processing matrices and splits into the testing
     * and training matrices
     * @param shuffle
     * @param normalize
     */
     private void finishDataProcessing(boolean shuffle, boolean normalize,double testFraction){
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
