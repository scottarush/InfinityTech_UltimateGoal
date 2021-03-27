package org.firstinspires.ftc.teamcode.ringdetect;

import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.media.Image;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.util.ImageUtils;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;

/**
 * This class extends JavaNeuralNetwork for the RingDetector .
 */
public class RingDetectorNeuralNetwork extends NeuralNetwork {

    // Detection value and output node index for single ring
    public static final int LABEL_NO_RING = 0;
    // Detection value and output node index for one ring
    public static final int LABEL_ONE_RING = 1;
    // Detection value and output nodex index for four rings
    public static final int LABEL_FOUR_RINGS = 2;

    // Label string for no ring
    public static final String LABEL_STRING_NO_RING = "NoRing";
    // Label string for no ring
    public static final String LABEL_STRING_ONE_RING = "OneRing";
    // Label string for no ring
    public static final String LABEL_STRING_FOUR_RINGS = "FourRings";

    // Detection value for unknown
    public static final int UNKNOWN = -1;

    // Configuration for camera sensor
//    public static final int CAMERA_ONLY_IMAGE_HEIGHT = 72;
//    public static final int CAMERA_ONLY_IMAGE_WIDTH = 94;
    public static final int CAMERA_ONLY_IMAGE_HEIGHT = 30;
    public static final int CAMERA_ONLY_IMAGE_WIDTH = 40;
    // Number of input rows is height * width * 3 color channels
    public static final int CAMERA_ONLY_INPUT_ROWS = CAMERA_ONLY_IMAGE_HEIGHT*CAMERA_ONLY_IMAGE_WIDTH*3;
    public static final String CAMERA_ONLY_NEURAL_NETWORK_FILE = "camera_ringnn_"+CAMERA_ONLY_IMAGE_WIDTH+"x"+CAMERA_ONLY_IMAGE_HEIGHT+".bin";
    private static final String CAMERA_ONLY_LOGGING_HEADER = "#,Result,y0,y1,y2";

    private FileWriter mLogWriter = null;
    private File mLogFileDirectory = null;

    private boolean mLogImages = false;

    private int mInferenceNum = 10;
    /**
     * @param nnFilePath path to neural network files
     * @param logFile path to logging file or null if logging disable
     * @param logImages if true, then log the images at each inference - also clears the logging directory first
     * @throws Exception if neural network cannot be read from the file
     */
    public RingDetectorNeuralNetwork(File nnFilePath,File logFile,boolean logImages) throws Exception {
        mLogImages = logImages;
        File nnFile = new File(nnFilePath,getNeuralNetworkFilename());
        try {
            if (!nnFile.exists()){
                throw new RuntimeException("Cannot find ring detector neural network file:"+nnFile.getPath());
            }
            // Open the file and deserialize the network
            InputStream is = new FileInputStream(nnFile);
            deserializeNetwork(is);
            is.close();

            // Initialize the logging file.  create the directory if it doesn't exist
            if (logFile != null) {
                // Save directory for use in saving camera images

                mLogFileDirectory = logFile.getParentFile();
                if (!mLogFileDirectory.exists()){
                    mLogFileDirectory.mkdir();
                }
                else{
                    if (logImages) {
                        // Clear out the directory if logging images
                        File[] files = mLogFileDirectory.listFiles();
                        for (int i = 0; i < files.length; i++) {
                            files[i].delete();
                        }
                    }
                }
                initLogFile(logFile);
            }
        }
        catch (Exception e) {
            throw new Exception("Error reading neural network file:" + e.getMessage());
        }

    }

    /**
     * called to close the log file on shutdown
     */
    public void closeLogFile(){
        try{
            if (mLogWriter != null){
                mLogWriter.close();
            }
        }
        catch(IOException e){

        }
    }

    /**
     * Utility used by the RingNeuralNetworkTrainer returns a network topology array based on sensor config
     */
    public static int[] getNetworkNodeTopology() {
        int[] nodes = new int[]{RingDetectorNeuralNetwork.CAMERA_ONLY_INPUT_ROWS,
                        (int) Math.round(RingDetectorNeuralNetwork.CAMERA_ONLY_INPUT_ROWS * 0.04), 3};
        return nodes;
    }

    /**
     * Return the neural network filename.y
     * @return filename of the neural network file
     */
    public static String getNeuralNetworkFilename(){
        return CAMERA_ONLY_NEURAL_NETWORK_FILE;
    }
    private void initLogFile(File logFile) throws IOException{
        if (logFile.exists()) {
            logFile.delete();
        }
        mLogWriter = new FileWriter(logFile);
        String header = CAMERA_ONLY_LOGGING_HEADER;
        mLogWriter.write(header);
        mLogWriter.write("\n");
    }









    /**
     * Performs a measurement for the CAMERA configuration.
     * @param cameraFrame bitmap from the camera
     * @returns NO_RING, ONE_RING, or FOUR_RINGS, or UNKNOWN
     */
    public int doInference(Bitmap cameraFrame) {
        // Scale the image down to the network resolutino
        int height = RingDetectorNeuralNetwork.CAMERA_ONLY_IMAGE_HEIGHT;
        int width = RingDetectorNeuralNetwork.CAMERA_ONLY_IMAGE_WIDTH;

        Bitmap inputFrame = Bitmap.createScaledBitmap(cameraFrame,width,height,false);

        // Now parse each line of resized pixels into the matrix scanning left->right and top->bottom
        int[] pixel = new int[3];
        SimpleMatrix input = new SimpleMatrix(CAMERA_ONLY_INPUT_ROWS,1);
        for(int py=0;py < inputFrame.getHeight();py++){
            for(int px=0;px < inputFrame.getWidth();px++){
                int startRow = 3*(py*inputFrame.getWidth()+px);
                int apixel = inputFrame.getPixel(px,py);
                // Extract channels and load in RGB order down the column vector
                int blue = apixel & 0x000000FF;
                apixel >>= 8;
                int green = apixel & 0x000000FF;
                apixel >>= 8;
                int red = apixel & 0x000000FF;

                input.set(startRow++,0,red);
                input.set(startRow++,0,green);
                input.set(startRow++,0,blue);
            }
        }
        // do the inference
        Bitmap loggedFrame = null;
        if (mLogImages){
            loggedFrame = inputFrame;
        }
        int result = doInference(input,loggedFrame);
        // And recycle the input frame
        inputFrame.recycle();
        return result;
    }

    /**
     * Internal method for inference
     * @param x input vector that must be in the format for the current configuration
     * @param inputBitmap  if non-null then the Bitmap used as input
     * @returns NO_RING, ONE_RING, or FOUR_RINGS, or UNKNOWN
     */
    private int doInference(SimpleMatrix x,Bitmap inputBitmap) {
        SimpleMatrix y = feedForward(x);
        int inference = decodeOutput(y);

        logInference(x, y, inference,inputBitmap);

        return inference;
    }

    private void logInference(SimpleMatrix x, SimpleMatrix y,int inference,Bitmap inputBitmap){
        if (mLogWriter == null)
            return;  // logging disabled
        try{
            mLogWriter.write(""+mInferenceNum +",");
            mLogWriter.write(RingDetectorNeuralNetwork.convertResultToString(inference));
            mLogWriter.write(",");
            for(int i=0;i < y.numRows();i++){
                mLogWriter.write(String.format("%1.5f",y.get(i,0)));
                mLogWriter.write(",");
            }
            int numRows = 0;

            // Write the inference count
            for(int i = 0; i < numRows; i++){
                mLogWriter.write(String.format("%1.5f",x.get(i,0)));
                if (i < numRows -1){
                    mLogWriter.write(",");
                }
            }
            mLogWriter.write("\n");

            // If the Bitmap is non-null then save it too
            if (inputBitmap != null){
                String filename = "NoRings_"+mInferenceNum+".png";
                File file = new File(mLogFileDirectory,filename);
                ImageUtils.savePNG(inputBitmap,file);
            }
            mInferenceNum++;
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

    public static String convertResultToString(int result) {
        switch (result) {
            case LABEL_NO_RING:
                return LABEL_STRING_NO_RING;
            case LABEL_ONE_RING:
                return LABEL_STRING_ONE_RING;
            case LABEL_FOUR_RINGS:
                return LABEL_STRING_FOUR_RINGS;
            default:
                return "Unknown";
        }
    }

}
