package org.firstinspires.ftc.teamcode.ringdetect;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Test class created to verify the neural network code
 */
public class NetworkVerifier {

    SimpleMatrix mX;
    SimpleMatrix mY;
    SimpleMatrix mScale;
    static final int NUM_INPUTS = 3;
    static final int NUM_OUTPUTS = 3;

    public void genData(){
        SimpleMatrix inputVectors[] = new SimpleMatrix[NUM_INPUTS];
        SimpleMatrix outputVectors[] = new SimpleMatrix[NUM_OUTPUTS];
        for(int i=0;i < inputVectors.length;i++){
            inputVectors[i] = new SimpleMatrix(inputVectors.length,1);
            inputVectors[i].set(i,0,1.0d);
            int opposite = outputVectors.length-i-1;
            outputVectors[i] = new SimpleMatrix(outputVectors.length,1);
            outputVectors[i].set(opposite,0,1.0d);
        }
        int num_samples = 1000;
         for(int col=0;col < num_samples;col++){
             int i = col % 3;
             if (mX == null){
                 mX = new SimpleMatrix(inputVectors[i]);
                 mY = new SimpleMatrix(outputVectors[i]);
             }
             else{
                 mX = mX.concatColumns(inputVectors[i]);
                 mY = mY.concatColumns(outputVectors[i]);
             }
         }
    }
     /**
     * Called to train the network
     */
    public void
    trainNetwork(File networkFile,File logFile){
        // Now create and pass training data to neural network
        NeuralNetwork ringnn = new NeuralNetwork(new int[]{3,5,3},NeuralNetwork.QUADRATIC_COST);

        final StringBuffer logBuffer = new StringBuffer();
        logBuffer.append("Epoch#,Normal Error\n");

        ringnn.addTrainingStatusListener(new NeuralNetwork.ITrainingStatusListener() {
            @Override
            public void trainingStatus(int epochNumber, double cost,double[] del_L,double normalError) {
                logBuffer.append(epochNumber+","+ normalError +"\n");
            }
        });

        genData();
        ringnn.train("NetworkVerifier",mX,mY,mScale, 10,0.1d,1.0d,5000);
         // Write out the network
        try {
            if (networkFile.exists()){
                networkFile.delete();
            }
            FileOutputStream fos = new FileOutputStream(networkFile);
            ringnn.serializeNetwork(fos);
        }
        catch(Exception e){
            e.printStackTrace();
        }
        // Save the training log
        try{
            if (logFile.exists()){
                logFile.delete();
            }
            FileWriter fw = new FileWriter(logFile);
            fw.write(logBuffer.toString());
            fw.close();
        }
        catch(IOException e){
            e.printStackTrace();
        }

    }

    public void testNetwork(File nnFilePath,File logFile,double testFraction){
       // Read the network
        RingDetectorNeuralNetwork ringnn=null;
        try {
            ringnn = new RingDetectorNeuralNetwork(nnFilePath,RingDetectorNeuralNetwork.ALL_SENSORS,logFile);
        }
        catch(Exception e){
            e.printStackTrace();
            return;
        }

        // Init the log buffer
        final StringBuffer logBuffer = new StringBuffer();
        logBuffer.append("Data#,Truth,yTruth,Detection,yDetection\n");

        genData();

        // And loop through the test data
        for(int column=0;column < mX.numCols();column++){
            SimpleMatrix input = mX.extractVector(false,column);
            SimpleMatrix ytruth = mX.extractVector(false,column);

            SimpleMatrix output = ringnn.doTestInference(input);

            String ytruths = NeuralNetworkMatrixUtils.printColumn(ytruth,0);
            String ydetections = NeuralNetworkMatrixUtils.printColumn(output,0);
            logBuffer.append(column+1+","+ytruths+","+ydetections+"\n");
        }

        // Save the test log
        try{
            if (logFile.exists()){
                logFile.delete();
            }
            FileWriter fw = new FileWriter(logFile);
            fw.write(logBuffer.toString());
            fw.close();
        }
        catch(IOException e){
            e.printStackTrace();
        }

    }

    @Test
    public void main() {
        NetworkVerifier trainer = new NetworkVerifier();

        Path currentRelativePath = Paths.get("");
        String trainingDataPath = currentRelativePath.toAbsolutePath().toString() + "/src/test/java/org/firstinspires/ftc/teamcode/ringdetect";
        File nnPath = new File(trainingDataPath);
        boolean train = true;
        if (train) {
            // Now save the network to the runtime assets directory
            File logFile = new File(trainingDataPath + "/top_bottom_ringnn_training_log.csv");
            trainer.trainNetwork(nnPath,logFile);
        }
        boolean test = true;
        if (test){
            File logFile = new File(trainingDataPath + "/top_bottom_testing_log.csv");
            testNetwork(nnPath,logFile,0.10);
        }

    }
}