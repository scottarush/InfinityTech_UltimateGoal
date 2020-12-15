package org.firstinspires.ftc.teamcode.ringdetect;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Test class to train the ring neural network.
 */
public class RingNeuralNetworkTrainer {

    RingDataProcessor mDataProcessor = null;

    /**
     * Reads data into the pre-processor for both training and testing.
     */
    public void readData(File trainingDataFile,double testFraction,boolean normalize){
        mDataProcessor.processData(trainingDataFile,testFraction,normalize);
    }
    /**
     * Called to train the network
     */
    public void trainNewNetwork(String description, File trainingDataFile, File neuralNetworkFilePath,int networkConfig, File logFile){
        mDataProcessor = new RingDataProcessor(networkConfig);
        File networkFile = new File(neuralNetworkFilePath,RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig));

        readData(trainingDataFile,0.10,true);

        // Now create the node structure according to the configuration
        int[] nodes = RingDetectorNeuralNetwork.getNetworkNodes(networkConfig);

        NeuralNetwork ringnn = new NeuralNetwork(nodes);

        final StringBuffer logBuffer = new StringBuffer();
        logBuffer.append("Epoch#,Normal Error\n");

        ringnn.addTrainingStatusListener(new NeuralNetwork.ITrainingStatusListener() {
            @Override
            public void trainingStatus(int epochNumber, double normalError) {
                logBuffer.append(epochNumber+","+ normalError +"\n");
            }
        });

        ringnn.train(description,mDataProcessor.getXTrainingData(), mDataProcessor.getYTrainingData(),
                mDataProcessor.getScaleFactors(), 10,0.1d,50000,false);
         // Write out the network
        try {
            if (networkFile.exists()){
                networkFile.delete();
            }
            FileOutputStream fos = new FileOutputStream(networkFile);
            ringnn.serializeNetwork(fos);
            fos.close();
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

    public void testNetwork(File trainingDataFile,File neuralNetworkFilePath,int networkConfig,File logFile,double testFraction){
       // Read the network
        RingDetectorNeuralNetwork ringnn=null;
        try {
            ringnn = new RingDetectorNeuralNetwork(neuralNetworkFilePath,networkConfig,logFile);
        }
        catch(Exception e){
            e.printStackTrace();
            return;
        }

        // Init the log buffer
        final StringBuffer logBuffer = new StringBuffer();
        logBuffer.append("Data#,Truth,yTruth,Detection,yDetection\n");

        // Read the test data but do not scale it
        readData(trainingDataFile,testFraction,false);
        SimpleMatrix x = mDataProcessor.getXTestData();
        SimpleMatrix y = mDataProcessor.getYTestData();

        // And loop through the test data
        int pass = 0;
        int fail = 0;
        for(int column=0;column < x.numCols();column++){
            SimpleMatrix input = x.extractVector(false,column);
            SimpleMatrix ytruth = y.extractVector(false,column);

            SimpleMatrix output = ringnn.doTestInference(input);
            int truth = ringnn.decodeOutput(ytruth);
            int inference = ringnn.decodeOutput(output);

            if (truth != inference){
                fail++;
                logBuffer.append("Fail:");
            }
            else {
                pass++;
                logBuffer.append("Pass:");

            }

            String ytruths = MatrixUtils.printColumn(ytruth,0);
            String truths = ringnn.convertToString(truth);

            String yinference = MatrixUtils.printColumn(output,0);
            String inferences = ringnn.convertToString(inference);
            logBuffer.append(","+truths+","+ytruths+","+inferences+","+yinference+"\n");
        }
        double percent = 100d * (double)pass/(double)x.numCols();
        logBuffer.append("Summary\n");
        logBuffer.append(String.format("%2.1f",percent)+".  Pass="+pass+".  Fail="+fail+". of "+x.numCols()+" test samples.\n");
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

    /**
     * The main method in this class configures the
     */
    @Test
    public void main() {
        RingNeuralNetworkTrainer trainer = new RingNeuralNetworkTrainer();

        Path currentRelativePath = Paths.get("");
        String trainingDataPath = currentRelativePath.toAbsolutePath().toString() + "/src/test/java/org/firstinspires/ftc/teamcode/ringdetect";
        File trainingFile = new File(trainingDataPath, "13DEC20_training_data.csv");
        File neuralNetFilePath = new File(trainingDataPath);

        boolean train = true;
        int networkConfig = RingDetectorNeuralNetwork.ALL_SENSORS;
        // Form training and testing log filenames using the configuration filename as a prefix
        String trainingLogFilename = RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig);
        trainingLogFilename = trainingLogFilename.substring(0,trainingLogFilename.length()-4);      // Strip .bin extension
        String testingLogFilename = trainingLogFilename+"_testing_log.cvs";
        trainingLogFilename= trainingLogFilename+"_training_log.cvs";
        if (train) {
            File logFile = new File(trainingDataPath,trainingLogFilename);
            trainer.trainNewNetwork(RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig),
                    trainingFile,neuralNetFilePath,networkConfig,logFile);
        }
        boolean test = true;
        if (test){
            File logFile = new File(trainingDataPath,testingLogFilename);
            trainer.testNetwork(trainingFile,neuralNetFilePath,networkConfig,logFile,0.10);
        }

    }
}