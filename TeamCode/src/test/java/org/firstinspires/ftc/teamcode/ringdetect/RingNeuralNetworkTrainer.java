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
 * Test class to train the ring neural network.
 *
 * This class is only for the color and distance sensor neural network.
 */
public class RingNeuralNetworkTrainer {

    /**
     * Called to train a new network
     */
    public void trainNewNetwork(String description, File trainingDataFile, File neuralNetworkFilePath, File logFile,double testFraction,double eta,double lambda,int numEpochs) {
        File networkFile = new File(neuralNetworkFilePath, RingDetectorNeuralNetwork.getNeuralNetworkFilename());
        if (networkFile.exists()) {
            throw new RuntimeException("File:" + networkFile + " exists.  Please delete first before training a new network.");
        }

         // Now create the node structure according to the configuration
        int[] nodes = RingDetectorNeuralNetwork.getNetworkNodeTopology();

        // Instantiate the network
        NeuralNetwork ringnn = new NeuralNetwork(nodes, NeuralNetwork.CROSS_ENTROPY_COST);

        // and train it
        trainNetwork(description,trainingDataFile,ringnn,networkFile,logFile,testFraction,eta,lambda,numEpochs);
    }

    /**
     * Actual network training function called for both new and update
     */
    private void trainNetwork(String description,File trainingDataFile,NeuralNetwork ringnn,File nnOutputFile,File logFile,double testFraction,double eta,double lambda,int numEpochs){
        RingDataProcessor processor = new RingDataProcessor();
        processor.processData(trainingDataFile, testFraction, true, true);


        final StringBuffer logBuffer = new StringBuffer();
        String header = "Epoch#,Cost,del_L[0],del_L[1],delL[2],Normal Error";
        logBuffer.append(header + "\n");
        System.out.println(header);

        ringnn.addTrainingStatusListener(new NeuralNetwork.ITrainingStatusListener() {
            @Override
            public void trainingStatus(int epochNumber,double cost, double[]del_L,double normalError) {
                StringBuffer buffer = new StringBuffer();
                buffer.append(epochNumber+",");
                buffer.append(cost+",");
                for(int i=0;i < del_L.length;i++){
                    buffer.append(del_L[i]);
                    buffer.append(",");
                }
                buffer.append(normalError);
                System.out.println(buffer.toString());
                buffer.append("\n");
                logBuffer.append(buffer.toString());
            }
        });

        ringnn.train(description,processor.getXTrainingData(), processor.getYTrainingData(),
                processor.getScaleFactors(), 10,eta,lambda,numEpochs);
        // Write out the network
        try {
            // Delete the old file if it exists
            if (nnOutputFile.exists()) {
                nnOutputFile.delete();
            }
            // And replace or create it with the trained network
            FileOutputStream fos = new FileOutputStream(nnOutputFile);
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
    /**
     * Called to update train an existing network
     */
    public void updateNetwork(String description, File trainingDataFile, File neuralNetworkFilePath, File logFile,double testFraction,double eta,double lambda,int numEpochs) {
        // Read the network
        RingDetectorNeuralNetwork ringnn=null;
        try {
            ringnn = new RingDetectorNeuralNetwork(neuralNetworkFilePath,logFile,false);
        }
        catch(Exception e){
            e.printStackTrace();
            return;
        }
        // Get the networkFile to output the updated network after training
        File networkFile = new File(neuralNetworkFilePath, RingDetectorNeuralNetwork.getNeuralNetworkFilename());
        // And train it
        trainNetwork(description,trainingDataFile,ringnn,networkFile,logFile,testFraction,eta,lambda,numEpochs);
    }


    public void testNetwork(File testDataFile,File neuralNetworkFilePath,File logFile){
       // Read the network
        RingDetectorNeuralNetwork ringnn=null;
        try {
            ringnn = new RingDetectorNeuralNetwork(neuralNetworkFilePath,logFile,false);
        }
        catch(Exception e){
            e.printStackTrace();
            return;
        }
        RingDataProcessor processor = new RingDataProcessor();

        // Init the log buffer
        final StringBuffer logBuffer = new StringBuffer();
        logBuffer.append("#,Status,Truth,yTruth,Detection,yDetection,Cost\n");

        // Read the test data  but do not scale or shuffle
        processor.processData(testDataFile,1.0d ,false,false);
        SimpleMatrix x = processor.getXTestData();
        SimpleMatrix y = processor.getYTestData();

        // And loop through the test data
        int pass = 0;
        int fail = 0;
        for(int column=0;column < x.numCols();column++){
            SimpleMatrix input = x.extractVector(false,column);
            SimpleMatrix ytruth = y.extractVector(false,column);
            SimpleMatrix output = ringnn.doTestInference(input);
            int truth = ringnn.decodeOutput(ytruth);
            int inference = ringnn.decodeOutput(output);

            double cost = ringnn.computeTotalCost(input.getDDRM(),ytruth.getDDRM(),true);

            logBuffer.append(column+1+",");
            if (truth != inference){
                fail++;
                logBuffer.append("Fail,");
            }
            else {
                pass++;
                logBuffer.append("Pass,");

            }
            String ytruths = NeuralNetworkMatrixUtils.printColumn(ytruth,0);
            String truths = ringnn.convertResultToString(truth);

            String yinference = NeuralNetworkMatrixUtils.printColumn(output,0);
            String inferences = ringnn.convertResultToString(inference);
            logBuffer.append(truths+","+ytruths+","+inferences+","+yinference+","+String.format("%2.5f",cost)+"\n");
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
     * This is the method used for the image capture camera
     */
    public static void doCameraSensorNetwork(){
        RingNeuralNetworkTrainer trainer = new RingNeuralNetworkTrainer();

        // Get the path to the data directory
        Path currentRelativePath = Paths.get("");
        String dataPath = currentRelativePath.toAbsolutePath().toString() + "/src/test/java/org/firstinspires/ftc/teamcode/ringdetect/data";
        // Form training and testing log filenames using the configuration filename as a prefix
        String fileprefix = RingDetectorNeuralNetwork.getNeuralNetworkFilename();
        fileprefix = fileprefix.substring(0,fileprefix.length()-4);      // Strip .bin extension

        File neuralNetFilePath = new File(dataPath);

        boolean TRAIN = false;
        if (TRAIN) {
            File trainingLogFile = new File(dataPath,fileprefix+"_training_log.csv");
            trainer.trainNewNetwork(fileprefix,
                    neuralNetFilePath,neuralNetFilePath,trainingLogFile,0d,0.1d,0,5000);
        }
        boolean UPDATE = true;
        if (UPDATE) {
            File trainingLogFile = new File(dataPath,fileprefix+"_training_log.csv");
            trainer.updateNetwork(fileprefix,
                    neuralNetFilePath,neuralNetFilePath,trainingLogFile,0d,0.1d,0,5000);
        }

        boolean test = true;
        if (test){
            File testingLogFile = new File(dataPath, fileprefix +"_testing_log.csv");
            //           File testingDataFile = new File(dataPath,"15DEC20_training_data with old setup.csv");
            trainer.testNetwork(neuralNetFilePath,neuralNetFilePath,testingLogFile);
        }


    }

    private void logSystemConsole(String s){
        System.out.println(s);
    }

    /**
     * The main method in this class configures the
     */
    @Test
    public void main() {
        doCameraSensorNetwork();
    }
}