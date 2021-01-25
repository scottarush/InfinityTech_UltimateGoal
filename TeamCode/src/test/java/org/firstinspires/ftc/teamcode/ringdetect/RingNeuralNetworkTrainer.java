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
    public void trainNewNetwork(String description, File trainingDataFile, File neuralNetworkFilePath,int networkConfig, File logFile){
        RingDataProcessor processor = new RingDataProcessor(networkConfig);
        File networkFile = new File(neuralNetworkFilePath,RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig));

        processor.processData(trainingDataFile,0.10 ,true,true);

        // Now create the node structure according to the configuration
        int[] nodes = RingDetectorNeuralNetwork.getNetworkNodeTopology(networkConfig);

        NeuralNetwork ringnn = new NeuralNetwork(nodes,NeuralNetwork.CROSS_ENTROPY_COST);

        final StringBuffer logBuffer = new StringBuffer();
        String header = "Epoch#,Cost,del_L[0],del_L[1],delL[2],Normal Error";
        logBuffer.append(header+"\n");
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
                processor.getScaleFactors(), 10,0.1d,0d,5000);
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

    public void testNetwork(File testDataFile,File neuralNetworkFilePath,int networkConfig,File logFile,double testFraction){
        RingDataProcessor processor = new RingDataProcessor(networkConfig);
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
        logBuffer.append("#,Status,Truth,yTruth,Detection,yDetection,Cost\n");

        // Read the test data  but do not scale or shuffle
        processor.processData(testDataFile,testFraction ,false,false);
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
     * This is the method used for the color and distance sensor network
     */
    public static void doColorDistanceSensorNetwork(){
        RingNeuralNetworkTrainer trainer = new RingNeuralNetworkTrainer();
        int networkConfig = RingDetectorNeuralNetwork.CONFIGURATION_NO_MID_COLOR_SENSOR;

        // Get the path to the data directory
        Path currentRelativePath = Paths.get("");
        String dataPath = currentRelativePath.toAbsolutePath().toString() + "/src/test/java/org/firstinspires/ftc/teamcode/ringdetect/data";
        // Form training and testing log filenames using the configuration filename as a prefix
        String fileprefix = RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig);
        fileprefix = fileprefix.substring(0,fileprefix.length()-4);      // Strip .bin extension
        File neuralNetFilePath = new File(dataPath);

        boolean TRAIN = true;
        if (TRAIN) {
            File trainingFile = new File(dataPath, "21DEC20_training_data.csv");
            File trainingLogFile = new File(dataPath,fileprefix+"_training_log.csv");
            trainer.trainNewNetwork(RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig),
                    trainingFile,neuralNetFilePath,networkConfig,trainingLogFile);
        }

        boolean test = true;
        if (test){
            File testingLogFile = new File(dataPath, fileprefix +"_testing_log.csv");
            File testingDataFile = new File(dataPath,"21DEC20_testing_data.csv");
            //           File testingDataFile = new File(dataPath,"15DEC20_training_data with old setup.csv");
            trainer.testNetwork(testingDataFile,neuralNetFilePath,networkConfig,testingLogFile,1.0);
        }


    }
    /**
     * This is the method used for the image capture camera
     */
    public static void doCameraSensorNetwork(){
        RingNeuralNetworkTrainer trainer = new RingNeuralNetworkTrainer();
        int networkConfig = RingDetectorNeuralNetwork.CONFIGURATION_CAMERA_ONLY;

        // Get the path to the data directory
        Path currentRelativePath = Paths.get("");
        String dataPath = currentRelativePath.toAbsolutePath().toString() + "/src/test/java/org/firstinspires/ftc/teamcode/ringdetect/data/images";
        // Form training and testing log filenames using the configuration filename as a prefix
        String fileprefix = RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig);
        fileprefix = fileprefix.substring(0,fileprefix.length()-4);      // Strip .bin extension

        File neuralNetFilePath = new File(dataPath);
        boolean TRAIN = true;
        if (TRAIN) {
            File trainingLogFile = new File(dataPath,fileprefix+"_training_log.csv");
            trainer.trainNewNetwork(fileprefix,
                    neuralNetFilePath,neuralNetFilePath,networkConfig,trainingLogFile);
        }

        boolean test = true;
        if (test){
            File testingLogFile = new File(dataPath, fileprefix +"_testing_log.csv");
            //           File testingDataFile = new File(dataPath,"15DEC20_training_data with old setup.csv");
            trainer.testNetwork(neuralNetFilePath,neuralNetFilePath,networkConfig,testingLogFile,1.0);
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