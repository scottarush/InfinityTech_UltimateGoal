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

    /**
     * Called to train a new network
     */
    public void trainNewNetwork(String description, File trainingDataFile, File neuralNetworkFilePath,int networkConfig, File logFile){
        RingDataProcessor processor = new RingDataProcessor(networkConfig);
        File networkFile = new File(neuralNetworkFilePath,RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig));

        processor.processData(trainingDataFile,0.10 ,true,true);

        // Now create the node structure according to the configuration
        int[] nodes = RingDetectorNeuralNetwork.getNetworkNodes(networkConfig);

        NeuralNetwork ringnn = new NeuralNetwork(nodes);

        final StringBuffer logBuffer = new StringBuffer();
        logBuffer.append("Epoch#,del_L[0],del_L[1],delL[2],Normal Error\n");

        ringnn.addTrainingStatusListener(new NeuralNetwork.ITrainingStatusListener() {
            @Override
            public void trainingStatus(int epochNumber, double[]del_L, double normalError) {
                StringBuffer buffer = new StringBuffer();
                buffer.append(epochNumber+",");
                for(int i=0;i < del_L.length;i++){
                    buffer.append(del_L[i]);
                    buffer.append(",");
                }
                buffer.append(normalError);
                buffer.append("\n");
                logBuffer.append(buffer.toString());
            }
        });

        ringnn.train(description,processor.getXTrainingData(), processor.getYTrainingData(),
                processor.getScaleFactors(), 10,0.1d,5000,true);
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
        logBuffer.append("#,Status,Truth,yTruth,Detection,yDetection\n");

        // Read the test data but do not scale or shuffle it
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

            logBuffer.append(column+1+",");
            if (truth != inference){
                fail++;
                logBuffer.append("Fail");
            }
            else {
                pass++;
                logBuffer.append("Pass");

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
        String dataPath = currentRelativePath.toAbsolutePath().toString() + "/src/test/java/org/firstinspires/ftc/teamcode/ringdetect/data";
        String trainingFileName = "15DEC20_training_data.csv";
        String testingFileName = "15DEC20_testing_data.csv";

        File trainingFile = new File(dataPath, trainingFileName);
        File neuralNetFilePath = new File(dataPath);

        boolean TRAIN = false;
        int networkConfig = RingDetectorNeuralNetwork.ALL_SENSORS;
        // Form training and testing log filenames using the configuration filename as a prefix
        String fileprefix = RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig);
        fileprefix = fileprefix.substring(0,fileprefix.length()-4);      // Strip .bin extension
        String trainingLogFilename= fileprefix+"_training_log.cvs";
        if (TRAIN) {
            File logFile = new File(dataPath,trainingLogFilename);
            trainer.trainNewNetwork(RingDetectorNeuralNetwork.getNeuralNetworkFilename(networkConfig),
                    trainingFile,neuralNetFilePath,networkConfig,logFile);
        }

        boolean test = true;
        String testingLogFilename = fileprefix +"_testing_log.cvs";
        if (test){
            File logFile = new File(dataPath,testingLogFilename);
            trainer.testNetwork(trainingFile,neuralNetFilePath,networkConfig,logFile,1.0);
        }

    }
}