package org.firstinspires.ftc.teamcode.ringdetect;
import org.junit.Test;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Test class to train the ring neural network.
 */
public class RingNeuralNetworkTrainer {

    @Test
    public void main() {
        Path currentRelativePath = Paths.get("");
        String trainingDataPath = currentRelativePath.toAbsolutePath().toString()+ "/src/test/java/org/firstinspires/ftc/teamcode/ringdetect";
        String inputFile = "06DEC20 initial training data.csv";
        RingTrainingDataPreProcessor processor = new RingTrainingDataPreProcessor(trainingDataPath+"/"+inputFile);
        processor.processData(0.10);
        // Now create and pass training data to neural network
        JavaNeuralNetwork ringnn = new JavaNeuralNetwork(new int[]{13,15,3});

        final StringBuffer logBuffer = new StringBuffer();
        logBuffer.append("Epoch#,Normal Error\n");
        ringnn.addTrainingStatusListener(new JavaNeuralNetwork.ITrainingStatusListener() {
            @Override
            public void trainingStatus(int epochNumber, double normalError) {
                logBuffer.append(epochNumber+","+ normalError +"\n");
            }
        });
        ringnn.train(processor.getXTrainingData(),processor.getYTrainingData(), processor.getScaleFactors(), 5,1.0d,5000);
        // Save the training log
        try{
            File logFile = new File(trainingDataPath+"/training_log.csv");
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

        // Now save the network to the runtime assets directory
        String runtimeNetworkPath = currentRelativePath.toAbsolutePath().toString()+ "/src/main/assets";

        try {
            File nnFile = new File(runtimeNetworkPath+"/neural_network.bin");
            if (nnFile.exists()){
                nnFile.delete();
            }
            FileOutputStream fos = new FileOutputStream(nnFile);
            ringnn.serializeNetwork(fos);
        }
        catch(Exception e){
            e.printStackTrace();
        }

        // print out the weights
 //       System.out.print(ringnn.printNetwork());

    }
}