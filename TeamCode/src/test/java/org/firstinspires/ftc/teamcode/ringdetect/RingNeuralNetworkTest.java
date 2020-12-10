package org.firstinspires.ftc.teamcode.ringdetect;

import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class RingNeuralNetworkTest {

    @Test
    public void main() {
        Path currentRelativePath = Paths.get("");
        String filePath = currentRelativePath.toAbsolutePath().toString()+ "/src/main/java/org/firstinspires/ftc/teamcode/ringdetect";
        String inputFile = "06DEC20 initial training data.csv";
        RingTrainingDataPreProcessor processor = new RingTrainingDataPreProcessor(filePath+"/"+inputFile);
        processor.processData();
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
        ringnn.train(processor.getXTrainingData(),processor.getYTrainingData(),5,0.005d,50000);
        try{
            File logFile = new File(filePath+"/training_log.csv");
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

        // print out the weights
//        System.out.print(ringnn.printNetwork());

    }
}