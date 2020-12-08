package org.firstinspires.ftc.teamcode.ringdetect;

import org.junit.Test;

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


//        ringnn.train(processor.getXTrainingData(),processor.getYTrainingData());


        // print out the weights
       System.out.print(ringnn.printNetwork());

    }
}