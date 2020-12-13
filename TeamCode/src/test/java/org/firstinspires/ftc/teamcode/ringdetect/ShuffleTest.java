package org.firstinspires.ftc.teamcode.ringdetect;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class ShuffleTest {

    @Test
    public void main() {
        Path currentRelativePath = Paths.get("");
        String filePath = currentRelativePath.toAbsolutePath().toString()+ "/src/main/java/org/firstinspires/ftc/teamcode/ringdetect/data";


        // Now create and pass training data to neural network
        JavaNeuralNetwork ringnn = new JavaNeuralNetwork(new int[]{10,5});

        final StringBuffer logBuffer = new StringBuffer();
        int cols = 20;
        SimpleMatrix x = new SimpleMatrix(10,cols);
        int offset =0;
        for(int i=0;i < x.numRows();i++){
            int increment = offset++;
            for(int j=0;j < x.numCols();j++){
                x.set(i,j,j+increment);
            }
        }
        SimpleMatrix y = new SimpleMatrix(5,cols);
        for(int i=0;i < y.numRows();i++){
            int increment = offset++;
            for(int j=0;j < y.numCols();j++){
                y.set(i,j,j+increment);
            }
        }

        logBuffer.append("Epoch#,Normal Error\n");
        ringnn.addTrainingStatusListener(new JavaNeuralNetwork.ITrainingStatusListener() {
            @Override
            public void trainingStatus(int epochNumber, double normalError) {
                logBuffer.append(epochNumber+","+ normalError +"\n");
            }
        });
        ringnn.train(x,y,null,10,1.0d,2,true);
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