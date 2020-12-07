package org.firstinspires.ftc.teamcode.ringdetect;

import org.junit.Test;

import java.nio.file.Path;
import java.nio.file.Paths;

public class RingTrainingDataProcessorTest {

    @Test
    public void main() {
        Path currentRelativePath = Paths.get("");
        String filePath = currentRelativePath.toAbsolutePath().toString()+ "/src/main/java/org/firstinspires/ftc/teamcode/ringdetect";
        String inputFile = "06DEC20 initial training data.csv";
        String outputFile = "ringdetect_model.bin";
        RingTrainingDataPreProcessor processor = new RingTrainingDataPreProcessor(filePath+"/"+inputFile);
        processor.processData();
    }
}