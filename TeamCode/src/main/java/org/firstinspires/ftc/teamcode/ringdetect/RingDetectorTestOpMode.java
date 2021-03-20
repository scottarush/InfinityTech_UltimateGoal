package org.firstinspires.ftc.teamcode.ringdetect;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RingDetectorTest", group="Robot")
@Disabled
public class RingDetectorTestOpMode extends OpMode {

    private boolean mLastConfigurationButtonState = false;

    private RingDetector mRingDetector = null;

    private int mLastResult = RingDetectorNeuralNetwork.UNKNOWN;
    @Override
    public void init() {
        mRingDetector = new RingDetector(this);
        String initErrs = "";
        try {
            mRingDetector.init();
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }
        if (initErrs.length() == 0){
            telemetry.addData("Status:","Ring Detector Init Success");
        }
        else {
            telemetry.addData("Init errors:", initErrs);
        }
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
        // Call stop to close the log file
        mRingDetector.stop();
    }

    public void loop() {

        long startTime = System.currentTimeMillis();
        mLastResult = mRingDetector.doSingleInference();
        int detectTime = (int)(System.currentTimeMillis()-startTime);
        String sinference = RingDetectorNeuralNetwork.convertResultToString(mLastResult);
        telemetry.addData("NNFile", RingDetectorNeuralNetwork.getNeuralNetworkFilename());
        telemetry.addData("Inference Result", sinference);
        telemetry.addData("Inference Time(ms)",detectTime);
        telemetry.update();
    }

}
