package org.firstinspires.ftc.teamcode.ringdetect;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.LogFile;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

@TeleOp(name="RingDetectorTest", group="Robot")
public class RingDetectorTestOpMode extends OpMode {

    private int mLastInferenceResult = RingDetectorNeuralNetwork.UNKNOWN;

    private int mInferenceNum = 0;

    private RingDetector mRingDetector = null;
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
        int inference = mRingDetector.readDetector();
        if (mLastInferenceResult != inference){
            mLastInferenceResult = inference;
            String sinference = RingDetectorNeuralNetwork.convertToString(inference);
            telemetry.addData("New Inference Result","#"+mInferenceNum++ +":"+sinference);
            telemetry.update();
        }
    }

}
