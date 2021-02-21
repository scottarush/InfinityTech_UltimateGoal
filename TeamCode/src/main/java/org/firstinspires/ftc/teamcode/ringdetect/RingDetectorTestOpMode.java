package org.firstinspires.ftc.teamcode.ringdetect;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RingDetectorTest", group="Robot")
@Disabled
public class RingDetectorTestOpMode extends OpMode {

    private int mRingDetectorConfiguration = RingDetectorNeuralNetwork.CONFIGURATION_CAMERA_ONLY;
    private boolean mLastConfigurationButtonState = false;

    private RingDetector mRingDetector = null;

    private int mLastResult = RingDetectorNeuralNetwork.UNKNOWN;
    @Override
    public void init() {
        mRingDetector = new RingDetector(mRingDetectorConfiguration,this);
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

        // detect the tag button edge and change tag state if rising edge
        /**       boolean edgeDetect = false;
         boolean buttonState = gamepad1.y;
         if (buttonState) {
         // button pressed. check if last false
         if (mLastConfigurationButtonState == false) {
         edgeDetect = true;
         }
         }
         // Save tag button for next time
         mLastConfigurationButtonState = buttonState;

         // if we had a rising edge, cycle to the next tag
         if (edgeDetect == true) {
         if (mRingDetectorConfiguration == RingDetectorNeuralNetwork.ALL_SENSORS) {
         mRingDetectorConfiguration = RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR;
         } else if (mRingDetectorConfiguration == RingDetectorNeuralNetwork.NO_DISTANCE_SENSOR) {
         mRingDetectorConfiguration = RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR;
         } else if (mRingDetectorConfiguration == RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR) {
         mRingDetectorConfiguration = RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR;
         } else if (mRingDetectorConfiguration == RingDetectorNeuralNetwork.NO_MID_COLOR_SENSOR) {
         mRingDetectorConfiguration = RingDetectorNeuralNetwork.ALL_SENSORS;
         }
         // Re-initialize the ring detector with the new configuration
         mRingDetector.stop();  // To close log file
         // Now re-init
         mRingDetector = new RingDetector(this);
         String initErrs = "";
         try {
         mRingDetector.init();
         }
         catch(Exception e){
         initErrs += e.getMessage();
         }
         }
         **/


        long startTime = System.currentTimeMillis();
        mLastResult = mRingDetector.doDetection();
        int detectTime = (int)(System.currentTimeMillis()-startTime);
        String sinference = RingDetectorNeuralNetwork.convertResultToString(mLastResult);
        telemetry.addData("NNFile", RingDetectorNeuralNetwork.getNeuralNetworkFilename(mRingDetector.getRingDetectorConfiguration()));
        telemetry.addData("Inference Result", sinference);
        telemetry.addData("Inference Time(ms)",detectTime);
        telemetry.update();
    }

}
