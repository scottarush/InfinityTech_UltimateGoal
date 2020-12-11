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


    private RingDetector mRingDetector = null;
    @Override
    public void init() {
        mRingDetector = new RingDetector(this);

    }

    public void stop() {
        super.stop();
    }

    @Override
    public void start() {
        super.start();
    }

    public void loop() {

    }

}
