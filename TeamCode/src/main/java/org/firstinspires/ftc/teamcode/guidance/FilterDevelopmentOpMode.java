package org.firstinspires.ftc.teamcode.guidance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.robot.MrRingsBot;

@Autonomous(name="FilterDevelopment", group="Robot")
@Disabled
public class FilterDevelopmentOpMode extends OpMode{
    private BaseAutonomousOpMode mAutonomousOpMode = null;

    private MrRingsBot mRingsBot = null;

    public FilterDevelopmentOpMode() {
        mAutonomousOpMode = new BaseAutonomousOpMode(this,0);
    }

    @Override
    public void init() {
        String initErrs = "";
        try {
            mAutonomousOpMode.init();
        }
        catch(Exception e){
            initErrs += ","+e.getMessage();
        }

        if (initErrs.length() == 0){
            telemetry.addData("Status:","Robot init complete");
        }
        else {
            telemetry.addData("Init errors:", initErrs);
        }
        telemetry.addData("IMU cal status", mRingsBot.getIMUCalibrationStatus());
        telemetry.update();

    }

    @Override
    public void start() {
        mAutonomousOpMode.start();
        super.start();
    }

    @Override
    public void stop() {
        mAutonomousOpMode.stop();
        super.stop();
    }

    public void loop() {
        // Call the autonomous opmod loop  and execute if quantization time met
        if (!mAutonomousOpMode.loop()){
            return;
        }
        // Otherwise, quantization time met

        telemetry.addData("KF Data","heading=%5.2f px=%4.1f py=%4.1f",
                mAutonomousOpMode.getKalmanTracker().getEstimatedHeading()*180d/Math.PI,
                mAutonomousOpMode.getKalmanTracker().getEstimatedXPosition(),
                mAutonomousOpMode.getKalmanTracker().getEstimatedYPosition());
        telemetry.update();
    }


}
