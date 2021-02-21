package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This class wraps the {@link BaseAutonomousOpMode} to execute the main
 * autonomous sequence.
 */
@Autonomous(name="Autonomous", group="Robot")
public class AutonomousModeOpMode extends OpMode  {

    BaseAutonomousOpMode mBaseOpMode = new BaseAutonomousOpMode(this,0);

    public AutonomousModeOpMode(){

    }
    @Override
    public void start() {
        mBaseOpMode.start();
        super.start();
    }

    @Override
    public void stop() {
        mBaseOpMode.stop();
        super.stop();
    }

    @Override
    public void init() {
        try {
            mBaseOpMode.init();
        }
        catch(Exception e){
            telemetry.addLine("Init Error:"+e.getMessage());
        }
    }

    @Override
    public void loop() {
        mBaseOpMode.loop();
    }
}
