package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.grabber.Grabber;
import org.firstinspires.ftc.teamcode.shooter.Shooter;

@TeleOp(name="TestGrabber", group="robot")
//@Disabled
public class TestGrabber extends OpMode {
    private Grabber mGrabber = null;


    @Override
    public void init() {
        mGrabber = new Grabber(this);
        // check for initialization errors
        try {
            mGrabber.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Initialization Error(s)", e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void init_loop() {
        super.init_loop();
        mGrabber.init_loop();
    }

    @Override
    public void loop() {

        // listen for gamepad input and do the corresponding function
        // x button starts the shooter
        boolean gpx = gamepad1.x;
        // a button shoots a ring
        boolean gpa = gamepad1.a;
        // b button deactivates the shooter
        boolean gpb = gamepad1.b;
        if (gpx) {
            // trigger the evActivate event to the shooter controller
            mGrabber.setGrabberPosition(Grabber.GRABBER_POSITION_RETRACTED);
        }
        if (gpa) {
            // trigger the evShoo event to the shooter controller
            mGrabber.setGrabberPosition(Grabber.GRABBER_POSITION_CARRY);
        }
        if (gpb) {
            // disable the shooter through the shooter controller
            mGrabber.setGrabberPosition(Grabber.GRABBER_POSITION_LOWERED);
        }
        if (gamepad1.y){
            if (mGrabber.isGrabberOpen()){
                mGrabber.closeGrabber();
            }
            else{
                mGrabber.openGrabber();
            }
        }
        float power = gamepad1.left_stick_y;
        mGrabber.setManualPower(power);
         // Call the grabber service loop
        mGrabber.loop();

    }
}
