package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

@TeleOp(name="TestShooter", group="robot")
//@Disabled
public class TestShooter extends OpMode {
    private Shooter mShooter = null;
    // Make sure the left motor is called "shooterL" in the configuration and "shooterR" for the right motor
    // the servo needs to be called "pusherServo", and can be disabled. see init() for more info

    @Override
    public void init() {
        mShooter = new Shooter();
        // check for initialization errors
        try {
            mShooter.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Initialization Error(s)", e.getMessage());
            telemetry.update();
        }
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
        // y button actuates the servo
        boolean gpy = gamepad1.y;
        if (gpx) {
            // activate the shooter through the controller
            mShooter.getShooterController().activateShooter();
            telemetry.addData("status:", "activating");
            telemetry.update();
        }
        if (gpa) {
            // check if the shooter is ready to shoot
            if (mShooter.isShooterReady() == false) {
                telemetry.addData("status:", "shooter not ready!");
                telemetry.update();
            }
            else {
                // Shoot through the shooter controller
                mShooter.getShooterController().shoot();
                telemetry.addData("status:", "shot the ring :)");
                telemetry.update();
            }
        }
        if (gpb) {
            // disable the shooter through the shooter controller
            mShooter.getShooterController().deactivateShooter();
            telemetry.addData("status:", "shooter deactivated");
            telemetry.update();
        }
         // Call the shooter service loop
        mShooter.serviceShooterLoop();

    }
}
