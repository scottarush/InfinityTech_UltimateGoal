package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.ShooterController;

@TeleOp(name="TestShooter", group="robot")
//@Disabled
public class TestShooter extends OpMode {
    private ShooterController mShooterController = null;
    private Shooter mShooter = null;
    // Make sure the left motor is called "shooterL" in the configuration and "shooterR" for the right motor
    // the servo needs to be called "pusherServo", and can be disabled. see init() for more info

    @Override
    public void init() {
        mShooter = new Shooter();
        mShooterController = new ShooterController(mShooter);
        // to disable the servos, uncomment (take out "//") this next line:

        //mShooter.disableServo();

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
            // start shooter and print telemetry
            mShooter.activateShooter(Shooter.SHOOTER_MIDFIELD_HIGH_SETTING);
            telemetry.addData("status:", "ready to shoot!");
            telemetry.update();
        }
        if (gpa) {
            boolean shootStatus = mShooter.shoot();
            // check if the shooter is ready to shoot
            if (shootStatus == false) {
                telemetry.addData("status:", "shooter not ready!");
                telemetry.update();
            } else if (shootStatus) {
                telemetry.addData("status:", "shot the ring :)");
                telemetry.update();
            }
        }
        if (gpb) {
            // brake the motors
            mShooter.deactivateShooter();
            telemetry.addData("status:", "shooter deactivated");
            telemetry.update();
        }
        if (gpy) {
            boolean servoEnabled = mShooter.checkServoStatus();
            if (servoEnabled) {
                mShooter.actuateServo();
            } else {
                // print error if not
                telemetry.addData("status:", "Servo not enabled! Cannot run the function!");
                telemetry.update();
            }
        }
        // Service the shooter controller loop
        mShooterController.loop();

    }
}
