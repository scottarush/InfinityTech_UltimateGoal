package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

@TeleOp(name="TestShooter", group="robot")
public class TestShooter extends OpMode {
    private Shooter mShooter = null;

    @Override
    public void init() {
        mShooter = new Shooter();
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
        boolean gpx = gamepad1.x;
        boolean gpa = gamepad1.a;
        boolean gpb = gamepad1.b;
        if (gpx) {
            mShooter.activateShooter();
        }
        if (gpa) {
            boolean shootStatus = mShooter.shoot();
            if (shootStatus == false) {
                telemetry.addData("status:", "shooter not ready");
                telemetry.update();
            } else if (shootStatus) {
                telemetry.addData("status:", "shot the ring :)");
                telemetry.update();
            }
        }
        if (gpb) {
            mShooter.deactivateShooter();
            telemetry.addData("status:", "shooter deactivated");
            telemetry.update();
        }
    }
}
