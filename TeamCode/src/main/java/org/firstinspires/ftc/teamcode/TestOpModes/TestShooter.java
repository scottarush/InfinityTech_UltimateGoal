package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

@TeleOp(name="TestShooter", group="robot")
//@Disabled
public class TestShooter extends OpMode {
    private Shooter mShooter = null;

    @Override
    public void init() {
        mShooter = new Shooter(this);
        // check for initialization errors
        try {
            mShooter.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Initialization Error(s)", e.getMessage());
            telemetry.update();
        }
        mShooter.setShooterDistance(Shooter.SETTING_MIDFIELD_HIGH);
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
            mShooter.getShooterController().evActivate();
        }
        if (gpa) {
            // trigger the evShoo event to the shooter controller
            mShooter.getShooterController().evShoot();
        }
        if (gpb) {
            // disable the shooter through the shooter controller
            mShooter.getShooterController().evDeactivate();
        }
        if (gamepad1.y){
            if (mShooter.getShooterSetting() == Shooter.SETTING_MIDFIELD_HIGH){
                mShooter.setShooterDistance(Shooter.SETTING_MIDFIELD_LOW);
            }
            else{
                mShooter.setShooterDistance(Shooter.SETTING_MIDFIELD_HIGH);
            }
        }
         // Call the shooter service loop
        mShooter.serviceShooterLoop();

    }
}
