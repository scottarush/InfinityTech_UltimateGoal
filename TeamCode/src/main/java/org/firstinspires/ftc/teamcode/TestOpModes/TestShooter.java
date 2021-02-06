package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.EdgeDetector;

@TeleOp(name="TestShooter", group="robot")
//@Disabled
public class TestShooter extends OpMode {
    private Shooter mShooter = null;

    private EdgeDetector mSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mActivateButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPulleyButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mShootButtonEdgeDetector = new EdgeDetector();

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

        // b button turns it on and off
        if (mActivateButtonEdgeDetector.sampleRisingEdge(gamepad1.b)) {
            if (mShooter.isShooterActive()){
                mShooter.deactivateShooter();
            }
            else {
                // trigger the evActivate event to the shooter controller
                mShooter.getShooterController().evActivate();
            }
        }
        if (mSettingButtonEdgeDetector.sampleRisingEdge(gamepad1.a)) {
            switch(mShooter.getShooterSetting()){
                case Shooter.SETTING_MIDFIELD_LOW:
                    mShooter.setShooterDistance(Shooter.SETTING_MIDFIELD_HIGH);
                    break;
                case Shooter.SETTING_MIDFIELD_HIGH:
                    mShooter.setShooterDistance(Shooter.SETTING_MIDFIELD_LOW);
                    break;
            }
        }
        if (mShootButtonEdgeDetector.sampleRisingEdge(gamepad1.y)) {
            mShooter.shoot();
        }
        if (mPulleyButtonEdgeDetector.sampleRisingEdge(gamepad1.x)) {
            // Toggle the pulley position
            switch (mShooter.getLoaderPulleyPosition()) {
                case Shooter.LOADER_PULLEY_POSITION_HIGH:
                    mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_LOW);
                    break;
                case Shooter.LOADER_PULLEY_POSITION_LOW:
                    mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_HIGH);
                    break;
                case Shooter.LOADER_PULLEY_POSITION_MIDDLE:
                    if (mShooter.isLoaderPulleyMoving()) {
                        mShooter.stopLoaderPulley();
                    } else {
                        mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_LOW);
                    }
                    break;
            }
        }

            // Call the shooter service loop
        mShooter.serviceShooterLoop();

    }
}
