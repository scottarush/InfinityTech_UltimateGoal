package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.SpeedBotMecanumDrive;
import org.firstinspires.ftc.teamcode.grabber.Grabber;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.EdgeDetector;

@TeleOp(name="ShooterBot", group="robot")
//@Disabled
public class TestShooterBot extends OpMode {
    private Shooter mShooter = null;
    private Grabber mGrabber = null;

    private EdgeDetector mSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mActivateButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPulleyButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mShootButtonEdgeDetector = new EdgeDetector();

    private EdgeDetector mGrabberRetractedEdgeDetector = new EdgeDetector();
    private EdgeDetector mGrabberClearEdgeDetector = new EdgeDetector();
    private EdgeDetector mGrabberCarryEdgeDetector = new EdgeDetector();
    private EdgeDetector mGrabberLoweredEdgeDetector = new EdgeDetector();

    private EdgeDetector mGrabberClampEdgeDetector = new EdgeDetector();

    private BaseMecanumDrive drivetrain = null;

    @Override
    public void init() {
        mShooter = new Shooter(this);
        mGrabber = new Grabber(this);
        // check for initialization errors
        try {
            mShooter.init(hardwareMap);
            mGrabber.init(hardwareMap);
            drivetrain = new SpeedBotMecanumDrive(this);
            drivetrain.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Initialization Error(s)", e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        super.stop();
        drivetrain.stop();
        // Stop the shooter to save the logging files (if any)
        mShooter.stop();
    }

    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double xleft = gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = gamepad1.right_stick_x;
        double yright = -gamepad1.right_stick_y;

        // the speeds with the new gamepad inputs
        drivetrain.setTankDriveJoystickInput(xleft,yleft,xright,yright);


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
        if (mShootButtonEdgeDetector.sampleRisingEdge(gamepad1.right_bumper)) {
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

        // Now process the grabber
        if (mGrabberRetractedEdgeDetector.sampleRisingEdge(gamepad1.dpad_up)){
            mGrabber.setGrabberPosition(Grabber.GRABBER_FULLY_RETRACTED);
        }
        if (mGrabberClearEdgeDetector.sampleRisingEdge(gamepad1.dpad_right)){
            mGrabber.setGrabberPosition(Grabber.GRABBER_CLEAR_PULLEY);
        }
        if (mGrabberLoweredEdgeDetector.sampleRisingEdge(gamepad1.dpad_down)){
            mGrabber.setGrabberPosition(Grabber.GRABBER_LOWERED);
        }
        if (mGrabberCarryEdgeDetector.sampleRisingEdge(gamepad1.dpad_left)){
            mGrabber.setGrabberPosition(Grabber.GRABBER_CARRY);
        }

        if (mGrabberClampEdgeDetector.sampleRisingEdge(gamepad1.left_bumper)){
            if (mGrabber.isGrabberOpen()) {
                mGrabber.closeGrabber();
            }
            else{
                mGrabber.openGrabber();
            }
        }
        // Call the grabber loop
        mGrabber.loop();
    }
}
