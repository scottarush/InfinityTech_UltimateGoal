package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.MrRingsBotMecanumDrive;
import org.firstinspires.ftc.teamcode.shooter.Grabber;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.EdgeDetector;

/**
 * This is the main TeleOp OpMode for the MrRingsBot.
 */
@TeleOp(name="TeleOpMrRingsBot", group="robot")
//@Disabled
public class TeleOpMrRingsBot extends OpMode {
    private Shooter mShooter = null;
    private Grabber mGrabber = null;

    private EdgeDetector mPad1DumpSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2DumpSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad1MidfieldLowGoalSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2MidfieldLowGoalSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad1MidfieldHighGoalSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2MidfieldHighGoalSpeedSettingButtonEdgeDetector = new EdgeDetector();

    private EdgeDetector mPad1ActivateButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2ActivateButtonEdgeDetector = new EdgeDetector();

    private EdgeDetector mPad1PulleyOverrideButtonEdgeDetector = new EdgeDetector();

    private EdgeDetector mPad1ShootButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2ShootButtonEdgeDetector = new EdgeDetector();

    private EdgeDetector mPad1GrabberRetractedEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad1GrabberClearEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad1GrabberCarryEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad1GrabberLoweredEdgeDetector = new EdgeDetector();

    private EdgeDetector mPad2GrabberRetractedEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2GrabberClearEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2GrabberCarryEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2GrabberLoweredEdgeDetector = new EdgeDetector();

    private EdgeDetector mPad2GrabberClampEdgeDetector = new EdgeDetector();

    private BaseMecanumDrive drivetrain = null;

    @Override
    public void init() {
        mShooter = new Shooter(this);
        mGrabber = new Grabber(this);
        // check for initialization errors
        try {
            mShooter.init(hardwareMap);
            mGrabber.init(hardwareMap);
            drivetrain = new MrRingsBotMecanumDrive(this);
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
        // ------------------------------------------------------
        // Do tank mode drive read on gamepad1.
        // ------------------------------------------------------
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        // 13-Feb trials: drivers consistently think of the grabber side as 'front'
        // pre-change: x's were positive, y's were negative
        double xleft = -gamepad1.left_stick_x;
        double yleft = gamepad1.left_stick_y;
        double xright = -gamepad1.right_stick_x;
        double yright = gamepad1.right_stick_y;

        // the speeds with the new gamepad inputs
        drivetrain.setTankDriveJoystickInput(xleft,yleft,xright,yright);

        // ------------------------------------------------------
        // b button on either pad activates or deactivates the shooter
        // ------------------------------------------------------
        if (mPad1ActivateButtonEdgeDetector.sampleRisingEdge(gamepad1.b) ||
                mPad2ActivateButtonEdgeDetector.sampleRisingEdge(gamepad2.b)) {
            if (mShooter.isShooterActive()){
                mShooter.deactivateShooter();
            }
            else {
                // trigger the evActivate event to the shooter controller
                mShooter.activateShooter();
            }
        }
        // ------------------------------------------------------
        // a, x, and y buttons set the shooter speed setting
        // ------------------------------------------------------
        if (mPad1DumpSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad1.a) ||
                mPad2DumpSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad2.a)) {
            mShooter.setShooterDistance((Shooter.SPEED_SETTING_DUMP));
        }
        if (mPad1MidfieldLowGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad1.x) ||
                mPad2MidfieldLowGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad2.x)) {
            mShooter.setShooterDistance((Shooter.SPEED_SETTING_MIDFIELD_LOW_GOAL));
        }
        if (mPad1MidfieldHighGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad1.y) ||
                mPad2MidfieldHighGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad2.y)) {
            mShooter.setShooterDistance((Shooter.SPEED_SETTING_MIDFIELD_HIGH_GOAL));
        }

        // Right bumper on either pad shoots
        if (mPad1ShootButtonEdgeDetector.sampleRisingEdge(gamepad1.right_bumper) ||
                mPad2ShootButtonEdgeDetector.sampleRisingEdge(gamepad2.right_bumper)){
            mShooter.shoot();
        }


        // ------------------------------------------------------
        // Grabber position controls on either dpad
        // ------------------------------------------------------
        if (mPad1GrabberRetractedEdgeDetector.sampleRisingEdge(gamepad1.dpad_up) ||
                mPad2GrabberRetractedEdgeDetector.sampleRisingEdge(gamepad2.dpad_up)){
            mGrabber.setGrabberPosition(Grabber.GRABBER_FULLY_RETRACTED);
        }
        if (mPad1GrabberClearEdgeDetector.sampleRisingEdge(gamepad1.dpad_right) ||
                mPad2GrabberClearEdgeDetector.sampleRisingEdge(gamepad2.dpad_right)){
            mGrabber.setGrabberPosition(Grabber.GRABBER_CLEAR_PULLEY);
        }
        if (mPad1GrabberLoweredEdgeDetector.sampleRisingEdge(gamepad1.dpad_down) ||
                mPad2GrabberLoweredEdgeDetector.sampleRisingEdge(gamepad2.dpad_down)){
            mGrabber.setGrabberPosition(Grabber.GRABBER_LOWERED);
        }
        if (mPad1GrabberCarryEdgeDetector.sampleRisingEdge(gamepad1.dpad_left) ||
                mPad2GrabberCarryEdgeDetector.sampleRisingEdge(gamepad2.dpad_left)){
            mGrabber.setGrabberPosition(Grabber.GRABBER_CARRY);
        }

        // ------------------------------------------------------
        // Manual pully override on gamepad1 left bumper
        // ------------------------------------------------------
        if (mPad1PulleyOverrideButtonEdgeDetector.sampleRisingEdge(gamepad1.left_bumper)){
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
        // ------------------------------------------------------
        // Grabber open close on gamepad 2 left bumper
        // ------------------------------------------------------
        if (mPad2GrabberClampEdgeDetector.sampleRisingEdge(gamepad2.left_bumper)) {
            if (mGrabber.isGrabberOpen()) {
                mGrabber.closeGrabber();
            } else {
                mGrabber.openGrabber();
            }

        }
        // ------------------------------------------------------
        // End of controls now
        // ------------------------------------------------------

        // Call the shooter service loop
        mShooter.serviceShooterLoop();
        // Call the grabber loop
        mGrabber.loop();
    }
}
