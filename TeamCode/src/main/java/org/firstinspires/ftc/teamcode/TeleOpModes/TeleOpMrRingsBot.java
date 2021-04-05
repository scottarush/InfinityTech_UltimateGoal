package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.MrRingsBotMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.MrRingsBot;
import org.firstinspires.ftc.teamcode.shooter.Grabber;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.EdgeDetector;

/**
 * This is the main TeleOp OpMode for the MrRingsBot.
 */
@TeleOp(name="TeleOpMrRingsBot", group="robot")
//@Disabled
public class TeleOpMrRingsBot extends OpMode {

    private MrRingsBot mRingsBot = null;

    private EdgeDetector mPad1DumpSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2DumpSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad1MidfieldLowGoalSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2MidfieldLowGoalSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad1MidfieldHighGoalSpeedSettingButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2MidfieldHighGoalSpeedSettingButtonEdgeDetector = new EdgeDetector();

    private EdgeDetector mPad1ActivateButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad2ActivateButtonEdgeDetector = new EdgeDetector();

    private EdgeDetector mPad1PulleyOverrideButtonEdgeDetector = new EdgeDetector();
    private EdgeDetector mPad1GrabberClampEdgeDetector = new EdgeDetector();

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

    @Override
    public void init() {
        mRingsBot = new MrRingsBot(this,false,false);
        try{
            mRingsBot.init();
        }
        catch(Exception e){
            telemetry.addLine("Error initialing MrRingsBot");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            return;
        }
        telemetry.addLine("MrRingsBot successfully initialized");
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        // call stop on the robot
        mRingsBot.stop();
    }

    @Override
    public void loop() {

        // ------------------------------------------------------
        // call loop function on robot first.  The robot's loop function quantizes to a fixed
        // T period and will return true only if the quantization time has been met.
        // ------------------------------------------------------
        if (!mRingsBot.loop()){
            return;  // Skip this one to force quantization on all controls
        }

        // Otherwise, quantization time met so execute teleop controls

        // ------------------------------------------------------
        // Do tank mode drive read on gamepad1.
        // ------------------------------------------------------
        // Run wheels in tank mode. Reverse signs on y since forward is negative on the controller
        double xleft = -gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = -gamepad1.right_stick_x;
        double yright = -gamepad1.right_stick_y;

        // the speeds with the new gamepad inputs
        mRingsBot.getDrivetrain().setTankDriveJoystickInput(xleft,yleft,xright,yright);

        // ------------------------------------------------------
        // b button on either pad activates or deactivates the shooter
        // ------------------------------------------------------
        if (mPad1ActivateButtonEdgeDetector.sampleRisingEdge(gamepad1.b) ||
                mPad2ActivateButtonEdgeDetector.sampleRisingEdge(gamepad2.b)) {
            if (mRingsBot.getShooter().isShooterActive()){
                mRingsBot.getShooter().deactivateShooter();
            }
            else {
                // trigger the evActivate event to the shooter controller
                mRingsBot.getShooter().activateShooter();
            }
        }
        // ------------------------------------------------------
        // a, x, and y buttons set the shooter speed setting
        // ------------------------------------------------------
        if (mPad1DumpSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad1.a) ||
                mPad2DumpSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad2.a)) {
            mRingsBot.getShooter().setShooterDistance((Shooter.SPEED_SETTING_DUMP));
        }
        if (mPad1MidfieldLowGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad1.x) ||
                mPad2MidfieldLowGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad2.x)) {
            mRingsBot.getShooter().setShooterDistance((Shooter.SPEED_SETTING_MIDFIELD_LOW_GOAL));
        }
        if (mPad1MidfieldHighGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad1.y) ||
                mPad2MidfieldHighGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad2.y)) {
            mRingsBot.getShooter().setShooterDistance((Shooter.SPEED_SETTING_MIDFIELD_HIGH_GOAL));
        }
        if (mPad1MidfieldHighGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad1.b) ||
                mPad2MidfieldHighGoalSpeedSettingButtonEdgeDetector.sampleRisingEdge(gamepad2.b)) {
            mRingsBot.getShooter().setShooterDistance((Shooter.SPEED_SETTING_MIDFIELD_POWER_SHOT));
        }

        // Right bumper on either pad shoots
        if (mPad1ShootButtonEdgeDetector.sampleRisingEdge(gamepad1.right_bumper) ||
                mPad2ShootButtonEdgeDetector.sampleRisingEdge(gamepad2.right_bumper)){
            mRingsBot.getShooter().shoot();
        }


        // ------------------------------------------------------
        // Grabber position controls on either dpad
        // ------------------------------------------------------
        if (mPad1GrabberRetractedEdgeDetector.sampleRisingEdge(gamepad1.dpad_up) ||
                mPad2GrabberRetractedEdgeDetector.sampleRisingEdge(gamepad2.dpad_up)){
            mRingsBot.getGrabber().setGrabberPosition(Grabber.GRABBER_FULLY_RETRACTED);
        }
        if (mPad1GrabberClearEdgeDetector.sampleRisingEdge(gamepad1.dpad_right) ||
                mPad2GrabberClearEdgeDetector.sampleRisingEdge(gamepad2.dpad_right)){
            mRingsBot.getGrabber().setGrabberPosition(Grabber.GRABBER_CLEAR_PULLEY);
        }
        if (mPad1GrabberLoweredEdgeDetector.sampleRisingEdge(gamepad1.dpad_down) ||
                mPad2GrabberLoweredEdgeDetector.sampleRisingEdge(gamepad2.dpad_down)){
            mRingsBot.getGrabber().setGrabberPosition(Grabber.GRABBER_LOWERED);
        }
        if (mPad1GrabberCarryEdgeDetector.sampleRisingEdge(gamepad1.dpad_left) ||
                mPad2GrabberCarryEdgeDetector.sampleRisingEdge(gamepad2.dpad_left)){
            mRingsBot.getGrabber().setGrabberPosition(Grabber.GRABBER_CARRY);
        }

        // ------------------------------------------------------
        // Manual pully override on gamepad1 left bumper
        // ------------------------------------------------------

        // Temporarily disable, make gamepad1.left_bumper the same as gamepad2
        /*
        if (mPad1PulleyOverrideButtonEdgeDetector.sampleRisingEdge(gamepad1.left_bumper)){
            // Toggle the pulley position
            switch (mRingsBot.getShooter().getLoaderPulleyPosition()) {
                case Shooter.LOADER_PULLEY_POSITION_HIGH:
                    mRingsBot.getShooter().setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_LOW);
                    break;
                case Shooter.LOADER_PULLEY_POSITION_LOW:
                    mRingsBot.getShooter().setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_HIGH);
                    break;
                case Shooter.LOADER_PULLEY_POSITION_MIDDLE:
                    if (mRingsBot.getShooter().isLoaderPulleyMoving()) {
                        mRingsBot.getShooter().stopLoaderPulley();
                    } else {
                        mRingsBot.getShooter().setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_LOW);
                    }
                    break;
            }
        }
        */

        // ------------------------------------------------------
        // Grabber open close on gamepad 2 left bumper
        // ------------------------------------------------------
        if (mPad2GrabberClampEdgeDetector.sampleRisingEdge(gamepad2.left_bumper) || mPad1GrabberClampEdgeDetector.sampleRisingEdge(gamepad1.left_bumper)) {
            if (mRingsBot.getGrabber().isGrabberOpen()) {
                mRingsBot.getGrabber().closeGrabber();
            } else {
                mRingsBot.getGrabber().openGrabber();
            }

        }


    }
}
