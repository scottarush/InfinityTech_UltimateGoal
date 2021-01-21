package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.grabber.Grabber;
import org.firstinspires.ftc.teamcode.shooter.Shooter;

import static java.lang.Thread.sleep;

@TeleOp(name="TestGrabber", group="robot")
//@Disabled
public class TestGrabber extends OpMode {
    private Grabber mGrabber = null;
    private Shooter mShooter = null;

    enum grabberMotorStates {
        UP, DOWN, LIFT, UNKNOWN
    }
    grabberMotorStates grabberMotorState;
    enum grabberStates {
        OPEN, CLOSED, WOBBLE, UNKNOWN
    }
    grabberStates grabberState;
    enum lifterStates {
        LOW, HIGH, UNKNOWN
    }
    lifterStates lifterState;
    enum motorSpeeds {
        LOW, MEDIUM, HIGH, OFF
    }
    motorSpeeds motorSpeed;

    // If initial grabberMotorState is .UP, then these encoder values should be valid
    // Note that the zero position of the grabber motor depends on the position
    // of the grabber when the power is first turned on
    private int grabberMotorEncoderPositionUp = 10;
    private int grabberMotorEncoderPositionDown = 50;
    // Right now, not enough power to stay in a holding (lifted) position
    private int grabberMotorEncoderPositionLift = 25;

    private int GRABBER_POSITION_RETRACTED = 0;
    private int GRABBER_POSITION_CARRY = 1;
    private int GRABBER_POSITION_LOWERED = 2;

    private int lifterMotorEncoderPositionLow = 0;
    private int lifterMotorEncoderPositionHigh = 100;

    private ElapsedTime runtime;

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
        // Set states to initial configuration states
        motorSpeed = motorSpeeds.OFF;
        grabberState = grabberStates.OPEN;
        lifterState = lifterStates.LOW;
        grabberMotorState = grabberMotorStates.UP;

    }

    @Override
    public void loop() {

         ElapsedTime runtime = new ElapsedTime();

        // listen for gamepad input and do the corresponding function
        // x button starts the shooter
        // boolean gpx = gamepad1.x;
        boolean buttonMotorSpeed = gamepad1.x;
        // a button shoots a ring
        // boolean gpa = gamepad1.a;
        float triggerShoot = gamepad1.right_trigger;
        // b button deactivates the shooter
        // boolean gpb = gamepad1.b;
        boolean dPadUp = gamepad1.dpad_up;
        boolean dPadDown = gamepad1.dpad_down;
        boolean buttonGrabberActivate = gamepad1.y;
        boolean buttonGrabberWobblePosition = gamepad1.right_bumper;

        // Check grabber limit switch to see if pressed
        if (mGrabber.isLimitSwitchPressed) {
            grabberMotorState = grabberMotorStates.DOWN;
        }

        // This code uses the motor sp
        if (buttonMotorSpeed){
            switch(motorSpeed){
                case LOW:
                    motorSpeed = motorSpeeds.MEDIUM;
                    break;
                case MEDIUM:
                    motorSpeed = motorSpeeds.HIGH;
                    break;
                case HIGH:
                    motorSpeed = motorSpeeds.OFF;
                    break;
                case OFF:
                    motorSpeed = motorSpeeds.LOW;
                    break;
                default:
                    motorSpeed = motorSpeeds.LOW;
                    break;
            }
            // Add code here to call the Shooter Motors
        }
        // if grabberActivate button is pressed -- currently gamepad1 Y button
        if (buttonGrabberActivate) {

            switch (grabberState){
                case OPEN:
                    mGrabber.closeGrabber();
                    grabberState = grabberStates.CLOSED;
                    break;
                case CLOSED:
                    mGrabber.openGrabber();
                    grabberState = grabberStates.OPEN;
                    break;
                default:
                    mGrabber.closeGrabber();
                    grabberState = grabberStates.CLOSED;
                    break;
            }

        }

        if (buttonGrabberWobblePosition) {
            mGrabber.gotoGrabberWobblePosition();
            grabberState = grabberStates.WOBBLE;
        }

        // If either dPadUP or dPadDown are true, move the grabber motor
        if (dPadUp || dPadDown){
            if (dPadUp && grabberMotorState == grabberMotorStates.DOWN){
                // Current State is DOWN, desired state is UP
                int currentEncoderPosition = mGrabber.getGrabberEncoderPosition();
                // Zero is nominally UP, DOWN is positive, UP is negative
                int desiredEncoderPosition = currentEncoderPosition - (grabberMotorEncoderPositionDown - grabberMotorEncoderPositionUp);
                mGrabber.setGrabberPositionByEncoderValue(grabberMotorEncoderPositionUp);
                grabberMotorState = grabberMotorStates.UP;
            }
            if (dPadDown && grabberMotorState==grabberMotorStates.UP){
                // Current state is UP, desired state is DOWN
                int currentEncoderPosition = mGrabber.getGrabberEncoderPosition();
                // Zero is nominally UP, DOWN is positive
                int desiredEncoderPosition = (grabberMotorEncoderPositionDown - grabberMotorEncoderPositionUp) + currentEncoderPosition;
                mGrabber.setGrabberPositionByEncoderValue(desiredEncoderPosition);
                grabberMotorState = grabberMotorStates.DOWN;
            }
        }

        if (triggerShoot >= 0.5){
            // Need to verify what the range of values is on the triggers
            switch (lifterState){
                case LOW:
                    // if low, raise the pulled


                case HIGH:

                case UNKNOWN:

            }
        }

        /*
        // temporarily not execute this code block
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
        */

        // float power = gamepad1.left_stick_y;
        //
        telemetry.addData("Grabber Motor Position", mGrabber.motorEncoderPosition);
        telemetry.update();
        // mGrabber.setManualPower(power);
         // Call the grabber service loop
        mGrabber.loop();

        // Add a delay
        long holdTime  = 200;
        double startDelayTime = runtime.milliseconds();
        double currentDelayTime = startDelayTime;
        double endDelayTime = startDelayTime + holdTime;

        while (endDelayTime >= currentDelayTime){
            currentDelayTime = runtime.milliseconds();
        }
    }
}
