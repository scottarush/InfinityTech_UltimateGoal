package org.firstinspires.ftc.teamcode.grabber;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class encapsulates front hook assembly that picks up rings on the front of the
 * robot.
 */
public class Grabber {

    /**
     * Retracted position.
     */
    public static final int GRABBER_POSITION_RETRACTED = 0;

    /**
     * carry position.
     */
    public static final int GRABBER_POSITION_CARRY = 1;

    /**
     * lowered position.
     */
    public static final int GRABBER_POSITION_LOWERED = 2;

    /**
     * Number of counts per 360 degree rotation for the grabber motor
     */
    private static final int GRABBER_MOTOR_COUNTS_PER_ROTATION = 288;

    /**
     * position for hook unknown which requires collapse to the limit switch to recalibrate.
     **/
    public static final int GRABBER_POSITION_UNKNOWN = 3;

    /**
     * encoder position at fully lowered.  Set during calibration.  All
     * other positions are relative to this one.
     */
    private int mLoweredEncoderPosition = 0;

    /**
     * delta angle in degrees from LOWERED to CARRY
     */
    private static final int CARRY_POSITION_ANGLE = 20;

    /**
     * delta angle in degrees from LOWERED to RETRACTED
     */
    private static final int RETRACTED_POSITION_ANGLE = 150;

    private int mGrabberPosition = GRABBER_POSITION_UNKNOWN;

    private Servo mLeftServo = null;
    private Servo mRightServo = null;
    private HardwareMap mHWMap = null;

    private DcMotor mGrabberMotor = null;
    private OpMode mOpMode = null;

    private IGrabberController mGrabberController = null;

    private DigitalChannel mLimitSwitch = null;

    private static final double LEFT_SERVO_OPEN_POSITION = 0.0d;
    private static final double LEFT_SERVO_CLOSED_POSITION = 1.0d;
    private double mLeftServoPosition = LEFT_SERVO_OPEN_POSITION;

    private static final double RIGHT_SERVO_OPEN_POSITION = 0d;
    private static final double RIGHT_SERVO_CLOSED_POSITION = 1.0d;
    private double mRightServoPosition = RIGHT_SERVO_CLOSED_POSITION;

    public Grabber(OpMode opMode) {
        opMode = mOpMode;
    }

    /**
     * Initializes the grabber
     * @param ahwMap hardwareMap
     * @throws Exception on any hardware detect error.
     */
    public void init(HardwareMap ahwMap) throws Exception {
        mHWMap = ahwMap;
        String initErrString = "";
        try {
            mLeftServo = mHWMap.get(Servo.class, "lgrabberservo");
            mLeftServo.setPosition(LEFT_SERVO_OPEN_POSITION);
        } catch (Exception e) {
            initErrString += "left grabber servo err";
        }
        try {
            mRightServo = mHWMap.get(Servo.class, "rgrabberservo");
            mRightServo.setPosition(RIGHT_SERVO_OPEN_POSITION);
        } catch (Exception e) {
            initErrString += "right grabber servo err";
        }
        try {
            mGrabberMotor = mHWMap.get(DcMotor.class, "grabbermotor");
            mGrabberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mGrabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } catch (Exception e) {
            initErrString += "grabber motor error";
        }
        try {
            mLimitSwitch = mHWMap.get(DigitalChannel.class, "grabberlsw");
            mLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            initErrString += "grabber limit sw error";
        }

        // Initialize the grabber controller
        mGrabberController = new GrabberController(this, mOpMode);
        // Trigger the calibration initialization in the controller
        mGrabberController.evInit();

        if (initErrString.length() > 0) {
            throw new Exception(initErrString);
        }
    }
    /**
     * must be called from overridden init_loop function within Opmode to drive the
     * calibration timer and limit switch checking.
     *
     */
    public void init_loop(){
        mGrabberController.loop();
        checkLimitSwitch();
    }

    /**
     * checks the limit switch during both calibration as well as continuously from the loop
     * to compensate for drift
     */
    private void checkLimitSwitch(){
        if (mLimitSwitch != null){
            if (mLimitSwitch.getState()){
                // limit switch trigger closed.  notify the controller
                mGrabberController.evLimitSwitchClosed();
                // and save the encoder position to set the reference
                if (mGrabberMotor != null) {
                    mLoweredEncoderPosition = mGrabberMotor.getCurrentPosition();
                    mGrabberPosition = GRABBER_POSITION_LOWERED;
                }
            }
        }
    }

    /**
     * must be called from OpMode loop function when using automatic mode.
     */
    public void loop(){
        if (mGrabberController != null){
            mGrabberController.loop();
        }
        // Check the limit switch each loop time to reset the reference encoder position
        checkLimitSwitch();
        // Check if grabber has stopped and notify the grabber
        if (!mGrabberMotor.isBusy()){
            mGrabberController.evGrabberStopped();
        }
    }

    /**
     *
     * @return true if grabbber is open, false if close
     */
    public boolean isGrabberOpen() {
        if (mLeftServoPosition == LEFT_SERVO_OPEN_POSITION) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * sets the grabber position automatically.  This function cannot be used in all manual mode.
     * @return true if position set was attempted, false if position currently unknown and cannot be set
     */
    public boolean setGrabberPosition(int position){
        if (mGrabberPosition == GRABBER_POSITION_UNKNOWN){
            // Error.  Can't set position when unknown
            return false;
        }
        if (mGrabberPosition == position) {
            return true;   // already at this position
        }
        // Otherwise it is a change.  Determine the angle delta to move relative to the
        // reference (lowered) position
        int deltaAngle = 0;
        switch(mGrabberPosition){
            case GRABBER_POSITION_RETRACTED:
                switch(position){
                    case GRABBER_POSITION_LOWERED:
                        deltaAngle = -RETRACTED_POSITION_ANGLE;
                        break;
                    case GRABBER_POSITION_CARRY:
                        deltaAngle = -RETRACTED_POSITION_ANGLE+CARRY_POSITION_ANGLE;
                        break;
                }
                break;
            case GRABBER_POSITION_LOWERED:
                switch(position){
                    case GRABBER_POSITION_RETRACTED:
                        deltaAngle = RETRACTED_POSITION_ANGLE;
                        break;
                    case GRABBER_POSITION_CARRY:
                        deltaAngle = CARRY_POSITION_ANGLE;
                        break;
                }
                break;
            case GRABBER_POSITION_CARRY:
                switch(position){
                    case GRABBER_POSITION_RETRACTED:
                        deltaAngle = -RETRACTED_POSITION_ANGLE+CARRY_POSITION_ANGLE;
                        break;
                    case GRABBER_POSITION_LOWERED:
                        deltaAngle = -CARRY_POSITION_ANGLE;
                        break;
                }
                break;
        }
        // The delta angle to move has been determined now compute the target position for this
        // delta
        double angled = (double)GRABBER_MOTOR_COUNTS_PER_ROTATION * (double)deltaAngle/360d;
        int deltaCounts = (int)Math.round(angled);

        int targetPosition = mLoweredEncoderPosition +deltaCounts;
        if (mGrabberMotor != null){
            mGrabberMotor.setTargetPosition(targetPosition);
            // And notify the controller
            mGrabberController.evGrabberMoving();
            return true;
        }
        else{
            // motor init error, can't move it
            return false;
        }
    }


    /**
     *
     * @return current grabber position
     */
    public int getGrabberPosition() {
        return mGrabberPosition;
    }

    public void openGrabber() {
        setLeftServoPosition(LEFT_SERVO_OPEN_POSITION);
        setRightServoPosition(RIGHT_SERVO_OPEN_POSITION);
    }

    public void closeGrabber() {
        setLeftServoPosition(LEFT_SERVO_CLOSED_POSITION);
        setRightServoPosition(RIGHT_SERVO_CLOSED_POSITION);
    }

    private void setLeftServoPosition(double position) {
        mLeftServoPosition = position;
        if (mLeftServo != null) {
            mLeftServo.setPosition(mLeftServoPosition);
        }
    }
    private void setRightServoPosition(double position) {
        mRightServoPosition = position;
        if (mRightServo != null) {
            mRightServo.setPosition(mRightServoPosition);
        }
    }

}

