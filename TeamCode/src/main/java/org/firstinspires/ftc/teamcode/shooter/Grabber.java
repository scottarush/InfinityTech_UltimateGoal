package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.EdgeDetector;

/**
 * This class encapsulates front grabber assembly that picks up rings on the front of the
 * robot.
 */
public class Grabber {

    /**
     * index/enum constant for fully retracted position.
     */
    public static final int GRABBER_FULLY_RETRACTED = 3;

    /**
     * index/enum constant for clear pulley position just slightly from RETRACTED used to make room for the
     * pulley to shoot
     */
    public static final int GRABBER_CLEAR_PULLEY = 2;
    /**
     * index/enum constant for carry position.
     */
    public static final int GRABBER_CARRY = 1;

    /**
     * index/enum constant for lowered position.
     */
    public static final int GRABBER_LOWERED = 0;

    /**
     * Number of counts per 360 degree rotation for the grabber motor
     */
    private static final int GRABBER_MOTOR_COUNTS_PER_ROTATION = 288;

    /**
     * encoder positions array indexed by the GRABBER_POSITION_ at fully lowered.  Set whenever the limit switch is triggerd.  All
     * other positions are then updated relative to this one.
     *
     * Assume 0 at startup is the RETRACTED position.  The other encoder values are determined
     * according to the following formula:
     *  encoder counts delta = GRABBER_MOTOR_COUNTS_PER_ROTATION * delta/360;
     **/
    private int mGrabberEncoderCounts[] = new int[] {0,93,28,109};

    /**
     * Fixed angles corresponding to the grabber positions.
     * order is LOWERED, CARRY, CLEAR, and RETRACTED
     */
    public static final int GRABBER_ANGLES[] = new int[] {0,20,116,136};

    // The initial position must be retracted at startup to keep it within the 18" cube
    private int mGrabberPosition = GRABBER_FULLY_RETRACTED;

    private boolean mGrabberMoving = true;

    private double mGrabberPower = 1.0d;

    private Servo mLeftServo = null;
    private Servo mRightServo = null;
    private HardwareMap mHWMap = null;

    private DcMotor mGrabberMotor = null;
    private OpMode mOpMode = null;

    private RevTouchSensor mLimitSwitch = null;
    private EdgeDetector mLimitSwitchEdgeDetector = new EdgeDetector();

    private static final double LEFT_SERVO_OPEN_POSITION = 0.25d;
    private static final double LEFT_SERVO_CLOSED_POSITION = 0.0d;
    private static final double LEFT_SERVO_WOBBLE_POSITION = 0.15d;
    private double mLeftServoPosition = LEFT_SERVO_OPEN_POSITION;

    private static final double RIGHT_SERVO_OPEN_POSITION = 0d;
    private static final double RIGHT_SERVO_CLOSED_POSITION = 0.25d;
    private static final double RIGHT_SERVO_WOBBLE_POSITION = 0.10d;
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
            mGrabberMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            mGrabberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mGrabberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Initialize the count positions using the current motor position
            int retractedCounts = mGrabberMotor.getCurrentPosition();
            int loweredCounts = retractedCounts- (int) Math.round((double)GRABBER_ANGLES[GRABBER_FULLY_RETRACTED]/360d * (double)GRABBER_MOTOR_COUNTS_PER_ROTATION);
            for(int i=0;i < GRABBER_ANGLES.length;i++){
                int countValue = (int) Math.round((double)GRABBER_ANGLES[i]/360d * (double)GRABBER_MOTOR_COUNTS_PER_ROTATION) + loweredCounts;
                mGrabberEncoderCounts[i] = countValue;
            }

        } catch (Exception e) {
            initErrString += "grabber motor error";
        }
        try {
            mLimitSwitch = mHWMap.get(RevTouchSensor.class, "grabberlsw");
        } catch (Exception e) {
            initErrString += "grabber limit sw error";
        }

        if (initErrString.length() > 0) {
            throw new Exception(initErrString);
        }
    }

    /**
     * checks the limit switch.  If pressed, the current encoder counts value from the
     * motor will be read and used to update the entire array.  This compensates for
     * drift over time.
     */
    private void checkLimitSwitch(){
        if (mLimitSwitch != null){
            // Feed the current state into the EdgeDetector
            boolean risingEdge = mLimitSwitchEdgeDetector.sampleRisingEdge(mLimitSwitch.isPressed());

            if (risingEdge) {
                // Limit switch just pressed.  Read the motor's current encoder counts and recalculate
                // the other positions based on this new offset which must be the counts values
                // at the LOWERED position.
                mGrabberPosition = GRABBER_LOWERED;  // Latch the lowered position
                if (mGrabberMotor != null){
                    int loweredCounts = mGrabberMotor.getCurrentPosition();
                    // Now recompute all the counts start from thise
                    for(int i=0;i < GRABBER_ANGLES.length;i++){
                        int countValue = (int) Math.round((double)GRABBER_ANGLES[i]/360d * (double)GRABBER_MOTOR_COUNTS_PER_ROTATION +(double)loweredCounts);
                        mGrabberEncoderCounts[i] = countValue;
                    }
                }
            }
         }
    }

    /**
     * must be called from OpMode loop function when using automatic mode.
     */
    public void loop(){
        // Check the limit switch and update the encoder the positions if pressed.
        checkLimitSwitch();

        // Check if we are moving the grabber to a new position and if we have reached that
        // new position, stop the movement
        if (mGrabberMoving){
            if (mGrabberMotor != null){
                if (!mGrabberMotor.isBusy()){
                    mGrabberMoving = false;
                    // Reached destination
                    mGrabberMotor.setPower(0d);  // Set power to 0 to brake
                    // Not sure if this is necessary, but example shows it
                    mGrabberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
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
    public boolean setGrabberPosition(int targetPosition){
        // Otherwise it is a change.  Check for validity
        if (targetPosition < 0){
            return false; // invalid
        }
        if (targetPosition > GRABBER_ANGLES.length){
            return false;
        }
        // Valid position.  Pull the target encoder position from the array and
        // run to position there.
        mGrabberPosition = targetPosition;  // Save the value (assuming we get there)

        // Target counts is just the value from the array
        int targetCounts = mGrabberEncoderCounts[targetPosition];

        if (mGrabberMotor != null){
            mGrabberMoving = true;
            mGrabberMotor.setTargetPosition(targetCounts);
            mGrabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mGrabberMotor.setPower(mGrabberPower);
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

    public void setGrabbleWobble(){
        setLeftServoPosition(LEFT_SERVO_WOBBLE_POSITION);
        setRightServoPosition(RIGHT_SERVO_WOBBLE_POSITION);
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

//   public void setManualPower(double power){
//        if (mGrabberMotor != null){
//            mGrabberMotor.setPower(power);
//        }
//   }

}

