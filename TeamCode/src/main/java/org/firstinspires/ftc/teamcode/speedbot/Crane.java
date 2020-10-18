package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * This class encapsulates the Crane and grabbing hand on the speed bot.
 *
 */
public class Crane {


    /**
     * Core hex motor from the specification
     */
    public static final int ENCODER_COUNTS_PER_ROTATION = 288;
    public static final double PULLY_CIRCUMFERENCE = 4.712d;
    public static final double COUNTS_PER_INCH = ENCODER_COUNTS_PER_ROTATION / PULLY_CIRCUMFERENCE;

    private OpMode mOpMode = null;

    private DcMotor mCraneMotor = null;

    public static final String CRANE_MOTOR_NAME = "crane";
    public static final String HAND_SERVO_NAME = "handservo";

    /** enum values for hand positions. **/
    public static final int HAND_RETRACTED = 0;
    public static final int HAND_OPEN = 1;
    public static final int HAND_CLOSED = 2;
    private int mHandPosition = 0;

    /** servo values for hand positions. **/
    private static final double HAND_CLOSE_POSITION = 0.0d;
    private static final double HAND_OPEN_POSITION = 0.4d;
    private static final double HAND_RETRACT_POSITION = 0.98d;

    private double mHandServoPosition = HAND_RETRACT_POSITION;

    private Servo mHandServo = null;

    private ElapsedTime mRampTimer = new ElapsedTime();

    private ArrayList<ICraneMovementStatusListener> mCraneMoveListeners = new ArrayList<>();
   // private DigitalChannel mLimitSwitch = null;
    private OneShotTimer mCraneMovementTimeoutTimer = null;
    // Constants

    public static final double RAISE_MAX_POWER = 0.5d;
    public static final double RAISE_START_POWER = 0.1d;
    public static final double RAISE_RAMP_UP_TIME = 1.0d;
    public static final double RAISE_POWER_SLOPE_PER_SEC = (RAISE_MAX_POWER - RAISE_START_POWER)/ RAISE_RAMP_UP_TIME;
    public static final double LOWER_MAX_POWER = 0.5d;
    public static final double LOWER_RAMP_UP_TIME = 0.5d;
    public static final double LOWER_START_POWER = 0.1d;
    public static final double LOWER_POWER_SLOPE_PER_SEC = (LOWER_MAX_POWER - LOWER_START_POWER)/ LOWER_RAMP_UP_TIME;


    /**
     * Constructs the Crane
     * @param opMode
     */
    public Crane(OpMode opMode) {
        mOpMode = opMode;
        mCraneMovementTimeoutTimer = new OneShotTimer(2000, new OneShotTimer.IOneShotTimerCallback() {
            @Override
            public void timeoutComplete() {
                stop();
                for(Iterator<ICraneMovementStatusListener> iter = mCraneMoveListeners.iterator(); iter.hasNext();){
                    ICraneMovementStatusListener listener = iter.next();
                    listener.moveTimeoutFailure();
                }
            }
        });

    }


    public void init(HardwareMap ahwMap) throws Exception {
        String initErrString = "";
        try {
            mCraneMotor = ahwMap.get(DcMotor.class, CRANE_MOTOR_NAME);
            mCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mCraneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            initErrString += "Crane motor err,";
        }

        try {
            mHandServo = ahwMap.get(Servo.class, HAND_SERVO_NAME);
            setHandServoPosition(HAND_CLOSED);
        } catch (Exception e) {
            initErrString += "Claw servo err";
        }
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }

    public void stop() {
        if (mCraneMotor != null){
            mCraneMotor.setPower(0d);
        }
    }

    public void addCraneMovementStatusListener(ICraneMovementStatusListener listener){
        if (mCraneMoveListeners.contains(listener)){
            return;
        }
        mCraneMoveListeners.add(listener);
    }
    /**
     * raises the crane continuously with manual power
     * @param power 0 to 1.0 power to raiseManualRamp
     */
    public void raiseManual(double power){
        if (mCraneMotor == null){
            return;
        }
        // Restore default mode (if it had been changed)
        if (mCraneMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            mCraneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (power < 0d){
            mCraneMotor.setPower(0d);
            return;
        }
        else if (power > 1.0d){
            power = 1.0d;
        }
        mCraneMotor.setPower(power);
    }
    /**
     * lowers the crane continuously with manual power
     * @param power 0 to 1.0 power to raiseManualRamp
     */
    public void lowerManual(double power){
        if (mCraneMotor == null){
            return;
        }
        // Restore default mode (if it had been changed)
        if (mCraneMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            mCraneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (power < 0d){
            mCraneMotor.setPower(0d);
            return;
        }
        else if (power > 1.0d){
            power = 1.0d;
        }
        mCraneMotor.setPower(-power);
    }

    /**
     * starts a raise of the crane by a linear distance.  Call checkRaiseStatus
     * continuously to determine when the raise has finished
     * @param deltaHeight height + or - in inches to move
     */
    public void moveByEncoder(double speed, double deltaHeight){
        if (mCraneMotor == null)
            return;
        if (mCraneMovementTimeoutTimer.isRunning()){
            mCraneMovementTimeoutTimer.cancel();
            mCraneMotor.setPower(0d);
        }

        double deltaCounts = COUNTS_PER_INCH * deltaHeight;

 //       mOpMode.telemetry.addData("counts: ",deltaCounts);
//        mOpMode.telemetry.update();

        mCraneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mCraneMotor.setTargetPosition(Math.round((float)deltaCounts));
        mCraneMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         speed = Math.abs(speed);
        if (Math.abs(speed) > 1.0d)
            speed = 1.0d;
        mCraneMotor.setPower(speed);
    }

    /*
    * Must be called from opmode loop to process automatic raise/lowerByEncoder operations
     */
    public void loop(){
        if (mCraneMotor == null)
            return;
        if (mCraneMovementTimeoutTimer.isRunning()){
            if (!mCraneMotor.isBusy()){
                // Success.  Cancel timeout timer
                mCraneMovementTimeoutTimer.cancel();
                // Restore default mode
                mCraneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // And notify listeners
                for(Iterator<ICraneMovementStatusListener> iter = mCraneMoveListeners.iterator(); iter.hasNext();){
                    ICraneMovementStatusListener listener = iter.next();
                    listener.moveComplete();
                }
            }
        }
    }

    /**
     * raises the crane continuously with automatic power ramping
     */
    public void raiseManualRamp(){
        if (mCraneMotor == null){
            return;
        }
        double abspower = 0d;
        double power = mCraneMotor.getPower();
        if (power == 0d){
            // We are starting a raiseManualRamp
            mRampTimer.reset();
            abspower = RAISE_START_POWER;
         }
        else if (power < 0d) {
            // We are lowering and reversing to a raiseManualRamp
            stop();
            mRampTimer.reset();
            abspower = RAISE_START_POWER;
        }
        if (mRampTimer.time() < RAISE_MAX_POWER){
            abspower = abspower + (RAISE_POWER_SLOPE_PER_SEC * mRampTimer.time());
        }
        if (abspower > RAISE_MAX_POWER) {
            abspower = RAISE_MAX_POWER;
        }
        mCraneMotor.setPower(power);
    }
    /**
     * lowers the crane continuously
     */
    public void lowerManualRamp(){
        if (mCraneMotor == null){
            return;
        }
        double abspower = 0d;
        double power = mCraneMotor.getPower();
        if (power == 0d){
            // We are starting a lowerManualRamp
            mRampTimer.reset();
            abspower = LOWER_START_POWER;
        }
        else if (power > 0d) {
            // We are raising so reverse to a lowerManualRamp
            stop();
            mRampTimer.reset();
            abspower = LOWER_START_POWER;
        }
        if (mRampTimer.time() < LOWER_MAX_POWER){
            abspower = abspower + (LOWER_POWER_SLOPE_PER_SEC * mRampTimer.time());
        }
        if (abspower > LOWER_MAX_POWER) {
            abspower = LOWER_MAX_POWER;
        }
        // Negate power to motor for lowering
        mCraneMotor.setPower(-power);

    }

    /**
     * sets the hand position
     * @param position HAND_OPEN_POSITION, HAND_CLOSE_POSITION, or HAND_RETRACT_POSITION
     */
    public void setHandPosition(int position){
        switch(position){
            case HAND_OPEN:
                setHandServoPosition(HAND_OPEN_POSITION);
                break;
            case HAND_CLOSED:
                setHandServoPosition(HAND_CLOSE_POSITION);
                break;
            case HAND_RETRACTED:
                setHandServoPosition(HAND_RETRACT_POSITION);
                break;
        }
    }
    /**
     * returns hand position
     */
    public int getHandPosition(){
        return mHandPosition;
    }

    private void setHandServoPosition(double position){
        if (mHandServo == null){
            return;
        }
        mHandServoPosition = position;
        if (position == HAND_RETRACT_POSITION){
            mHandPosition = HAND_RETRACTED;
        }
        else if (position == HAND_CLOSE_POSITION){
            mHandPosition = HAND_CLOSED;
        }
        else{
            mHandPosition = HAND_OPEN;
        }
        mHandServo.setPosition(position);
    }
}