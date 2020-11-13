package org.firstinspires.ftc.teamcode.hook;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class encapsulates front hook assembly that picks up rings on the front of the
 * robot.
 */
public class Hook {

    /**
     * Retracted hook position.
     */
    public static final int HOOK_POSITION_RETRACTED = 0;

    /**
     * vertical hook position.
     */
    public static final int HOOK_POSITION_VERTICAL = 1;

    /**
     * lowered hook position.
     */
    public static final int HOOK_POSITION_LOWERED = 2;

    /**
     * position for hook unknown which requires collapse to the limit switch to recalibrate.
     **/
    public static final int HOOK_POSITION_UNKNOWN = 3;
    private int mHookPosition = HOOK_POSITION_UNKNOWN;

    private Servo mClampServo = null;
    private HardwareMap mHWMap = null;

    private DcMotor mHookMotor = null;
    private OpMode mOpMode = null;

    private DigitalChannel mHookLimitSwitch = null;

    public static final String CLAMP_SERVO_NAME = "clampservo";

    public static final String HOOK_MOTOR_NAME = "hookmotor";
    public static final String HOOK_LIMIT_SWITCH_NAME = "hooklsw";
    private static final double OPEN_POSITION = 0.0d;
    private static final double CLOSED_POSITION = 1.0d;
    private double mServoPosition = OPEN_POSITION;

    public Hook(OpMode opMode) {
        opMode = mOpMode;
    }

    public void init(HardwareMap ahwMap) throws Exception {
        mHWMap = ahwMap;
        String initErrString = "";
        try {
            mClampServo = mHWMap.get(Servo.class, CLAMP_SERVO_NAME);
            mClampServo.setPosition(OPEN_POSITION);
        } catch (Exception e) {
            initErrString += "clamp hook servo err";
        }
        try {
            mHookMotor = mHWMap.get(DcMotor.class, HOOK_MOTOR_NAME);
            mHookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mHookMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } catch (Exception e) {
            initErrString += "hook motor error";
        }
        try {
            mHookLimitSwitch = mHWMap.get(DigitalChannel.class, HOOK_LIMIT_SWITCH_NAME);
            mHookLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            initErrString += "hook limit sw error";
        }

        if (initErrString.length() > 0) {
            throw new Exception(initErrString);
        }
    }

    public boolean isClampOpen() {
        if (mServoPosition == OPEN_POSITION) {
            return true;
        } else {
            return false;
        }
    }

    /**
     *
     * @return current hook position
     */
    public int getHookPosition() {
        return mHookPosition;
    }

    public void openClamp() {
        setServoPosition(OPEN_POSITION);
    }

    public void closeClamp() {
        setServoPosition(CLOSED_POSITION);
    }

    private void setServoPosition(double position) {
        mServoPosition = position;
        if (mClampServo != null) {
            mClampServo.setPosition(mServoPosition);
        }
    }

}

