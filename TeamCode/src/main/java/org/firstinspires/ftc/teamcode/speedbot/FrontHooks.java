package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class encapsulates the two servos on the front of the robot.
 * left hook servo is "lhook"
 * right hook servo is "rhook"
 */
public class FrontHooks {
    private Servo mLeftServo = null;
    private Servo mRightServo = null;
    private HardwareMap mHWMap = null;
    private OpMode mOpMode = null;

    private DigitalChannel mLimitSwitch = null;

    public static final String LEFT_HOOK_SERVO_NAME = "lhook";
    public static final String RIGHT_HOOK_SERVO_NAME = "rhook";

    private static final double OPEN_POSITION = 0.0d;
    private static final double CLOSED_POSITION = 1.0d;
    private double mServoPosition = OPEN_POSITION;

    public FrontHooks(OpMode opMode){
        opMode = mOpMode;
    }

    public void init(HardwareMap ahwMap) throws Exception {
        mHWMap = ahwMap;
        String initErrString = "";
        try {
            mLeftServo = mHWMap.get(Servo.class, LEFT_HOOK_SERVO_NAME);
            mLeftServo.setPosition(OPEN_POSITION);
        } catch (Exception e) {
            initErrString += "left hook servo err";
        }
        try {
            mRightServo = mHWMap.get(Servo.class, RIGHT_HOOK_SERVO_NAME);
            mRightServo.setPosition(OPEN_POSITION);
        } catch (Exception e) {
            initErrString += "right hook servo err";
        }

        if  (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
     }

     public boolean isOpen(){
        if (mServoPosition == OPEN_POSITION){
            return true;
        }
        else{
            return false;
        }
     }

    public void openHooks(){
        setServoPosition(OPEN_POSITION);
    }
    public void closeHooks(){
        setServoPosition(CLOSED_POSITION);
    }
    private void setServoPosition(double position){
        mServoPosition = position;
        if (mLeftServo != null){
            mLeftServo.setPosition(mServoPosition);
        }
        if (mRightServo != null){
            mRightServo.setPosition(mServoPosition);
        }
    }

}

