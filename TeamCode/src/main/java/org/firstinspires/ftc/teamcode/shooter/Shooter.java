package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter{
    // left motor is to the left when looking directly at the robot from the front and vice versa for right
    private DcMotor mLeftMotor = null;
    private DcMotor mRightMotor = null;
    private Servo mPusherServo = null;
    private boolean servoActuated = false;
    public boolean servoEnabled = true;
    public static final int PUSHER_RETRACTED = 0;
    public static final int PUSHER_EXTENDED = 1;

    public Shooter() {}

    public void disableServo() { servoEnabled = false; }

    public boolean checkServoStatus() {
        if (servoEnabled) {
            return true;
        } else {
            return false;
        }
    }
    // init the devices
    public void init(HardwareMap ahwMap) throws Exception {
        String initErrString = "";
        try {
            mLeftMotor = ahwMap.get(DcMotor.class, "shooterL");
            mLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            initErrString += "Left Shooter Motor error";
        }
        try {
            mRightMotor = ahwMap.get(DcMotor.class, "shooterR");
            mRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            initErrString += ", Right Shooter Motor error";
        }
        if (servoEnabled) {
            try {
                mPusherServo = ahwMap.get(Servo.class, "pusherServo");
                mPusherServo.setPosition(1.0d);
            } catch (Exception e) {
                initErrString += ", Pusher servo error";
            }
        } else {
            initErrString += "WARNING: PusherServo not enabled!";
        }
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }
    // TODO: add an led on the robot
    boolean readyToShoot = false;
    public void activateShooter() {
        if (readyToShoot) return;
        readyToShoot = true;
        mLeftMotor.setPower(1.0d);
        mRightMotor.setPower(1.0d);
        if (servoEnabled) {
            servoActuated = false;
            setPusher(PUSHER_RETRACTED);
        }
    }
    public void deactivateShooter() {
        if (readyToShoot == false) return;
        readyToShoot = false;
        mLeftMotor.setPower(0d);
        mLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mRightMotor.setPower(0d);
        mRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (servoEnabled) {
            servoActuated = false;
            setPusher(PUSHER_RETRACTED);
        }
    }

    /**
     * Shoots a ring if the shooter is ready.
     * @return True if the shooter shot a ring. False if the shooter isn't ready to shoot.
     */
    public boolean shoot() {
        if (readyToShoot == false) return false;
        readyToShoot = false;
        if (servoEnabled) {
            setPusher(PUSHER_EXTENDED);
        }
        return true;
    }
    public void actuateServo() {
        if (servoActuated) {
            setPusher(PUSHER_RETRACTED);
        } else {
            servoActuated = true;
            setPusher(PUSHER_EXTENDED);
        }
    }
    public void setPusher(int state) {
        float position = 0;
        switch (state) {
            case PUSHER_RETRACTED:
                position = 1.0f;
                break;
            case PUSHER_EXTENDED:
                position = -1.0f;
                break;
        }
        mPusherServo.setPosition(position);
    }
}
