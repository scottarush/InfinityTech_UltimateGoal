package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter{
    private DcMotor mLeftMotor = null;
    private DcMotor mRightMotor = null;
    private Servo mPusherServo = null;

    public Shooter(){

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
        try {
            mPusherServo = ahwMap.get(Servo.class, "pusherServo");
        } catch (Exception e) {
            initErrString += ", Pusher servo error";
        }
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }
    // TODO: add an led on the robot

    public void activateShooter() {

    }
    public void deactivateShooter() {

    }

    /**
     * Shoots a ring if the shooter is ready.
     * @return True if the shooter shot a ring. False if the shooter isn't ready to shoot.
     */
    boolean shot = false;
    public boolean shoot() {
        if (shot) {
            shot = false;
            return true;
        } else {
            shot = true;
            return false;
        }
    }
}
