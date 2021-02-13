package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is the Mr. Rings Bot with the speed gearing wih a 60:90 up gearing output ratio
 * used for the 2020 and 2021 season.
 */
public class MrRingsBotMecanumDrive extends BaseMecanumDrive {

    /*
    * With core hex motors there are 288 counts/axle rotation x 90:60 gear output ratio
     */
    @Override
    protected int getEncoderCountsPerRev() {
        return (int) Math.round(ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION/1.7d);
    }

    @Override
    public double getWheelRadius() {
        return 0.098d/2d;
    }

    @Override
    protected double getWheelSlipFactor() {
        return 0d;
    }

    @Override
    public double getLX() {
        return 0.29d/2d;
    }

    @Override
    public double getLY() {
        return 0.275d/2d;
    }

    /**
     * Core hex motor from the specification
     */
    public static final int ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION = 288;

    public MrRingsBotMecanumDrive(OpMode opMode){
        super(opMode);
    }

    /* Initialize standard Hardware interfaces.
     * NOTE:  This class throws Exception on any hardware initIMU error so be sure to catch and
     * report to Telemetry in your initialization. */
    public void init(HardwareMap ahwMap) throws Exception {
        // Save reference to Hardware map
        mHWMap = ahwMap;

        // Define and Initialize Motors
        String motorInitError = "";
        DcMotor motor = null;
        try {
            motor = tryMapMotor("lf");
//            lfMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mLFMotor = motor;
            mLFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            mLFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e){
            motorInitError += "lf,";
        }
        try {
            motor = tryMapMotor("rf");
//            rfMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mRFMotor = motor;
            mRFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            mRFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        catch(Exception e){
            motorInitError += "rf,";
        }
        try {
            motor = tryMapMotor("lr");
//            lrMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mLRMotor = motor;
            mLRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            mLRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        catch(Exception e){
            motorInitError += "lr,";
        }
        try {
            motor = tryMapMotor("rr");
//            rrMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mRRMotor = motor;
            mRRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            mRRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        catch(Exception e){
            motorInitError += "rr,";
        }

        // Set all motors to zero power
        setPower(0, 0, 0, 0);

        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if (motorInitError.length() > 0){
            throw new Exception("Motor initIMU errs: '"+motorInitError+"'");
        }

    }


}
