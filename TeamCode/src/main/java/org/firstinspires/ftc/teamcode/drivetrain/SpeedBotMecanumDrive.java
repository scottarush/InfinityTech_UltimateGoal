package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is the speed bot with the 1-4 gear.
 */
public class SpeedBotMecanumDrive extends BaseMecanumDrive {



    /*
    * With core hex motors there are 288 counts/axle rotation x 1.6:1 gear output ratio
     */
    @Override
    protected int getEncoderCountsPerRev() {
        return (int) Math.round(ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION/1.6d);
    }

    /**
     * Core hex motor from the specification
     */
    public static final int ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION = 288;

    public SpeedBotMecanumDrive(OpMode opMode){
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
            mLFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            mLFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e){
            motorInitError += "lf,";
        }
        try {
            motor = tryMapMotor("rf");
//            rfMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mRFMotor = motor;
            mRFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        catch(Exception e){
            motorInitError += "rf,";
        }
        try {
            motor = tryMapMotor("lr");
//            lrMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mLRMotor = motor;
            mLRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            mLRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        catch(Exception e){
            motorInitError += "lr,";
        }
        try {
            motor = tryMapMotor("rr");
//            rrMotor = new WrappedDCMotor(motor,ENCODER_COUNTS_PER_MOTOR_SHAFT_ROTATION, PIMOTOR_KP, PIMOTOR_KI);
            mRRMotor = motor;
            mRRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        catch(Exception e){
            motorInitError += "rr,";
        }

        // Set all motors to zero power
        setPower(0, 0, 0, 0);

        // Set all motors to run without encoders for maximum power
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if (motorInitError.length() > 0){
            throw new Exception("Motor initIMU errs: '"+motorInitError+"'");
        }

    }


}
