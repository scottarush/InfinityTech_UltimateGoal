package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * This class encapsulates the shooter and loader.  It is primarily called by the ShooterController
 * that coordinates the activation/deactivation of the motors.
 */
public class Shooter {
    // left motor is to the left when looking directly at the robot from the front and vice versa for right
    private DcMotor mLeftMotor = null;
    private DcMotor mRightMotor = null;

    private MiniPID mLeftMotorSpeedPID = null;
    private MiniPID mRightMotorSpeedPID = null;

    private double mLeftMotorSpeed = 0d;
    private int mLastLeftMotorPosition = 0;
    private double mRightMotorSpeed = 0d;
    private int mLastRightMotorPosition = 0;

    private static final int NUM_ENCODER_COUNTS_PER_REV = 72;
    private static final double PER_NS_TO_PER_MINUTE = 1e9*60d;
    private long mLastSystemTimeNS = 0;

    private Servo mLoaderServo = null;

    public static final int LOADER_RETRACTED = 0;
    public static final int LOADER_EXTENDED = 1;

    // Shooter wheel settings
    public static final int SETTING_MIDFIELD_HIGH = 1;
    public static final int SETTING_MIDFIELD_LOW = 20;
    private int mShooterSetting = SETTING_MIDFIELD_HIGH;

    private boolean mShooterActivated = false;

    // Speed thresholds for shooter wheels in RPM
    private static final int SHOOTER_MIDFIELD_LOW_SPEED = 750;
    private static final int SHOOTER_MIDFIELD_HIGH_SPEED = 1500;

    private double mSetSpeed = 0;

    private boolean mShooterReady = false;

    private static final double SPEED_PROP_GAIN = 1d;
    private static final double SPEED_INTEGRAL_GAIN = 0d;
    private static final double SPEED_DERIVATIVE_GAIN = 0d;

    private ArrayList<IShooterStatusListener> mShooterStatusListeners = new ArrayList<>();

    private ShooterController mShooterController = null;

    /**
     * Constructor
     * @param opMode needed for state machine logging in ShooterController
     */
    public Shooter(OpMode opMode) {
        // Create the PIDs for speed control
        mLeftMotorSpeedPID = new MiniPID(SPEED_PROP_GAIN,SPEED_INTEGRAL_GAIN,SPEED_DERIVATIVE_GAIN);
        mLeftMotorSpeedPID.setOutputLimits(0d,1.0d);
        mRightMotorSpeedPID = new MiniPID(SPEED_PROP_GAIN,SPEED_INTEGRAL_GAIN,SPEED_DERIVATIVE_GAIN);
        mRightMotorSpeedPID.setOutputLimits(0d,1.0d);
        // Create the shooter controller
        mShooterController = new ShooterController(this,opMode);
    }

    public IShooterController getShooterController(){
        return mShooterController;
    }

    // init the devices
    public void init(HardwareMap ahwMap) throws Exception {
        String initErrString = "";
        try {
            mLeftMotor = ahwMap.get(DcMotor.class, "shooterL");
            mLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            initErrString += "Left Shooter Motor error";
        }
        try {
            mRightMotor = ahwMap.get(DcMotor.class, "shooterR");
            mRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            initErrString += ", Right Shooter Motor error";
        }
        try {
            mLoaderServo = ahwMap.get(Servo.class, "loader");
            mLoaderServo.setPosition(1.0d);
        } catch (Exception e) {
            initErrString += ", loader servo error";
        }
        mShooterController.init();

        if (initErrString.length() > 0) {
            throw new Exception(initErrString);
        }
    }

    public void addShooterStatusListener(IShooterStatusListener listener){
        if (!mShooterStatusListeners.contains(listener)){
            mShooterStatusListeners.add(listener);
        }
    }

    /**
     * Must be called from opmode loop to service PID speed control on the motors and
     * service the controller loop
     */
    public void serviceShooterLoop() {
        // Call the shooter controller loop first
        mShooterController.loop();

        // Now do the service for the shooter

        // Compute the delta since the last read
        long systemTime = System.nanoTime();

        // Compute the delta T and quantize to
        int deltat_ns = (int)(systemTime-mLastSystemTimeNS);
        // Compute the delta
        int position = getLeftCurrentPosition();
        double countsDelta = Math.abs(position-mLastLeftMotorPosition);
        // Convert to revolutions
        countsDelta = countsDelta / (double)NUM_ENCODER_COUNTS_PER_REV;
        // Divide by the delta and convert to Rev/Min
        mLeftMotorSpeed = countsDelta/deltat_ns * PER_NS_TO_PER_MINUTE;
        mLastLeftMotorPosition = position;

        // Now the right motor speed
        position = getRightCurrentPosition();
        countsDelta = Math.abs(getRightCurrentPosition()-mLastRightMotorPosition);
        // Convert to revolutions
        countsDelta = countsDelta / (double)NUM_ENCODER_COUNTS_PER_REV;
        // Divide by the delta and convert to Rev/Min
        mRightMotorSpeed = countsDelta/deltat_ns * PER_NS_TO_PER_MINUTE;
        mLastRightMotorPosition = position;

        // compute the power from the PID unless we are deactivated
        if (mShooterActivated){
            double left = mLeftMotorSpeedPID.getOutput(mLeftMotorSpeed,mSetSpeed);
            double right = mLeftMotorSpeedPID.getOutput(mRightMotorSpeed,mSetSpeed);
            setPower(left,right);
            // Compute and update the shooter ready status if it has changed
            updateShooterStatus();
        }

    }

    private void updateShooterStatus() {
        double highThreshold = 0;
        double lowThreshold = 0;
        double delta = 20;
        switch (mShooterSetting) {
            case SETTING_MIDFIELD_HIGH:
                highThreshold = SHOOTER_MIDFIELD_HIGH_SPEED + delta;
                lowThreshold = SHOOTER_MIDFIELD_HIGH_SPEED - delta;
                break;
            case SETTING_MIDFIELD_LOW:
                highThreshold = SHOOTER_MIDFIELD_HIGH_SPEED + delta;
                lowThreshold = SHOOTER_MIDFIELD_HIGH_SPEED - delta;
                break;
       }
        // Now check if the speeds are within the delta
        boolean left = checkSpeedThreshold(mLeftMotorSpeed,highThreshold,lowThreshold);
        boolean right = checkSpeedThreshold(mRightMotorSpeed,highThreshold,lowThreshold);
        if (left && right) {
            // shooter is ready.  check if status has changed
            if (!mShooterReady){
                mShooterReady = true;
                updateShooterStatusListeners();
            }
        }
        else{
            // not ready.  check if status changed
            if (mShooterReady){
                mShooterReady = false;
                updateShooterStatusListeners();
            }
        }
    }

    /**
     * returns whether shooter is ready or not
     */
    public boolean isShooterReady(){
        return mShooterReady;
    }

    /**
     * utility to check if a speed is within two limits
     * @param lowThreshold
     * @param highThreshold
     * @param speed
     * @return
     */
    private boolean checkSpeedThreshold(double lowThreshold,double highThreshold,double speed){
        if (speed >= lowThreshold){
            if (speed <= highThreshold){
                return true;
            }
            else{
                return false;
            }
        }
        return false;
    }

    // TODO: add an LED on the robot

    /**
     * public function to set the activation speed of the shooter
     * @param setting to either SETTING_MIDFIELD_HIGH or SETTING_MIDFIELD_LOW
     */
    public void setShooterSpeed(int setting) {
        switch (setting) {
            case SETTING_MIDFIELD_HIGH:
                mSetSpeed = SHOOTER_MIDFIELD_HIGH_SPEED;
                break;
            case SETTING_MIDFIELD_LOW:
                mSetSpeed = SHOOTER_MIDFIELD_LOW_SPEED;
                break;
            default:
                return;  // Invalid setting
        }
        mShooterSetting = setting;
    }

    public void activateShooter(){
        mShooterActivated = true;
    }

    /**
     * Public function deactivates the shooter in order to stop the wheels between shooting.
     * Stops the motors and retracts the pusher.
     */
    public void deactivateShooter() {
        mShooterActivated = false;
        setPower(0d,0d);
        setLoaderPosition(LOADER_RETRACTED);
        // If the shooter was ready than mark it not ready and notify listeners.
        if (mShooterReady){
            mShooterReady = false;
            updateShooterStatusListeners();
         }
    }

    private void updateShooterStatusListeners(){
        for(Iterator<IShooterStatusListener>iter=mShooterStatusListeners.iterator();iter.hasNext();){
            IShooterStatusListener listener = iter.next();
            listener.shooterReady(mShooterReady);
        }
    }

    private void setPower(double leftPower,double rightPower){
        if (mLeftMotor != null){
            mLeftMotor.setPower(leftPower);
        }
        if (mRightMotor != null){
            mRightMotor.setPower(rightPower);
        }
    }

    private int getLeftCurrentPosition(){
        if (mLeftMotor != null){
            return mLeftMotor.getCurrentPosition();
        }
        return 0;
    }
    private int getRightCurrentPosition(){
        if (mRightMotor != null){
            return mRightMotor.getCurrentPosition();
        }
        return 0;
    }

    public void setLoaderPosition(int state) {
        float position = 0;
        switch (state) {
            case LOADER_RETRACTED:
                position = 1.0f;
                break;
            case LOADER_EXTENDED:
                position = -1.0f;
                break;
        }
        if (mLoaderServo != null)
            mLoaderServo.setPosition(position);
    }

}
