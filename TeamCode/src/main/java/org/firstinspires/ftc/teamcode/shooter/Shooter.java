package org.firstinspires.ftc.teamcode.shooter;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.MiniPID;

/**
 * This class encapsulates the shooter and loader pulley.  It is primarily called by the ShooterController
 * that coordinates the activation/deactivation of the motors.
 */
public class Shooter {

    public static final boolean SHOOTER_WHEEL_SPEED_TELEMETRY_ENABLED = false;

    // left motor is to the left when looking directly at the robot from the front and vice versa for right
    private DcMotor mLeftMotor = null;
    private DcMotor mRightMotor = null;

    private MiniPID mLeftMotorSpeedPID = null;
    private MiniPID mRightMotorSpeedPID = null;

    private RevColorSensorV3 mPulleyColorSensor = null;

    private double mLeftMotorSpeed = 0d;
    private int mLastLeftMotorPosition = 0;
    private double mRightMotorSpeed = 0d;
    private int mLastRightMotorPosition = 0;

    private static final int NUM_ENCODER_COUNTS_PER_REV = 72;
    private static final double PER_NS_TO_PER_MINUTE = 1e9*60d;
    private long mLastSystemTimeNS = 0;

    private DcMotor mLoaderPulley = null;

    // Loader pulley position enumeration
    public static final int LOADER_PULLEY_POSITION_UNKNOWN = 0;
    public static final int LOADER_PULLEY_POSITION_LOW = 0;
    public static final int LOADER_PULLEY_POSITION_MIDDLE = 1;
    public static final int LOADER_PULLEY_POSITION_HIGH = 2;
    private int mLoaderPulleyPosition = LOADER_PULLEY_POSITION_LOW;

    // Encoder value for loader pulley low postion.  Assume at startup it is at zero
    private int mLoaderPulleyEncoderValueLow = 0;
    // Encoder value for loader pulley high postion.
    private int mLoaderPulleyEncoderValueHigh = 100;

    // Shooter wheel settings
    public static final int SETTING_MIDFIELD_HIGH = 1;
    public static final int SETTING_MIDFIELD_LOW = 20;
    private int mShooterDistanceSetting = SETTING_MIDFIELD_HIGH;


    // Speed thresholds for shooter wheels in RPM
    private static final int SHOOTER_MIDFIELD_LOWGOAL_WHEEL_SPEED = 475;
    private static final int SHOOTER_MIDFIELD_HIGHGOAL_WHEEL_SPEED = 800;

    // 1st order lag filter constant - same for both wheels
    private static final double SHOOTER_SPEED_LAG_FILTER_K = 1.0d;

    // Spin RPM.  + is clockwise spin, - is counterclockwise.
    private static final int SPIN_RPM = 50;

    private double mRightWheelSetSpeed = 0;
    private double mLeftWheelSetSpeed = 0;

    /**
     * Speed loop is closed in units of "RPM" where max output of 1.0 is ~1500 rpm
     * and 0 is stopped.  Gains below should be scaled with the RPM units and max range in
     * mind
     */

    private ElapsedTime runtime;

    private static final double SPEED_PROP_GAIN = 0.05d;
    private static final double SPEED_INTEGRAL_GAIN = 0d;
    private static final double SPEED_DERIVATIVE_GAIN = 0.0d;

    private IShooterController mShooterController = null;

    private OpMode mOpMode = null;

    /**
     * Constructor
     * @param opMode needed for state machine logging in ShooterController
     */
    public Shooter(OpMode opMode) {
        mOpMode = opMode;
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
            mLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            initErrString += "Left Shooter Motor error";
        }
        try {
            mRightMotor = ahwMap.get(DcMotor.class, "shooterR");
            mRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            initErrString += ", Right Shooter Motor error";
        }
        try {
            mLoaderPulley = ahwMap.get(DcMotor.class, "loader");
            mLoaderPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mLoaderPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mLoaderPulley.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            initErrString += ", Loader Pully Motor error";
        }

        if (initErrString.length() > 0) {
            throw new Exception(initErrString);
        }
    }

    /**
     * Must be called from opmode loop to service PID speed control on the motors and
     * service the controller loop
     */
    public void serviceShooterLoop() {
        // Call the shooter controller loop first
        mShooterController.loop();

        // Now the wheels
        serviceShooterWheels();
        // And the pulley
        serviceLoaderPulley();

    }

    /**
     * services the shooter wheels.   Must be called once per loop from serviceShooterLoop
     */
    private void serviceShooterWheels(){
        //----------------------------------------------
        // 1.  Compute the wheel speeds.
        //----------------------------------------------

        // Compute the delta since the last read
        long systemTime = System.nanoTime();

        // Compute the delta T and quantize to
        int deltat_ns = (int)(systemTime-mLastSystemTimeNS);
        // Save time for next time
        mLastSystemTimeNS = systemTime;

        // Compute the delta
        int position = getLeftWheelMotorCurrentPosition();
        double countsDelta = Math.abs(position-mLastLeftMotorPosition);
        // Convert to revolutions
        countsDelta = countsDelta / (double)NUM_ENCODER_COUNTS_PER_REV;
        // Divide by the delta and convert to Rev/Min
        double leftRawSpeed = countsDelta/deltat_ns * PER_NS_TO_PER_MINUTE;
        mLastLeftMotorPosition = position;
        // now filter by the lag filter
        mLeftMotorSpeed = leftRawSpeed * SHOOTER_SPEED_LAG_FILTER_K +
                (1d-SHOOTER_SPEED_LAG_FILTER_K) * mLeftMotorSpeed;


        // Now the right motor speed
        position = getRightWheelMotorCurrentPosition();
        countsDelta = Math.abs(getRightWheelMotorCurrentPosition()-mLastRightMotorPosition);
        // Convert to revolutions
        countsDelta = countsDelta / (double)NUM_ENCODER_COUNTS_PER_REV;
        // Divide by the delta and convert to Rev/Min
        double rightRawSpeed = countsDelta/deltat_ns * PER_NS_TO_PER_MINUTE;
        mLastRightMotorPosition = position;
        // now filter by the lag filter
        mRightMotorSpeed = rightRawSpeed * SHOOTER_SPEED_LAG_FILTER_K +
                (1d-SHOOTER_SPEED_LAG_FILTER_K) * mRightMotorSpeed;

        //-----------------------------------------------------------
        // Update the PID controlled speed unless we are deactivated
        //-----------------------------------------------------------
        if (mShooterController.isActivated()){
            double left = mLeftMotorSpeedPID.getOutput(mLeftMotorSpeed, mRightWheelSetSpeed);
            double right = mRightMotorSpeedPID.getOutput(mRightMotorSpeed, mRightWheelSetSpeed);
            setPower(left,right);
            // Compute and update the shooter ready status if it has changed
            updateShooterSpinStatus();
        }
        else{
            // Stop the motors
            setPower(0d,0d);
        }
        if (SHOOTER_WHEEL_SPEED_TELEMETRY_ENABLED) {
            mOpMode.telemetry.addData("Right Motor Speed", mRightMotorSpeed);
            mOpMode.telemetry.addData("Left Motor Speed", mLeftMotorSpeed);
            mOpMode.telemetry.update();
        }
    }

    private void updateShooterSpinStatus() {
        double delta = 10;
        // compute thresholds for both wheels using the set speeds
        double highRightThreshold = mRightWheelSetSpeed+delta/2;
        double lowRightThreshold = mRightWheelSetSpeed-delta/2;
        double highLeftThreshold = mLeftWheelSetSpeed+delta/2;
        double lowLeftThreshold = mLeftWheelSetSpeed-delta/2;

        // Now check if the filtered speeds are within the delta
        boolean left = checkSpeedThreshold(mLeftMotorSpeed,highLeftThreshold,lowLeftThreshold);
        boolean right = checkSpeedThreshold(mRightMotorSpeed,highRightThreshold,lowRightThreshold);
        if (left && right) {
            // shooter is ready.  trigger event to controller
            mShooterController.evReadyToShoot();
        }
        else{
            // not ready.  call evActivate to wait until speed has stabilized again
            mShooterController.evActivate();
        }
    }

    /**
     * returns current loader pulley position enumeration
     * @return LOADER_PULLY_POSITION_LOW, LOADER_PULLY_POSITION_MIDDLE, LOADER_PULLY_POSITION_MIDDLE, or LOADER_PULLY_POSITION_UNKNOWN
     */
    public int getLoaderPulleyPosition(){
        return mLoaderPulleyPosition;
    }

    /**
     * services the loader pulley.  Must be called once per loop from the serviceShooterLoop.
     */
    private void serviceLoaderPulley(){

        int lastLoaderPulleyPosition = mLoaderPulleyPosition;

        // Read the color sensors and latch encoder positions
        NormalizedRGBA colors = mPulleyColorSensor.getNormalizedColors();
        float hsvValues[] = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] >= 200) {
            // blue tape detected.
            mLoaderPulleyPosition = LOADER_PULLEY_POSITION_LOW;
            // And set the position
            mLoaderPulleyEncoderValueLow = mLoaderPulley.getCurrentPosition();
        } else if (hsvValues[0] >= 155) {
            // white tape detected
            mLoaderPulleyPosition = LOADER_PULLEY_POSITION_HIGH;
            mLoaderPulleyEncoderValueHigh = mLoaderPulley.getCurrentPosition();
        }
        else{
            // TODO.  Need to add table in the middle of the plastic sheet so we know the
            // pulley is in the middle.
            mLoaderPulleyPosition = LOADER_PULLEY_POSITION_MIDDLE;
        }
        // Now determine if this is a change and take transition action according to last
        // position
        if (lastLoaderPulleyPosition != mLoaderPulleyPosition){
            switch(mLoaderPulleyPosition){
                case LOADER_PULLEY_POSITION_LOW:
                    // For a transition into Low position, stop the pulley and notify the
                    // state machine.
                    stopLoaderPulley();
                    mShooterController.evLoaderPulleyLow();
                    break;
                case LOADER_PULLEY_POSITION_HIGH:
                    // For a transition into Low position, stop the pulley and notify the
                    // state machine.
                    stopLoaderPulley();
                    mShooterController.evLoaderPulleyHigh();
                    break;
                case LOADER_PULLEY_POSITION_MIDDLE:
                    // For a transition into interim, just notify the state machine
                    mShooterController.evLoaderPulleyMiddle();
                    break;
            }
        }
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

    // TODO: add an LED on the robot to indicate when it is ready

    /**
     * public function to set the activation speed to a calibrated position constant
     * @param setting to either SETTING_MIDFIELD_HIGH or SETTING_MIDFIELD_LOW
     */
    public void setShooterDistance(int setting) {
        // Compute the left and right speeds adding 1/2 the SPIN_RPM
        // opposite to each wheel
        switch (setting) {
            case SETTING_MIDFIELD_HIGH:
                mRightWheelSetSpeed = SHOOTER_MIDFIELD_HIGHGOAL_WHEEL_SPEED-SPIN_RPM/2;
                mLeftWheelSetSpeed = SHOOTER_MIDFIELD_HIGHGOAL_WHEEL_SPEED+SPIN_RPM/2;
                break;
            case SETTING_MIDFIELD_LOW:
                mRightWheelSetSpeed = SHOOTER_MIDFIELD_LOWGOAL_WHEEL_SPEED-SPIN_RPM/2;
                mLeftWheelSetSpeed = SHOOTER_MIDFIELD_LOWGOAL_WHEEL_SPEED+SPIN_RPM/2;
                break;
            default:
                return;  // Invalid setting
        }
        // valid change so change the setting
        mShooterDistanceSetting = setting;
        // And update shooter status
        updateShooterSpinStatus();
    }
    /**
     * public function to set the speed to a specific RPM value.
     * @param rpm center set speed
     * @param spinOffset + is clockwise, - counterclockwise
     **/
    public void setShooterSpeed(int rpm,int spinOffset) {
        // Compute the left and right speeds adding 1/2 the spinOffset to each
        mRightWheelSetSpeed = rpm-spinOffset/2;
        if (mRightWheelSetSpeed < 0) {
            mRightWheelSetSpeed = 0;
        }
        mLeftWheelSetSpeed = rpm+spinOffset/2;
        if (mLeftWheelSetSpeed < 0) {
            mLeftWheelSetSpeed = 0;
        }
        // And update shooter status
        updateShooterSpinStatus();
    }

    /**
     * Returns current shooter speed.
     */
    public int getShooterSetting(){
        return mShooterDistanceSetting;
    }

    /**
     * activates the shooter.
     */
    public void activateShooter(){
        mShooterController.evActivate();
    }

    /**
     * Public function deactivates the shooter in order to stop the wheels between shooting.
     * Stops the motors and sets the loader pulley position to LOADER_PULLY_POSITION_LOW.
     */
    public void deactivateShooter() {
        mShooterController.evDeactivate();
        setPower(0d,0d);
        setLoaderPulleyPosition(LOADER_PULLEY_POSITION_LOW);
    }

    private void setPower(double leftPower,double rightPower){
        if (mLeftMotor != null){
            mLeftMotor.setPower(leftPower);
        }
        if (mRightMotor != null){
            mRightMotor.setPower(rightPower);
        }
    }

    private int getLeftWheelMotorCurrentPosition(){
        if (mLeftMotor != null){
            return mLeftMotor.getCurrentPosition();
        }
        return 0;
    }
    private int getRightWheelMotorCurrentPosition(){
        if (mRightMotor != null){
            return mRightMotor.getCurrentPosition();
        }
        return 0;
    }

    /**
     * commands the loader pulley to move to the High position (presumably to shoot a ring).
     * @param position either LOADER_PULLY_POSITION_LOW OR LOADER_PULLY_POSITION_HIGH. Any other
     *                 value is invalid and will be ignored.
     * @return true if valid position, false if notl.
     */
    public boolean setLoaderPulleyPosition(int position){

        int targetEncoderValue = 0;
        switch(position){
            case LOADER_PULLEY_POSITION_HIGH:
                targetEncoderValue = mLoaderPulleyEncoderValueHigh;
                break;
            case LOADER_PULLEY_POSITION_LOW:
                targetEncoderValue = mLoaderPulleyEncoderValueLow;
                break;
            default:
                // invalid
                return false;
        }
        // Now do a run to encoder with the new target value.

        // If the pulley is moving then stop it.
        stopLoaderPulley();
        // And run it to the new encoder value.
        mLoaderPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLoaderPulley.setTargetPosition(targetEncoderValue);

        return true;
    }


    /**
     * Stops the loader pulley
     */
    public void stopLoaderPulley(){
        if (mLoaderPulley != null) {
            mLoaderPulley.setPower(0d);
        }
    }
    /**
     * @return true if loader pulley moving, false if not.
     */
    public boolean isLoaderPulleyMoving(){
        if (mLoaderPulley != null) {
            return (mLoaderPulley.getPower() != 0d);
        }
        return false;
    }

}
