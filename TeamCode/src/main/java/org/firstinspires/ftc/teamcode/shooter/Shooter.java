package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MiniPID;

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

    private DcMotor mLoaderPully = null;

    private enum loaderPulleyStates{
        LOW, HIGH, UNKNOWN
    }
    private loaderPulleyStates loaderPulleyState;

    private final int loaderPulleyEncoderValueLow = 0;
    private final int loaderPulleyEncoderValueHigh = 100;

    // Shooter wheel settings
    public static final int SETTING_MIDFIELD_HIGH = 1;
    public static final int SETTING_MIDFIELD_LOW = 20;
    private int mShooterSetting = SETTING_MIDFIELD_HIGH;

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

    private ShooterController mShooterController = null;

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
            mLoaderPully = ahwMap.get(DcMotor.class, "loader");
            mLoaderPully.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mLoaderPully.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mLoaderPully.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            initErrString += ", Loader Pully Motor error";
        }
        // Assume starting at Low pulley state for now
        // To do: add color sensor behind shooter ramp to detect state LOW or HIGH
        loaderPulleyState = loaderPulleyStates.LOW;

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
        int position = getLeftCurrentPosition();
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
        position = getRightCurrentPosition();
        countsDelta = Math.abs(getRightCurrentPosition()-mLastRightMotorPosition);
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
            updateShooterStatus();
        }
        else{
            // Stop the motors
            setPower(0d,0d);
        }
        mOpMode.telemetry.addData("Right Motor Speed", mRightMotorSpeed);
        mOpMode.telemetry.addData("Left Motor Speed", mLeftMotorSpeed);
        mOpMode.telemetry.update();
    }

    private void updateShooterStatus() {
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
     * public function to set the activation speed of the shooter
     * @param setting to either SETTING_MIDFIELD_HIGH or SETTING_MIDFIELD_LOW
     */
    public void setShooterSpeed(int setting) {

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
        mShooterSetting = setting;
        // And update shooter status
        updateShooterStatus();
    }

    /**
     * Returns current shooter speed.
     */
    public int getShooterSetting(){
        return mShooterSetting;
    }

    /**
     * activates the shooter.
     */
    public void activateShooter(){
        mShooterController.evActivate();
    }

    /**
     * Public function deactivates the shooter in order to stop the wheels between shooting.
     * Stops the motors and retracts the pusher.
     */
    public void deactivateShooter() {
        mShooterController.evDeactivate();
        setPower(0d,0d);
        stopLoaderPulley();
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

    /**
     * Starts the loader pulley to shoot a ring
     */
    public void startLoaderPulley(){
        int currentLoaderPulleyEncoderValue;
        int lastLoaderPulleyEncoderValue;
        int targetLoaderPulleyEncoderValue;
        int threshold = 10;
        int currentEncoderDifference;
        int lastEncoderDifference;
        double power = 1.0d;
        double pauseStart;
        double pauseStop;
        double pause = 100;
        if (mLoaderPully != null) {
            targetLoaderPulleyEncoderValue = loaderPulleyEncoderValueHigh;
            currentLoaderPulleyEncoderValue = mLoaderPully.getCurrentPosition();
            currentEncoderDifference = targetLoaderPulleyEncoderValue - currentLoaderPulleyEncoderValue;
            while (currentEncoderDifference > threshold){
                mLoaderPully.setPower(power);
                lastLoaderPulleyEncoderValue = currentLoaderPulleyEncoderValue;
                lastEncoderDifference = currentEncoderDifference;
                currentLoaderPulleyEncoderValue = mLoaderPully.getCurrentPosition();
                currentEncoderDifference = targetLoaderPulleyEncoderValue - currentLoaderPulleyEncoderValue;
                // Check to see if going in the wrong direction
                if (currentEncoderDifference > lastEncoderDifference){
                    // change the sign of the power variable
                    power = -power;
                }
            }
            stopLoaderPulley();
            loaderPulleyState = loaderPulleyStates.HIGH;

            // add Pause
            pauseStart = runtime.milliseconds();
            pauseStop = pauseStart + pause;
            while (runtime.milliseconds() < pauseStop){
                // just wait
            }

            // Automatically drop the Pulley back down to low
            targetLoaderPulleyEncoderValue = loaderPulleyEncoderValueLow;
            currentLoaderPulleyEncoderValue = mLoaderPully.getCurrentPosition();
            currentEncoderDifference = targetLoaderPulleyEncoderValue - currentLoaderPulleyEncoderValue;
            while (currentEncoderDifference > threshold){
                mLoaderPully.setPower(power);
                lastLoaderPulleyEncoderValue = currentLoaderPulleyEncoderValue;
                lastEncoderDifference = currentEncoderDifference;
                currentLoaderPulleyEncoderValue = mLoaderPully.getCurrentPosition();
                currentEncoderDifference = targetLoaderPulleyEncoderValue - currentLoaderPulleyEncoderValue;
                // Check to see if going in the wrong direction
                if (currentEncoderDifference > lastEncoderDifference){
                    // change the sign of the power variable
                    power = -power;
                }
            }
            stopLoaderPulley();
            loaderPulleyState = loaderPulleyStates.LOW;
        }
    }


    /**
     * Stops the loader pulley
     */
    public void stopLoaderPulley(){
        if (mLoaderPully != null) {
            mLoaderPully.setPower(0d);
        }
    }


}
