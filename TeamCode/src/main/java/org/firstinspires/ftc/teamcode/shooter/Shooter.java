package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.LogFile;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * This class encapsulates the shooter and loader pulley.  It is primarily called by the ShooterController
 * that coordinates the activation/deactivation of the motors.
 */
public class Shooter {

    public static final boolean WHEEL_SPEED_TELEMETRY_ENABLED = false;

    // delta RPM for acceptable shooting
    public static final double DELTA_RPM_SHOOTING_WINDOW = 50d;

    // Logging file setup for Shooter wheels
    public static final boolean LOGGING_ENABLED = false;
    public static final String LOG_PATHNAME = "/sdcard";
    public static final String LOG_FILENAME = "shooter.csv";
    public static final String[] LOG_COLUMNS = {"time", "left_set","left_raw","left_actual","left_pwrcmd","right_set","right_raw","right_actual","right_pwrcmd"};
    private LogFile mLogFile;

    // Time variables
    private long mElapsedTimeNS = 0l;
    private boolean mInitElapsedTime = true;
    private long mStartTimeNS = 0l;
    private long mLastSystemTimeNS = 0l;
    private long mDeltat_ns = 0l;

    // left motor is to the left when looking directly at the robot from the front and vice versa for right
    private DcMotor mLeftMotor = null;
    private DcMotor mRightMotor = null;

    private MiniPID mLeftMotorSpeedPID = null;
    private MiniPID mRightMotorSpeedPID = null;

    private RevColorSensorV3 mPulleyColorSensor = null;
    private RevTouchSensor mPulleyHighTouchSensor = null;
    private RevTouchSensor mPulleyLowTouchSensor = null;

    private double mLeftWheelSpeed = 0d;
    private double mLeftRawSpeed = 0d;
    private int mLastLeftMotorPosition = 0;
    private double mLeftCommandedPower = 0d;

    private double mRightWheelSpeed = 0d;
    private double mRightRawSpeed = 0d;
    private int mLastRightMotorPosition = 0;
    private double mRightCommandedPower = 0d;

    private static final int NUM_ENCODER_COUNTS_PER_REV = 72;
    private static final double PER_NS_TO_PER_MINUTE = 1e9*60d;

    private DcMotor mLoaderPulley = null;

    // Loader pulley position enumeration
    public static final int LOADER_PULLEY_POSITION_LOW = 0;
    public static final int LOADER_PULLEY_POSITION_MIDDLE = 1;
    public static final int LOADER_PULLEY_POSITION_HIGH = 2;

    // Loader pulley state enumeration
    public static final int LOADER_PULLEY_STATE_READY = 0;
    public static final int LOADER_PULLEY_STATE_SHOOTING = 1;
    public static final int LOADER_PULLEY_STATE_SHOT = 2;
    public static final int LOADER_PULLEY_STATE_RETURNING = 3;
    private int mLoaderPulleyState;

    private int mLoaderPulleyCurrentPosition = LOADER_PULLEY_POSITION_LOW;
    private int mLoaderPulleyTargetPostion = LOADER_PULLEY_POSITION_LOW;

    private static final double LOADER_PULLY_POWER = 0.4d;

    // Delta in encoder counts between loader pulley position High and Low
    private static final int LOADER_PULLEY_ENCODER_VALUE_LOW_TO_HIGH_DELTA = 145;

    private int mLoaderPulleyEncoderValueLow = 0;
    private int mLoaderPulleyEncoderValueHigh = LOADER_PULLEY_ENCODER_VALUE_LOW_TO_HIGH_DELTA;
    private int mLoaderPulleyEncoderValueCurrent;
    private double power = 0.5;

    /**
     * Speed setting for dumping into the low goal from right by the low goal
     */
    public static final int SPEED_SETTING_DUMP = 0;
    /**
     * Speed setting for hitting the low goal from just behind the midfield white line.
     */
    public static final int SPEED_SETTING_MIDFIELD_LOW_GOAL = 1;
    /*
     * Speed setting for hitting the high goal from just behind the midfield white line.
     */
    public static final int SPEED_SETTING_MIDFIELD_HIGH_GOAL = 2;

    /*
     * Speed setting for hitting the power shot just behind the midfield white line.
     */
    public static final int SPEED_SETTING_MIDFIELD_POWER_SHOT = 3;

    private int mShooterDistanceSetting = SPEED_SETTING_MIDFIELD_HIGH_GOAL;

    /**
     * From 04MAR21 Meeting, here are the best values from the white line:
     * Low goal:  300 w/o spin
     * Mid goal:  600 w/o spin
     * High goal:  665 w/o spin
     * Power shot:  630 w/ 50 rpm clockwise spin
     */
    private static final int[] SHOOTER_SETTING_SPEEDS = new int[]{300,600,665,630};

    // Spin offsets in total RPM, + is clockwise, - is counterclockwise
    private static final int[] SHOOTER_SETTING_SPIN_OFFSET = new int[]{0,0,0,50};

    // 1st order lag filter constant - same for both wheels
    private static final double SHOOTER_SPEED_LAG_FILTER_K = 1.0d;

    private double mRightWheelSetSpeed = 0;
    private double mLeftWheelSetSpeed = 0;

    /**
     * Speed loop is closed in units of "RPM" where max output of 1.0 is ~1500 rpm
     * and 0 is stopped.  Gains below should be scaled with the RPM units and max range in
     * mind
     */

    private ElapsedTime runtime;

    private static final double SPEED_PROP_GAIN = 0.0001d;
    private static final double SPEED_INTEGRAL_GAIN = 0.0001d;
    private static final double SPEED_DERIVATIVE_GAIN = 0.0005d;

    private ShooterController mShooterController = null;

    private OpMode mOpMode = null;

    private ArrayList<IShooterListener> mShooterListeners = new ArrayList<>();
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

    // init the devices
    public void init() throws Exception {
        String initErrString = "";
        try {
            mLeftMotor = mOpMode.hardwareMap.get(DcMotor.class, "shooterL");
            mLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Float to reduce stress on output bearing
            mLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            initErrString += "Left Shooter Motor error";
        }
        try {
            mRightMotor = mOpMode.hardwareMap.get(DcMotor.class, "shooterR");
            mRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            initErrString += ", Right Shooter Motor error";
        }
        try {
            mLoaderPulley = mOpMode.hardwareMap.get(DcMotor.class, "loader");
            mLoaderPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Set initial state
            mLoaderPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // with motor stopped and encoder set to zero, set the EncoderValueLow variable
            mLoaderPulleyEncoderValueCurrent = mLoaderPulley.getCurrentPosition();
            mLoaderPulleyEncoderValueLow = mLoaderPulleyEncoderValueCurrent;
            //
            mLoaderPulley.setDirection(DcMotor.Direction.REVERSE);
            mLoaderPulley.setTargetPosition(mLoaderPulleyEncoderValueLow);
            mLoaderPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mLoaderPulleyState = LOADER_PULLEY_STATE_READY;
        } catch (Exception e) {
            initErrString += ", Loader Pulley Motor error";
        }
        try {
            mPulleyColorSensor = mOpMode.hardwareMap.get(RevColorSensorV3.class, "pulleyClr");
        } catch (Exception e) {
            initErrString += ", pulley color sensor init err";
        }
        try {
            mPulleyHighTouchSensor = mOpMode.hardwareMap.get(RevTouchSensor.class,"highlsw");
        } catch (Exception e) {
            initErrString += ", High Touch Sensor init err";
        }
        try {
            mPulleyLowTouchSensor = mOpMode.hardwareMap.get(RevTouchSensor.class,"lowlsw");
        } catch (Exception e) {
            initErrString += ", Low Touch Sensor init err";
        }
         // open the log file if enabled
        if (LOGGING_ENABLED) {
            mLogFile = new LogFile(LOG_PATHNAME, LOG_FILENAME, LOG_COLUMNS);
            mLogFile.openFile();
        }
        // Initialize set speeds
        setShooterDistance(mShooterDistanceSetting);
        if (initErrString.length() > 0) {
            throw new Exception(initErrString);
        }


    }

    /**
     * Must be called at shutdown to save the logging file if enabled.
     */
    public void stop(){
        if (LOGGING_ENABLED) {
            mLogFile.closeFile();
        }
        stopLoaderPulley();
        deactivateShooter();
    }

    /**
     * Must be called from robot's/OpMode's loop to service PID speed control on the motors and
     * service the controller loop
     */
    public void serviceShooterLoop() {
        // Call the shooter controller loop first
        mShooterController.loop();

        //  update the elapsed time for logging and compute the mDeltaT
        if (mInitElapsedTime){
            mElapsedTimeNS = 0l;
            mStartTimeNS = System.nanoTime();
            mLastSystemTimeNS = mStartTimeNS;
            mDeltat_ns = 20000l;   // Initialize to dummy default of 20 ms for first iteration
            mInitElapsedTime = false;
        }
        else {
            // Just update the times
            mElapsedTimeNS = System.nanoTime() - mStartTimeNS;
            mDeltat_ns = System.nanoTime()-mLastSystemTimeNS;
            mLastSystemTimeNS = System.nanoTime();
        }

        // Now service the wheels
        serviceShooterWheels();
        // And the pulley
        // Changed to New code:
        serviceLoaderPulley();

        // And log data for this cycle if enabled
        if (LOGGING_ENABLED){
            logData();
        }
    }

    /**
     * services the shooter wheels.   Must be called once per loop from serviceShooterLoop
     * prior to entry the mDeltat_ns must be updated.
     */
    private void serviceShooterWheels(){
        //----------------------------------------------
        // 1.  Compute the wheel speeds.
        //----------------------------------------------
        // Compute the left motor speed and quantize to an integer RPM
        int position = getLeftWheelMotorCurrentPosition();
        double countsDelta = Math.abs(position-mLastLeftMotorPosition);
        // Convert to revolutions
        countsDelta = countsDelta / (double)NUM_ENCODER_COUNTS_PER_REV;
        // Divide by the delta and convert to Rev/Min
        mLeftRawSpeed = countsDelta/ (double)mDeltat_ns * PER_NS_TO_PER_MINUTE;
        mLastLeftMotorPosition = position;
        // now filter by the lag filter
        mLeftWheelSpeed = mLeftRawSpeed * SHOOTER_SPEED_LAG_FILTER_K +
                (1d-SHOOTER_SPEED_LAG_FILTER_K) * mLeftWheelSpeed;
        mLeftWheelSpeed = Math.round(mLeftWheelSpeed);

        // Now the right motor speed
        position = getRightWheelMotorCurrentPosition();
        countsDelta = Math.abs(getRightWheelMotorCurrentPosition()-mLastRightMotorPosition);
        // Convert to revolutions
        countsDelta = countsDelta / (double)NUM_ENCODER_COUNTS_PER_REV;
        // Divide by the delta and convert to Rev/Min
        mRightRawSpeed = countsDelta/ (double)mDeltat_ns * PER_NS_TO_PER_MINUTE;
        mLastRightMotorPosition = position;
        // now filter by the lag filter
        mRightWheelSpeed = mRightRawSpeed * SHOOTER_SPEED_LAG_FILTER_K +
                (1d-SHOOTER_SPEED_LAG_FILTER_K) * mRightWheelSpeed;
        mRightWheelSpeed = Math.round(mRightWheelSpeed);

        //-----------------------------------------------------------
        // Update the PID controlled speed unless we are deactivated
        //-----------------------------------------------------------
        if (mShooterController.isActivated()){
            double left = mLeftMotorSpeedPID.getOutput(mLeftWheelSpeed, mLeftWheelSetSpeed);
            double right = mRightMotorSpeedPID.getOutput(mRightWheelSpeed, mRightWheelSetSpeed);
            setWheelMotorPower(left,right);
            // Compute and update the shooter ready status if it has changed
            double delta = DELTA_RPM_SHOOTING_WINDOW;
            // compute thresholds for both wheels using the set speeds
            double highRightThreshold = mRightWheelSetSpeed+delta/2;
            double lowRightThreshold = mRightWheelSetSpeed-delta/2;
            double highLeftThreshold = mLeftWheelSetSpeed+delta/2;
            double lowLeftThreshold = mLeftWheelSetSpeed-delta/2;

            // Now check if the filtered speeds are within the delta
            boolean leftOK = checkSpeedThreshold(lowLeftThreshold,highLeftThreshold, mLeftWheelSpeed);
            boolean rightOK = checkSpeedThreshold(lowRightThreshold,highRightThreshold, mRightWheelSpeed);
            if (leftOK && rightOK) {
                // shooter is ready.  trigger event to controller
                mShooterController.evShooterSpeedReady();
                notifyShooterReadyToShoot();
            }
            else{
                // not ready.  call evActivate to wait until speed has stabilized again
                mShooterController.evActivate();
            }
        }
        else{
            // Stop the motors
            setWheelMotorPower(0d,0d);
        }
        if (WHEEL_SPEED_TELEMETRY_ENABLED) {
            mOpMode.telemetry.addData("Right Motor Set Speed", mRightWheelSetSpeed);
            mOpMode.telemetry.addData("Right Motor Actual Speed", mRightWheelSpeed);

            mOpMode.telemetry.addData("Left Motor Set Speed", mLeftWheelSetSpeed);
            mOpMode.telemetry.addData("Left Motor Actual Speed", mLeftWheelSpeed);

            mOpMode.telemetry.addData("ShooterState",mShooterController.getState());
            mOpMode.telemetry.update();
        }
    }

    /**
     * returns current loader pulley position enumeration
     * @return LOADER_PULLY_POSITION_LOW, LOADER_PULLY_POSITION_MIDDLE, LOADER_PULLY_POSITION_MIDDLE, or LOADER_PULLY_POSITION_UNKNOWN
     */
    public int getLoaderPulleyPosition(){
        return mLoaderPulleyCurrentPosition;
    }

    private void serviceLoaderPulley(){
        // Read the pulley motor encoder value
        mLoaderPulleyEncoderValueCurrent = mLoaderPulley.getCurrentPosition();

        switch (mLoaderPulleyState){
            case LOADER_PULLEY_STATE_READY:
                if (mLoaderPulleyCurrentPosition == LOADER_PULLEY_POSITION_LOW && mLoaderPulleyTargetPostion == LOADER_PULLEY_POSITION_HIGH) {
                    // shoot();
                    mLoaderPulley.setDirection(DcMotorSimple.Direction.REVERSE);
                    mLoaderPulley.setTargetPosition(mLoaderPulleyEncoderValueHigh);
                    mLoaderPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mLoaderPulley.setPower(power);
                    mLoaderPulleyState = LOADER_PULLEY_STATE_SHOOTING;
                    mShooterController.evLoaderPulleyMiddle();
                }
                break;
            case LOADER_PULLEY_STATE_SHOOTING:
                if (mPulleyHighTouchSensor.isPressed()){
                    // We've hit the high touch sensor.  Latch the pulley high touch sensor position
                    // to absorb drift.
                    mLoaderPulleyEncoderValueHigh = mLoaderPulley.getCurrentPosition();
                    // And update the low position as a delta from the high
                    mLoaderPulleyEncoderValueLow = mLoaderPulleyEncoderValueHigh- LOADER_PULLEY_ENCODER_VALUE_LOW_TO_HIGH_DELTA;
                    // Stop the motor
                    mLoaderPulley.setPower(0.0);
                    // Update the loader pulley state
                    mLoaderPulleyState = LOADER_PULLEY_STATE_SHOT;
                    // And send the event to the shooter controller that the pulley reach high
                    mShooterController.evLoaderPulleyHigh();
                }
                else if (!mLoaderPulley.isBusy()) {
                    // we reached the target without hitting the switch so just shut off the
                    // pulley.
                    mLoaderPulley.setPower(0.0);
                    // Update the loader pulley state
                    mLoaderPulleyState = LOADER_PULLEY_STATE_SHOT;
                    mShooterController.evLoaderPulleyHigh();
                }
                break;
            case LOADER_PULLEY_STATE_SHOT:
                mLoaderPulleyTargetPostion = LOADER_PULLEY_POSITION_LOW;
                mLoaderPulley.setDirection(DcMotorSimple.Direction.FORWARD);
                mLoaderPulley.setTargetPosition(mLoaderPulleyEncoderValueLow);
                mLoaderPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mLoaderPulley.setPower(power);
                mLoaderPulleyState = LOADER_PULLEY_STATE_RETURNING;
                mShooterController.evLoaderPulleyMiddle();
                break;
            case LOADER_PULLEY_STATE_RETURNING:
                if (!mLoaderPulley.isBusy()) {
                    mLoaderPulley.setPower(0.0);
                    mLoaderPulleyState = LOADER_PULLEY_STATE_READY;
                    mShooterController.evLoaderPulleyLow();
                }
                break;
            default:
                break;
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
     * @return true on success, false on invalid setting
     */
    public boolean setShooterDistance(int setting) {
        if ((setting < 0) || (setting >= SHOOTER_SETTING_SPEEDS.length)){
            return false;
        }
        // valid change so change the setting
        mShooterDistanceSetting = setting;
        // Compute the left and right speeds adding 1/2 the SPIN_RPM
        // opposite to each wheel
        mRightWheelSetSpeed = SHOOTER_SETTING_SPEEDS[mShooterDistanceSetting]-
                SHOOTER_SETTING_SPIN_OFFSET[mShooterDistanceSetting]/2;
        mLeftWheelSetSpeed = SHOOTER_SETTING_SPEEDS[mShooterDistanceSetting]+
                SHOOTER_SETTING_SPIN_OFFSET[mShooterDistanceSetting]/2;

        return true;
    }
//    /**
//     * public function to set the speed to a specific RPM value.
//     * @param rpm center set speed
//     * @param spinOffset + is clockwise, - counterclockwise
//     **/
//    public void setShooterSpeed(int rpm,int spinOffset) {
//        // Compute the left and right speeds adding 1/2 the spinOffset to each
//        mRightWheelSetSpeed = rpm-spinOffset/2;
//        if (mRightWheelSetSpeed < 0) {
//            mRightWheelSetSpeed = 0;
//        }
//        mLeftWheelSetSpeed = rpm+spinOffset/2;
//        if (mLeftWheelSetSpeed < 0) {
//            mLeftWheelSetSpeed = 0;
//        }
//    }

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
     * triggers an automated shooter
     */
    public void shoot(){
        mLoaderPulleyTargetPostion = LOADER_PULLEY_POSITION_HIGH;
        mShooterController.evShoot();
    }

    /**
     * Public function deactivates the shooter in order to stop the wheels between shooting.
     * Stops the motors and sets the loader pulley position to LOADER_PULLY_POSITION_LOW.
     */
    public void deactivateShooter() {
        mShooterController.evDeactivate();
        setWheelMotorPower(0d,0d);
        setLoaderPulleyPosition(LOADER_PULLEY_POSITION_LOW);
    }

    private void setWheelMotorPower(double leftPower, double rightPower){
        mLeftCommandedPower = leftPower;
        mRightCommandedPower = rightPower;
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
        switch(position){
            case LOADER_PULLEY_POSITION_HIGH:
                mLoaderPulleyTargetPostion = position;
                break;
            case LOADER_PULLEY_POSITION_LOW:
                mLoaderPulleyTargetPostion = position;
                break;
            default:
                return false;
        }
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
    /*
        Return loader pulley to LOW position
     */
    public void goToLoaderPulleyPositionLow(){
        if (mLoaderPulley != null){
            mLoaderPulleyTargetPostion = LOADER_PULLEY_POSITION_LOW;
            mLoaderPulley.setDirection(DcMotorSimple.Direction.FORWARD);
            mLoaderPulley.setTargetPosition(mLoaderPulleyEncoderValueLow);
            mLoaderPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mLoaderPulley.setPower(LOADER_PULLY_POWER);
        }
    }
    /**
     * @return true if loader pulley moving, false if not.
     */
    public boolean isLoaderPulleyMoving(){
        if (mLoaderPulley != null) {
            return (mLoaderPulley.isBusy());
        }
        return false;
    }

    /**
     * @return true if shooter active
     */
    public boolean isShooterActive(){
        return mShooterController.isActivated();
    }

    /**
     * @return true if shooter ready to shooter
     */
    public boolean isShooterReady(){
        return mShooterController.isShooterReady();
    }

    private void logData(){
        // Now form the record for the log
        String[] logRecord = new String[LOG_COLUMNS.length];
        int logIndex = 0;
        double time = (double)mElapsedTimeNS/1e9d;
        logRecord[logIndex++] = String.format("%4.3f",time);
        logRecord[logIndex++] = String.format("%4.0f",mLeftWheelSetSpeed);
        logRecord[logIndex++] = String.format("%4.0f",mLeftWheelSpeed);
        logRecord[logIndex++] = String.format("%4.0f",mLeftRawSpeed);
        logRecord[logIndex++] = String.format("%4.2f",mLeftCommandedPower);

        logRecord[logIndex++] = String.format("%4.0f",mRightWheelSetSpeed);
        logRecord[logIndex++] = String.format("%4.0f",mRightWheelSpeed);
        logRecord[logIndex++] = String.format("%4.0f",mRightRawSpeed);
        logRecord[logIndex++] = String.format("%4.2f",mRightCommandedPower);
        mLogFile.writeLogRow(logRecord);
    }

    /**
     * notifies listeners of shooter ready to shoot
     */
    private void notifyShooterReadyToShoot(){
        for(Iterator<IShooterListener>iter=mShooterListeners.iterator();iter.hasNext();){
            IShooterListener listener = iter.next();
            listener.readyToShoot();
        }
    }
    /**
     * adds a listener for shooter status
     */
    public void addShooterListener(IShooterListener listener){
        if (!mShooterListeners.contains(listener)){
            mShooterListeners.add(listener);
        }
    }
}

