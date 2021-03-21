package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.MrRingsBotMecanumDrive;
import org.firstinspires.ftc.teamcode.ringdetect.RingDetector;
import org.firstinspires.ftc.teamcode.ringdetect.RingDetectorNeuralNetwork;
import org.firstinspires.ftc.teamcode.shooter.Grabber;
import org.firstinspires.ftc.teamcode.shooter.Shooter;

import java.io.File;

/**
 * This is the Mr. Rings Bot for the 2020-2021 season
 * -------------------------------------------------
 * HUB Layout:
 * -------------------------------------------------
 * Port0:  left front wheel hex motor "lf"
 * Port1:  right front wheel hex motor "rf"
 * Port2:  left rear wheel hex motor "lr"
 * Port3:  right rear wheel hex motor "rr"
 *
 */
public class MrRingsBot {

    private static final String IMU_CAL_FILE_NAME = "IMUCalCtrlHubOne.json";

    protected OpMode mOpMode;

    private BNO055IMU mIMU;
    private boolean mIMUInitialized = false;

    protected MrRingsBotMecanumDrive mDrivetrain = null;

    private boolean mEnableIMU = false;
    private Shooter mShooter = null;
    private Grabber mGrabber = null;

    private RingDetector mRingDetector = null;

    private boolean mEnableRingDetector = false;
    /**
     * Sampling time.  The loop function will try to quantize to this time.  KalmanTracker
     * must have quantized loop calls.  Should also improve stability of various PID control
     * loops with uniform time updates.
     */
    public static final double T = 0.050d;
    private static final int T_NS = Math.round((float)(T * 1e9d));

    private long mLastSystemTimeNS = 0;
    private long mElapsedTimeNS = 0;
    private long mStartTimeNS = 0;

    public MrRingsBot(OpMode opMode, boolean enableIMU,boolean enableRingDetector){
        this.mOpMode = opMode;
        mEnableIMU = enableIMU;
        mEnableRingDetector = enableRingDetector;
    }

    public BaseMecanumDrive getDrivetrain(){
        return mDrivetrain;
    }


    /**
     * Returns direct reference to the grabber
     * @return
     */
    public Grabber getGrabber(){
        return mGrabber;
    }

    /**
     * @return reference to RingDetector
     */
    public RingDetector getRingDetector(){
        return mRingDetector;
    }
    /**
     * Loop function must be called from the OpMode loop in order to service
     * the shooter and grabber loops for PID control and state machines.\
     *
     * This loop enforces the quantization time and returns true if the loop executed,
     * false if the loop skipped if the quantization time wasn't met on this call.
     * Calling OpMode should evaluate return value and only do its own loop processing
     * if this loop executed to further synchronize control.
     * @return true if the quantization time was met and this loop executed, false if loop call skipped
     */
    public boolean loop(){
        long systemTime = System.nanoTime();
        // update elapsed time
        mElapsedTimeNS = systemTime-mStartTimeNS;

        // Compute the delta T and quantize to the sample period
        int deltat_ns = (int)(systemTime-mLastSystemTimeNS);
        if (deltat_ns < T_NS) {
            return false;   // Skip this call due to quantization time not met.
        }
        // Otherwise, quantization time met.  Save last time for next time
        mLastSystemTimeNS = systemTime;

        // Call the robot service loop
        mShooter.serviceShooterLoop();
        // Call the grabber loop
        mGrabber.serviceGrabberLoop();

        // Service the ring detector if enabled
        if (mEnableRingDetector) {
            mRingDetector.serviceRingDetector();
        }

        return true;
    }

    /**
     * init must be called from OpMode to initialize the robot.
     * @throws Exception
     */
    public void init() throws Exception {
        String initErrString = "";
        // Initialize the IMU if enabled.  We only enable the IMU in Autonomous mode.
        if (mEnableIMU){
            try{
                BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
                imuParameters.mode                = BNO055IMU.SensorMode.IMU;
                imuParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
                imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                // Load calibrations from file
                File file = AppUtil.getInstance().getSettingsFile(IMU_CAL_FILE_NAME);
                BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(file));
//                imuParameters.calibrationData   = calibrationData;   // Doesn't work TODO:  Figure this out to reduce startup times
//            parameters.loggingEnabled      = true;
//            parameters.loggingTag          = "IMU";
                imuParameters.loggingEnabled      = false;

                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                mIMU = mOpMode.hardwareMap.get(BNO055IMU.class, "imu");

                mIMU.initialize(imuParameters);
                // Must write the calibrations each time - Parameter method doesn't work.
                mIMU.writeCalibrationData(calibrationData);
                mIMUInitialized = true;
            }
            catch(Exception e){
                initErrString += e.getMessage();
            }
        }
        // Initialize the ring detector if enabled
        if (mEnableRingDetector) {
            mRingDetector = new RingDetector(mOpMode);
            try {
                mRingDetector.init();
            } catch (Exception e) {
                initErrString += e.getMessage();
            }
        }


        // Create and initialize the drivetrain
        mDrivetrain = new MrRingsBotMecanumDrive(mOpMode);
        try {
            mDrivetrain.init();
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }

        // Init shooter
        mShooter = new Shooter(mOpMode);
        try {
            mShooter.init();
        } catch (Exception e) {
            initErrString += e.getMessage();
        }

        // Init grabber
        mGrabber = new Grabber(mOpMode);
        try {
            mGrabber.init();
        } catch (Exception e) {
            initErrString += e.getMessage();
        }

        // End of inits, check if there were any errors and if so, throw an exception to
        // communicate to opmode
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }
    /**
     * Must be called from OpMode's start() function
     */
    public void start() {
        mElapsedTimeNS = 0;
        mStartTimeNS = System.nanoTime();
    }

    /**
     * must be called from the opmode's stop function.
     */
    public void stop(){
        getDrivetrain().stop();
        mShooter.stop();
        // Call stop to close the log file
        if (mEnableRingDetector) {
            mRingDetector.stop();
        }
    }
    /**
     *
     * @return true if IMU successfully initialized, false otherwise
     */
    public boolean isIMUInitialized(){
        return mIMUInitialized;
    }

    /**
     * returns elapsed time in milliseconds.  Used for development logging.
     */
    public int getElapsedTimeMS(){
        return (int)Math.round((double)mElapsedTimeNS / 1000000d);
    }

    /**
     * Returns the IMU calibration status string for development
     */
    public String getIMUCalibrationStatus(){
        if (mIMUInitialized){
            return mIMU.getCalibrationStatus().toString();
        }
        else{
            return "Error:  IMU not initialized";
        }
    }

    /**
     * returns direct reference to the shooter
     * @return
     */
    public Shooter getShooter(){
        return mShooter;
    }

    public BNO055IMU getIMU(){
        return mIMU;
    }
}
