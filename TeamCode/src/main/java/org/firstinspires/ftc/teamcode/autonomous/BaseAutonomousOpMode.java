package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.guidance.GuidanceController;
import org.firstinspires.ftc.teamcode.guidance.KalmanTracker;
import org.firstinspires.ftc.teamcode.robot.MrRingsBot;
import org.firstinspires.ftc.teamcode.util.LogFile;

/**
 * This is the main AutonomousOpMode class.  It is not extended from
 * OpMode because we don't want it to show in the menus.
 * The actual OpModes shown in the menu create an instance of this
 * class and pass themselves into the constructor.
 */
public class BaseAutonomousOpMode {
    private OpMode mOpmode = null;

    public static final String LOG_PATHNAME = "/sdcard";

    public static final boolean LOGGING_ENABLED = true;
    public static final String LOG_FILENAME = "kflog.csv";
    public static final String[] LOG_COLUMNS = {"time", "w_lf", "w_rf", "w_lr", "w_rr", "vx","vy","theta_imu",
            "kf_px", "kf_py", "kf_wz", "kf_heading",
            "mode","rot_cmd", "steering_cmd","cmd_pwr"};
    private LogFile mLogFile;


    private int mSequence = 0;

    private MrRingsBot mRingsBot = null;

    private KalmanTracker mKalmanTracker = null;
    private KalmanTracker.KalmanParameters mKalmanParameters = null;
    private Orientation mIMUOrientation;

    private GuidanceController mGuidanceController = null;

    private AutonomousController mAutonomousController = null;

    private int mReadWheelSpeedCount = 0;
    private static final int WHEEL_SPEED_SKIP_COUNT = 1;

    private boolean mSequenceStarted = false;

    /**
     *
     * @param opMode
     * @param sequence
     */
    public BaseAutonomousOpMode(OpMode opMode, int sequence){
        mOpmode = opMode;
        // lengthen init timeout to give time to initialize the IMU
        mOpmode.msStuckDetectInit = 40000;
        mSequence = sequence;
    }

    /**
     * Must be called from owning OpMode's init function.
     * @throws Exception on any initialization error
     */
    public void init() throws Exception {
        String initErrs = "";
        try {
            mRingsBot= new MrRingsBot(mOpmode, true,true);
            mRingsBot.init();
        }
        catch(Exception e){
            initErrs += ","+e.getMessage();
        }

        // Create and initialize the KalmanTracker
        mKalmanTracker = new KalmanTracker();
        mKalmanParameters = new KalmanTracker.KalmanParameters();
        mKalmanParameters.PX0 = 0d;
        mKalmanParameters.PY0 = 0d;
        mKalmanParameters.THETA0 = 0d;
        mKalmanParameters.LX = mRingsBot.getDrivetrain().getLX();
        mKalmanParameters.LY = mRingsBot.getDrivetrain().getLY();
        mKalmanParameters.WHEEL_RADIUS = mRingsBot.getDrivetrain().getWheelRadius();
        mKalmanTracker.init(mKalmanParameters);

        // Create and nitialize the guidance controller
        mGuidanceController = new GuidanceController(new GuidanceController.GuidanceControllerParameters(),
                mKalmanTracker);

        // add the drivetrain as a listener for guidance controller commands.
        mGuidanceController.addGuidanceControllerCommandListener(mRingsBot.getDrivetrain());

        // Create the autonomous state machine controller
        mAutonomousController = new AutonomousController(mOpmode,mGuidanceController, mRingsBot);

        // open the log file if enabled
        if (LOGGING_ENABLED) {
            mLogFile = new LogFile(LOG_PATHNAME, LOG_FILENAME, LOG_COLUMNS);
            mLogFile.openFile();
        }

        if (initErrs.length() > 0){
            throw new Exception(initErrs.toString());
        }
    }

    /**
     * loop function calls robot loop and also executes autonomous updates if sample time met.
     * @return true if the quantization time was met and this loop executed, false if loop call skipped
     */
    public boolean loop(){
        if (!mRingsBot.loop()){
            return false;
        }
        // Otherwise, quantization time met so compute wheelspeeds and update drivetrain
        if (mReadWheelSpeedCount >= WHEEL_SPEED_SKIP_COUNT) {
            // Service the drivetrain loop to update wheel speed measurements
            mRingsBot.getDrivetrain().loop();
            mReadWheelSpeedCount = 0;
        }
        else{
            mReadWheelSpeedCount++;
        }
        // update the tracker
        updateTracker();

        // Update the guidance controller by called updateCommand
        mGuidanceController.updateCommand();

        // Now service the controller and call it's loop function.
        mAutonomousController.loop();

        // If we haven't started then trigger the start event
        if (!mSequenceStarted){
            mAutonomousController.startSequence(AutonomousController.RINGS_SEQUENCE);
            mSequenceStarted = true;
        }


        if (LOGGING_ENABLED) {
            logData();
        }

        // And update the telemetry with the state
        mOpmode.telemetry.addLine("Current state: "+mAutonomousController.getState());

        return true;
    }
    /**
     * Must be called from OpMode's start() function.
     */
    public void start() {
        mRingsBot.start();
    }

    /**
     * Must be called from OpMode's stop() function.
     */
    public void stop() {
        if (LOGGING_ENABLED) {
            mLogFile.closeFile();
        }
        mRingsBot.stop();
    }

     /**
     * helper function to update the Tracker
     */
    private void updateTracker(){
        // Now get the IMU orientation
        mIMUOrientation = mRingsBot.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double theta_imu = mIMUOrientation.firstAngle + mKalmanParameters.THETA0;
        // Update the tracker.
        mKalmanTracker.updateMeasurement(mRingsBot.getDrivetrain().getVx(),
                mRingsBot.getDrivetrain().getVy(),
                mRingsBot.getDrivetrain().getWzw(),
                theta_imu);
    }

    /**
     * Retrieves the guidance controller for use in logging for development
     */
    public GuidanceController getGuidanceController(){
        return mGuidanceController;
    }
    /**
     * Retrieves the KalmanTracker  for use in logging for development
     */
    public KalmanTracker getKalmanTracker(){
        return mKalmanTracker;
    }

    private void logData(){
        // Now form the record for the log
        String[] logRecord = new String[LOG_COLUMNS.length];
        int logIndex = 0;
        double time = (double)mRingsBot.getElapsedTimeMS()/1e9d;
        logRecord[logIndex++] = String.format("%4.3f",time);
        // Now the speeds
        double[] speeds = mRingsBot.getDrivetrain().getWheelSpeeds();
        for(int i=0;i < speeds.length;i++){
            logRecord[logIndex++] = String.format("%4.2f",speeds[i]);
        }
        logRecord[logIndex++] = String.format("%4.2f",mKalmanTracker.getVx());
        logRecord[logIndex++] = String.format("%4.2f",mKalmanTracker.getVy());

        // IMU data
        double theta_imu = mIMUOrientation.firstAngle + mKalmanParameters.THETA0;
        logRecord[logIndex++] = String.format("%4.2f",theta_imu*180d/Math.PI);

        // Kalman outputs
        logRecord[logIndex++] = String.format("%4.2f",mKalmanTracker.getEstimatedXPosition());
        logRecord[logIndex++] = String.format("%4.2f",mKalmanTracker.getEstimatedYPosition());
        logRecord[logIndex++] = String.format("%4.2f",mKalmanTracker.getEstimatedAngularVelocity()*180d/Math.PI);
        logRecord[logIndex++] = String.format("%5.2f",mKalmanTracker.getEstimatedHeading()*180d/Math.PI);
        logRecord[logIndex++] = mGuidanceController.getModeString();
        logRecord[logIndex++] = String.format("%4.2f",mGuidanceController.getRotationCommand());
        logRecord[logIndex++] = String.format("%4.2f",mGuidanceController.getPathSteeringCommand());
        logRecord[logIndex++] = String.format("%4.2f",mGuidanceController.getPowerCommand());
        mLogFile.writeLogRow(logRecord);

    }

}
