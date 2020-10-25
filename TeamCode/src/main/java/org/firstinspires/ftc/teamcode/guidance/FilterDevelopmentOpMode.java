package org.firstinspires.ftc.teamcode.guidance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousController;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;

import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;
import org.firstinspires.ftc.teamcode.util.LogFile;

@Autonomous(name="FilterDevelopment", group="Robot")
public class FilterDevelopmentOpMode extends OpMode{
    public static final String LOG_PATHNAME = "/sdcard";

    public static final boolean LOGGING_ENABLED = true;
    public static final String LOG_FILENAME = "kflog.csv";
    public static final String[] LOG_COLUMNS = {"time", "w_lf", "w_rf", "w_lr", "w_rr", "vx","vy","theta_imu",
            "kf_px", "kf_py", "kf_wz", "kf_heading",
            "mode","rot_cmd", "steering_cmd","cmd_pwr"};
    private LogFile mLogFile;

    public static final double T = 0.050d;
    private static final int T_NS = Math.round((float)(T * 1e9d));

    private int mReadWheelSpeedCount = 0;
    private static final int WHEEL_SPEED_SKIP_COUNT = 1;

    private BaseSpeedBot mSpeedBot = null;

    private KalmanTracker mKalmanTracker = null;
    private KalmanTracker.KalmanParameters mKalmanParameters = null;
    private Orientation mIMUOrientation;
    private long mLastSystemTimeNS = 0;
    private long mElapsedTimeNS = 0;
    private long mStartTimeNS = 0;

    private GuidanceController mGuidanceController = null;

    private AutonomousController mAutonomousController;

    @Override
    public void init() {
        msStuckDetectInit = 1000000;
        mElapsedTimeNS = 0;

        String initErrs = "";
        try {
            mSpeedBot = new BaseSpeedBot(this, true);
            mSpeedBot.init("IMUCalCtrlHubOne.json");
         }
        catch(Exception e){
            initErrs += ","+e.getMessage();
        }

        if (initErrs.length() == 0){
            telemetry.addData("Status:","Robot init complete");
        }
        else {
            telemetry.addData("Init errors:", initErrs);
        }
        telemetry.addData("IMU cal status",mSpeedBot.getIMUCalibrationStatus());
        telemetry.update();

        // Initialize the KalmanTracker
        mKalmanTracker = new KalmanTracker();
        mKalmanParameters = new KalmanTracker.KalmanParameters();
        mKalmanParameters.PX0 = 0d;
        mKalmanParameters.PY0 = 0d;
        mKalmanParameters.THETA0 = 0d;
        mKalmanParameters.LX = mSpeedBot.getDrivetrain().getLX();
        mKalmanParameters.LY = mSpeedBot.getDrivetrain().getLY();
        mKalmanParameters.WHEEL_RADIUS = mSpeedBot.getDrivetrain().getWheelRadius();
        mKalmanTracker.init(mKalmanParameters);

        // Initialize the guidance controller
        mGuidanceController = new GuidanceController(new GuidanceController.GuidanceControllerParameters(),mKalmanTracker);

        // add the drivetrain as a listener for guidance controller commands
        mGuidanceController.addGuidanceControllerCommandListener(mSpeedBot.getDrivetrain());

        // Create the state machine controller
        mAutonomousController = new AutonomousController(this,mGuidanceController,mSpeedBot);
        // open the log file if enabled
        if (LOGGING_ENABLED) {
            mLogFile = new LogFile(LOG_PATHNAME, LOG_FILENAME, LOG_COLUMNS);
            mLogFile.openFile();
        }

    }

    @Override
    public void stop() {
        if (LOGGING_ENABLED) {
            mLogFile.closeFile();
        }
        super.stop();

        mSpeedBot.getDrivetrain().stop();
        mGuidanceController.closeLogFile();
    }

    @Override
    public void start() {
        mLastSystemTimeNS = System.nanoTime();
        mElapsedTimeNS = 0;
        mStartTimeNS = mLastSystemTimeNS;

//      mGuidanceController.strafe(24.0d/39.37d,1.0d);
//        mGuidanceController.rotateToHeading(90*Math.PI/180);
//        mGuidanceController.moveStraight(24d/39.37d,1.0d);
        super.start();
    }

    public void loop() {
        // Compute the delta time and update the Tracker if we are at the sample period T
        long systemTime = System.nanoTime();

        // Compute the delta T and quantize to the sample period
        int deltat_ns = (int)(systemTime-mLastSystemTimeNS);
        if (deltat_ns >= T_NS){
            if (mReadWheelSpeedCount >= WHEEL_SPEED_SKIP_COUNT) {
                // Service the drivetrain loop to update wheel speed measurements
                mSpeedBot.getDrivetrain().loop();
                mReadWheelSpeedCount = 0;
            }
            else{
                mReadWheelSpeedCount++;
            }

            mElapsedTimeNS = systemTime-mStartTimeNS;
            updateTracker();
            mLastSystemTimeNS = systemTime;   // save for next loop

            if (mSpeedBot.isIMUInitialized()) {
                // update the guidance controller commnd
                mGuidanceController.updateCommand();
             }
            // Service the autononomous controller
            mAutonomousController.loop();

  //          telemetry.addData("KF Data","heading=%5.2f px=%4.1f py=%4.1f",mKalmanTracker.getEstimatedHeading()*180d/Math.PI,mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
    //        telemetry.update();
            // Log a record of data if enabled
            if (LOGGING_ENABLED) {
                logData();
            }
        }
     }

    private void logData(){
         // Now form the record for the log
        String[] logRecord = new String[LOG_COLUMNS.length];
        int logIndex = 0;
        double time = (double)mElapsedTimeNS/1e9d;
        logRecord[logIndex++] = String.format("%4.3f",time);
        // Now the speeds
        double[] speeds = mSpeedBot.getDrivetrain().getWheelSpeeds();
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

    /**
     * helper function to update the Tracker
     */
    private void updateTracker(){
        // Now get the IMU orientation
        mIMUOrientation = mSpeedBot.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double theta_imu = mIMUOrientation.firstAngle + mKalmanParameters.THETA0;
        // Update the tracker.
        mKalmanTracker.updateMeasurement(mSpeedBot.getDrivetrain().getVx(),
                mSpeedBot.getDrivetrain().getVy(),
                mSpeedBot.getDrivetrain().getWzw(),
                theta_imu);

     }


}
