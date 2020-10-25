package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.SpeedBotMecanumDrive;

import java.io.File;
import java.io.IOException;

/**
 * This is the Speed Bot Chassis version used for development of the autonomous filter
 * -------------------------------------------------
 * HUB Layout:
 * -------------------------------------------------
 * Port0:  left front wheel hex motor "lf"
 * Port1:  right front wheel hex motor "rf"
 * Port2:  left rear wheel hex motor "lr"
 * Port3:  right rear wheel hex motor "rr"
 *
 */
public class BaseSpeedBot {

    protected OpMode mOpMode;

    private BNO055IMU mIMU;
    private boolean mIMUInitialized = false;

    protected SpeedBotMecanumDrive mDrivetrain = null;

    private  FrontHooks mFrontHooks = null;

    private boolean mEnableIMU = false;

    public BaseSpeedBot(OpMode opMode, boolean enableIMU){
        this.mOpMode = opMode;
        mEnableIMU = enableIMU;
     }

    public BaseMecanumDrive getDrivetrain(){
        return mDrivetrain;
    }


    public FrontHooks getFrontHooks(){
        return mFrontHooks;
    }
    /**
     *
     * @throws Exception
     */
    public void init(String imuCalibrationDataFilename) throws Exception {
        String initErrString = "";

        if (mEnableIMU){
            try{

                // Initialize the IMU
                BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

  //              mIMU.writeCalibrationData(calibrationData);

                imuParameters.mode                = BNO055IMU.SensorMode.IMU;
                imuParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
                imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//                imuParameters.calibrationData   = calibrationData;   // Doesn't work TODO:  Figure this out to reduce startup times
//            parameters.loggingEnabled      = true;
//            parameters.loggingTag          = "IMU";
                imuParameters.loggingEnabled      = false;

                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                try {
                    mIMU = mOpMode.hardwareMap.get(BNO055IMU.class, "imu");

                    mIMU.initialize(imuParameters);

                    // Now load calibrations - not sure why parameter method didn't work
                    File file = AppUtil.getInstance().getSettingsFile(imuCalibrationDataFilename);
                    BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(file));
                    mIMU.writeCalibrationData(calibrationData);

                }
                catch(Exception e){
                    throw new Exception("IMU Initialization error:"+e.getMessage());
                }

                mIMUInitialized = true;
            }
            catch(Exception e){
                initErrString += e.getMessage();
            }
        }
        try {
            mDrivetrain = new SpeedBotMecanumDrive(mOpMode);
            mDrivetrain.init(mOpMode.hardwareMap);

        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
//         try{
//             mFrontHooks = new FrontHooks(mOpMode);
//             mFrontHooks.init(mOpMode.hardwareMap);
//        }
//        catch (Exception e){
//            initErrString += e.getMessage();
//        }

        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }

    }
     public boolean isIMUInitialized(){
        return mIMUInitialized;
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

    public BNO055IMU getIMU(){
        return mIMU;
    }
}
