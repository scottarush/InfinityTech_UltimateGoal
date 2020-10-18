package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.util.ArrayList;
import java.util.Iterator;

public abstract class Drivetrain {
    /* local OpMode members. */
    public HardwareMap mHWMap = null;

    /** OpMode in order to access telemetry from subclasses. **/
    protected OpMode mOpMode;


    public Drivetrain(OpMode opMode) {
        init(opMode);
    }


    private void init(OpMode opMode){
        this.mOpMode = opMode;
    }


    /* Initialize standard Hardware interfaces.
     * NOTE:  This class throws Exception on any hardware initIMU error so be sure to catch and
     * report to Telemetry in your initialization. */
    public abstract void init(HardwareMap ahwMap) throws Exception;


    /**
     * Sets a steering command
     * @param steering 0 = straight ahead, +1.0 max left, -1.0 max right
     * @param power -1.0..1.0 backward to forward
     */
    public abstract void setSteeringCommand(double steering,double power);

        /**
            *  Utility function to handle motor initialization.  initIMU must have been called
            *  with a non-null mHWMap or exception will be thrown.
            *
            */
    protected DcMotor tryMapMotor(String motorName) throws Exception {
        DcMotor motor = null;
        try {
            motor = mHWMap.get(DcMotor.class, motorName);
        }
        catch(Exception e){
            // Throw an exception for the caller to catch so we can debug.
            throw new Exception ("Cannot map motor: "+motorName);
        }
        return motor;
    }


}
