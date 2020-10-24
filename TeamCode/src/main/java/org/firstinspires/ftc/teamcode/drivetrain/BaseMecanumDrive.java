/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.guidance.IGuidanceControllerCommandListener;
import org.firstinspires.ftc.teamcode.util.LogFile;

import java.util.ArrayList;

/**
 * This is NOT an opmode.
 *
 * This is the base class for a Mecanum Drive robot.  Because the motor direction configurations
 * must be different between the GrabberBot and the development frame bot, this class has
 * been made with an abstract initIMU method.  The rest of the behavior is the same between the
 * subclasses.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lowerManualRamp case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:    "lf"
 * Motor channel:  Right front drive motor:  "rf"
 * Motor channel:  Left rear drive motor:    "lr"
 * Motor channel:  Right rear drive motor:   "r"
 *
 */
public abstract class BaseMecanumDrive extends Drivetrain implements IGuidanceControllerCommandListener {

    protected DcMotor mLFMotor = null;
    protected DcMotor mRFMotor = null;
    protected DcMotor mLRMotor = null;
    protected DcMotor mRRMotor = null;

    private ArrayList<DcMotor> mMotorList = new ArrayList<>();

    /**
     * Index of LF wheel in returned wheel speed array.
     */
    public static int LF_WHEEL_ARRAY_INDEX = 0;
    /**
     * Index of RF wheel in returned wheel speed array.
     */
    public static int RF_WHEEL_ARRAY_INDEX = 1;
    /**
     * Index of LR wheel in returned wheel speed array.
     */
    public static int LR_WHEEL_ARRAY_INDEX = 2;
    /**
     * Index of RR wheel in returned wheel speed array.
     */
    public static int RR_WHEEL_ARRAY_INDEX = 3;

    private int mMotorPositions[] = new int[4];
    private double mWheelSpeeds[] = new double[4];

    public static final String[] LOG_COLUMNS = {"time","counts_lf", "w_lf","counts_rf","w_rf","counts_lr" ,"w_lr","counts_rr" ,"w_rr"};

    private boolean mFirstLoopInit = false;

    private LogFile mLogFile;
    private final static boolean ENABLE_LOGGING = false;
    private long mLastLoopTimeNS = 0;

    /**
     * Wheel circumference in inches
     **/
    public static final double MECANUM_WHEEL_CIRCUMFERENCE = 12.1211;

    /**
     * @param
     **/
    public BaseMecanumDrive(OpMode opMode) {
        super(opMode);
        // And the wheel speed log file
        if (ENABLE_LOGGING) {
            mLogFile = new LogFile("/sdcard", "slog.csv", LOG_COLUMNS);
            mLogFile.openFile();
        }
    }


    /**
     * Must be implemented by subclasses to provide the number of counts per wheel revolution
     */
    protected abstract int getEncoderCountsPerRev();

    /**
     * Must be called to update wheel speed computations
     */
    public void loop() {
        long newtime = System.nanoTime();
        if (!mFirstLoopInit) {
            mLastLoopTimeNS = newtime;
            computeWheelSpeeds(0d);
            mFirstLoopInit = true;
            return;
        }

        double deltat = (newtime-mLastLoopTimeNS)*1e-9d;
        computeWheelSpeeds(deltat);
        // Compute delta t since last computation
        mLastLoopTimeNS = newtime;  // save for next time

        if (ENABLE_LOGGING)
            logData();
    }

    /**
     * helper to compute wheel speeds called from loop
     */
    private void computeWheelSpeeds(double deltaT){
        if (!mFirstLoopInit){
            // This is the first call so initialize everything
            mMotorList.add(mLFMotor);
            mMotorList.add(mRFMotor);
            mMotorList.add(mLRMotor);
            mMotorList.add(mRRMotor);

            // so just pull the positions from the motors
            for(int i=0;i < mMotorList.size();i++){
                mMotorPositions[i] = getCurrentPosition(mMotorList.get(i));
                mWheelSpeeds[i] = 0d;
            }
            return;
        }
        // Otherwise compute the angular velocities
        for(int i=0;i < mMotorList.size();i++){
            int newpos = getCurrentPosition(mMotorList.get(i));
            double angle = (double)(newpos - mMotorPositions[i])/(double)getEncoderCountsPerRev() * 2d*Math.PI;
            mWheelSpeeds[i] = angle /deltaT;
            mMotorPositions[i] = newpos;  // Transfer to current pos array for next time
        }
 //       Log.i(getClass().getCanonicalName(),"positions=" + mMotorPositions[0]+","+mMotorPositions[1]+","+mMotorPositions[2]+","+mMotorPositions[3]);
    }

    /**
     * returns array list of wheel angular speeds in radians/sec
     * array order is:  {LF,RF,LR,RR}
     */
    public double[] getWheelSpeeds(){
        double speeds[] = new double[4];
        for(int i=0;i < mWheelSpeeds.length;i++){
            speeds[i] = mWheelSpeeds[i];
        }
        return speeds;
    }

    /**
     * private helper function for get current position.  Returns 0 if
     * null to allow fail-op operation.
     */
    private int getCurrentPosition(DcMotor motor){
        if (motor != null){
            return motor.getCurrentPosition();
        }
        return 0;
    }
    /**
     * Sets a straight command to the motors either forward or backward
     * @param power -1.0..1.0 backward to forward
     */
    @Override
    public void setStraightCommand(double power){
        double motorPower[] = new double[4];

        power = limitUnity(power);
        motorPower[0] = power;
        motorPower[1] = power;
        motorPower[2] = power;
        motorPower[3] = power;
        for(int i=0;i < mMotorList.size();i++){
            mMotorList.get(i).setPower(motorPower[i]);
        }
    }

    @Override
    public void setStrafeCommand(double power,double rotateCorrection) {
        double motorPower[] = new double[4];

        power = limitUnity(power);
        motorPower[0] = power+rotateCorrection;
        motorPower[1] = -power-rotateCorrection;
        motorPower[2] = -power+rotateCorrection;
        motorPower[3] = power-rotateCorrection;
        for(int i=0;i < mMotorList.size();i++){
            mMotorList.get(i).setPower(motorPower[i]);
        }

    }

    /**
     * Sets a composite steering command to the motors with both steering and power input
     * @param steering 0 = straight ahead, -1.0 max left, +1.0 max right
     * @param power -1.0..1.0 backward to forward
     */
    public void setSteeringCommand(double steering,double power){
        double motorPower[] = new double[4];
        // Do a rotation scaled by the steering command
        // limit steering
        if (Math.abs(steering) > 1.0d){
            steering = Math.signum(steering);
        }
        double powerAbs = Math.abs(power);
        double steering_gain = 2.0d;
        motorPower[0] = powerAbs* limitUnity(1.0d + steering_gain*steering);
        motorPower[1] = powerAbs* limitUnity(1.0d - steering_gain*steering);
        motorPower[2] = powerAbs* limitUnity(1.0d + steering_gain*steering);
        motorPower[3] = powerAbs* limitUnity(1.0d - steering_gain*steering);
        for(int i=0;i < mMotorList.size();i++){
            mMotorList.get(i).setPower(motorPower[i]);
        }
    }
    /**
     * sets the motors to rotation mode with a rotation gain onlky
     * @param rotation 0 = stop, >0..+1.0 turn to right
     *                 >0..-1.0 turn to left
     */
    @Override
    public void setRotationCommand(double rotation){
        double motorPower[] = new double[4];
        // Do a rotation scaled by the steering command
        // limit steering
        if (Math.abs(rotation) > 1.0d){
            rotation = Math.signum(rotation);
        }
        motorPower[0] = rotation;
        motorPower[1] = -rotation;
        motorPower[2] = rotation;
        motorPower[3] = -rotation;
        for(int i=0;i < mMotorList.size();i++){
            mMotorList.get(i).setPower(motorPower[i]);
        }
    }

    /**
     * helper computes power to each and limits to +/-
     */
    private double limitUnity(double number){
        if (Math.abs(number) > 1.0d){
            number = Math.signum(number);
        }
        return number;
    }
    /**
     * dumps the current encoder positions for development in same order
     * as internal motor list lf,rf,lr,rr
     */
    public int[] getEncoderPositions(){
        int retarray[] = new int[mMotorList.size()];
        for(int i = 0;i < mMotorList.size();i++){
            retarray[i] = mMotorList.get(i).getCurrentPosition();
        }
        return retarray;
    }

    /**
     * helper function to stop all motors on the robot and return motor modes to
     * manual mode if they had been changed to run to position.
     * Make sure to call base class.
     */
    public void stop() {
        setPower(0.0, 0.0, 0.0, 0.0);
        // Return motors to manual control
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (ENABLE_LOGGING) {
            mLogFile.closeFile();
        }
    }

    private void logData(){
        // Now form the record for the log
        String[] logRecord = new String[LOG_COLUMNS.length];
        int logIndex = 0;
        double time = (double)mLastLoopTimeNS/1e9d;
        logRecord[logIndex++] = String.format("%4.3f",time);
        // Now the speeds
        double[] speeds = getWheelSpeeds();
        for(int i=0;i < speeds.length;i++){
            logRecord[logIndex++] = String.format("%5d",mMotorPositions[i]);
            logRecord[logIndex++] = String.format("%4.2f",speeds[i]);
        }

        mLogFile.writeLogRow(logRecord);

    }

    /**
     * This is a helper function that takes input from a dual joy stick and computes the speed
     * of each Mecanum wheel motor.
     *
     * positive x is to the right
     * positive y is up
     *
     * @param xleft x coordinate of left stick
     * @param yleft y coordinate of left stick
     * @param xright x coordinate of right stick
     * @param yright y coordinate of right stick
     *
     **/
    public void setTankDriveJoystickInput(double xleft, double yleft, double xright, double yright) {

        double lfPower = yleft+(xleft+xright)/2;
        double rfPower = yright - (xleft+xright)/2;
        double lrPower = yleft-(xleft + xright)/2;
        double rrPower = yright + (xleft+xright)/2;

        /**
         * Now normalize the wheel speed commands:
         * Let speedmax be the maximum absolute value of the four wheel speed commands.
         * If speedmax is greater than 1, then divide each of the four wheel speed commands by speedmax.
         **/
        double speedmax = Math.abs(lfPower + rfPower + lrPower + rrPower);
        if (speedmax > 4.0){
            lfPower = lfPower /speedmax;
            rfPower = rfPower / speedmax;
            lrPower = lrPower / speedmax;
            rrPower = rrPower / speedmax;
        }
//        mOpMode.telemetry.addData("power","lf %1.2f rf %1.2f lr %1.2f rr %1.2f",lfPower,rfPower,lrPower,rrPower);
//        mOpMode.telemetry.update();

        setPower(lfPower,rfPower,lrPower,rrPower);
    }


    /**
     * Helper function to set power to the wheel drive motors
     *
     * @param lf left front motor power
     * @param rf right front motor power
     * @param lr left rear motor power
     * @param rr right rear motor power
     */
    public void setPower(double lf, double rf, double lr, double rr) {
        if (mLFMotor != null)
            mLFMotor.setPower(lf);
        if (mRFMotor != null)
            mRFMotor.setPower(rf);
        if (mLRMotor != null)
        mLRMotor.setPower(lr);
        if (mRRMotor != null)
            mRRMotor.setPower(rr);
    }

    /**
     * Helper function sets all motor modes to the same mode
     *
     * @param mode
     */
    public void setMotorModes(DcMotor.RunMode mode) {
        if (mLFMotor != null)
            mLFMotor.setMode(mode);
        if (mRFMotor != null)
            mRFMotor.setMode(mode);
        if (mLRMotor != null)
            mLRMotor.setMode(mode);
        if (mRRMotor != null)
            mRRMotor.setMode(mode);
    }

}