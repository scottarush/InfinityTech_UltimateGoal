package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;

import org.firstinspires.ftc.teamcode.guidance.GuidanceController;
import org.firstinspires.ftc.teamcode.guidance.IGuidanceControllerStatusListener;
import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;
import org.firstinspires.ftc.teamcode.speedbot.CraneSpeedBot;
import org.firstinspires.ftc.teamcode.util.BaseStateMachineController;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;

import statemap.FSMContext;
import statemap.State;

public class AutonomousController extends BaseStateMachineController implements IGuidanceControllerStatusListener {

    /**
     * Max rotation power.
     */
    private static final double MAX_ROTATION_POWER = 0.75d;

    /**
     * rotation timeout. same value used for all transitions
     */
    private static final int ROTATION_TIMEOUTMS = 1000;

     private BaseSpeedBot mSpeedBot = null;

    private GuidanceController mGuidanceController;

    private OneShotTimer mRotationTimeoutTimer = new OneShotTimer(ROTATION_TIMEOUTMS, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evRotationTimeoutError");
        }
    });
      /**
     * Constructor
     * @param opMode
     * @param speedBot
     */
    public AutonomousController(final OpMode opMode,
                                GuidanceController guidanceController,
                                BaseSpeedBot speedBot) {
        mGuidanceController = guidanceController;
        mSpeedBot = speedBot;

        // initialize the base class
        init(opMode,new FilterDevStateMachineContext(this));

        mGuidanceController.addGuidanceControllerStatusListener(this);
     }


    @Override
    public void rotationComplete() {
        // Cancel the timeout error timer
        mRotationTimeoutTimer.cancel();
        // And notify the state machine
        transition("evRotationComplete");
    }

    @Override
    public void pathFollowComplete() {
        transition("evPathFollowComplete");
    }

    @Override
    public void moveComplete() {
        transition("evMoveComplete");
    }

    /**
     * called from state machine to start a rotation using the guidance controller.
     * If the rotation fails to complete in the default rotation timeout an evRotationTimeoutError
     * will be triggered
     * @param angle rotation in degrees with positive angles to the right
     */
    public void rotateToHeading(int angle){
        // Convert angle to floating point radians
        double radianAngle = (double)angle * Math.PI/180d;
        mGuidanceController.rotateToHeading(radianAngle);
        mRotationTimeoutTimer.start();
    }
    /**
     * called from state machine to do a rotation to point to a target point using the
     * guidance controller.
     * @param targetx x coordinate of target point
     * @param targety y coordinate of target point
     **/
    public void rotateToTarget(double targetx,double targety){
        mGuidanceController.rotateToTarget(targetx,targety);
        mRotationTimeoutTimer.start();
    }

    /**
     * called from state machine to drive forward or rearward
     * @param distance in inches + for forward, - for rearward
     */
    public void moveStraight(double distance){
        double meters = distance / 39.37d;
        mGuidanceController.moveStraight(meters,1.0d);
    }
    /**
     * Strafes the robot left (negative distance) or right (positive distance).
     * Listeners are notified on completion via the IGuidanceControllerStatusListener interface.
     * @param distance distance to strafe in inches + for right, - for left
     **/
    public void strafe(double distance) {
        double meters = distance / 39.37d;
        mGuidanceController.strafe(meters,1.0d);
    }


    /**
     * Called from the OpMode loop until the state machine reaches Success
     */
    public void loop(){
        // If we haven't started then kick if off.
        boolean triggerStart = false;
        if (mStateMachineContext != null){
            if (((FilterDevStateMachineContext)mStateMachineContext).getState() ==
                    FilterDevStateMachineContext.FilterDevStateMachine.Idle){
                triggerStart = true;
            }
        }
        // Start event depends upon the selected sequence
        if (triggerStart){
            transition("evStartDemo");
        }

        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
     }


     /**
     * Called from state machine to stop the robot
     */
    public void stop(){
        mSpeedBot.getDrivetrain().stop();
    }

}

