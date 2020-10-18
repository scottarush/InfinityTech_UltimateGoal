package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;

import org.firstinspires.ftc.teamcode.guidance.GuidanceController;
import org.firstinspires.ftc.teamcode.guidance.IGuidanceControllerStatusListener;
import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;
import org.firstinspires.ftc.teamcode.speedbot.CraneSpeedBot;
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

public class AutonomousController implements IGuidanceControllerStatusListener {
    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private FilterDevStateMachineContext mStateMachineContext = null;

    private OpMode opMode = null;

    /**
     * Max rotation power.
     */
    private static final double MAX_ROTATION_POWER = 0.75d;

    /**
     * rotation timeout. same value used for all transitions
     */
    private static final int ROTATION_TIMEOUTMS = 1000;

     private BaseSpeedBot mSpeedBot = null;

    /**
     * Common reference used for the MecanumDrive on either bot
     */
    private BaseMecanumDrive mMecanumDrive = null;
    private GuidanceController mGuidanceController;

    private ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

    /**
     * Common timer used to open or close the hooks on either bot
     */
    private OneShotTimer mTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evTimeout");
        }
    });

    private OneShotTimer mRotationTimeoutTimer = new OneShotTimer(ROTATION_TIMEOUTMS, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evRotationTimeoutError");
        }
    });
    /***
     * Needed to queue events into the state machine
     */
    private static HashMap<String, Method> mTransition_map;
    private LinkedList<String> mTransition_queue;
     /**
     * Constructor
     * @param opMode
     * @param speedBot
     */
    public AutonomousController(final OpMode opMode,
                                GuidanceController guidanceController,
                                BaseSpeedBot speedBot) {
        mStateMachineContext = new FilterDevStateMachineContext(this);
        this.opMode = opMode;
        mGuidanceController = guidanceController;
        mSpeedBot = speedBot;

        mMecanumDrive = speedBot.getDrivetrain();
        mGuidanceController.addGuidanceControllerStatusListener(this);
        // Add timers to be checked
        mStateTimers.add(mTimer);

        // Now do common initializations
        init();
    }


    /**
     * Does initializations common to both robots.
     */
    private void init(){
        buildTransitionTable();


        // And add a listener to the state machine to send the state transitions to telemtry
        mStateMachineContext.addStateChangeListener(new PropertyChangeListener() {
            @Override
            public void propertyChange(PropertyChangeEvent event) {
                FSMContext fsm = (FSMContext) event.getSource();
                String propertyName = event.getPropertyName();
                State previousStatus = (State) event.getOldValue();
                State newState = (State) event.getNewValue();
                if (opMode != null){
                    opMode.telemetry.addData("Current State: ",newState.getName());
                    opMode.telemetry.update();
                }
            }
        });
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
    public void moveStraightComplete() {
        transition("evMoveStraightComplete");
    }

    /**
     * called from state machine to start a rotation using the guidance controller.
     * If the rotation fails to complete in the default rotation timeout an evRotationTimeoutError
     * will be triggered
     * @param angle rotation in degrees with positive angles to the right
     */
    public void doRotation(int angle){
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
     * called from state machine to determine if heading is OK for path follow
     * @param px x coordinate of target point
     * @param py y coordinate of target point
     */
    public boolean isPathFollowValid(double px,double py){
        return mGuidanceController.isPathFollowValid(px,py);
    }
    /**
     * called from state machine to start a path follow using the guidance controller.
     * This function assumes that isPathFollowValid has first been called to make
     * @param targetx x coordinate of target point
     * @param targety y coordinate of target point
     */
    public void doPathFollow(double targetx,double targety){
        mGuidanceController.followPath(targetx,targety);
    }

    /**
     * helper method to build the transition table so that we can trigger events from
     * within state machine handlers.
     */
    private void buildTransitionTable(){
        // Initialize the transition table for use in queueing events
        mTransition_map = new HashMap<>();
        try
        {
            Class context = mStateMachineContext.getClass();
            Method[] transitions = context.getDeclaredMethods();
            String name;
            int i;

            for (i = 0; i < transitions.length; ++i)
            {
                name = transitions[i].getName();

                // Ignore the getState and getOwner methods.
                if (name.compareTo("getState") != 0 &&
                        name.compareTo("getOwner") != 0)
                {
                    mTransition_map.put(name, transitions[i]);
                }
            }
        }
        catch (Exception ex)
        {}

        mTransition_queue = new LinkedList<>();
    }


    /**
     * Transition method needed to make transition calls within the Controller
     * @param trans_name
     */
    private synchronized void transition(String trans_name)
    {
        // Add the transition to the queue.
        mTransition_queue.add(trans_name);

        // Only if a transition is not in progress should a
        // transition be issued.
        if (mStateMachineContext.isInTransition() == false)
        {
            String name;
            Method transition;
            Object[] args = new Object[0];

            while (mTransition_queue.isEmpty() == false)
            {
                name = (String) mTransition_queue.remove(0);
                transition = (Method) mTransition_map.get(name);
                try
                {
                    transition.invoke(mStateMachineContext, args);
                }
                catch (Exception ex)
                {
                    String msg = ex.getMessage();
                    if (msg == null)
                        msg = "exception in state machine";
                    Log.e(getClass().getSimpleName(),msg);
                }
            }
        }

        return;
    }



    /**
     * Called from the OpMode loop until the state machine reaches Success
     */
    public void loop(){
        // If we haven't started then kick if off.
        boolean triggerStart = false;
        if (mStateMachineContext != null){
            if (mStateMachineContext.getState() == FilterDevStateMachineContext.FilterDevStateMachine.Idle){
                triggerStart = true;
            }
        }
        // Start event depends upon the selected sequence
        if (triggerStart){
            transition("");
          }

        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
        // Call loop method on drivetrain to support drive and rotation controls
        mMecanumDrive.loop();
    }


     /**
     * Called from state machine to stop the robot
     */
    public void stop(){
        mMecanumDrive.stop();
        if (mSpeedBot instanceof CraneSpeedBot) {
            ((CraneSpeedBot) mSpeedBot).getCrane().stop();
        }
    }

    /**
     * Starts a timer that will fire the evTimeout event
     * @param timeoutms
     */
    public void startTimer(int timeoutms){
       mTimer.setTimeout(timeoutms);
       mTimer.start();
    }
    private void serviceTimers(){
        for(Iterator<OneShotTimer> iter = mStateTimers.iterator(); iter.hasNext();){
            OneShotTimer timer = iter.next();
            timer.checkTimer();
        }
    }

    public void setLogMessage(String msg){
        if (TELEMETRY_STATE_LOGGING_ENABLED) {
            opMode.telemetry.addData("Status", msg);
            opMode.telemetry.update();
        }
    }


}

