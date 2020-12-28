package org.firstinspires.ftc.teamcode.shooter;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

/**
 * this is the controller for the shooter state machine that coordinates the loading and firing for
 * the shooter
 */
public class ShooterController implements IShooterStatusListener, IShooterController {

    private ShooterStateMachineContext mStateMachineContext = null;
    private Shooter mShooter = null;

    private ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private OpMode mOpMode = null;

    /**
     * Common timer used to retract the loader
     */
    private OneShotTimer mTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evTimeout");
        }
    });

    /***
     * Needed to queue events into the state machine
     */
    private static HashMap<String, Method> mTransition_map;
    private LinkedList<String> mTransition_queue;

    public ShooterController(Shooter shooter, OpMode opMode) {
        mShooter = shooter;
        mOpMode = opMode;

        mStateMachineContext = new ShooterStateMachineContext(this);
        mStateTimers.add(mTimer);
        // add this class as a listener for shooter status updates.
        mShooter.addShooterStatusListener(this);

        buildTransitionTable();

        // And add a listener to the state machine to send the state transitions to telemtry
        mStateMachineContext.addStateChangeListener(new PropertyChangeListener() {
            @Override
            public void propertyChange(PropertyChangeEvent event) {
                FSMContext fsm = (FSMContext) event.getSource();
//                String propertyName = event.getPropertyName();
//                State previousStatus = (State) event.getOldValue();
                State newState = (State) event.getNewValue();
                setLogMessage(newState.getName());
            }
        });
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
    @Override
    public void shooterReady(boolean ready) {
        transition("evShooterReady");
    }

    public boolean isShooterReady(){
        return mShooter.isShooterReady();
    }

    /**
     * Called from the OpMode loop to service timers
     */
    public void loop() {

        serviceTimers();
    }

    /**
     * called from opmodes to trigger the evActivate event
     */
    public void evActivate() {
        transition("evActivate");
    }

    /**
     * called from opmodes to trigger the evDeactivate event
     */
    public void evDeactivate() {
        transition("evDeactivate");
    }

    /**
     * Called only from state machine to activate and deactivate the shooter.
     */
    public void setShooterActivation(boolean active){
        if (active) {
            mShooter.activateShooter();
        }
        else {
            mShooter.deactivateShooter();
        }
    }
    /**
     * called from opmodes to trigger the evShoot event
     */
    public void evShoot() {
        transition("evShoot");
    }

    /**
     * Starts the loader pully to shoot the ring
     */
    public void startLoaderPully() {
        mShooter.startLoaderPully();
    }

    /**
     * Stops the loader pully
     */
    public void stopLoaderPully(){
        mShooter.stopLoaderPully();
    }
    /**
     * Starts a timer that will fire the evTimeout event
     * @param timeoutms
     */
    public void startTimer(int timeoutms){
        mTimer.setTimeout(timeoutms);
        mTimer.start();
    }

    /**
     * This method just looks for all timers and checks them with the checkTimer() method
     */
    private void serviceTimers(){
        for(Iterator<OneShotTimer> iter = mStateTimers.iterator(); iter.hasNext();){
            OneShotTimer timer = iter.next();
            timer.checkTimer();
        }
    }

    public void setLogMessage(String msg){
        if (TELEMETRY_STATE_LOGGING_ENABLED) {
            mOpMode.telemetry.addData("State", msg);
            mOpMode.telemetry.update();
        }
    }
}
