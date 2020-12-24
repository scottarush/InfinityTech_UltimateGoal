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
public class ShooterController implements IShooterStatusListener {

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

    @Override
    public void shooterReady(boolean ready) {
        transition("evShooterReady");
    }

    public boolean isShooterReady(){
        return mShooter.isShooterReady();
    }
    /**
     * Called from the opMode's init method to initialize the shooter
     */
    public void init() {
        transition("evInit");
        // And add a listener to the state machine to send the state transitions to telemtry
        mStateMachineContext.addStateChangeListener(new PropertyChangeListener() {
            @Override
            public void propertyChange(PropertyChangeEvent event) {
                FSMContext fsm = (FSMContext) event.getSource();
//                String propertyName = event.getPropertyName();
//                State previousStatus = (State) event.getOldValue();
                State newState = (State) event.getNewValue();
                setLogMessage("CurrentState="+newState.getName());
             }
        });
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
     * Called from the state machine to activate the shooter
     */
    public void activateShooter(){
        mShooter.enableShooter();
    }

    /**
     * Called from the state machine to deactivate the shooter
     */
    public void deactivateShooter(){
        mShooter.deactivateShooter();
    }

    /**
     * This method actuates the servo and shoots a ring, or calls the activateShooter() function
     * and waits for the function to be called a second time
     */
    public void shoot() {
        mShooter.setLoaderPosition(Shooter.LOADER_EXTENDED);
    }

    /**
     * Starts the retraction of the loader and sets a timer to trigger the end of the retraction
     */
    public void retractLoader() {
        mShooter.setLoaderPosition(Shooter.LOADER_RETRACTED);
        startTimer(1000);
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
            mOpMode.telemetry.addData("Status", msg);
            mOpMode.telemetry.update();
        }
    }
}
