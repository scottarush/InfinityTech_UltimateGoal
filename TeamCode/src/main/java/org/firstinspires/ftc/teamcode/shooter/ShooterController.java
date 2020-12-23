package org.firstinspires.ftc.teamcode.shooter;

import android.util.Log;

import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;

/**
 * this is the controller for the shooter state machine that coordinates the loading and firing for
 * the shooter
 */
public class ShooterController {

    private ShooterStateMachineContext mStateMachineContext = null;
    private Shooter mShooter = null;

    private ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

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

    public ShooterController(Shooter shooter) {
        mShooter = shooter;
        mStateMachineContext = new ShooterStateMachineContext(this);
        mStateTimers.add(mTimer);
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
     * Called from the opMode's init method to initialize the shooter
     */
    public void init() {
        transition("evInit");
    }
    /**
     * Called from the OpMode loop to allow the controller service timers and other functions
     */
    public void loop() {
        // If activating, then check if shooter ready
        if (mStateMachineContext.getState() == ShooterStateMachineContext.ShooterStateMachine.Activating) {
            if (mShooter.isShooterReady()) {
                transition("evReady");
            }
        }
        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
    }

    /**
     * This method starts the motors up, or is called when the shooter is not ready, called
     * by the shoot() function
     */
    public void activateShooter() {
        mShooter.setPusher(Shooter.PUSHER_RETRACTED);
    }

    /**
     * This method brakes the motors if they are spinning, and doesn't do anything if the motors
     * are not moving
     */
    public void deactivateShooter() {
        mShooter.deactivateShooter();
    }

    /**
     * This method actuates the servo and shoots a ring, or calls the activateShooter() function
     * and waits for the function to be called a second time
     */
    public void shoot() {
        mShooter.setPusher(Shooter.PUSHER_EXTENDED);
    }

    /**
     * Starts the retraction of the loader and sets a timer to trigger the end of the retraction
     */
    public void retractLoader() {
        mShooter.setPusher(Shooter.PUSHER_RETRACTED);
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
}
