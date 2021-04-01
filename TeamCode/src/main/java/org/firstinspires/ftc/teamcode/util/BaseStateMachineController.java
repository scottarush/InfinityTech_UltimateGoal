package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
 * This is the base class for an SMC state machine controller that
 * has all of the common functionality needed for a Controller
 *
 */
public class BaseStateMachineController {
    protected OpMode mOpMode = null;
    protected ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

    protected statemap.FSMContext mStateMachineContext;

    protected String mCurrentStateName = "unknown";

    private boolean mInitialized = false;

    protected boolean mDebuggingEnabled = false;

    protected boolean mLoggingEnabled = false;
    private LogFile mLogFile = null;

    public static final String LOG_PATHNAME = "/sdcard";

    public static final String[] LOG_COLUMNS = {"time", "type","state","event"};
    public static final String LOG_FILENAME = "statelog.csv";

    private long mStartTimeMS = 0l;
    /**
     * Common timer
     */
    protected OneShotTimer mTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evTimeout");
        }
    });

    public BaseStateMachineController(boolean debuggingEnabled, boolean loggingEnabled){
        mDebuggingEnabled = debuggingEnabled;
        mLoggingEnabled = loggingEnabled;

    }
    /**
     * This method must be called by subclasses to service the common mTimer as well as
     * any subclass specific timers.
     */
    protected void serviceTimers(){
        for(Iterator<OneShotTimer> iter = mStateTimers.iterator(); iter.hasNext();){
            OneShotTimer timer = iter.next();
            timer.checkTimer();
        }
    }

    /**
     * must be called from the OpMode loop to service timers  if enabled.
     * Can be overridden by subclasses for additional functionality.
     */
    public void loop() {
        serviceTimers();

    }
    /**
     * Starts the common timer that will fire the evTimeout event
     * @param timeoutms
     */
    public void startTimer(int timeoutms){
        mTimer.setTimeout(timeoutms);
        mTimer.start();
    }

    /**
     * Stops the common timer if running.
     */
    public void stopTimer(){
        mTimer.cancel();
    }

    /**
     * closes the log file if logging was enabled.  Must be called my child class at shutdown.
     */
    protected void closeLogFile(){
        if (mLoggingEnabled){
            mLogFile.closeFile();
        }
    }

    /**
     * Must be called from subclass constructor to pass the stateMachineContext and opmode
     */
    protected void init(OpMode opMode, FSMContext stateMachineContext){
        if (mInitialized){
            return;
        }
        mInitialized = true;

        mOpMode = opMode;
        mStateMachineContext = stateMachineContext;
        // Add the common timer to the state timer list
        mStateTimers.add(mTimer);

        buildTransitionTable();
        // add a listener to the state machine to send the state transitions to telemtry
        mStateMachineContext.addStateChangeListener(new PropertyChangeListener() {
            @Override
            public void propertyChange(PropertyChangeEvent event) {
                FSMContext fsm = (FSMContext) event.getSource();
//                String propertyName = event.getPropertyName();
//                State previousStatus = (State) event.getOldValue();
                State newState = (State) event.getNewValue();
                mCurrentStateName = newState.getName();
                if (mDebuggingEnabled){
                    System.out.println("Transition to state:"+mCurrentStateName);
                }
                if (mLoggingEnabled){
                    String[] logRecord = new String[LOG_COLUMNS.length];
                    int logIndex = 0;
                    long elapsedTime = System.currentTimeMillis()-mStartTimeMS;
                    double mstime = (double)elapsedTime/1e3d;
                    logRecord[logIndex++] = String.format("%4.3f",mstime);
                    logRecord[logIndex++] = "State Transition";
                    logRecord[logIndex++] = mCurrentStateName;
                    logRecord[logIndex++] = "-";
                    mLogFile.writeLogRow(logRecord);
                }
            }
        });
        // open the log file if enabled
        if (mLoggingEnabled) {
            mLogFile = new LogFile(LOG_PATHNAME, LOG_FILENAME, LOG_COLUMNS);
            mLogFile.openFile();
        }
        // And initialize the starting time
        mStartTimeMS = System.currentTimeMillis();

    }
    /***
     * Needed to queue events into the state machine
     */
    private HashMap<String, Method> mTransition_map;
    private LinkedList<String> mTransition_queue;

    /**
     * Transition method needed to make transition calls within the Controller
     * @param trans_name
     */
    protected synchronized void transition(String trans_name)
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
                    if (mLoggingEnabled){
                        String[] logRecord = new String[LOG_COLUMNS.length];
                        int logIndex = 0;
                        long elapsedTime = System.currentTimeMillis()-mStartTimeMS;
                        double mstime = (double)elapsedTime/1e3d;
                        logRecord[logIndex++] = String.format("%4.3f",mstime);
                        logRecord[logIndex++] = "Event";
                        logRecord[logIndex++] = mCurrentStateName;
                        logRecord[logIndex++] = name;
                        mLogFile.writeLogRow(logRecord);
                    }
                }
                catch (Exception ex)
                {
                    if (mDebuggingEnabled){
                        ex.printStackTrace();
                    }
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

    public String getState(){
        return mCurrentStateName;
    }
}
