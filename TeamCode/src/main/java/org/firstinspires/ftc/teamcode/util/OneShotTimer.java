package org.firstinspires.ftc.teamcode.util;

/**
 * One-shot timer that triggers a IOneShotTimerCallback after the timeout from .start occurs.
 * This class does not spawn a separate thread and must receive service after .start() via
 * repeated calls to .checkTimer()
 */
public class OneShotTimer {
    private int mTimeoutMS;
    private IOneShotTimerCallback callback;
    private long mTimeoutSystemTimeMS = 0;

    private boolean mIsRunning = false;

    public OneShotTimer(int timeoutms, IOneShotTimerCallback callback){
        this.callback = callback;
        mTimeoutMS = timeoutms;
    }

    public interface IOneShotTimerCallback {
        void timeoutComplete();
    }

    /**
     * changes the timeout to the supplied value
     */
    public void setTimeout(int timeoutms){
        mTimeoutMS = timeoutms;
    }
    /**
     * Returns true if running, false if expired
     */
    public boolean isRunning(){
        return mIsRunning;
    }

    /**
     * Gets the time since the last .start() call in ms
     */
    public int getElapsedTimeMS(){
        int starttimems = (int)(mTimeoutSystemTimeMS-mTimeoutMS);
        return (int)(System.currentTimeMillis()-starttimems);
    }
    /**
     * Cancels the timer.
     */
    public void cancel(){
        mIsRunning = false;
    }
    /**
     * Re-enables and restarts the timer.
     */
    public void start(){
        mTimeoutSystemTimeMS = System.currentTimeMillis() + mTimeoutMS;
        mIsRunning = true;
    }


    public boolean checkTimer(){
        if (mIsRunning) {
            if (System.currentTimeMillis() >= mTimeoutSystemTimeMS){
                // Disable for next time and make the callback
                mIsRunning = false;
                callback.timeoutComplete();
            }
        }
        return mIsRunning;
    }

 /**   public static void main(String[] args){
        OneShotTimer timer = new OneShotTimer(2000, new IOneShotTimerCallback() {

            @Override
            public void timeoutComplete() {
                System.out.println("complete");
            }
        });
        timer.setTimeout(3000);
        timer.start();
        System.out.println("Starting");
        while(timer.isRunning()) {
            timer.checkTimer();
        }

    }
**/
}
