package org.firstinspires.ftc.teamcode.speedbot;

/**
 * Interface implemented by listeners for status of encoder movement of the crane
 */
public interface ICraneMovementStatusListener {
    /**  completed successfully. **/
    void moveComplete();
    /** Drive session failed. **/
    void moveTimeoutFailure();
}
