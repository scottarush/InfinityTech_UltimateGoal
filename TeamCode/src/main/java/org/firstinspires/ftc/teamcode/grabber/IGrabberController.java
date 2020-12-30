package org.firstinspires.ftc.teamcode.grabber;

public interface IGrabberController {

    /**
     * initializes the GrabberController
     */
    void evInit();

    /**
     * called at the start of the position change
     */
    void evGrabberMoving();

    /**
     * called when the grabber has stopped at a new target position
     */
    void evGrabberStopped();

    /**
     * Triggered when limit switch is closed
     */
    void evLimitSwitchClosed();

    /**
     * must be called from OpMode init_loop and loop methods.
     */
    void loop();

}
