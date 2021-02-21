package org.firstinspires.ftc.teamcode.guidance;

public interface IGuidanceControllerStatusListener {

    /**
     * called when a rotation is complete.
     */
    void rotationComplete();

    /**
     * Called when a path follow is complete.
     */
    void pathFollowComplete();

    /**
     * Called when a straight or strafe mode maneuver is complete
     */
    void moveComplete();

}
