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
     * Called when a straight mode maneuver is complete
     */
    void moveComplete();
    /**
     * Called when a strafe mode maneuver is complete
     */
    void strafeComplete();

}
