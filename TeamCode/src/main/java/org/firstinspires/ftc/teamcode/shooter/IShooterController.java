package org.firstinspires.ftc.teamcode.shooter;

/**
 * this is the event interface used by opmodes using the ShooterController and state machine
 */
public interface IShooterController {
    /**
     * called to activate the shooter or when the speed setting has been changed by the user
     */
    void evActivate();
    /**
     * called to deactivate the shooter
     */
    void evDeactivate();
    /**
     * called to shoot
     */
    void evShoot();

    /**
     * called to indicate the shooter motor speeds are ready to shoot
     */
    void evReadyToShoot();

    /**
     * returns true if shooter is activated, false if deactivated.
     */
    boolean isActivated();

    /**
     * returns true if shooter is ready to shoot, false if not
     */
    boolean isShooterReady();
}
