package org.firstinspires.ftc.teamcode.shooter;

/**
 * this is the event interface used by opmodes using the ShooterController and state machine
 */
public interface IShooterController {
    /**
     * triggers the evActivate method
     */
    void evActivate();
    /**
     * triggers the evDeactivate method
     */
    void evDeactivate();
    /**
     * triggers the evShoot method
     */
    void evShoot();
}
