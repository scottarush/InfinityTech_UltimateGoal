package org.firstinspires.ftc.teamcode.shooter;

/**
 * Notification interface for the shooter used by AutonomousController to
 * listen to the shooter status.
 */
public interface IShooterListener {
    /**
     * triggered when the shooter is ready to shoot from not ready to shoot
     */
    void readyToShoot();
}
