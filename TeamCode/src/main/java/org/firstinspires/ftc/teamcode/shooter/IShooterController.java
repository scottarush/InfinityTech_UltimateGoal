package org.firstinspires.ftc.teamcode.shooter;

/**
 * this is the event interface used by opmodes using the ShooterController and state machine
 */
public interface IShooterController {
    /**
     * must be called to allow ShooterController to service timers
     */
    void loop();

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
     * triggered when the loader pulley has reached the high position
     */
    void evLoaderPulleyHigh();
    /**
     * triggered when the loader pulley has reached the low position
     */
    void evLoaderPulleyLow();
    /**
     * triggered when the loader pulley has entered the middle region
     * from either extreme
     */
    void evLoaderPulleyMiddle();
    /**
     * called to indicate the shooter motor speed is stable
     */
    void evShooterSpeedReady();

    /**
     * returns true if shooter is activated, false if deactivated.
     */
    boolean isActivated();

    /**
     * returns true if shooter is ready to shoot, false if not
     */
    boolean isShooterReady();

    /**
     * returns the state as a string.
     */
    String getState();
}
