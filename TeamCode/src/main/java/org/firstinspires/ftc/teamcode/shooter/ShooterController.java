package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.BaseStateMachineController;

/**
 * this is the controller for the shooter state machine that coordinates the loading and firing for
 * the shooter
 */
public class ShooterController extends BaseStateMachineController {

    private Shooter mShooter = null;

    private static final boolean DEBUGGING_ENABLED = false;

    public ShooterController(Shooter shooter, OpMode opMode) {
        super(DEBUGGING_ENABLED);
        mShooter = shooter;
        // initialize the base class
        init(opMode,new ShooterStateMachineContext(this));
    }

    public boolean isShooterReady() {
        ShooterStateMachineContext shooterContext = (ShooterStateMachineContext)mStateMachineContext;
        if (shooterContext.getState() == ShooterStateMachineContext.ShooterStateMachine.ReadyToShoot){
            return true;
        }
        else{
            return false;
        }

    }

    public void evShooterSpeedReady() {
        transition("evShooterSpeedReady");
    }

    /**
     * called from opmodes to trigger the evActivate event
     */
    public void evActivate() {
        transition("evActivate");
    }

    public boolean isActivated() {
        ShooterStateMachineContext shooterContext = (ShooterStateMachineContext)mStateMachineContext;
        if (shooterContext.getState() == ShooterStateMachineContext.ShooterStateMachine.Deactivated){
            return false;
        }
        else{
            return true;
        }
    }


    /**
     * called from opmodes to trigger the evDeactivate event
     */
    public void evDeactivate() {
        transition("evDeactivate");
    }

    /**
     * called from opmodes to trigger the evShoot event
     */
    public void evShoot() {
        transition("evShoot");
    }

    public void evLoaderPulleyHigh() {
        transition("evLoaderPulleyHigh");
    }

    public void evLoaderPulleyLow() {
        transition("evLoaderPulleyLow");

    }

    public void evLoaderPulleyMiddle() {
        transition("evLoaderPulleyMiddle");
    }

    /**
     * sets the loader pulley position to High (to shoot the ring)
     */
    public void moveLoaderToHigh() {
        mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_HIGH);
    }
    /**
     * sets the loader pulley position to Low
     */
    public void moveLoaderToLow() {
        mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_LOW);
    }

    /**
     * Stops the loader pully
     */
    public void stopLoaderPully(){
        mShooter.stopLoaderPulley();
    }

    /**
     * Sets the loader position to the Low position
     */
    public void setLoaderPulleyLow() {
        mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_LOW);
    }
    /**
     * Sets the loader position to the High position
     */
    public void setLoaderPulleyHigh() {
        mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_HIGH);
    }


}
