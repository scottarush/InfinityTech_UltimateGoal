package org.firstinspires.ftc.teamcode.shooter;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.BaseStateMachineController;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;

import statemap.FSMContext;
import statemap.State;

/**
 * this is the controller for the shooter state machine that coordinates the loading and firing for
 * the shooter
 */
public class ShooterController extends BaseStateMachineController implements IShooterController {

    private Shooter mShooter = null;

    public ShooterController(Shooter shooter, OpMode opMode) {
        mShooter = shooter;
        // initialize the base class
        init(opMode,new ShooterStateMachineContext(this));
    }
    @Override
    public boolean isShooterReady() {
        ShooterStateMachineContext shooterContext = (ShooterStateMachineContext)mStateMachineContext;
        if (shooterContext.getState() == ShooterStateMachineContext.ShooterStateMachine.ReadyToShoot){
            return true;
        }
        else{
            return false;
        }

    }

    @Override
    public void evReadyToShoot() {
        transition("evReadyToShoot");
    }

    /**
     * called from opmodes to trigger the evActivate event
     */
    public void evActivate() {
        transition("evActivate");
    }

    @Override
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

    /**
     * Starts the loader pully to shoot the ring
     */
    public void startLoaderPully() {
        mShooter.startLoaderPully();
    }

    /**
     * Stops the loader pully
     */
    public void stopLoaderPully(){
        mShooter.stopLoaderPully();
    }

}
