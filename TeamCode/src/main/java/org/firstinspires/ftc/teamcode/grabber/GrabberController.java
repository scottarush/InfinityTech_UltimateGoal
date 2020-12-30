package org.firstinspires.ftc.teamcode.grabber;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.BaseStateMachineController;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

public class GrabberController extends BaseStateMachineController implements IGrabberController {

    private Grabber mGrabber = null;

    public GrabberController(Grabber grabber,OpMode opMode){
        mGrabber = grabber;
        // initialize the base class and create the context
        init(opMode,new GrabberStateMachineContext(this));

    }

    @Override
    public void evInit() {
        transition("evInit");
    }


    @Override
    public void evLimitSwitchClosed() {
        transition("evLimitSwitchClosed");
    }

    @Override
    public void evGrabberMoving() {

    }

    @Override
    public void evGrabberStopped() {

    }

    /**
     * Proxy for setting the grabber position from the state machine.
     * @param position
     */
    public void setGrabberPosition(int position){
        mGrabber.setGrabberPosition(position);
    }

    public int getGrabberPosition() {
        return mGrabber.getGrabberPosition();
    }

}
