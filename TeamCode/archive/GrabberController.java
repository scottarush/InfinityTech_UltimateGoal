package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.grabber.Grabber;
import org.firstinspires.ftc.teamcode.util.BaseStateMachineController;

public class GrabberController extends BaseStateMachineController implements IGrabberListener {

    private Grabber mGrabber = null;

    public GrabberController(Grabber grabber,OpMode opMode){
        super(false);
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
        transition("evGrabberMoving");
    }

    @Override
    public void evGrabberStopped() {
        transition("evGrabberStopped");
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
