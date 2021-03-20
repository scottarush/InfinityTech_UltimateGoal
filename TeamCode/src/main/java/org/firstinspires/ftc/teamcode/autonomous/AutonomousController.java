package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.guidance.GuidanceController;
import org.firstinspires.ftc.teamcode.guidance.IGuidanceControllerStatusListener;
import org.firstinspires.ftc.teamcode.ringdetect.IRingDetectorResultListener;
import org.firstinspires.ftc.teamcode.ringdetect.RingDetectorNeuralNetwork;
import org.firstinspires.ftc.teamcode.robot.MrRingsBot;
import org.firstinspires.ftc.teamcode.shooter.Grabber;
import org.firstinspires.ftc.teamcode.shooter.IShooterListener;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.BaseStateMachineController;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

public class AutonomousController extends BaseStateMachineController
        implements IShooterListener, IGuidanceControllerStatusListener, IRingDetectorResultListener {

    public static final int DEMO_SEQUENCE = 0;
    public static final int RINGS_SEQUENCE = 1;

    /**
     * Max rotation power.
     */
    private static final double MAX_ROTATION_POWER = 0.75d;

    /**
     * rotation timeout. same value used for all transitions
     */
    private static final int ROTATION_TIMEOUTMS = 2000;

    private MrRingsBot mRingsBot = null;

    private GuidanceController mGuidanceController;

    private OneShotTimer mRotationTimeoutTimer = new OneShotTimer(ROTATION_TIMEOUTMS, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evRotationTimeoutError");
        }
    });
    /**
     * Constructor
     * @param opMode
     * @param ringsBot
     */
    public AutonomousController(final OpMode opMode,
                                GuidanceController guidanceController,
                                MrRingsBot ringsBot) {
        super(false);
        mGuidanceController = guidanceController;
        mRingsBot = ringsBot;

        // initialize the base class
        init(opMode,new AutonomousStateMachineContext(this));

        mGuidanceController.addGuidanceControllerStatusListener(this);

        mRingsBot.getShooter().addShooterListener(this);
    }

    /**
     * starts the selected sequence.
     */
    public void startSequence(int sequence){
        switch(sequence){
            case RINGS_SEQUENCE:
                transition("evStart");
                break;
            case DEMO_SEQUENCE:
            default:
                transition("evStartDemo");
        }
    }

    @Override
    public void rotationComplete() {
        // Cancel the timeout error timer
        mRotationTimeoutTimer.cancel();
        // And notify the state machine
        transition("evRotationComplete");
    }

    @Override
    public void pathFollowComplete() {
        transition("evPathFollowComplete");
    }

    @Override
    public void moveComplete() {
        transition("evMoveComplete");
    }

    @Override
    public void readyToShoot() {
        transition("evReadyToShoot");
    }

    /**
     * called from state machine to start a rotation using the guidance controller.
     * If the rotation fails to complete in the default rotation timeout an evRotationTimeoutError
     * will be triggered
     * @param angle rotation in degrees with positive angles to the right
     */
    public void rotateToHeading(int angle){
        // Convert angle to floating point radians
        double radianAngle = (double)angle * Math.PI/180d;
        mGuidanceController.rotateToHeading(radianAngle);
        mRotationTimeoutTimer.start();
    }
    /**
     * called from state machine to do a rotation to point to a target point using the
     * guidance controller.
     * @param targetx x coordinate of target point
     * @param targety y coordinate of target point
     **/
    public void rotateToTarget(double targetx,double targety){
        mGuidanceController.rotateToTarget(targetx,targety);
        mRotationTimeoutTimer.start();
    }

    /**
     * called from state machine to drive forward or rearward
     * @param distance in inches + for forward, - for rearward
     */
    public void moveStraight(double distance){
        double meters = distance / 39.37d;
        mGuidanceController.moveStraight(meters,1.0d);
    }
    /**
     * Strafes the robot left (negative distance) or right (positive distance).
     * Listeners are notified on completion via the IGuidanceControllerStatusListener interface.
     * @param distance distance to strafe in inches + for right, - for left
     **/
    public void strafe(double distance) {
        double meters = distance / 39.37d;
        mGuidanceController.strafe(meters,1.0d);
    }

    /**
     * Called to start ring measurement.  RingDetector will trigger
     * a detection after an average period.
     */
    public void startRingsMeasurement(){
        mRingsBot.getRingDetector().doAveragedInference(10,this);
    }

    @Override
    public void averageDetectionResult(int result) {

        switch(result){
            case RingDetectorNeuralNetwork.LABEL_NO_RING:
                transition("evNoRings");
                break;
            case RingDetectorNeuralNetwork.LABEL_ONE_RING:
                transition("evOneRing");
                break;
            case RingDetectorNeuralNetwork.LABEL_FOUR_RINGS:
                transition("evFourRings");
                break;
        }
    }

    /**
     * Called to activate the ring detector.
     */
    public void activateRingDetector(){
        mRingsBot.getRingDetector().startDetection();
    }
    /**
     * Called to deactivate the ring detector.  Actually pauses
     * the ring detector.
     */
    public void deactivateRingDetector(){
        mRingsBot.getRingDetector().pauseDetection();
    }

    /**
     * Opens the grabber
     */
    public void openGrabber(){
        mRingsBot.getGrabber().openGrabber();
    }
    /**
     * Closes the grabber
     */
    public void closeGrabber(){
        mRingsBot.getGrabber().closeGrabber();
    }
    /**
     * Activates shooter
     */
    public void activateShooter(){
        mRingsBot.getShooter().activateShooter();
    }
    /**
     * Deactivates shooter
     */
    public void deactivateShooter(){
        mRingsBot.getShooter().deactivateShooter();
    }

    /**
     * moves grabber to retracted position
     */
    public void setGrabberRetracted(){
        mRingsBot.getGrabber().setGrabberPosition(Grabber.GRABBER_FULLY_RETRACTED);
    }
    /**
     * moves grabber to clear position
     */
    public void setGrabberClearPulley(){
        mRingsBot.getGrabber().setGrabberPosition(Grabber.GRABBER_CLEAR_PULLEY);
    }
    /**
     * moves grabber to lowered position
     */
    public void setGrabberLowered(){
        mRingsBot.getGrabber().setGrabberPosition(Grabber.GRABBER_LOWERED);
    }
    /**
     * moves grabber to carry position
     */
    public void setGrabberCarry(){
        mRingsBot.getGrabber().setGrabberPosition(Grabber.GRABBER_CARRY);
    }

    /**
     *
     * Shoots a ring. Presumes that a ring has been loaded in the pulley
     */
    public void shootRing(){
        mRingsBot.getShooter().shoot();
    }

    /**
     * Returns true if ready to shoot, false if not for any reason
     */
    public boolean isShooterReady(){
        return mRingsBot.getShooter().isShooterReady();
    }
    /**
     * Sets the shooter distance to midfield high
     */
    public void setShooterDistanceMidfieldHighGoal(){
        mRingsBot.getShooter().setShooterDistance(Shooter.SPEED_SETTING_MIDFIELD_HIGH_GOAL);
    }
    /**
     * Sets the shooter distance to midfield low goal
     */
    public void setShooterDistanceMidfieldLowGoal(){
        mRingsBot.getShooter().setShooterDistance(Shooter.SPEED_SETTING_MIDFIELD_LOW_GOAL);
    }

    /**
     * Called from state machine to stop the robot and release resources.  This can
     * only be called once at shutdown.
     */
    public void stop(){
        mRingsBot.stop();
    }

}

