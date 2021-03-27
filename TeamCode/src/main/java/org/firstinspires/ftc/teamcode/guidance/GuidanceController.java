package org.firstinspires.ftc.teamcode.guidance;

import org.firstinspires.ftc.teamcode.util.LogFile;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * GuidanceController encapsulates a PID algorithm that generates corrective
 * gain for guidance to an x,y field target position.
 */
public class GuidanceController {

    private MiniPID mRotationModePID = null;
     private MiniPID mPathSteeringPID = null;
    private MiniPID mPathPowerPID = null;
    private MiniPID mStraightPowerPID = null;
    private MiniPID mStraightHeadingPID = null;
    private MiniPID mStrafePowerPID = null;

    private double mRotationCommand = 0d;
    private double mPathSteeringCommand = 0d;

    private double mPowerCommand = 0d;
    public static final int STOPPED = 0;
    public static final int ROTATION_MODE = 1;
    public static final int PATH_MODE = 2;
    public static final int STRAIGHT_MODE = 3;
    public static final int STRAFE_MODE = 4;
    private int mMode = STOPPED;

    public static final double MAX_STRAFE_HEADING_ERROR = Math.PI/180;

    private double mTargetHeading = 0d;

    // direction in strafe and straight modes.  false = left/backward, true = right/forward
    private boolean mDirection = true;

    private KalmanTracker mKalmanTracker = null;

    private Point mPathLineStart = null;
    private Point mPathLineEnd = null;

    private Point mTargetPoint = null;

    private ArrayList<IGuidanceControllerCommandListener> mCommandListeners = new ArrayList<>();
    private ArrayList<IGuidanceControllerStatusListener> mStatusListeners = new ArrayList<>();

    private GuidanceControllerParameters mGCParameters = new GuidanceControllerParameters();

    public static class GuidanceControllerParameters {
        /**
         * Minimum angle threshold for rotation mode to complete
         */
        public double rotationModeStopAngleError = 7d*Math.PI/180;

        public double rotationModePropGain = 0.6d;
        public double rotationModeIntegGain = 0.01d;
        public double rotationModeMaxIntegGain = 0.2d;
        public double rotationModeDerivGain = 7.0d;

        /**
         * Maximum angle to target to be able to enter path mode.
         */
        public double pathModeMaxEntryAngle = 20.0d*Math.PI/180;

        public double pathModeSteeringPropGain = 0.1d;
        public double pathModeSteeringIntegGain = 0d;
        public double pathModeSteeringMaxIntegOutput = 0.2d;
        public double pathModeSteeringDerivGain = 0d;

        public double pathModePowerPropGain = 0.7d;
        public double pathModePowerIntegGain = 0.01d;
        public double pathModePowerMaxIntegOutput = 0.2d;
        public double pathModePowerDerivGain = 0d;

        // Time to stop in seconds in straight mode.
        public double straightModeStopTime = 0.3;
        public double straightModePowerPropGain = 2.0d;
        public double straightModePowerIntegGain = 0d;
        public double straightModePowerMaxIntegOutput = 0.3d;
        public double straightModePowerDerivGain = 1.0d;

        public double straightModeHeadingPropGain = 0.7d;
        public double straightModeHeadingIntegGain = 0.05d;
        public double straightModeHeadingMaxIntegOutput = 0.2d;
        public double straightModeHeadingDerivGain = 4.0d;

        public double strafeModeStopTime = 0.15;
        public double strafeModePowerPropGain = 0.1d;
        public double strafeModePowerIntegGain = 0.3d;
        public double strafeModePowerMaxIntegOutput = 0.8d;
        public double strafeModePowerDerivGain = 0d;
    }

    public GuidanceController(GuidanceControllerParameters gcParameters, KalmanTracker kalmanTracker){
        mKalmanTracker = kalmanTracker;
        mGCParameters = gcParameters;

        // Rotation controller limited to -1.0 to 1.0
        mRotationModePID = new MiniPID(mGCParameters.rotationModePropGain,mGCParameters.rotationModeIntegGain,mGCParameters.rotationModeDerivGain);
        mRotationModePID.setMaxIOutput(mGCParameters.rotationModeMaxIntegGain);
        mRotationModePID.setOutputLimits(-1.0,1.0d);

        mPathSteeringPID = new MiniPID(mGCParameters.pathModeSteeringPropGain,mGCParameters.pathModeSteeringIntegGain,mGCParameters.pathModeSteeringDerivGain);
        mPathSteeringPID.setMaxIOutput(mGCParameters.pathModeSteeringMaxIntegOutput);
        mPathSteeringPID.setOutputLimits(-1.0,1.0d);

        mPathPowerPID = new MiniPID(mGCParameters.pathModePowerPropGain,mGCParameters.pathModePowerIntegGain,mGCParameters.pathModePowerDerivGain);
        mPathPowerPID.setMaxIOutput(mGCParameters.pathModePowerMaxIntegOutput);
        mPathPowerPID.setOutputLimits(0d,1.0d);

        mStraightPowerPID = new MiniPID(mGCParameters.straightModePowerPropGain,mGCParameters.straightModePowerIntegGain,mGCParameters.straightModePowerDerivGain);
        mStraightPowerPID.setMaxIOutput(mGCParameters.straightModePowerMaxIntegOutput);

        mStraightHeadingPID = new MiniPID(mGCParameters.straightModeHeadingPropGain,mGCParameters.straightModeHeadingIntegGain,mGCParameters.straightModeHeadingDerivGain);
        mStraightHeadingPID.setMaxIOutput(mGCParameters.straightModeHeadingMaxIntegOutput);
        mStraightHeadingPID.setOutputLimits(-1.0d,1.0d);

        mStrafePowerPID = new MiniPID(mGCParameters.strafeModePowerPropGain,mGCParameters.strafeModePowerIntegGain,mGCParameters.strafeModePowerDerivGain);
        mStrafePowerPID.setMaxIOutput(mGCParameters.strafeModePowerMaxIntegOutput);
        mStrafePowerPID.setOutputLimits(-1.0d,1.0d);


    }

    /**
     * Utility returns true if current heading is acceptable to do a path follow
     * @return true if successful, false if the current robot heading is beyond the maximum allowed entry angle (and must be rotated first)
     */
    public boolean isPathFollowValid(double px,double py) {
        // Compute the heading angle of the robot relative to the end point and determine
        // if we are within the maximum entry angle for stability
        double theta = mKalmanTracker.getEstimatedHeading();
        Point pathLineStart = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        Point pathLineEnd = new Point(px,py);
        Point rotatedEnd = pathLineStart.rotate(theta);
        Point rotatedStart = pathLineEnd.rotate(theta);
        Point robotPos = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        double m = (rotatedEnd.y - rotatedStart.y) / (rotatedEnd.x - rotatedStart.x);
        double angle = -Math.atan(m);
        if (Math.abs(angle) >= mGCParameters.pathModeMaxEntryAngle){
            // Robot heading is too far for stability so return flase
            return false;
        }
        else{
            return true;
        }
    }

        /**
         * Engages path mode to a line
         * Starting point of the line is the current center of the robot.
         * @param px x coordinate of line terminus
         * @param py y coordinate of line terminus
         * @return true if successful, false if the current robot heading is beyond the maximum allowed entry angle (and must be rotated first)
         */
    public boolean followPath(double px, double py){
        if (!isPathFollowValid(px,py)){
            return false;
        }
        mPathLineStart = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        mPathLineEnd = new Point(px,py);
        // Start the path follow
        mPathSteeringPID.reset();
        mPathPowerPID.reset();

        // Stop the robot in case it was moving
        clearAllCommands();
        // Set mode to path and wait for update
        mMode = PATH_MODE;
        return true;
    }
    /**
     * Moves the robot straight forward (+) or backward (-) at a specific target heading.
     * Listeners are notified on completion via the IGuidanceControllerStatusListener interface.
     * @param distance distance to move in meters + for forward, - for backward
     * @param maxPower maximum power from 0d to 1.0d
     **/
    public void moveStraight(double distance,double maxPower,double targetHeading) {
        mMode = STRAIGHT_MODE;
        mTargetHeading = targetHeading;
        mStraightPowerPID.setOutputLimits(-maxPower,maxPower);
        // Compute the target point
        double px = mKalmanTracker.getEstimatedXPosition() + Math.sin(mTargetHeading)*distance;
        double py = mKalmanTracker.getEstimatedYPosition() + Math.cos(mTargetHeading)*distance;
        mTargetPoint = new Point(px,py);
        // Clear all commands in case robot was moving
        clearAllCommands();
        mStraightPowerPID.reset();
        mStraightHeadingPID.reset();
    }

    /**
     * Strafes the robot left (negative distance) or right (positive distance) maintaining
     * the target heading
     * Listeners are notified on completion via the IGuidanceControllerStatusListener interface.
     * @param distance distance to strafe in meters + for right, - for left
     * @param maxPower maximum power from 0d to 1.0d
     **/
    public void strafe(double distance,double maxPower,double targetHeading){
        mMode = STRAFE_MODE;
        mStrafePowerPID.setOutputLimits(-maxPower,maxPower);

        // Compute the heading angle to strafe base on left or right
        double strafeHeading = targetHeading;
        if (distance >= 0d) {
            strafeHeading += Math.PI / 2;
        }
        else{
            strafeHeading -= Math.PI / 2;
        }
        // Now compute the target point using the strafe heading
        double px = mKalmanTracker.getEstimatedXPosition() + Math.sin(strafeHeading)*Math.abs(distance);
        double py = mKalmanTracker.getEstimatedYPosition() + Math.cos(strafeHeading)*Math.abs(distance);
        mTargetPoint = new Point(px,py);
        // Save target heading to be able to correct at end if need be.
        mTargetHeading = mKalmanTracker.getEstimatedHeading();
        // Clear all commands in case robot was moving
        clearAllCommands();
        mStrafePowerPID.reset();
        mRotationModePID.reset();
    }


    /**
     * Does a rotation maneuver The rotation will be completed and notified to listeners via the
     * IGuidanceControllerStatusListener interface.
     **/
    public void rotateToHeading(double heading){
        mMode = ROTATION_MODE;
        mTargetHeading = heading;
        // Clear all commands in case robot was moving
        clearAllCommands();
        mRotationModePID.reset();
    }
    /**
     * Does a rotation maneuver to point the robot at the provided point.
     * If the heading is already within the minimum a rotation complete
     * will be triggered without an actual rotation
     * @param targetx x coordinate of target point
     * @param targety y coordinate of target point
     **/
    public void rotateToTarget(double targetx,double targety){
        double deltaAngle = mKalmanTracker.getEstimatedHeading()-getAngleToTarget(targetx,targety);
        if (Math.abs(deltaAngle) <= mGCParameters.rotationModeStopAngleError){
            mMode = STOPPED;
            notifyRotationComplete();
            return;
        }
        // Otherwise, do the rotation by the angle
        rotateToHeading(deltaAngle);
    }
    /**
     * utility computes the angle between the current robot position and the
     * provided position from 0..PI = N->E->S and -PI..0 = N->W->S
     */
    private double getAngleToTarget(double targetx,double targety){
        double xrel = targetx-mKalmanTracker.getEstimatedXPosition();
        double yrel = targety-mKalmanTracker.getEstimatedYPosition();
        // Compute the angle assuming the robot is pointed straight north and add
        // the current heading afterward
        double angleToTarget = 0d;
        double invtan = 0d;
        // Handle straight line case with yrel = 0 as inverse tan will blow up at 0 and PI
        if (yrel == 0d){
            if (xrel >= 0d){
                return Math.PI/2d;
            }
            else{
                return -Math.PI/2d;
            }
        }
        // Otherwise compute angle from the inverse tangent
        invtan = Math.abs(Math.atan(Math.abs(xrel)/Math.abs(yrel)));
        if (xrel >= 0d){
            if (yrel >= 0d){
                // northeast quadrant
                angleToTarget = invtan;
            }
            else{
                // southeast quadrant
                angleToTarget = Math.PI-invtan;
            }
        }
        else{
            // xrel is negative
            if (yrel >= 0d){
                // northwest quadrant
                angleToTarget = -invtan;
            }
            else{
                // southwest quadrant
                angleToTarget = -(Math.PI-invtan);
            }

        }
        return angleToTarget;
    }

    /**
     * adds  listener for guidance controller command output notifications.
     */
    public void addGuidanceControllerCommandListener(IGuidanceControllerCommandListener listener){
        if (mCommandListeners.contains(listener)){
            return;
        }
        mCommandListeners.add(listener);
    }

    /**
     * adds  listener for guidance controller status notifications.
     */
    public void addGuidanceControllerStatusListener(IGuidanceControllerStatusListener listener){
        if (mStatusListeners.contains(listener)){
            return;
        }
        mStatusListeners.add(listener);
    }

    /**
     * update function triggers update of the controller.  Should ideally be called at the
     * sample rate.
     *
     */
    public void updateCommand(){
        // check if we need to make a mode transition
        switch(mMode){
             case ROTATION_MODE: updateRotationMode();
               break;
             case PATH_MODE:
                 updatePathMode();
                break;
            case STRAIGHT_MODE:
                updateStraightMode();
                break;
            case STRAFE_MODE:
                updateStrafeMode();
                break;
            case STOPPED:
                break;
        }
     }
    /**
     * update function used to rotate toward the current target heading.
     */
    private void updateRotationMode(){
        // Compute current heading to target error threshold to decide if we need to stop
        double error = mKalmanTracker.getEstimatedHeading()-mTargetHeading;
        if (Math.abs(error) <= mGCParameters.rotationModeStopAngleError){
            mMode = STOPPED;
            mRotationCommand = 0d;
            // Notify command listeners to stop the rotation command
            for (Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext(); ) {
                IGuidanceControllerCommandListener listener = iter.next();
                listener.setRotationCommand(0d);
            }
            // And status listeners that the maneuver is complete
            notifyRotationComplete();
            return;
        }
        // Otherwise, drop through to compute regular rotation command
        mRotationCommand = mRotationModePID.getOutput(mKalmanTracker.getEstimatedHeading(),mTargetHeading);

        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setRotationCommand(mRotationCommand);
        }
    }

    private void notifyRotationComplete(){
        for (Iterator<IGuidanceControllerStatusListener> iter = mStatusListeners.iterator(); iter.hasNext(); ) {
            IGuidanceControllerStatusListener listener = iter.next();
            listener.rotationComplete();
        }

    }
    /**
     * utility sends a zero to null all commands to zero
     */
    private void clearAllCommands(){
        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setStraightCommand(0d,0d);
            listener.setRotationCommand(0d);
            listener.setSteeringCommand(0d,0d);
        }
    }
    /**
     * Returns the current guidance controller operating mode.
     */
    public int getMode(){
        return mMode;
    }

    /**
     * updates the path mode PID
     */
    private void updatePathMode(){
        double theta = mKalmanTracker.getEstimatedHeading();
        // rotate the coordinates of the robot and the line points.
        Point rotatedEnd = mPathLineEnd.rotate(theta);
        Point rotatedStart = mPathLineStart.rotate(theta);

        Point robotPos = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        Point rotRobotPos = robotPos.rotate(theta);

        // First check if we have pass the y end of the line and stop
        boolean stop = false;
        if (rotatedEnd.y < 0){
            if (rotRobotPos.y <= rotatedEnd.y){
                stop = true;
            }
        }
        else{
            if (rotRobotPos.y >= rotatedEnd.y){
                stop = true;
            }
        }
        if (stop){
            // Set path and power to 0 and drop through to send command
            mPathSteeringCommand = 0d;
            mPowerCommand = 0d;
            mMode = STOPPED;
        }
        else {
            // Not yet at the target so control steering using the angle between the rotated
            // heading (which is now = 0) and the angle of the line (which is the arctan
            // of the slope.
            // Line formula:  y = mx + b where m = (endy-starty)/(endx-startx) and b = starty
            double m = (rotatedEnd.y - rotatedStart.y) / (rotatedEnd.x - rotatedStart.x);
            // Angle is negative because our steering command is + to the right.
            double angle = -Math.atan(m);
            mPathSteeringCommand = mPathSteeringPID.getOutput(angle, 0);
            // And control the power based on the distance to the target's y coordinate
            mPowerCommand = mPathPowerPID.getOutput(rotRobotPos.y,rotatedEnd.y);
        }

        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setSteeringCommand(mPathSteeringCommand, mPowerCommand);
        }
        if (mMode == STOPPED){
            // notify that the path follow was complete
            for(Iterator<IGuidanceControllerStatusListener> iter = mStatusListeners.iterator(); iter.hasNext();){
                IGuidanceControllerStatusListener listener = iter.next();
                listener.pathFollowComplete();
            }
        }

    }
    /**
     * updates the straight mode PID
     */
    private void updateStraightMode(){
        double theta = mKalmanTracker.getEstimatedHeading();
        // To simplify processing, first rotate the coordinates of both robot and target by the heading.
        // Rotated coordinates will then point straight up and only the y coordinate needs to be checked
        // to determine when we have reached the target.
        Point robotPos = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        Point rotRobotPos = robotPos.rotate(theta);
        Point rotatedTarget = mTargetPoint.rotate(theta);

        //  check if we have passed the target.  Only need to check the y coordinate, but leave
        // stopDistance based on the stop time and estimated speed
        boolean stop = false;
        double stopDistance = mGCParameters.straightModeStopTime * mKalmanTracker.getEstimatedSpeed();
        if (rotatedTarget.y < 0){
            if (rotRobotPos.y <= rotatedTarget.y+stopDistance){
                stop = true;
            }
        }
        else{
            if (rotRobotPos.y >= rotatedTarget.y-stopDistance){
                stop = true;
            }
        }
        if (stop){
            // Set power to 0 and drop through to send command
            mPowerCommand = 0d;
            mRotationCommand = 0d;
            mMode = STOPPED;
        }
        else {
            // Not yet at the target control the power based on the distance to the target's y coordinate
            mPowerCommand = mStraightPowerPID.getOutput(rotRobotPos.y,rotatedTarget.y);
            // and compute a heading command as an offset for any deviation from heading.
            mRotationCommand = mStraightHeadingPID.getOutput(mKalmanTracker.getEstimatedHeading(),mTargetHeading);
        }

        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setStraightCommand(mPowerCommand,mRotationCommand);
        }
        if (mMode == STOPPED){
            // notify that the command is complete
            for(Iterator<IGuidanceControllerStatusListener> iter = mStatusListeners.iterator(); iter.hasNext();){
                IGuidanceControllerStatusListener listener = iter.next();
                listener.moveComplete();
            }
        }
    }

    /**
     * updates the strafe mode PID
     */
    private void updateStrafeMode(){
        double theta = mKalmanTracker.getEstimatedHeading();
        // To simplify processing, first rotate the coordinates of both robot and target by the heading.
        // Rotated coordinates will then point straight up and only the x coordinate needs to be checked
        // to determine when we have reached the target.
        Point robotPos = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        Point rotRobotPos = robotPos.rotate(theta);
        Point rotatedTarget = mTargetPoint.rotate(theta);

        //  check if we have passed the target.  Only need to check the x coordinate, but leave
        // stopDistance based on the stop time and estimated speed
        boolean stop = false;
        double stopDistance = mGCParameters.strafeModeStopTime * mKalmanTracker.getEstimatedSpeed();
        if (rotatedTarget.x < 0){
            if (rotRobotPos.x <= rotatedTarget.x+stopDistance){
                stop = true;
            }
        }
        else{
            if (rotRobotPos.x >= rotatedTarget.x-stopDistance){
                stop = true;
            }
        }
        if (stop){
            mPowerCommand = 0d;
            mRotationCommand = 0d;
            mMode = STOPPED;
        }
        else {
            // the power based on the distance to the target's x coordinate
            mPowerCommand = mStrafePowerPID.getOutput(rotRobotPos.x,rotatedTarget.x);
            // and compute a rotation command as an offset for any deviation from heading.
            mRotationCommand = mRotationModePID.getOutput(mKalmanTracker.getEstimatedHeading(),mTargetHeading);
        }

        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setStrafeCommand(mPowerCommand,mRotationCommand);
        }
        if (mMode == STOPPED){
            // notify that the command is complete
            for(Iterator<IGuidanceControllerStatusListener> iter = mStatusListeners.iterator(); iter.hasNext();){
                IGuidanceControllerStatusListener listener = iter.next();
                listener.strafeComplete();
            }
        }
    }

    /**
     * returns the rotation command  for logging
     */
    public double getRotationCommand(){
        return mRotationCommand;
    }
    /**
     * returns the steering command for logging
     */
    public double getPathSteeringCommand(){
        return mPathSteeringCommand;
    }
    /**
     * returns the power command in Straight and Path modes for logging
     */
    public double getPowerCommand(){
        return mPowerCommand;
    }
    /**
     * returns the current mode as a string for logging.
     */
    public String getModeString(){
        switch(mMode) {
            case ROTATION_MODE:
                return "ROTATION";
             case PATH_MODE:
                return "PATH";
            case STRAIGHT_MODE:
                return "STRAIGHT";
            case STRAFE_MODE:
                return "STRAFE";
            case STOPPED:
                return "STOPPED";
            default:
                return "INVALID_MODE";  // Can't happen
        }
    }

}
