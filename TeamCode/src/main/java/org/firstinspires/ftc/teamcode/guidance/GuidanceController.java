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

    private class Point {
        public Point(double x, double y){
            this.x = x;
            this.y = y;
        }
        public double x = 0d;
        public double y = 0d;

        public Point rotate(double theta){
            double rx = Math.cos(theta) * x - Math.sin(theta) * y;
            double ry = Math.sin(theta) * x + Math.cos(theta) * y;
            return new Point(rx,ry);
        }
    }

    private MiniPID mRotationModePID = null;
    private MiniPID mPathSteeringPID = null;
    private MiniPID mPathPowerPID = null;
    private MiniPID mStraightPowerPID = null;

    private double mRotationCommand = 0d;
    private double mPathSteeringCommand = 0d;

    private double mPowerCommand = 0d;

    public static final int STOPPED = 0;
    public static final int ROTATION_MODE = 1;
    public static final int PATH_MODE = 2;
    public static final int STRAIGHT_MODE = 3;
    private int mMode = STOPPED;

    private double mTargetHeading = 0d;

    private KalmanTracker mKalmanTracker = null;
    private LogFile mLogFile = null;
    private static boolean ENABLE_LOGGING = false;
    public static final String[] LOG_COLUMNS = {"px","py","theta","distance","projection","angle"};

    private Point mPathLineStart = null;
    private Point mPathLineEnd = null;

    private Point mTargetPoint = null;
    /** Direction for straight mode.  true = forward, false = backward. **/
    private boolean mStraightModeDirection = false;
    /**
     * maximum power for current straight mode maneuver.
     */
    private double mMaxStraightModePower = 1.0d;

    private ArrayList<IGuidanceControllerCommandListener> mCommandListeners = new ArrayList<>();
    private ArrayList<IGuidanceControllerStatusListener> mStatusListeners = new ArrayList<>();

    private GuidanceControllerParameters mGCParameters = new GuidanceControllerParameters();

    public static class GuidanceControllerParameters {
        /**
         * Minimum angle threshold for rotation mode to complete
         */
        public double rotationModeStopAngleError = 3d*Math.PI/180;
        /**
         * Angular velocity threshold to stop in radians per sec.
         */
        public double rotationModeStopAngularVelocityThreshold = 10.0d*Math.PI/180;
        public double rotationModePropGain = 0.5d;
        public double rotationModeIntegGain = 0d;
        public double rotationModeMaxIntegGain = 0.2d;
        public double rotationModeDerivGain = 0d;

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

        public double straightModePowerPropGain = 0.7d;
        public double straightModePowerIntegGain = 0.01d;
        public double straightModePowerMaxIntegOutput = 0.2d;
        public double straightModePowerDerivGain = 0d;
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

        mPathPowerPID = new MiniPID(mGCParameters.straightModePowerPropGain,mGCParameters.straightModePowerIntegGain,mGCParameters.straightModePowerDerivGain);
        mPathPowerPID.setMaxIOutput(mGCParameters.straightModePowerMaxIntegOutput);
        mPathPowerPID.setOutputLimits(0d,1.0d);

        if (ENABLE_LOGGING){
            mLogFile = new LogFile("/sdcard","gclog.csv",LOG_COLUMNS);
            mLogFile.openFile();
        }
    }

    public void closeLogFile(){
        if (ENABLE_LOGGING)
            mLogFile.closeFile();
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
     * Moves the robot straight forward (+) or backward (-) at the current heading.
     * Listeners are notified on completion via the IGuidanceControllerStatusListener interface.
     * @param distance distance to move in meters + for forward, - for backward
     * @param maxPower maximum power from 0d to 1.0d
     **/
    public void moveStraight(double distance,double maxPower){
        mMode = STRAIGHT_MODE;
        mMaxStraightModePower = maxPower;
        if (distance >= 0d){
            // forward
            mStraightModeDirection = true;
        }
        else{
            // backward
            mStraightModeDirection = false;
        }
        // Compute the target point
        double px = mKalmanTracker.getEstimatedXPosition() + Math.sin(mKalmanTracker.getEstimatedHeading())*distance;
        double py = mKalmanTracker.getEstimatedYPosition() + Math.cos(mKalmanTracker.getEstimatedHeading())*distance;
        mTargetPoint = new Point(px,py);
        // Clear all commands in case robot was moving
        clearAllCommands();
        mStraightPowerPID.reset();
        mStraightPowerPID.setOutputLimits(-maxPower,maxPower);
        
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
             case ROTATION_MODE:
                 updateRotationMode();
               break;
             case PATH_MODE:
                 updatePathMode();
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
            // Now check if the angular velocity has slowed enough to stop
            if (Math.abs(mKalmanTracker.getEstimatedAngularVelocity()) <= mGCParameters.rotationModeStopAngularVelocityThreshold) {
                mMode = STOPPED;
                // Notify command listeners to stop the rotation command
                for (Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext(); ) {
                    IGuidanceControllerCommandListener listener = iter.next();
                    listener.setRotationCommand(0d);
                }
                // And status listeners that the maneuver is complete
                notifyRotationComplete();
             }
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
            listener.setStraightCommand(0d);
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
        // rotate the coordinates of the robot and the target point
        Point robotPos = new Point(mKalmanTracker.getEstimatedXPosition(),mKalmanTracker.getEstimatedYPosition());
        Point rotRobotPos = robotPos.rotate(theta);
        Point rotatedTarget = mTargetPoint.rotate(theta);

        //  check if we have pass the y end of the line and stop
        boolean stop = false;
        if (rotatedTarget.y < 0){
            if (rotRobotPos.y <= rotatedTarget.y){
                stop = true;
            }
        }
        else{
            if (rotRobotPos.y >= rotatedTarget.y){
                stop = true;
            }
        }
        if (stop){
            // Set power to 0 and drop through to send command
            mPowerCommand = 0d;
            mMode = STOPPED;
        }
        else {
            // Not yet at the target control the power based on the distance to the target's y coordinate
            mPowerCommand = mStraightPowerPID.getOutput(rotRobotPos.y,rotatedTarget.y);
            // Reverse the sign of the command if going backward
            if (!mStraightModeDirection){
                mPowerCommand = Math.signum(mPowerCommand) * mPowerCommand;
            }
        }

        for(Iterator<IGuidanceControllerCommandListener> iter = mCommandListeners.iterator(); iter.hasNext();){
            IGuidanceControllerCommandListener listener = iter.next();
            listener.setStraightCommand(mPowerCommand);
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
            case STOPPED:
                return "STOPPED";
            default:
                return "INVALID_MODE";  // Can't happen
        }
    }

}
