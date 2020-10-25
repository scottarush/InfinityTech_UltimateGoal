package org.firstinspires.ftc.teamcode.guidance;


import org.ejml.data.DMatrixRMaj;

/**
 * Encapsulates a Kalman filter state estimator for a mecanum wheel robot.
 * The filter estimates position (returned by getEstimatedPosition()) and
 * heading (getEstimatedHeading()).
 *
 * The init method must be called with the initial position and heading of the resting
 * robot.
 *
 * When position or heading is known from additional inputs, call the
 * setKnownXXX functions to update the estimate.
 *
 * Measurement vector z=[vx,vy,w_zw,ax,ay,w_zi]^T
 * Estimate vector xhat=[px,vx,vy,ax,ay,w_z,theta]^
 */
public class KalmanTracker {

    private static final int XHAT_PX_INDEX = 0;
    private static final int XHAT_PY_INDEX = 1;
    private static final int XHAT_VX_INDEX = 2;
    private static final int XHAT_VY_INDEX = 3;
    private static final int XHAT_OMEGAZ_INDEX = 4;
    private static final int XHAT_THETA_INDEX = 5;


    private KalmanFilterOperations mFilter = null;

    // Process matrix
    private DMatrixRMaj A;
    // Process noise covariance matrix
    private DMatrixRMaj Q;
    // Measurment noise covariance matrix
    private DMatrixRMaj R;
    // Measurement matrix
    private DMatrixRMaj H;

    public KalmanTracker(){

    }

    private KalmanParameters mKalmanParameters;

    public static class KalmanParameters {
        // T sampling interval in seconds
        public double T = 0.050d;
        // px0 x coordinate of robot initial position in meters
        public double PX0 = 0d;
        //  py0 y coordinate of robot initial position in meters
        public double PY0 = 0d;
        //  theta0 initial heading in radians where 0=judge side, PI/2=right side, PI=audience side, 3PI/2=left side
        public double THETA0 = 0d;
        //  lx  Lateral distance from wheel axle to imu center in meters
        public double LX = 0.5d;
        //  ly Lateral distance from wheel axle to imu center in meters
        public double LY = 0.5d;
        //  r radius of wheel in meters
        public double WHEEL_RADIUS = 0.098d;
        // Initial P covariance values
        public double SIGMA_POSITION = 0.1;
        public double VAR_POSITION = Math.pow(SIGMA_POSITION,2.0d);
        public double SIGMA_VELOCITY = 0.1;
        public double VAR_VELOCITY = Math.pow(SIGMA_VELOCITY,2.0d);
        public double SIGMA_W = 0.1;
        public double VAR_W = Math.pow(SIGMA_W,2.0d);
        public double SIGMA_THETA = 0.1;
        public double VAR_THETA = Math.pow(SIGMA_THETA,2.0d);
    }

    // last calculated vx
    private double mVx = 0d;
    // last calculated vy
    private double mVy = 0d;

    /**
     * Initializes the Kalman filter
     */
    public void init(KalmanParameters parameters){
        mKalmanParameters = parameters;

        final double[][] A_MATRIX = {
                {1,0,mKalmanParameters.T,0,0,0},
                {0,1,0,mKalmanParameters.T,0,0},
                {0,0,1,0,0,0},
                {0,0,0,1,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,mKalmanParameters.T,1} };

        //-------------------------------------------------------------
        // Q process noise covariance matrix
        //-------------------------------------------------------------

        final double[][] Q_MATRIX = {
                {mKalmanParameters.VAR_POSITION,0,0,0,0,0},
                {0, mKalmanParameters.VAR_POSITION,0,0,0,0},
                {0,0, mKalmanParameters.VAR_VELOCITY,0,0,0},
                {0,0,0, mKalmanParameters.VAR_VELOCITY,0,0},
                {0,0,0,0,mKalmanParameters.VAR_W,0},
                {0,0,0,0,0,mKalmanParameters.VAR_THETA} };

        //-------------------------------------------------------------
        // R measurement noise covariance matrix
        //-------------------------------------------------------------
        final double WHEEL_RADIUS = 0.050;
        final double SIGMA_W_WHL = 0.001;
        final double R2_VAR_W_WHL = Math.pow(WHEEL_RADIUS,2.0d) * Math.pow(SIGMA_W_WHL,2.0d);
        final double VAR_VX = R2_VAR_W_WHL;
        final double VAR_VY = R2_VAR_W_WHL;
        final double VAR_W_WHL = R2_VAR_W_WHL/((Math.pow(mKalmanParameters.LY,2.0d) + Math.pow(mKalmanParameters.LX,2.0d)));
        final double SIGMA_W_IMU = 0.1;
        final double VAR_W_IMU = Math.pow(SIGMA_W_IMU,2.0d);
        final double[][] R_MATRIX = {
                {VAR_VX,0,0,0},
                {0,VAR_VY,0,0},
                {0,0,VAR_W_WHL,0},
                {0,0,0,VAR_W_IMU}};

        final double[][] H_MATRIX = {
                {0,0,1,0,0,0},
                {0,0,0,1,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,0,1}};


        // Initialize constant matrices
        A = new DMatrixRMaj(A_MATRIX);
        R = new DMatrixRMaj(R_MATRIX);
        Q = new DMatrixRMaj(Q_MATRIX);
        H = new DMatrixRMaj(H_MATRIX);

        // Create and configure the filter
        mFilter = new KalmanFilterOperations();
        mFilter.configure(A,Q,H);

        // Initialize the state estimate vector to the supplied position and orientation
        DMatrixRMaj xhat = new DMatrixRMaj(new double[][] {{mKalmanParameters.PX0},{mKalmanParameters.PY0},{0d},{0d},{0d},{mKalmanParameters.THETA0}});
        // Initialize P to the process noise covariance matrix
        DMatrixRMaj p = new DMatrixRMaj(Q_MATRIX);

        mFilter.setState(xhat,p);
    }

    /**
     * Called at the sampling rate T to update the filter with a new measurement.
     * @param w_lf angular velocity of LF wheel in radians/sec
     * @param w_lr angular velocity of LR wheel in radians/sec
     * @param w_rf angular velocity of RF wheel in radians/sec
     * @param w_rr angular velocity of RR wheel in radians/sec
      * @param theta_imu z coordinate of imu measured orientation in radians
     */
    public void updateMecanumMeasurement(double w_lf,
                                         double w_lr,
                                         double w_rf,
                                         double w_rr,
                                         double theta_imu) {
        // Compute the robot velocity from the wheel velocities which are in the local
        // frame
        double rover4 = mKalmanParameters.WHEEL_RADIUS/4.0d;
        mVx = rover4*(w_lf-w_rf-w_lr+w_rr);
        mVy = rover4*(w_lf+w_rf+w_lr+w_rr);
        // And then rotate into the global frame using the Point classes rotation function
        Point vp = new Point(mVx,mVy);
        Point rotated = vp.rotate(-getEstimatedHeading());
        mVx = rotated.x;
        mVy = rotated.y;

        double wzw = rover4*(-w_lf+w_rf-w_lr+w_rr)/(mKalmanParameters.LX+mKalmanParameters.LY);

        // Have to negate the wzw and wz_imu because we want to use left-handed orientation angles
        // instead of the right-handed angles produced by the measurements

        DMatrixRMaj z = new DMatrixRMaj(new double[][] {{mVx},{mVy},{-wzw},{-theta_imu}});

        // Do Kalman predict step
        mFilter.predict();
        // Do Kalman correct step
        mFilter.update(z,R);
    }
    /**
     * Returns the current estimated x position
     */
    public Double getEstimatedXPosition(){
        return mFilter.getState().get(XHAT_PX_INDEX);
    }
    /**
     * Returns the current estimated y position
     */
    public Double getEstimatedYPosition(){
        return mFilter.getState().get(XHAT_PY_INDEX);
    }

    /*
     * Returns the current estimated linear speed
     */
    public Double getEstimatedSpeed(){
        return Math.sqrt(Math.pow(mFilter.getState().get(XHAT_VX_INDEX),2)+Math.pow(mFilter.getState().get(XHAT_VY_INDEX),2));
    }

    /**
     * returns last calculated input Vx.  Used for logging
     */
    public Double getVx(){
        return mVx;
    }
    /**
     * returns last calculated input Vy.  Used for logging
     */
    public Double getVy(){
        return mVy;
    }
    /**
     *
     * @return Estimated heading from 0 to 2*PI in radians.  Note that this is a left-handed angle
     */
    public Double getEstimatedHeading() {
        return mFilter.getState().get(XHAT_THETA_INDEX, 0);
    }

    /**
     *
     * @return Estimated angular velocity in radians/sec.  Note that this is a left-handed angular velocity
     */
    public Double getEstimatedAngularVelocity(){
        return mFilter.getState().get(XHAT_OMEGAZ_INDEX,0);
    }

    /**
     * Called to set the position x coordinate when known from objects in the field
     * @param px updated x position
     */
    public void setKnownXPosition(double px){
        DMatrixRMaj xhat = new DMatrixRMaj(mFilter.getState());
        xhat.set(XHAT_PX_INDEX,0,px);
        // And reset the px variance to the default
        DMatrixRMaj p = new DMatrixRMaj(mFilter.getCovariance());
        p.set(XHAT_PX_INDEX,XHAT_PX_INDEX, mKalmanParameters.VAR_POSITION);
        mFilter.setState(xhat,p);
    }
    /**
     * Called to update the position y coordinate when known from objects in the field
     * @param py updated y position
     */
    public void setKnownYPosition(double py){
        DMatrixRMaj xhat = new DMatrixRMaj(mFilter.getState());
        xhat.set(XHAT_PY_INDEX,0,py);
        // And reset the px variance to the default
        DMatrixRMaj p = new DMatrixRMaj(mFilter.getCovariance());
        p.set(XHAT_PY_INDEX,XHAT_PY_INDEX, mKalmanParameters.VAR_POSITION);
        mFilter.setState(xhat,p);
    }
    /**
     * Called to update the heading when detected from objects in the field
     * @param theta updated heading angle
     */
    public void setKnownHeading(double theta){
        DMatrixRMaj xhat = new DMatrixRMaj(mFilter.getState());
        xhat.set(XHAT_THETA_INDEX,0,theta);
        // And reset the px variance to the default
        DMatrixRMaj p = new DMatrixRMaj(mFilter.getCovariance());
        p.set(XHAT_THETA_INDEX,XHAT_THETA_INDEX, mKalmanParameters.VAR_THETA);
        mFilter.setState(xhat,p);
    }

}
