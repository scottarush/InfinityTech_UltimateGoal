package org.firstinspires.ftc.teamcode.ringdetect;

/**
 * This class implements the neural network.
 */
public class RingDetectorNeuralNetwork {
    public static final int NO_RING = 1;
    public static final int ONE_RING = 2;
    public static final int FOUR_RINGS = 3;

    // indexes into the data array
    public static final int DATA_DIMENSION = 14;
    public static final int TAG_INDEX = 0;
    public static final int DISTANCE_INDEX = 1;
    public static final int TOP_RED_INDEX = 2;
    public static final int TOP_GREEN_INDEX = 3;
    public static final int TOP_BLUE_INDEX = 4;
    public static final int TOP_DISTANCE_INDEX = 5;
    public static final int MID_RED_INDEX = 6;
    public static final int MID_GREEN_INDEX = 7;
    public static final int MID_BLUE_INDEX = 8;
    public static final int MID_DISTANCE_INDEX = 9;
    public static final int BOTTOM_RED_INDEX = 10;
    public static final int BOTTOM_GREEN_INDEX = 11;
    public static final int BOTTOM_BLUE_INDEX = 12;
    public static final int BOTTOM_DISTANCE_INDEX = 13;
}
