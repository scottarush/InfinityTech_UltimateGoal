package org.firstinspires.ftc.teamcode.ringdetect;

import java.io.InputStream;

/**
 * This class extends JavaNeuralNetwork for the RingDetector specific network.
 */
public class RingDetectorNeuralNetwork extends JavaNeuralNetwork {
    public static final int NO_RING = 1;
    public static final int ONE_RING = 2;
    public static final int FOUR_RINGS = 3;

    public RingDetectorNeuralNetwork(InputStream is) throws Exception {
        super(is);
    }

    static class MeasurementData {

    }

    /**
     * Performs a measurement with another sample of data.
     * @returns NO_RING, ONE_RING, or FOUR_RINGS
     */
    public int doInference(MeasurementData measurementData){
        return NO_RING;
    }
}
