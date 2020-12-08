package org.firstinspires.ftc.teamcode.ringdetect;

public class RingNeuralNetwork extends JavaNeuralNetwork {
    public static final int INPUT_LAYER_NODES = 13;
    public static final int OUTPUT_LAYER_NODES = 3;

    public RingNeuralNetwork() {
        super(new int[]{INPUT_LAYER_NODES, OUTPUT_LAYER_NODES});
    }
}