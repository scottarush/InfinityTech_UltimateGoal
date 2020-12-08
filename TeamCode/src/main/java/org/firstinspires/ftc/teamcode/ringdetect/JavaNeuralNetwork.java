package org.firstinspires.ftc.teamcode.ringdetect;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Random;

/**
 * This class implements the neural network.
 */
public class JavaNeuralNetwork {
    // Array of weights in each layer from layer 2..L (total of L-1 elements)
    ArrayList<SimpleMatrix> mWeights = null;
    // Array of biases in each layer from layer 2..L (total of L-1 elements)
    ArrayList<SimpleMatrix> mBiases = null;

    // Array of activations for each layer from layer 1..L
    ArrayList<SimpleMatrix> mActivations = null;

    // Array of delta errors for each layer from layer 2..L (total of L-1 elements)
    ArrayList<SimpleMatrix> mDeltas = null;

    // Batch size
    private int mBatchSize = 5;

    // Learning Rate
    private double mETA = 0.005d;

    // Matrix of training data
    private SimpleMatrix mX = null;
    // Index of current column for batch in mXBatch and mYBatch arrays
    private int mBatchColumnIndex = 0;
    // Current batch of training data extracted from mX
    private SimpleMatrix mXBatch = null;
    // Row Vector of tag data corresponding to X
    private SimpleMatrix mY = null;
    // Current batch of training data extracted from mY
    private SimpleMatrix mYBatch = null;

    // Number of nodes in each layer starting from the input layer to the output
    private int[] mNetwork = null;

    /**
     * Creates a new neural network with randomized weights
     * @param network array containing number of nodes in each layer
     */
    public JavaNeuralNetwork(int network[]) {
        mNetwork = network;
        mWeights = new ArrayList<>();
        mBiases = new ArrayList<>();
        // Just set the size of the activation L with null entries
        mActivations = new ArrayList<>(network.length);
        // And deltas to L-1
        mDeltas = new ArrayList<>(network.length-1);

        // And initialize the weigth and bias matrices
        for (int i = 0; i < network.length - 1; i++) {
            // Add a total of L-1 weight and bias matrices
            SimpleMatrix w = SimpleMatrix.random_DDRM(network[i + 1], network[i], -1.0d, 1.0d, new Random());
            mWeights.add(w);
            SimpleMatrix b = SimpleMatrix.random_DDRM(network[i + 1], 1, -1.0d, 1.0d, new Random());
            mBiases.add(b);
        }
    }

    /**
     * Feedforward method loops through
     * z^x,l = w^l * a^x,l−1 + b^l and a^x,l=σ(z^x,l).
     */
    public void feedForward(SimpleMatrix input) {
        // Set the input and save in the activations array
        SimpleMatrix a_xlm1 = input;
        mActivations.set(0, a_xlm1);
        // and loop L-1 times starting with layer 0 up to L-1
        SimpleMatrix z_xl = null;
        for (int l = 0; l < mNetwork.length - 1; l++) {
            SimpleMatrix wl = mWeights.get(l);
            SimpleMatrix bl = mBiases.get(l);
            z_xl = wl.mult(a_xlm1).plus(bl);

            SimpleMatrix a_xl = sigma(z_xl);
            mActivations.set(l+1,a_xl);
        }
    }
    /**
     * backpropogate
     */
    public void backprop(SimpleMatrix x, SimpleMatrix y) {
        // Step 1.  Compute the output error delta according to δ^x,L=∇aCx ⊙ σ′(z^x,L)

            // ∇aCx is just the difference between the output activations and the y vector
        SimpleMatrix cost = mActivations.get(mNetwork.length-1).minus(y);
            // the last term is just the output layer activation primed
        SimpleMatrix sigma_prime = sigma_prime(mActivations.get(mNetwork.length-1));
        // Compute and save the layer L error
        SimpleMatrix del_xl = cost.elementMult(sigma_prime);
        mDeltas.set(mNetwork.length-1,del_xl);

        // Step 2. back propogate the error from L-1,...2 according to
        // δ^x,l=((w^l+1)^T * δ^x,l+1) ⊙ σ′(z^x,l)
        // where σ′(z^x,l) = σ(z^x,l)(1-σ(z^x,l)) = a^x,l(1-a^x,l)
        for(int l=mNetwork.length-2;l > 0;l--){
            SimpleMatrix wl1_T = mWeights.get(l+1).transpose();
            sigma_prime = sigma_prime(mActivations.get(l));
            SimpleMatrix del_xlm1 = wl1_T.mult(del_xl).elementMult(sigma_prime);
            // Save error delta for use in Step 3
            mDeltas.set(l,del_xlm1);
            // shift delta for next layer down
            del_xl = del_xlm1;
        }
        // Step 3.  Gradient descent, for each l=L,L−1,…,2
        // update weights according to: w^l→w^l−η/m * ∑ δ^x,l(ax,l−1)T
        // and biases according to:  b^l→b^l−η/m ∑ δ^x,l

    }
    /**
     * Computes the sigmoid of a matrix
     *
     * @param z
     * @return vector of the sigmoided matrix
     */
    private SimpleMatrix sigma(SimpleMatrix z) {
        // sigma = 1 / (1+e^-z)
        SimpleMatrix retsigma = new SimpleMatrix(z);
        for(int row=0;row < z.numRows();row++){
            double s = 1/(1+Math.exp(-z.get(row,0)));
            retsigma.set(row,0,s);
        }
        return retsigma;
    }

    /**
     * Computes the derivative of the sigmoid from the sigmoid
     * @param sigmaVector sigmoid vector computed from sigma function
     * @return derivative vector
     */
    private SimpleMatrix sigma_prime(SimpleMatrix sigmaVector) {
        // sigma' = sigma * (1 - sigma)
        SimpleMatrix identity = SimpleMatrix.identity(sigmaVector.numRows());
        SimpleMatrix sigmap = sigmaVector.elementMult(identity.minus(sigmaVector));
        return sigmap;
    }

    /**
     * Called to train
     *
     * @param x         X values must be numInputLayerNodes x NumSamples
     * @param y         Y values must be numOutputLayerNodes x NumSamples
     * @param batchSize number of samples per epoch
     * @param eta       learning rate
     * @param numEpochs
     */
    public void train(SimpleMatrix x, SimpleMatrix y, int batchSize, double eta, int numEpochs) {
        mX = x;
        mY = y;
        mBatchSize = batchSize;
        mETA = eta;

        // First shuffle the training data
        int[] columns = genShuffleColumnIndexVector(mX.numCols());
        mX = shuffleMatrix(mX, columns);
        mY = shuffleMatrix(mY, columns);
        // Reset the batch column index
        mBatchColumnIndex = 0;

        // And loop through the epochs
        for (int epochIndex = 0; epochIndex < numEpochs; epochIndex++) {
            // Get next batch of training data
            extractNextBatch();
            for (int batchIndex = 0; batchIndex < mXBatch.numCols(); batchIndex++) {
                SimpleMatrix xt = mXBatch.cols(batchIndex, batchIndex + 1);
                // And do the feed forward
                feedForward(xt);
            }

        }
    }

    /**
     * Extracts the next batch and updates indexes for next time
     */
    private void extractNextBatch() {
        int batchSize = mBatchSize;
        int remain = mX.numCols() - mBatchColumnIndex;
        if (remain < batchSize) {
            batchSize = remain;
        }
        int batchEndColumn = mBatchColumnIndex + batchSize;
        mXBatch = mX.cols(mBatchColumnIndex, batchEndColumn);
    }

    /**
     * generates a shuffled vector of columns for use in shuffling data
     */
    private int[] genShuffleColumnIndexVector(int length) {
        int[] retarray = new int[length];
        for (int i = 0; i < retarray.length; i++) {
            retarray[i] = i;
        }
        // Now shuffle them in a loop
        for (int i = 0; i < 20 * length; i++) {
            int swapCol1 = 1 + (int) (Math.random() * length - 1);
            int swapCol2 = 1 + (int) (Math.random() * length - 1);
            int col2Val = retarray[swapCol2];
            retarray[swapCol2] = retarray[swapCol1];
            retarray[swapCol1] = col2Val;
        }
        return retarray;
    }

    /**
     * Shuffles the columns of a matrix and returns a shuffled copy.
     */
    private SimpleMatrix shuffleMatrix(SimpleMatrix source, int[] columns) {
        SimpleMatrix shuffle = null;
        for (int i = 0; i < columns.length; i++) {
            SimpleMatrix col = source.extractVector(false, columns[i]);
            if (shuffle == null) {
                shuffle = col;
            } else {
                shuffle = shuffle.concatColumns(col);
            }
        }
        return shuffle;
    }

    public String printNetwork() {
        StringBuffer buffer = new StringBuffer();
        for (int i = 0; i < mWeights.size(); i++) {
            SimpleMatrix w = mWeights.get(i);
            SimpleMatrix b = mBiases.get(i);
            buffer.append(printMatrix(w, "w_layer" + i));
            buffer.append("\n\r");
            buffer.append(printMatrix(b, "b_layer" + i));
            buffer.append("\n\r");
        }
        return buffer.toString();
    }

    /**
     * Utility to format a matrix in java 2D array initializer format for outputting weights
     * to use as an initializer at runtime
     *
     * @param matrix
     * @return
     */
    public static String printMatrix(SimpleMatrix matrix, String name) {
        StringBuffer buffer = new StringBuffer();
        buffer.append("final double " + name + "[][]= {\n\r");
        for (int row = 0; row < matrix.numRows(); row++) {
            buffer.append("{");
            for (int column = 0; column < matrix.numCols(); column++) {
                double value = matrix.get(row, column);
                buffer.append(String.format("%1.10f", value));
                if (column < matrix.numCols() - 1) {
                    buffer.append(",");
                }
            }
            buffer.append("}");
            if (row < matrix.numRows() - 1) {
                buffer.append(",\n\r");
            } else {
                buffer.append("};");
            }
        }
        return buffer.toString();
    }
}
