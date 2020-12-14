package org.firstinspires.ftc.teamcode.ringdetect;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Random;
import static org.ejml.dense.row.CommonOps_DDRM.mult;
import static org.ejml.dense.row.CommonOps_DDRM.add;
import static org.ejml.dense.row.CommonOps_DDRM.elementMult;
import static org.ejml.dense.row.CommonOps_DDRM.subtract;
import static org.ejml.dense.row.CommonOps_DDRM.transpose;
import static org.ejml.dense.row.CommonOps_DDRM.scale;
import static org.ejml.dense.row.CommonOps_DDRM.extractColumn;
import static org.ejml.dense.row.CommonOps_DDRM.fill;

/**
 * This class implements the neural network.  Comments notation is from:
 * http://neuralnetworksanddeeplearning.com/chap2.html
 */
@SuppressWarnings("unchecked")
public class NeuralNetwork implements Serializable {

    public interface ITrainingStatusListener{
        void trainingStatus(int epochNumber, double normalError);
    }
    private String mDescription = "No Description";
    // List weight matrices in each layer from layer 2..L (total of L-1 elements)
    // element 0 => Layer 2
    private ArrayList<DMatrixRMaj> mWeights = null;
    // List of biase matrices in each layer from layer 2..L (total of L-1 elements)
    // element 0 => Layer 2
    private ArrayList<DMatrixRMaj> mBiases = null;

    // vector of scale vectors to scale input data.
    private DMatrixRMaj mInputScaleVector = null;

    // Array of activation matrices used in backpropogation.  Each element contains L layers
    private ArrayList<DMatrixRMaj>[] mActivations = null;

    // Array of delta arrays used in backpropogation.  Each element contains L layers
    private ArrayList<DMatrixRMaj>[] mDeltas = null;

    // Learning Rate
    private double mETA = 0.005d;

    // Number of nodes in each layer starting from the input layer to the output
    int[] mNetwork = null;

    // Listeners for status updates at end of each epoch
    private ArrayList<ITrainingStatusListener> mTrainingStatusListeners = new ArrayList<>();

    /**
     * Default constructor used only with an existing network that will be deserialized via deserializeNetwork()
     */
    public NeuralNetwork(){

    }

    /**
     * Deserializes an existing neural network from the FileInputStream.  This is the function
     * used at runtime feedforward deployment.  It can also be used to further train an existing
     * network.
     * @param is FileInputStream pointing to the Serialized JavaNeuralNetwork object
     * @throws Exception if there was a problem reading the serialized object from the fis
     */
    public void deserializeNetwork(InputStream is) throws Exception {
        try{
            ObjectInputStream ois = new ObjectInputStream(is);
            mDescription = (String) ois.readObject();
            mNetwork = (int[]) ois.readObject();
            mWeights = (ArrayList<DMatrixRMaj>) ois.readObject();
            mBiases = (ArrayList<DMatrixRMaj>) ois.readObject();
            mInputScaleVector = (DMatrixRMaj) ois.readObject();
        }
        catch(IOException e){
            throw new Exception("IOException reading neural network:"+e.getMessage());
        }
        catch(ClassNotFoundException e){
            throw new Exception("ClassNotFoundException reading neural network:"+e.getMessage());
        }
    }

    /**
     * Serialzies the neural network to the supplied FileOutputStream
     * @param fos FileOutputStream to save the network
     * @throws Exception if there was a problem writing the serialized object
     */
    public void serializeNetwork(FileOutputStream fos) throws Exception{
        try{
            ObjectOutputStream oos = new ObjectOutputStream(fos);
            oos.writeObject(mDescription);
            oos.writeObject(mNetwork);
            oos.writeObject(mWeights);
            oos.writeObject(mBiases);
            oos.writeObject(mInputScaleVector);
            oos.close();
        }
         catch(IOException e) {
             throw new Exception("IOException writing neural network:" + e.getMessage());
         }
    }
    /**
     * Returns the name of the network at the time that it was last trained.
     */
    public String getDescription(){
        return mDescription;
    }
    /**
     * Creates a new neural network with randomized weights.   This constructor is
     * used only for training a new network.
     * @param network array containing number of nodes in each layer
     */
    public NeuralNetwork(int network[]) {
        mNetwork = network;
        mWeights = new ArrayList<>();
        mBiases = new ArrayList<>();

        // And initialize the weigth and bias matrices
        for (int i = 0; i < network.length - 1; i++) {
            // Add a total of L-1 weight and bias matrices
            SimpleMatrix w = SimpleMatrix.random_DDRM(network[i + 1], network[i], -1.0d, 1.0d, new Random());
            mWeights.add(w.getDDRM());
            SimpleMatrix b = SimpleMatrix.random_DDRM(network[i + 1], 1, -1.0d, 1.0d, new Random());
            mBiases.add(b.getDDRM());
        }
    }

    /**
    * @return the column vector of scale factors used to scale the input data for feedforward runtime operation.
    *
     **/
    public DMatrixRMaj getInputScaleVector(){
        return mInputScaleVector;
    }
    /**
     * Adds a training status listeners
     */
    public void addTrainingStatusListener(ITrainingStatusListener listener){
        if (mTrainingStatusListeners.contains(listener))
            return;
        mTrainingStatusListeners.add(listener);
    }

    /**
     * Runtime feedforward method. This method uses raw input data that has NOT been scaled with
     * the normalization factors for this network.
     * @param rawInput vector of raw input node data that has NOT  been normalized.
     *             Must have same number of rows as input layer nodes and single column.
     * @returns column vector of output node activations
     */
    public SimpleMatrix feedForward(SimpleMatrix rawInput){
        // Scale the data first and then run the feedForward algorithm
        DMatrixRMaj dm = rawInput.getDDRM();
        elementMult(dm,mInputScaleVector);
        SimpleMatrix output = SimpleMatrix.wrap(feedForward(dm,null));
        return output;
    }
    /**
     * Training  feedfordward method. This method is used for training back-propogation only.
     * This method does NOT scale the data and must have normalized input data.
     *
     * Feedforward method loops through
     * z^x,l = w^l * a^x,l−1 + b^l and a^x,l=σ(z^x,l).
     * This is the actual feedforward method that requires normalized input data and is
     * called directly for training.  The other feedforward signature is used for runtime
     * which takes unscaled raw input data.
     *
     * @param normalizedInput vector of normalized input node activations.
     * @param activations reference to activation matrix to fill with values for training or
     *                    nullfor run-time (and values will not be saved).
     * @return vector of output node activations.  Only use for runtime.  Training uses the supplied list
     */
    private DMatrixRMaj feedForward(DMatrixRMaj normalizedInput,ArrayList<DMatrixRMaj> activations) {
        // Set the input and save in the activations array
        DMatrixRMaj a_xlm1 = normalizedInput;
        if (activations != null) {
            activations.set(0, a_xlm1);
        }
        // and loop L-1 times starting with layer 0 up to L-1
        for (int l = 0; l < mNetwork.length - 1; l++) {
            DMatrixRMaj wl = mWeights.get(l);
            DMatrixRMaj bl = mBiases.get(l);

            DMatrixRMaj z_xl = new DMatrixRMaj(bl);
            mult(wl,a_xlm1,z_xl);
            add(z_xl,bl,z_xl);

            DMatrixRMaj a_xl = sigma(z_xl);
            // If training then save the resultant activations to the supplied matrix
            if (activations != null)
                activations.set(l+1,a_xl);
            // and save a^x,l as a^x,l-1 for time
            a_xlm1 = a_xl;
        }
        // Return last computed activation matrix as the output node values.
        return a_xlm1;
    }
    /**
     * backpropogate
     * @param x numInputs X batchsize matrix of input batch
     * @param y numOutputs x batchsize matrix of output values
     */
    private void backprop(DMatrixRMaj x, DMatrixRMaj y) {
        // Loop through the mini-batch one sample at a time saving the activations and
        // error deltas along the way
        for(int column=0;column < x.numCols;column++) {
            // Do the feedforward with a single column from x
            DMatrixRMaj inputVector = new DMatrixRMaj(x.numRows,1);
            extractColumn(x,column,inputVector);
            feedForward(inputVector,mActivations[column]);

            // Step 1.  Compute the output error delta according to δ^x,L=∇aCx ⊙ σ′(z^x,L)
            // ∇aCx is just the difference between the output activations and the y vector
            DMatrixRMaj outputVector = new DMatrixRMaj(y.numRows,1);
            extractColumn(y,column,outputVector);
            DMatrixRMaj cost = new DMatrixRMaj(outputVector);
            DMatrixRMaj a_xL = mActivations[column].get(mNetwork.length - 1);
            subtract(a_xL,outputVector,cost);
            // the last term is just the output layer activation primed
            DMatrixRMaj sigma_prime = sigma_prime(a_xL);
            // Compute and save the layer L error
            DMatrixRMaj del_xl = new DMatrixRMaj(mNetwork[mNetwork.length-1],1);
            elementMult(cost,sigma_prime,del_xl);
            mDeltas[column].set(mNetwork.length - 1, del_xl);

            // Step 2. back propogate the error from L-1,...2 according to
            // δ^x,l=((w^l+1)^T * δ^x,l+1) ⊙ σ′(z^x,l)
            // where σ′(z^x,l) = σ(z^x,l)(1-σ(z^x,l)) = a^x,l(1-a^x,l)
            for (int l = mNetwork.length - 2; l >= 0; l--) {
                DMatrixRMaj wl1_T = new DMatrixRMaj(mNetwork[l],mNetwork[l+1]);
                transpose(mWeights.get(l),wl1_T);

                DMatrixRMaj del_xlm1 = new DMatrixRMaj(mNetwork[l],1);
                mult(wl1_T,del_xl,del_xlm1);

                sigma_prime = sigma_prime(mActivations[column].get(l));
                elementMult(del_xlm1,sigma_prime);

                // Save error delta for use in Step 3
                mDeltas[column].set(l, del_xlm1);
                // shift delta for next layer down
                del_xl = del_xlm1;
            }
        }
    }
    /**
     * stochastic gradient descent
     * Step 3.  Gradient descent, for each l=L,L−1,…,2
     * update weights according to: w^l→w^l−η/m * ∑ δ^x,l(a^x,l−1)^T
     * and biases according to:  b^l→b^l−η/m ∑ δ^x,l
     *
     * Upon entry mActivations and mDeltas contain lists of the activation and delta matrices
     * from the last backpropogated network.
     * @param batchSize The number of valid values in this batches mActivations and mDeltas lists
     */
    private void sgd(int batchSize) {
        double eta_m = mETA/batchSize;

        // Loop through the layers in reverse from layer=L, L-1,...,2
        // index l=0 corresponds to Layer 1
        for(int l=mNetwork.length-2;l >= 0;l--){
            // Create sum matrices for this layer
            DMatrixRMaj wsum = new DMatrixRMaj(mWeights.get(l));
            wsum.zero();
            DMatrixRMaj bsum = new DMatrixRMaj(mBiases.get(l));
            bsum.zero();
            for(int i=0;i < batchSize;i++){
                // Do the weights first
                DMatrixRMaj del_xl = mDeltas[i].get(l+1);
                DMatrixRMaj a_xlm1 = mActivations[i].get(l);

                DMatrixRMaj a_xlm1_T = new DMatrixRMaj(a_xlm1.numCols,a_xlm1.numRows);
                transpose(a_xlm1,a_xlm1_T);

                DMatrixRMaj wtemp = new DMatrixRMaj(wsum);
                mult(del_xl,a_xlm1_T,wtemp);
                add(wsum,wtemp,wsum);
                scale(eta_m,wsum,wtemp);
                subtract(mWeights.get(l),wtemp,mWeights.get(l));

                // Now the bias updates
                DMatrixRMaj btemp = new DMatrixRMaj(bsum);
                add(bsum,del_xl,bsum);
                scale(eta_m,bsum,btemp);

                subtract(mBiases.get(l),btemp,mBiases.get(l));
            }
        }
    }

    /**
     * Computes the sigmoid of a matrix
     *
     * @param z
     * @return vector of the sigmoided matrix
     */
    private DMatrixRMaj sigma(DMatrixRMaj z) {
        // sigma = 1 / (1+e^-z)
        DMatrixRMaj retsigma = new DMatrixRMaj(z);
        for (int row = 0; row < z.numRows; row++) {
            double s = 1d / (1d + Math.exp(-z.get(row, 0)));
            retsigma.set(row, 0, s);
        }
        return retsigma;
    }

    /**
     * Computes the derivative of the sigmoid from the sigmoid
     * @param sigmaVector sigmoid vector computed from sigma function
     * @return derivative vector
     */
    private DMatrixRMaj sigma_prime(DMatrixRMaj sigmaVector) {
        // sigma' = sigma * (1 - sigma)
        DMatrixRMaj sigmap = new DMatrixRMaj(sigmaVector.numRows,1);
        fill(sigmap,1d);
        subtract(sigmap,sigmaVector,sigmap);
        elementMult(sigmap,sigmaVector);
        return sigmap;
    }

    /**
     * Called to train
     *
     * @param x         X values must be numInputLayerNodes x NumSamples
     * @param y         Y values must be numOutputLayerNodes x NumSamples
     * @param xscale        column vector of values used to scale each X row.
     * @param batchSize number of samples per epoch
     * @param eta       learning rate
     * @param numEpochs
     * @param shuffleEachEpoch shuffle training data on each Epoch when true (usually need to do this)
     */
    public void train(String description,SimpleMatrix x, SimpleMatrix y, SimpleMatrix xscale,int batchSize, double eta, int numEpochs,boolean shuffleEachEpoch) {
        mETA = eta;
        mInputScaleVector = xscale.getDDRM();
        mDescription = description;

        // Allocate the activation and delta matrices needed for backpropogation according to the batch size
        mActivations = new ArrayList[batchSize];
        mDeltas = new ArrayList[batchSize];
        for (int i = 0; i < batchSize; i++) {
            mActivations[i] = new ArrayList<>(mNetwork.length);
            mDeltas[i] = new ArrayList<>(mNetwork.length);
            // Initialize the lists with matrices
            for(int l=0;l < mNetwork.length;l++){
                mActivations[i].add(new DMatrixRMaj(mNetwork[l]));
                mDeltas[i].add(new DMatrixRMaj(mNetwork[l]));
            }
        }
        // Loop through the epochs with one mini-batch at a time
        for (int epochIndex = 0; epochIndex < numEpochs; epochIndex++) {
            // Start a new epoch

             // Normally we want to shuffle data every epoch, but allow the caller to
            // determine.
            if (shuffleEachEpoch) {
                // shuffle the training data to randomize the batch picks
                int[] columns = MatrixUtils.genShuffleColumnIndexVector(y.numCols());
                x = MatrixUtils.shuffleMatrix(x, columns);
                y = MatrixUtils.shuffleMatrix(y, columns);
            }
            int batchColumnIndex = 0;

            boolean continueBatch = true;
            int lastBatchSize = 0;
            while (continueBatch) {
                // Get next batch of training data
                int remain = x.numCols() - batchColumnIndex;
                lastBatchSize = batchSize;
                if (remain <= batchSize) {
                    lastBatchSize = remain;
                    // Clear continueBatch to drop out of the loop after this one
                    continueBatch = false;
                }
                int batchEndColumn = batchColumnIndex + lastBatchSize;
                DMatrixRMaj xbatch = x.cols(batchColumnIndex, batchEndColumn).getDDRM();
                DMatrixRMaj ybatch = y.cols(batchColumnIndex, batchEndColumn).getDDRM();
                // backprop the batch passing the activations
                backprop(xbatch, ybatch);

                // do stochastic gradient descent iteration on this batch
                sgd(lastBatchSize);

                // Update the batchColumnIndex for next run
                batchColumnIndex = batchEndColumn;
             }
            // Compute and save the output error for this epoch to the log array using
            // the error deltas from the last layer of the last entry in the last batch
            DMatrixRMaj lastDelta = mDeltas[lastBatchSize - 1].get(mNetwork.length - 1);
            double normalError = SimpleMatrix.wrap(lastDelta).normF();
            // And notify the status listeners
            for(Iterator<ITrainingStatusListener>iter=mTrainingStatusListeners.iterator();iter.hasNext();){
                ITrainingStatusListener listener = iter.next();
                listener.trainingStatus(epochIndex+1,normalError);
            }
        }
    }

    public String printNetwork() {
        StringBuffer buffer = new StringBuffer();
        for (int i = 0; i < mWeights.size(); i++) {
            DMatrixRMaj w = mWeights.get(i);
            DMatrixRMaj b = mBiases.get(i);
            buffer.append(MatrixUtils.printMatrix(w, "w_layer" + i));
            buffer.append("\n\r");
            buffer.append(MatrixUtils.printMatrix(b, "b_layer" + i));
            buffer.append("\n\r");
        }
        return buffer.toString();
    }

}
