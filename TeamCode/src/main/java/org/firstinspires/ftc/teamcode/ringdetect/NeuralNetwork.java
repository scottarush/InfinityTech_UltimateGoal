/*
 * Copyright (c) 2020, Scott Rush. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This software uses the open source Efficient Java Matrix Library.
 * See:
 * https://github.com/lessthanoptimal/ejml
 */

package org.firstinspires.ftc.teamcode.ringdetect;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.NormOps_DDRM;
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
 *
 * @author Scott Rush
 */
@SuppressWarnings("unchecked")
public class NeuralNetwork implements Serializable {

    public interface ITrainingStatusListener{
        void trainingStatus(int epochNumber, double cost, double[]del_L,double normDel_L);
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

    public static final int QUADRATIC_COST = 0;
    public static final int CROSS_ENTROPY_COST = 1;

    // parameters class
    static class Parameters implements Serializable{
        // Learning Rate
        public double eta = 0.005d;
        // Cost function for this network
        public int costFunction = CROSS_ENTROPY_COST;
    }

    Parameters mParameters = new Parameters();

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
            mParameters = (Parameters)ois.readObject();
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
            oos.writeObject(mParameters);
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
     * @param costFunction to use either QUADRATIC_COST or CROSS_ENTROPY_COST
     */
    public NeuralNetwork(int network[],int costFunction) {
        mNetwork = network;
        mParameters.costFunction = costFunction;
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
     * Computes current cost in the network.  This method is used for testing only.
     * @param a_xL Layer L activations returned from feedForward function
     * @param yk ground truth vector
     * @return current cost in network
     */
    public double computeCurrentCost(DMatrixRMaj yk,DMatrixRMaj a_xL){
        DMatrixRMaj dela_C = yk.createLike();
        subtract(a_xL, yk, dela_C);
        double cost = computeUnscaledCostTerm(dela_C,a_xL,yk);
        if (mParameters.costFunction == QUADRATIC_COST){
            // Scale by 1/2
            cost = cost/2d;
        }
        return cost;
    }

    /**
     * Utility computes the unscaled cost term.  Separate function called during both
     * backpropogation as well as feedforward testing
     * @param dela_C vector of∇aC
     * @param a_xL Lth layer activation vector
     * @param yk truth vector
     * @return current cost in network.
     */
    private double computeUnscaledCostTerm(DMatrixRMaj dela_C, DMatrixRMaj a_xL, DMatrixRMaj yk){
        double cost = 0;
        // Now compute and accumulate the cost on this cycle
        switch (mParameters.costFunction) {
            case QUADRATIC_COST:
                // quadratic cost according to:
                // C=1/2n * ∑ ∥y(x)−a^L(x)∥2 = 1/2n * ∑ ∥dela_C∥2
                cost = NormOps_DDRM.normF(dela_C);
                break;
            case CROSS_ENTROPY_COST:
            default:
                // Compute and accumulate the cross-entropy cost for this iteration according to:
                // C=−1/n ∑x ∑j [y,j *ln(a^L,j)+(1−y,j)ln(1−a^L,j)]
                cost = 0d;
                for (int j = 0; j < yk.numCols; j++) {
                    cost = cost + (yk.get(j) * Math.log(a_xL.get(j)) + (1d - yk.get(j)) * Math.log(1d - a_xL.get(j)));
                }
                break;
        }
        return cost;
    }
    /**
     * backpropogate
     * @param x numInputs X batchsize matrix of input batch
     * @param y numOutputs x batchsize matrix of output values
     * @return cost computed for this minibatch
     */
    private double backprop(DMatrixRMaj x, DMatrixRMaj y) {
        // Loop through the mini-batch one sample at a time saving the activations and
        // error deltas along the way

        // Allocate the cost sum for the batch cost.
        double costSum = 0;

        for (int column = 0; column < x.numCols; column++) {
            // Do the feedforward with a single column from x
            DMatrixRMaj inputVector = new DMatrixRMaj(x.numRows, 1);
            extractColumn(x, column, inputVector);
            feedForward(inputVector, mActivations[column]);

            // Step 1.  Compute the output error delta according to the cost function
            // used for this network:
            // For quadratic cost:  δ^x,L=∇aCx ⊙ σ′(z^x,L)
            // For cross-entropy:  δ^x,L=∇aCx

            // ∇aCx is just a^L(x)-y(x), the difference between the output activations and the y vector
            DMatrixRMaj yk = new DMatrixRMaj(y.numRows, 1);
            extractColumn(y, column, yk);
            DMatrixRMaj dela_C = yk.createLike();
            DMatrixRMaj a_xL = mActivations[column].get(mNetwork.length - 1);
            subtract(a_xL, yk, dela_C);

            // Now compute δ^x,L according to the cost function.
            // Compute reused vectors first
            DMatrixRMaj del_xl = new DMatrixRMaj(mNetwork[mNetwork.length - 1], 1);
            DMatrixRMaj sigma_prime = sigma_prime(a_xL);
            switch (mParameters.costFunction) {
                case QUADRATIC_COST:
                    // For quadratic, complete ⊙ σ′(z^x,L)
                    elementMult(dela_C, sigma_prime, del_xl);
                     break;
                case CROSS_ENTROPY_COST:
                default:
                    // for cross entropy,  δ^x,L=∇aCx
                    del_xl = dela_C;
                    break;
            }
            //  save δ^x,L to the mDeltas matrix
            mDeltas[column].set(mNetwork.length - 1, del_xl);

            // Now compute and accumulate the cost on this cycle
            costSum += computeUnscaledCostTerm(dela_C,a_xL,yk);

            // Step 2. back propogate the error from L-1,...2 according to
            // δ^x,l=((w^l+1)^T * δ^x,l+1) ⊙ σ′(z^x,l)
            // where σ′(z^x,l) = σ(z^x,l)(1-σ(z^x,l)) = a^x,l(1-a^x,l)
            for (int l = mNetwork.length - 2; l >= 0; l--) {
                DMatrixRMaj wl1_T = new DMatrixRMaj(mNetwork[l], mNetwork[l + 1]);
                transpose(mWeights.get(l), wl1_T);

                DMatrixRMaj del_xlm1 = new DMatrixRMaj(mNetwork[l], 1);
                mult(wl1_T, del_xl, del_xlm1);

                sigma_prime = sigma_prime(mActivations[column].get(l));
                elementMult(del_xlm1, sigma_prime);

                // Save error delta for use in Step 3
                mDeltas[column].set(l, del_xlm1);
                // shift delta for next layer down
                del_xl = del_xlm1;
            }
        }
        // complete finalize cost sum and return
        double n = (double) x.numCols;
        switch (mParameters.costFunction) {
            case QUADRATIC_COST:
                costSum = costSum / (2.0d * n);
                return costSum;
            case CROSS_ENTROPY_COST:
            default:
                costSum = -1d * costSum / n;
                return costSum;
        }
    }

    /**
     * stochastic gradient descent
     * Step 3.  Gradient descent, for each l=L,L−1,…,2
     * update weights according to: w^l→w^l−η/m * ∑ δ^x,l * (a^x,l−1)^T
     * and biases according to:  b^l→b^l−η/m ∑ δ^x,l
     *
     * Upon entry mActivations and mDeltas contain lists of the activation and delta matrices
     * from the last backpropogated network.
     * @param batchSize The number of valid values in this batches mActivations and mDeltas lists
     */
    private void sgd(int batchSize) {
        double eta_m = mParameters.eta /batchSize;

        // Loop through the layers in reverse from L, L-1,...,2
        // index l=0 corresponds to Layer 1
        // Start with l->L-1 (then l+1 -> L)
        for(int l=mNetwork.length-2;l >= 0;l--){
            // Create sum matrices for this layer
            DMatrixRMaj wsum = mWeights.get(l).createLike();
            wsum.zero();
            DMatrixRMaj bsum = mBiases.get(l).createLike();
            bsum.zero();
            for(int i=0;i < batchSize;i++){
                // Do the weights first
                DMatrixRMaj del_xl = mDeltas[i].get(l+1);
                DMatrixRMaj a_xlm1 = mActivations[i].get(l);

                // compute (a^x,l−1)^T
                DMatrixRMaj a_xlm1_T = new DMatrixRMaj(a_xlm1.numCols,a_xlm1.numRows);
                transpose(a_xlm1,a_xlm1_T);
                // δ^x,l * (a^x,l−1)^T
                DMatrixRMaj wtemp = wsum.createLike();
                mult(del_xl,a_xlm1_T,wtemp);
                // And add the product
                add(wsum,wtemp,wsum);
                // multiply by eta/m
                scale(eta_m,wsum,wtemp);
                // and subtract off of the weight layer
                subtract(mWeights.get(l),wtemp,mWeights.get(l));

                // Now the bias updates
                DMatrixRMaj btemp = bsum.createLike();
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
        DMatrixRMaj retsigma = z.createLike();
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
     */
    public void train(String description,SimpleMatrix x, SimpleMatrix y, SimpleMatrix xscale,int batchSize, double eta, int numEpochs) {
        mParameters.eta = eta;
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

            // shuffle data every epoch
            // shuffle the training data to randomize the batch picks
            int[] columns = NeuralNetworkMatrixUtils.genShuffleColumnIndexVector(y.numCols());
            x = NeuralNetworkMatrixUtils.shuffleMatrix(x, columns);
            y = NeuralNetworkMatrixUtils.shuffleMatrix(y, columns);

            int batchColumnIndex = 0;

            boolean continueBatch = true;
            int lastBatchSize = 0;
            double cost = 0d;
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
                // backprop the batch and accumulate the cost on this batch for the epoch
                cost += backprop(xbatch, ybatch);

                // do stochastic gradient descent iteration on this batch
                sgd(lastBatchSize);

                // Update the batchColumnIndex for next run
                batchColumnIndex = batchEndColumn;
            }
            // Compute and save the output error for this epoch to the log array using
            // the error deltas from the last layer of the last entry in the last batch
            DMatrixRMaj lastDelta = mDeltas[lastBatchSize - 1].get(mNetwork.length - 1);
            double del_L[] = new double[lastDelta.numRows];
            for (int i = 0; i < lastDelta.numRows; i++) {
                del_L[i] = lastDelta.get(i, 0);
            }
            double normalError = SimpleMatrix.wrap(lastDelta).normF();
            // And notify the status listeners
            for (Iterator<ITrainingStatusListener> iter = mTrainingStatusListeners.iterator(); iter.hasNext(); ) {
                ITrainingStatusListener listener = iter.next();
                listener.trainingStatus(epochIndex + 1, cost,del_L, normalError);
            }
        }
    }

    public String printNetwork() {
        StringBuffer buffer = new StringBuffer();
        for (int i = 0; i < mWeights.size(); i++) {
            DMatrixRMaj w = mWeights.get(i);
            DMatrixRMaj b = mBiases.get(i);
            buffer.append(NeuralNetworkMatrixUtils.printMatrix(w, "w_layer" + i));
            buffer.append("\n\r");
            buffer.append(NeuralNetworkMatrixUtils.printMatrix(b, "b_layer" + i));
            buffer.append("\n\r");
        }
        return buffer.toString();
    }

}
