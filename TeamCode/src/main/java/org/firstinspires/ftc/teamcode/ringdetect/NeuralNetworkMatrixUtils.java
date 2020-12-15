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
import org.ejml.simple.SimpleMatrix;

public class NeuralNetworkMatrixUtils {
    /**
     * generates a shuffled vector of columns for use in shuffling data
     */
    public static int[] genShuffleColumnIndexVector(int length) {
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
     *
     * @param source  source matrix to shuffle
     * @param columns array of shuffled column indexes to shuffle
     * @return shuffled matrix
     */
    public static SimpleMatrix shuffleMatrix(SimpleMatrix source, int[] columns) {
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

    /**
     * Utility to format a matrix in java 2D array initializer format for outputting weights
     * to use as an initializer at runtime
     *
     * @param matrix
     * @return
     */
    public static String printMatrix(DMatrixRMaj matrix, String name) {
        StringBuffer buffer = new StringBuffer();
        buffer.append("final double " + name + "[][]= {\n\r");
        for (int row = 0; row < matrix.numRows; row++) {
            buffer.append("{");
            for (int column = 0; column < matrix.numCols; column++) {
                double value = matrix.get(row, column);
                buffer.append(String.format("%1.5f", value));
                if (column < matrix.numCols - 1) {
                    buffer.append(",");
                }
            }
            buffer.append("}");
            if (row < matrix.numRows - 1) {
                buffer.append(",\n\r");
            } else {
                buffer.append("};");
            }
        }
        return buffer.toString();
    }

    /**
     * Utility to print a column of a matrix
     *
     * @param matrix
     * @return
     */
    public static String printColumn(SimpleMatrix matrix, int column) {
        StringBuffer buffer = new StringBuffer();
        buffer.append("{");
        for (int row = 0; row < matrix.numRows(); row++) {
            double value = matrix.get(row, column);
            buffer.append(String.format("%1.5f", value));
            if (row < matrix.numRows() - 1) {
                buffer.append(",");
            } else {
                buffer.append("}");
            }
        }
        return buffer.toString();
    }

    /**
     * @param x matrix to normalize
     * @return column vector of scaling coefficients for each row.
     */
    public static SimpleMatrix normalizeRows(SimpleMatrix x) {
        double[] maxArray = new double[x.numRows()];
        for (int i = 0; i < maxArray.length; i++) {
            maxArray[i] = 0f;
        }
        // Compute the maxes
        for (int column = 0; column < x.numCols(); column++) {
            for (int row = 0; row < x.numRows(); row++) {
                if (x.get(row, column) > maxArray[row]) {
                    maxArray[row] = x.get(row, column);
                }
            }
        }
        // Compute the scale factors to normalize to 1.000
        SimpleMatrix scaleFactors = new SimpleMatrix(maxArray.length, 1);
        double max = 1.0d;
        for (int i = 0; i < scaleFactors.numRows(); i++) {
            scaleFactors.set(i, 0, max / maxArray[i]);
        }
        // Now normalize to using the scaleFactors
        for (int column = 0; column < x.numCols(); column++) {
            for (int row = 0; row < x.numRows(); row++) {
                double value = x.get(row, column) * scaleFactors.get(row, 0);
                x.set(row, column, value);
            }
        }
        return scaleFactors;
    }
}
