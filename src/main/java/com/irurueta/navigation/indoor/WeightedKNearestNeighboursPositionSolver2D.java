/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.indoor;

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;

/**
 * Estimates 2D position using Wi-Fi signals indoor and the Weighted k-Nearest
 * Neighbours (WkNN) algorithm.
 * WkNN algorithm is based on <a href="https://github.com/ajnas/WiFiPS">https://github.com/ajnas/WiFiPS</a>.
 */
public class WeightedKNearestNeighboursPositionSolver2D extends WeightedKNearestNeighboursPositionSolver<Point2D> {

    /**
     * Constructor.
     */
    public WeightedKNearestNeighboursPositionSolver2D() {
        super();
    }

    /**
     * Constructor.
     * Sets known located Wi-Fi fingerprints and Euclidean distances between Wi-Fi
     * signal fingerprints.
     *
     * @param fingerprints known located Wi-Fi fingerprints.
     * @param distances    Euclidean distances between Wi-Fi signal fingerprints
     *                     (expressed in dB's).
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     */
    public WeightedKNearestNeighboursPositionSolver2D(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] fingerprints,
            final double[] distances) {
        super(fingerprints, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    public WeightedKNearestNeighboursPositionSolver2D(
            final WeightedKNearestNeighboursPositionSolverListener<Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets known located Wi-Fi fingerprints and Euclidean distances between Wi-Fi
     * signal fingerprints.
     *
     * @param fingerprints known located Wi-Fi fingerprints.
     * @param distances    Euclidean distances between Wi-Fi signal fingerprints
     *                     (expressed in dB's).
     * @param listener     listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     */
    public WeightedKNearestNeighboursPositionSolver2D(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] fingerprints,
            final double[] distances, final WeightedKNearestNeighboursPositionSolverListener<Point2D> listener) {
        super(fingerprints, distances, listener);
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    @Override
    public Point2D getEstimatedPosition() {
        if (estimatedPositionCoordinates == null) {
            return null;
        }

        final var result = new InhomogeneousPoint2D();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Gets number of dimensions of location points.
     *
     * @return number of dimensions of location points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }
}
