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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;

/**
 * Estimates 3D position using WiFi signals indoor and the Weighted k-Nearest
 * Neighbours (WkNN) algorithm.
 * WkNN algorithm is based on https://github.com/ajnas/WiFiPS.
 */
public class WeightedKNearestNeighboursPositionSolver3D extends
        WeightedKNearestNeighboursPositionSolver<Point3D> {

    /**
     * Constructor.
     */
    public WeightedKNearestNeighboursPositionSolver3D() {
        super();
    }

    /**
     * Constructor.
     * Sets known located WiFi fingerprints and euclidean distances between WiFi
     * signal fingerprints.
     *
     * @param fingerprints known located WiFi fingerprints.
     * @param distances    euclidean distances between WiFi signal fingerprints
     *                     (expressed in dB's).
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     */
    public WeightedKNearestNeighboursPositionSolver3D(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] fingerprints,
            final double[] distances) {
        super(fingerprints, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    public WeightedKNearestNeighboursPositionSolver3D(
            final WeightedKNearestNeighboursPositionSolverListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets known located WiFi fingerprints and euclidean distances between WiFi
     * signal fingerprints.
     *
     * @param fingerprints known located WiFi fingerprints.
     * @param distances    euclidean distances between WiFi signal fingerprints
     *                     (expressed in dB's).
     * @param listener     listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     */
    public WeightedKNearestNeighboursPositionSolver3D(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] fingerprints,
            final double[] distances,
            final WeightedKNearestNeighboursPositionSolverListener<Point3D> listener) {
        super(fingerprints, distances, listener);
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    @Override
    public Point3D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        final Point3D result = new InhomogeneousPoint3D();
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
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }
}
