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

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;

/**
 * Estimates position using Wi-Fi signals indoor and the Weighted k-Nearest
 * Neighbours (WkNN) algorithm.
 * WkNN algorithm is based on <a href="https://github.com/ajnas/WiFiPS">https://github.com/ajnas/WiFiPS</a>.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class WeightedKNearestNeighboursPositionSolver<P extends Point<?>> {

    /**
     * Default minimum allowed distance between received Wi-Fi fingerprints.
     */
    public static final double DEFAULT_EPSILON = 1e-7;

    /**
     * Minimum required number of fingerprints and their distances.
     * If only 1 fingerprint is used, this algorithm will return provided fingerprint
     * position, however, some accuracy might be lost due to numerical computations.
     * For that reason, when only one fingerprint is provided, this algorithm will
     * simply return the fingerprint position.
     */
    public static final int MIN_FINGERPRINTS = 1;

    /**
     * Known located Wi-Fi fingerprints.
     */
    protected RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints;

    /**
     * Euclidean distances between WiFi signal fingerprints (expressed in dB's).
     */
    protected double[] distances;

    /**
     * Listener to be notified of events raised by this instance.
     */
    protected WeightedKNearestNeighboursPositionSolverListener<P> listener;

    /**
     * Estimated inhomogeneous position coordinates.
     */
    protected double[] estimatedPositionCoordinates;

    /**
     * Indicates if this instance is locked because indoor is being
     * estimated.
     */
    protected boolean locked;

    /**
     * Minimum allowed distance between received Wi-Fi signal strengths.
     */
    private double epsilon = DEFAULT_EPSILON;

    /**
     * Constructor.
     */
    protected WeightedKNearestNeighboursPositionSolver() {
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
    protected WeightedKNearestNeighboursPositionSolver(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints,
            final double[] distances) {
        internalSetFingerprintsAndDistances(fingerprints, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    protected WeightedKNearestNeighboursPositionSolver(
            final WeightedKNearestNeighboursPositionSolverListener<P> listener) {
        this.listener = listener;
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
    protected WeightedKNearestNeighboursPositionSolver(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints,
            final double[] distances, final WeightedKNearestNeighboursPositionSolverListener<P> listener) {
        this(fingerprints, distances);
        this.listener = listener;
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     *
     * @return listener to be notified of events raised by this instance.
     */
    public WeightedKNearestNeighboursPositionSolverListener<P> getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     *
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if instance is busy solving the position.
     */
    public void setListener(final WeightedKNearestNeighboursPositionSolverListener<P> listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Gets known located Wi-Fi fingerprints.
     *
     * @return known located Wi-Fi fingerprints.
     */
    public RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] getFingerprints() {
        return fingerprints;
    }

    /**
     * Gets euclidean distances between WiFi signal fingerprints
     * (expressed in dB's).
     *
     * @return euclidean distances between WiFi signal fingerprints.
     */
    public double[] getDistances() {
        return distances;
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    public boolean isReady() {
        return fingerprints != null && distances != null && fingerprints.length >= MIN_FINGERPRINTS;
    }

    /**
     * Returns boolean indicating if estimator is locked because estimation is under
     * progress.
     *
     * @return true if solver is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Sets known located Wi-Fi fingerprints and Euclidean distances between Wi-Fi
     * signal fingerprints.
     *
     * @param fingerprints known located Wi-Fi fingerprints.
     * @param distances    Euclidean distances between Wi-Fi signal fingerprints
     *                     (expressed in dB's).
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     * @throws LockedException          if instance is busy solving the position.
     */
    public void setFingerprintsAndDistances(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints,
            final double[] distances) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetFingerprintsAndDistances(fingerprints, distances);
    }

    /**
     * Gets minimum allowed distance between Wi-Fi signal fingerprints.
     *
     * @return minimum allowed distance between Wi-Fi signal fingerprints.
     */
    public double getEpsilon() {
        return epsilon;
    }

    /**
     * Sets minimum allowed distance between WiFi signal fingerprints.
     *
     * @param epsilon minimum allowed distance between WiFi signal fingerprints.
     *                strengths.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if instance is busy solving the indoor problem.
     */
    public void setEpsilon(final double epsilon) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        if (epsilon <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.epsilon = epsilon;
    }

    /**
     * Estimates position.
     *
     * @throws NotReadyException if solver is not ready.
     * @throws LockedException   if instance is busy solving position.
     */
    public void solve() throws NotReadyException, LockedException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onSolveStart(this);
            }

            final var num = fingerprints.length;
            final var dims = getNumberOfDimensions();
            if (num == 1) {
                // only one fingerprint available
                estimatedPositionCoordinates = new double[dims];
                for (var i = 0; i < dims; i++) {
                    final var p = fingerprints[0].getPosition();
                    estimatedPositionCoordinates[i] = p.getInhomogeneousCoordinate(i);
                }
            } else {
                // multiple fingerprints available
                final var coords = new double[dims];
                var sum = 0.0;
                double w;
                for (var i = 0; i < num; i++) {
                    // weighted average and weight summation
                    w = 1.0 / distances[i];
                    sum += w;

                    final var p = fingerprints[i].getPosition();
                    for (var j = 0; j < dims; j++) {
                        coords[j] += w * p.getInhomogeneousCoordinate(j);
                    }
                }

                // normalize by weight summation
                if (sum != 0.0) {
                    for (var j = 0; j < dims; j++) {
                        coords[j] /= sum;
                    }
                }

                estimatedPositionCoordinates = coords;
            }

            if (listener != null) {
                listener.onSolveEnd(this);
            }
        } finally {
            locked = false;
        }
    }

    /**
     * Gets estimated inhomogeneous position coordinates.
     *
     * @return estimated inhomogeneous position coordinates.
     */
    public double[] getEstimatedPositionCoordinates() {
        return estimatedPositionCoordinates;
    }

    /**
     * Gets estimated position and stores result into provided instance.
     *
     * @param estimatedPosition instance where estimated position will be stored.
     */
    public void getEstimatedPosition(final P estimatedPosition) {
        if (estimatedPositionCoordinates != null) {
            for (var i = 0; i < estimatedPositionCoordinates.length; i++) {
                estimatedPosition.setInhomogeneousCoordinate(i, estimatedPositionCoordinates[i]);
            }
        }
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public abstract P getEstimatedPosition();

    /**
     * Gets number of dimensions of location points.
     *
     * @return number of dimensions of location points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Sets known located Wi-Fi fingerprints and Euclidean distances between Wi-Fi
     * signal fingerprints.
     *
     * @param fingerprints known located Wi-Fi fingerprints.
     * @param distances    Euclidean distances between Wi-Fi signal fingerprints
     *                     (expressed in dB's).
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     */
    protected void internalSetFingerprintsAndDistances(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints,
            final double[] distances) {
        if (fingerprints == null || distances == null) {
            throw new IllegalArgumentException();
        }

        if (fingerprints.length < MIN_FINGERPRINTS) {
            throw new IllegalArgumentException();
        }

        if (fingerprints.length != distances.length) {
            throw new IllegalArgumentException();
        }

        this.fingerprints = fingerprints;
        this.distances = distances;

        // fix distances if needed
        for (var i = 0; i < this.distances.length; i++) {
            if (this.distances[i] < epsilon) {
                this.distances[i] = epsilon;
            }
        }
    }
}
