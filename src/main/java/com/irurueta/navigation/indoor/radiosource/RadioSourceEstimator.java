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
package com.irurueta.navigation.indoor.radiosource;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.ReadingLocated;

import java.util.List;

/**
 * Estimates a radio source. Usually this implies at least the estimation of
 * the radio source location, however, implementations of this class might
 * estimate additional parameters.
 *
 * @param <P> a {@link Point} type.
 * @param <R> a {@link ReadingLocated} type.
 * @param <L> a {@link RadioSourceEstimatorListener} type.
 */
public abstract class RadioSourceEstimator<P extends Point<?>, R extends ReadingLocated<P>,
        L extends RadioSourceEstimatorListener<? extends RadioSourceEstimator<?, ?, ?>>> {

    /**
     * Estimated position.
     */
    protected double[] estimatedPositionCoordinates;

    /**
     * Covariance of estimated parameters (position, transmitted power and path-loss exponent).
     * Size of this matrix will depend on which parameters estimation is enabled.
     */
    protected Matrix estimatedCovariance;

    /**
     * Covariance of estimated position.
     * Size of this matrix will depend on the number of dimensions
     * of estimated position (either 2 or 3).
     * This value will only be available when position estimation is enabled.
     */
    protected Matrix estimatedPositionCovariance;


    /**
     * Located signal readings belonging to the same radio source to be estimated.
     */
    protected List<? extends R> readings;

    /**
     * Indicates whether estimator is locked during estimation.
     */
    protected boolean locked;

    /**
     * Listener in charge of attending events raised by this instance.
     */
    protected L listener;

    /**
     * Constructor.
     */
    protected RadioSourceEstimator() {
    }

    /**
     * Constructor.
     * Sets located radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RadioSourceEstimator(final List<? extends R> readings) {
        internalSetReadings(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    protected RadioSourceEstimator(final L listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if fingerprints are not valid.
     */
    protected RadioSourceEstimator(final List<? extends R> readings, final L listener) {
        this(readings);
        this.listener = listener;
    }

    /**
     * Indicates whether estimator is locked during estimation.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Gets radio signal readings belonging to the same radio source to be estimated.
     *
     * @return radio signal readings belonging to the same radio source.
     */
    public List<R> getReadings() {
        //noinspection unchecked
        return (List<R>) readings;
    }

    /**
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings Wi-Fi signal readings belonging to the same radio source.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public void setReadings(final List<? extends R> readings) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetReadings(readings);
    }

    /**
     * Gets listener in charge of attending events raised by this instance.
     *
     * @return listener in charge of attending events raised by this instance.
     */
    public L getListener() {
        return listener;
    }

    /**
     * Sets listener in charge of attending events raised by this instance.
     *
     * @param listener listener in charge of attending events raised by this
     *                 instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(final L listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Indicates whether readings are valid or not.
     * Readings are considered valid when there are enough readings.
     *
     * @param readings readings to be validated.
     * @return true if readings are valid, false otherwise.
     */
    public boolean areValidReadings(final List<? extends R> readings) {
        return readings != null && readings.size() >= getMinReadings();
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
            for (int i = 0; i < estimatedPositionCoordinates.length; i++) {
                estimatedPosition.setInhomogeneousCoordinate(i, estimatedPositionCoordinates[i]);
            }
        }
    }

    /**
     * Gets covariance for estimated position and power.
     * Matrix contains information in the following order:
     * Top-left sub-matrix contains covariance of position,
     * then follows transmitted power variance, and finally
     * the last element contains path-loss exponent variance.
     *
     * @return covariance for estimated parameters.
     */
    public Matrix getEstimatedCovariance() {
        return estimatedCovariance;
    }

    /**
     * Gets estimated position covariance.
     * Size of this matrix will depend on the number of dimensions
     * of estimated position (either 2 or 3).
     * This value will only be available when position estimation is enabled.
     *
     * @return estimated position covariance or null.
     */
    public Matrix getEstimatedPositionCovariance() {
        return estimatedPositionCovariance;
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    public abstract boolean isReady();

    /**
     * Gets minimum required number of readings to estimate
     * power, position and path-loss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings for 2D, and 4 readings for 3D.
     *
     * @return minimum required number of readings.
     */
    public abstract int getMinReadings();

    /**
     * Gets number of dimensions of position points.
     *
     * @return number of dimensions of position points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Gets estimated radio source position.
     *
     * @return estimated radio source position.
     */
    public abstract P getEstimatedPosition();

    /**
     * Gets estimated located radio source.
     *
     * @param <S> type of located radio source.
     * @return estimated located radio source.
     */
    public abstract <S extends RadioSourceLocated<P>> S getEstimatedRadioSource();

    /**
     * Estimate radio source.
     *
     * @throws RadioSourceEstimationException if estimation fails.
     * @throws NotReadyException              if estimator is not ready.
     * @throws LockedException                if estimator is locked.
     */
    public abstract void estimate() throws RadioSourceEstimationException, NotReadyException, LockedException;

    /**
     * Internally sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are null or not enough readings
     *                                  are available.
     */
    protected void internalSetReadings(final List<? extends R> readings) {
        if (!areValidReadings(readings)) {
            throw new IllegalArgumentException();
        }

        this.readings = readings;
    }
}
