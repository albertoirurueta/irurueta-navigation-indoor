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

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RssiReadingLocated;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class to robustly estimate position, transmitted power and path-loss
 * exponent of a radio source (e.g. Wi-Fi access point or bluetooth beacon), by discarding
 * outliers and assuming that the radio source emits isotropically following the
 * expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antenna
 * Gr is the Gain of the receiver antenna
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the radio signal.
 * Because usually information about the antenna of the radio source cannot be
 * retrieved (because many measurements are made on unknown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If RssiReadings contain RSSI standard deviations, those values will be used,
 * otherwise it will be assumed an RSSI standard deviation of 1 dB.
 * Implementations of this class should be able to detect and discard outliers in
 * order to find the best solution.
 * <p>
 * IMPORTANT: Implementations of this class can choose to estimate a
 * combination of radio source position, transmitted power and path loss
 * exponent. However enabling all three estimations usually achieves
 * inaccurate results. When using this class, estimation must be of at least
 * one parameter (position, transmitted power or path loss exponent) when
 * initial values are provided for the other two, and at most it should consist
 * of two parameters (either position and transmitted power, position and
 * path loss exponent or transmitted power and path loss exponent), providing an
 * initial value for the remaining parameter.
 *
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 */
public abstract class RobustRssiRadioSourceEstimator<S extends RadioSource, P extends Point<P>> extends
        RobustRadioSourceEstimator<P, RssiReadingLocated<S, P>, RobustRssiRadioSourceEstimatorListener<S, P>> {

    /**
     * Initial transmitted power to start the estimation of radio source
     * transmitted power.
     * If not defined, average value of received power readings will be used.
     */
    protected Double initialTransmittedPowerdBm;

    /**
     * Initial position to start the estimation of radio source position.
     * If not defined, centroid of provided located readings will be used.
     */
    protected P initialPosition;

    /**
     * Initial exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * <p>
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
     */
    protected double initialPathLossExponent = RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Indicates whether transmitted power estimation is enabled or not.
     */
    protected boolean transmittedPowerEstimationEnabled =
            RssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED;

    /**
     * Indicates whether radio source position estimation is enabled or not.
     */
    protected boolean positionEstimationEnabled = RssiRadioSourceEstimator.DEFAULT_POSITION_ESTIMATION_ENABLED;

    /**
     * Indicates whether path loss estimation is enabled or not.
     */
    protected boolean pathLossEstimationEnabled;

    /**
     * Estimated transmitted power expressed in dBm's.
     */
    protected double estimatedTransmittedPowerdBm;

    /**
     * Estimated exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link RssiRadioSourceEstimator#DEFAULT_PATH_LOSS_EXPONENT}
     */
    protected double estimatedPathLossExponent = RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Variance of estimated transmitted power.
     * This value will only be available when transmitted power
     * estimation is enabled.
     */
    protected Double estimatedTransmittedPowerVariance;

    /**
     * Variance of estimated path loss exponent.
     * This value will only be available when path-loss
     * exponent estimation is enabled.
     */
    protected Double estimatedPathLossExponentVariance;

    /**
     * Constructor.
     */
    protected RobustRssiRadioSourceEstimator() {
        super();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(final List<? extends RssiReadingLocated<S, P>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    protected RobustRssiRadioSourceEstimator(final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings,
            final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings, final P initialPosition) {
        super(readings);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    protected RobustRssiRadioSourceEstimator(final P initialPosition) {
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    protected RobustRssiRadioSourceEstimator(
            final P initialPosition, final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings, final P initialPosition,
            final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     */
    protected RobustRssiRadioSourceEstimator(final Double initialTransmittedPowerdBm) {
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings, final Double initialTransmittedPowerdBm) {
        super(readings);
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    protected RobustRssiRadioSourceEstimator(
            final Double initialTransmittedPowerdBm, final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings, final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings, final P initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(readings);
        this.initialPosition = initialPosition;
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    protected RobustRssiRadioSourceEstimator(
            final P initialPosition, final Double initialTransmittedPowerdBm) {
        this.initialPosition = initialPosition;
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   in charge of attending events raised by this instance.
     */
    protected RobustRssiRadioSourceEstimator(
            final P initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        this.initialPosition = initialPosition;
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings, final P initialPosition,
            final Double initialTransmittedPowerdBm, final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        this.initialPosition = initialPosition;
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings, final P initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent) {
        this(readings, initialPosition, initialTransmittedPowerdBm);
        this.initialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     */
    protected RobustRssiRadioSourceEstimator(
            final P initialPosition, final Double initialTransmittedPowerdBm, final double initialPathLossExponent) {
        this(initialPosition, initialTransmittedPowerdBm);
        this.initialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    protected RobustRssiRadioSourceEstimator(
            final P initialPosition, final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        this(initialPosition, initialTransmittedPowerdBm, listener);
        this.initialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRssiRadioSourceEstimator(
            final List<? extends RssiReadingLocated<S, P>> readings, final P initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final RobustRssiRadioSourceEstimatorListener<S, P> listener) {
        this(readings, initialPosition, initialTransmittedPowerdBm, listener);
        this.initialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Gets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
     *
     * @return initial transmitted power to start the estimation of radio source
     * transmitted power.
     */
    public Double getInitialTransmittedPowerdBm() {
        return initialTransmittedPowerdBm;
    }

    /**
     * Sets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted
     *                                   power.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialTransmittedPowerdBm(final Double initialTransmittedPowerdBm) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Gets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in mW).
     * If not defined, average value of received power readings will be used.
     *
     * @return initial transmitted power to start the estimation of radio source
     * transmitted power.
     */
    public Double getInitialTransmittedPower() {
        return initialTransmittedPowerdBm != null ? Utils.dBmToPower(initialTransmittedPowerdBm) : null;
    }

    /**
     * Sets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in mW).
     * If not defined, average value of received power readings will be used.
     *
     * @param initialTransmittedPower initial transmitted power to start the
     *                                estimation of radio source transmitted power.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setInitialTransmittedPower(final Double initialTransmittedPower) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (initialTransmittedPower != null) {
            if (initialTransmittedPower < 0.0) {
                throw new IllegalArgumentException();
            }
            initialTransmittedPowerdBm = Utils.powerTodBm(initialTransmittedPower);
        } else {
            initialTransmittedPowerdBm = null;
        }
    }

    /**
     * Gets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided fingerprints will be used.
     *
     * @return initial position to start the estimation of radio source position.
     */
    public P getInitialPosition() {
        return initialPosition;
    }

    /**
     * Sets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided fingerprints will be used.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(final P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.initialPosition = initialPosition;
    }

    /**
     * Gets initial exponent typically used on free space for path loss propagation
     * in terms of distance.
     * On different environments path loss exponent might have different value:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * <p>
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
     *
     * @return initial path loss exponent.
     */
    public double getInitialPathLossExponent() {
        return initialPathLossExponent;
    }

    /**
     * Sets initial exponent typically used on free space for path loss propagation
     * in terms of distance.
     * On different environments path loss exponent might have different value:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * <p>
     * If path loss exponent estimation is enabled, estimation will start at this
     * value and will converge to the most appropriate value.
     * If path loss exponent estimation is disabled, this value will be assumed
     * to be exact and the estimated path loss exponent will be equal to this
     * value.
     *
     * @param initialPathLossExponent initial path loss exponent.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPathLossExponent(final double initialPathLossExponent) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.initialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Indicates whether transmitted power estimation is enabled or not.
     *
     * @return true if transmitted power estimation is enabled, false otherwise.
     */
    public boolean isTransmittedPowerEstimationEnabled() {
        return transmittedPowerEstimationEnabled;
    }

    /**
     * Specifies whether transmitted power estimation is enabled or not.
     *
     * @param transmittedPowerEstimationEnabled true if transmitted power estimation is enabled,
     *                                          false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setTransmittedPowerEstimationEnabled(final boolean transmittedPowerEstimationEnabled)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.transmittedPowerEstimationEnabled = transmittedPowerEstimationEnabled;
    }

    /**
     * Indicates whether radio source position estimation is enabled or not.
     *
     * @return true if position estimation is enabled, false otherwise.
     */
    public boolean isPositionEstimationEnabled() {
        return positionEstimationEnabled;
    }

    /**
     * Specifies whether radio source position estimation is enabled or not.
     *
     * @param positionEstimationEnabled true if position estimation is enabled,
     *                                  false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPositionEstimationEnabled(final boolean positionEstimationEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.positionEstimationEnabled = positionEstimationEnabled;
    }

    /**
     * Indicates whether path loss estimation is enabled or not.
     *
     * @return true if path loss estimation is enabled, false otherwise.
     */
    public boolean isPathLossEstimationEnabled() {
        return pathLossEstimationEnabled;
    }

    /**
     * Specifies whether path loss estimation is enabled or not.
     *
     * @param pathLossEstimationEnabled true if path loss estimation is enabled,
     *                                  false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPathLossEstimationEnabled(final boolean pathLossEstimationEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.pathLossEstimationEnabled = pathLossEstimationEnabled;
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        // at least one parameter estimation must be enabled
        return (positionEstimationEnabled || transmittedPowerEstimationEnabled || pathLossEstimationEnabled)
                // if position estimation is disabled, an initial position must be provided
                && !(!positionEstimationEnabled && initialPosition == null)
                // if transmitted power estimation is disabled, an initial transmitted power must be provided
                && !(!transmittedPowerEstimationEnabled && initialTransmittedPowerdBm == null)
                // readings must also be valid
                && areValidReadings(readings);
    }

    /**
     * Gets estimated transmitted power variance.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated transmitted power variance.
     */
    public Double getEstimatedTransmittedPowerVariance() {
        return estimatedTransmittedPowerVariance;
    }

    /**
     * Gets estimated path loss exponent variance.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated path loss exponent variance.
     */
    public Double getEstimatedPathLossExponentVariance() {
        return estimatedPathLossExponentVariance;
    }

    /**
     * Gets estimated transmitted power expressed in milli watts (mW).
     *
     * @return estimated transmitted power expressed in milli watts.
     */
    public double getEstimatedTransmittedPower() {
        return Utils.dBmToPower(estimatedTransmittedPowerdBm);
    }

    /**
     * Gets estimated transmitted power expressed in dBm's.
     *
     * @return estimated transmitted power expressed in dBm's.
     */
    public double getEstimatedTransmittedPowerdBm() {
        return estimatedTransmittedPowerdBm;
    }

    /**
     * Gets estimated exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link RssiRadioSourceEstimator#DEFAULT_PATH_LOSS_EXPONENT}
     *
     * @return estimated path loss exponent.
     */
    public double getEstimatedPathLossExponent() {
        return estimatedPathLossExponent;
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Solves preliminary solution for a subset of samples.
     *
     * @param samplesIndices indices of subset samples.
     * @param solutions      instance where solution will be stored.
     */
    protected abstract void solvePreliminarySolutions(final int[] samplesIndices, final List<Solution<P>> solutions);

    /**
     * Estimates residual for a solution obtained for a subset of samples.
     *
     * @param currentEstimation solution obtained for a subset of samples.
     * @param i                 i-th fingerprint to obtain residual for.
     * @return difference between measured and expected RSSI value.
     */
    @SuppressWarnings("DuplicatedCode")
    protected double residual(final Solution<P> currentEstimation, final int i) {
        // Model fitted internally is equal to:
        // Pr (dBm) = 10 * log(Pte * k^n / d^n) = 10*n*log(k) + 10*log(Pte) - 5*n*log(d^2)
        // where:
        // Pr is received, expressed in dBm
        // Pte is equivalent transmitted power, expressed in dBm
        // k is a constant equal to k = c^2 / (pi * f)^2, where c is speed of light
        // and d is equal to distance between fingerprint and estimated position
        final var reading = readings.get(i);
        final var frequency = reading.getSource().getFrequency();

        final var pathLossExponent = currentEstimation.getEstimatedPathLossExponent();

        // compute k as the constant part of the isotropic received power formula
        // so that: Pr = Pte*k^n/d^n
        final var k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        final var kdB = 10.0 * pathLossExponent * Math.log10(k);

        // get distance from estimated radio source position and reading position
        final var readingPosition = reading.getPosition();
        final var radioSourcePosition = currentEstimation.getEstimatedPosition();

        final var sqrDistance = radioSourcePosition.sqrDistanceTo(readingPosition);

        final var transmittedPowerdBm = currentEstimation.getEstimatedTransmittedPowerdBm();

        // compute expected received power assuming isotropic transmission
        // and compare against measured RSSI at fingerprint location
        final var expectedRSSI = kdB + transmittedPowerdBm - 5.0 * pathLossExponent * Math.log10(sqrDistance);
        final double rssi = reading.getRssi();

        return Math.abs(expectedRSSI - rssi);
    }

    /**
     * Contains a solution obtained during robust estimation for a subset of
     * samples.
     *
     * @param <P> a {@link Point} type.
     */
    protected static class Solution<P extends Point<?>> {
        /**
         * Estimated position for a subset of samples.
         */
        private final P mEstimatedPosition;

        /**
         * Estimated transmitted power expressed in dBm's for a subset of samples.
         */
        private final double mEstimatedTransmittedPowerdBm;

        /**
         * Estimated path loss exponent for a subset of samples.
         */
        private final double mEstimatedPathLossExponent;

        /**
         * Constructor.
         *
         * @param estimatedPosition            estimated position for a subset of samples.
         * @param estimatedTransmittedPowerdBm estimated transmitted power expressed
         *                                     in dBm's for a subset of samples.
         * @param estimatedPathLossExponent    estimated path loss exponent.
         */
        public Solution(final P estimatedPosition, final double estimatedTransmittedPowerdBm,
                        final double estimatedPathLossExponent) {
            mEstimatedPosition = estimatedPosition;
            mEstimatedTransmittedPowerdBm = estimatedTransmittedPowerdBm;
            mEstimatedPathLossExponent = estimatedPathLossExponent;
        }

        /**
         * Gets estimated position for a subset of samples.
         *
         * @return estimated position for a subset of samples.
         */
        public P getEstimatedPosition() {
            return mEstimatedPosition;
        }

        /**
         * Gets estimated transmitted power expressed in dBm's for a subset of
         * samples.
         *
         * @return estimated transmitted power expressed in dBm's for a subset
         * of samples.
         */
        public double getEstimatedTransmittedPowerdBm() {
            return mEstimatedTransmittedPowerdBm;
        }

        /**
         * Gets estimated path loss exponent.
         *
         * @return estimated path loss exponent.
         */
        public double getEstimatedPathLossExponent() {
            return mEstimatedPathLossExponent;
        }
    }
}
