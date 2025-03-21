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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated;
import com.irurueta.navigation.indoor.RangingReadingLocated;
import com.irurueta.navigation.indoor.ReadingLocated;
import com.irurueta.navigation.indoor.RssiReadingLocated;
import com.irurueta.navigation.indoor.Utils;

import java.util.ArrayList;
import java.util.List;

/**
 * This is an abstract class to robustly estimate position, transmitted power and
 * path loss exponent of a radio source (e.g. Wi-Fi access point or bluetooth
 * beacon) assuming that the ranging data is available to obtain position with greater
 * accuracy and that the radio source emits isotropically following the
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
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be assumed an RSSI standard deviation of 1 dB.
 * <p>
 * This implementation is like RangingAndRssiRadioSourceEstimator but allows mixing
 * different kinds of located radio source readings (ranging, RSSI and ranging+RSSI).
 *
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 */
public abstract class MixedRadioSourceEstimator<S extends RadioSource, P extends Point<P>>
        extends RadioSourceEstimator<P, ReadingLocated<P>, MixedRadioSourceEstimatorListener<S, P>> {

    /**
     * Speed of light expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = 299792458.0;

    /**
     * Default exponent typically used on free space for path loss propagation in
     * terms of distance. This value is used for free space environments.
     */
    public static final double DEFAULT_PATH_LOSS_EXPONENT = 2.0;

    /**
     * Indicates whether radio source transmitted power estimation is enabled or not by
     * default. Typically, this data is required for Wi-Fi Access points, but it is already
     * provided for Beacons (and hence its estimation is not needed).
     */
    public static final boolean DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED = true;

    /**
     * Indicates whether path loss estimation is enabled or not by default.
     */
    public static final boolean DEFAULT_PATHLOSS_ESTIMATION_ENABLED = false;

    /**
     * Indicates that by default position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    public static final boolean DEFAULT_USE_READING_POSITION_COVARIANCES = true;

    /**
     * RSSI radio source estimator.
     */
    protected RssiRadioSourceEstimator<S, P> rssiInnerEstimator;

    /**
     * Ranging radio source estimator.
     */
    protected RangingRadioSourceEstimator<S, P> rangingInnerEstimator;

    /**
     * Indicates whether transmitted power estimation is enabled or not.
     */
    protected boolean transmittedPowerEstimationEnabled = DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED;

    /**
     * Indicates whether path loss estimation is enabled or not.
     */
    protected boolean pathLossEstimationEnabled = DEFAULT_PATHLOSS_ESTIMATION_ENABLED;

    /**
     * Estimated transmitted power expressed in dBm's or null if not available.
     */
    private Double estimatedTransmittedPowerdBm;

    /**
     * Estimated exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link #DEFAULT_PATH_LOSS_EXPONENT}
     */
    private double estimatedPathLossExponent = DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Variance of estimated transmitted power.
     * This value will only be available when transmitted power
     * estimation is enabled.
     */
    private Double estimatedTransmittedPowerVariance;

    /**
     * Variance of estimated path loss exponent.
     * This value will only be available when path-loss
     * exponent estimation is enabled.
     */
    private Double estimatedPathLossExponentVariance;

    /**
     * Initial transmitted power to start the estimation of radio source
     * transmitted power.
     * If not defined, average value of received power readings will be used.
     */
    private Double initialTransmittedPowerdBm;

    /**
     * Initial position to start the estimation of radio source position.
     * If not defined, centroid of provided readings will be used.
     */
    private P initialPosition;

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
    private double initialPathLossExponent = DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    private boolean useReadingPositionCovariances = DEFAULT_USE_READING_POSITION_COVARIANCES;

    /**
     * Number of ranging readings available among all readings.
     */
    private int numRangingReadings;

    /**
     * Number of RSSI readings available among all readings.
     */
    private int numRssiReadings;

    /**
     * Indicates whether position is estimated using RSSI data.
     * If enough ranging readings are available, this is false and position is estimated using ranging readings,
     * otherwise this is true and position is estimated using RSSI data in a less reliable way.
     */
    private boolean rssiPositionEnabled;

    /**
     * Indicates whether an homogeneous linear solver is used to estimate an initial
     * position for the internal ranging radio source estimator.
     */
    private boolean useHomogeneousRangingLinearSolver =
            RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Constructor.
     */
    protected MixedRadioSourceEstimator() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same
     *                 radio sources.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(final List<? extends ReadingLocated<P>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    protected MixedRadioSourceEstimator(final MixedRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings, final MixedRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    protected MixedRadioSourceEstimator(final P initialPosition) {
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings        radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings, final P initialPosition) {
        super(readings);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    protected MixedRadioSourceEstimator(
            final P initialPosition, final MixedRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings        radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings, final P initialPosition,
            final MixedRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerDbm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    protected MixedRadioSourceEstimator(final Double initialTransmittedPowerDbm) {
        initialTransmittedPowerdBm = initialTransmittedPowerDbm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings, final Double initialTransmittedPowerdBm) {
        super(readings);
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    protected MixedRadioSourceEstimator(
            final Double initialTransmittedPowerdBm, final MixedRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings, final Double initialTransmittedPowerdBm,
            final MixedRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings, final P initialPosition,
            Double initialTransmittedPowerdBm) {
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
    protected MixedRadioSourceEstimator(final P initialPosition, final Double initialTransmittedPowerdBm) {
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
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    protected MixedRadioSourceEstimator(
            final P initialPosition, final Double initialTransmittedPowerdBm,
            final MixedRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        this.initialPosition = initialPosition;
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings, final P initialPosition,
            final Double initialTransmittedPowerdBm, final MixedRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        this.initialPosition = initialPosition;
        this.initialTransmittedPowerdBm = initialTransmittedPowerdBm;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings, final P initialPosition,
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
    protected MixedRadioSourceEstimator(
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
    protected MixedRadioSourceEstimator(
            final P initialPosition, final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final MixedRadioSourceEstimatorListener<S, P> listener) {
        this(initialPosition, initialTransmittedPowerdBm, listener);
        this.initialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected MixedRadioSourceEstimator(
            final List<? extends ReadingLocated<P>> readings,
            final P initialPosition, final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final MixedRadioSourceEstimatorListener<S, P> listener) {
        this(readings, initialPosition, initialTransmittedPowerdBm, listener);
        this.initialPathLossExponent = initialPathLossExponent;
    }

    /**
     * Gets initial transmitted power to start the estimation of radio source
     * transmitted power (expressed in dBm's).
     * If not defined, average value of received power readings will be used.
     * <p>
     * If transmitted power estimation is enabled, estimation will start at this
     * value and will be converted to the most appropriate value.
     * If transmitted power estimation is disabled, this value will be assumed to be
     * exact and the estimated transmitted power will be equal to this value
     * (converted to dBm's).
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
     * <p>
     * If transmitted power estimation is enabled, estimation will start at this
     * value and will be converted to the most appropriate value.
     * If transmitted power estimation is disabled, this value will be assumed to be
     * exact and the estimated transmitted power will be equal to this value
     * (converted to dBm's).
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
     * <p>
     * If transmitted power estimation is enabled, estimation will start at this
     * value and will be converted to the most appropriate value.
     * If transmitted power estimation is disabled, this value will be assumed to be
     * exact and the estimated transmitted power will be equal to this value
     * (converted to dBm's).
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
     * <p>
     * If transmitted power estimation is enabled, estimation will start at this
     * value and will be converted to the most appropriate value.
     * If transmitted power estimation is disabled, this value will be assumed to be
     * exact and the estimated transmitted power will be equal to this value
     * (converted to dBm's).
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
     * Gets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided readings will be used.
     * <p>
     * If position estimation is enabled, estimation will start at this value
     * and will converge to the most appropriate value.
     * If position estimation is disabled, this value will be assumed to
     * be exact and the estimated position will be equal to this value.
     *
     * @return initial position to start the estimation of radio source position.
     */
    public P getInitialPosition() {
        return initialPosition;
    }

    /**
     * Sets initial position to start the estimation of radio source position.
     * If not defined, centroid of provided fingerprints will be used.
     * <p>
     * If position estimation is enabled, estimation will start at this value
     * and will converge to the most appropriate value.
     * If position estimation is disabled, this value will be assumed to
     * be exact and the estimated position will be equal to this value.
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
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard
     * deviation assuming that both measures are statistically independent.
     *
     * @return true to take into account reading position covariances, false otherwise.
     */
    public boolean getUseReadingPositionCovariance() {
        return useReadingPositionCovariances;
    }

    /**
     * Specifies whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard
     * deviation assuming that both measures are statistically independent.
     *
     * @param useReadingPositionCovariances true to take into account reading position covariances, false
     *                                      otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setUseReadingPositionCovariances(final boolean useReadingPositionCovariances) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.useReadingPositionCovariances = useReadingPositionCovariances;
    }

    /**
     * Indicates whether an homogeneous linear solver is used to estimate an initial
     * position for the internal ranging radio source estimator.
     *
     * @return true if homogeneous linear solver is used, false if an inhomogeneous linear
     * one is used instead.
     */
    public boolean isHomogeneousRangingLinearSolverUsed() {
        return useHomogeneousRangingLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used to estimate an initial
     * position for the internal ranging radio source estimator.
     *
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used, false
     *                                   if an inhomogeneous linear one is used instead.
     * @throws LockedException if estimator is locked.
     */
    public void setHomogeneousRangingLinearSolverUsed(final boolean useHomogeneousLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        useHomogeneousRangingLinearSolver = useHomogeneousLinearSolver;
    }

    /**
     * Gets minimum required number of ranging or ranging+rssi readings
     * required to start estimation.
     *
     * @return minimum required number of ranging or ranging+rssi readings.
     */
    public int getMinRangingReadings() {
        return getNumberOfDimensions() + 1;
    }

    /**
     * Gets minimum required number of rssi or ranging+rssi readings
     * required to start estimation.
     *
     * @return minimum required number of rssi or ranging+rssi readings.
     */
    public int getMinRssiReadings() {
        return getMinReadings();
    }

    /**
     * Gets minimum required number of readings to estimate
     * power, position and path-loss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings.
     *
     * @return minimum required number of readings.
     * @throws IllegalStateException if inner RSSI estimator is busy.
     */
    @Override
    public int getMinReadings() {
        createInnerEstimatorsIfNeeded();

        var result = getNumberOfDimensions();
        if (rssiInnerEstimator != null && (transmittedPowerEstimationEnabled || pathLossEstimationEnabled)) {
            try {
                rssiInnerEstimator.setPositionEstimationEnabled(rssiPositionEnabled);
                rssiInnerEstimator.setTransmittedPowerEstimationEnabled(transmittedPowerEstimationEnabled);
                rssiInnerEstimator.setPathLossEstimationEnabled(pathLossEstimationEnabled);
            } catch (final LockedException e) {
                throw new IllegalStateException(e);
            }

            result += rssiInnerEstimator.getMinReadings();
        } else {
            result++;
        }

        return result;
    }

    /**
     * Gets estimated radio source position.
     *
     * @return estimated radio source position.
     */
    public P getEstimatedPosition() {
        return rssiPositionEnabled ? rssiInnerEstimator.getEstimatedPosition()
                : rangingInnerEstimator.getEstimatedPosition();
    }

    /**
     * Indicates whether readings are valid or not.
     * Readings are considered valid when there are enough readings.
     *
     * @param readings readings to be validated.
     * @return true if readings are valid, false otherwise.
     */
    @Override
    public boolean areValidReadings(final List<? extends ReadingLocated<P>> readings) {
        if (readings == null) {
            return false;
        }

        checkReadings(readings);

        // if enough ranging data is available, we check validity both for ranging and RSSI readings
        return ((!rssiPositionEnabled && numRangingReadings >= getMinRangingReadings()
                && numRssiReadings >= getMinRssiReadings())
                // if not enough ranging data is available, we check validity only for RSSI readings
                || (rssiPositionEnabled && numRssiReadings >= getMinRssiReadings())
                // if only position is enabled, then only check for ranging readings
                || (!transmittedPowerEstimationEnabled && !pathLossEstimationEnabled
                && numRangingReadings >= getMinRangingReadings()))
                // in both upper cases enough general readings must be available
                && super.areValidReadings(readings);
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return areValidReadings(readings);
    }

    /**
     * Estimate position, transmitted power and path loss exponent.
     *
     * @throws RadioSourceEstimationException if estimation fails.
     * @throws NotReadyException              if estimator is not ready.
     * @throws LockedException                if estimator is locked.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void estimate() throws RadioSourceEstimationException, NotReadyException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onEstimateStart(this);
            }

            createInnerEstimatorsIfNeeded();

            final var rangingReadings = new ArrayList<RangingReadingLocated<S, P>>();
            final var rssiReadings = new ArrayList<RssiReadingLocated<S, P>>();
            for (final var reading : readings) {
                if (reading instanceof RangingReadingLocated) {
                    rangingReadings.add((RangingReadingLocated<S, P>) reading);

                } else if (reading instanceof RssiReadingLocated) {
                    rssiReadings.add((RssiReadingLocated<S, P>) reading);

                } else if (reading instanceof RangingAndRssiReadingLocated) {
                    rangingReadings.add(createRangingReading((RangingAndRssiReadingLocated<S, P>) reading));
                    rssiReadings.add(createRssiReading((RangingAndRssiReadingLocated<S, P>) reading));
                }
            }

            // estimate position using ranging data, if possible
            P estimatedPosition = null;
            if (!rssiPositionEnabled) {
                rangingInnerEstimator.setUseReadingPositionCovariances(useReadingPositionCovariances);
                rangingInnerEstimator.setHomogeneousLinearSolverUsed(useHomogeneousRangingLinearSolver);
                rangingInnerEstimator.setReadings(rangingReadings);
                rangingInnerEstimator.setInitialPosition(initialPosition);

                rangingInnerEstimator.estimate();

                estimatedPositionCoordinates = rangingInnerEstimator.getEstimatedPositionCoordinates();
                estimatedPositionCovariance = rangingInnerEstimator.getEstimatedPositionCovariance();
                estimatedPosition = rangingInnerEstimator.getEstimatedPosition();
            }

            // estimate transmitted power and/or path-loss if enabled
            if (transmittedPowerEstimationEnabled || pathLossEstimationEnabled || rssiPositionEnabled) {
                rssiInnerEstimator.setPositionEstimationEnabled(rssiPositionEnabled);
                rssiInnerEstimator.setInitialPosition(estimatedPosition);

                rssiInnerEstimator.setTransmittedPowerEstimationEnabled(transmittedPowerEstimationEnabled);
                rssiInnerEstimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);

                rssiInnerEstimator.setPathLossEstimationEnabled(pathLossEstimationEnabled);
                rssiInnerEstimator.setInitialPathLossExponent(initialPathLossExponent);

                rssiInnerEstimator.setReadings(rssiReadings);

                rssiInnerEstimator.estimate();

                if (rssiPositionEnabled) {
                    estimatedPositionCoordinates = rssiInnerEstimator.getEstimatedPositionCoordinates();
                    estimatedPositionCovariance = rssiInnerEstimator.getEstimatedPositionCovariance();
                }

                if (transmittedPowerEstimationEnabled) {
                    // transmitted power estimation enabled
                    estimatedTransmittedPowerdBm = rssiInnerEstimator.getEstimatedTransmittedPowerdBm();
                    estimatedTransmittedPowerVariance = rssiInnerEstimator.getEstimatedTransmittedPowerVariance();
                } else {
                    // transmitted power estimation disabled
                    estimatedTransmittedPowerdBm = initialTransmittedPowerdBm;
                    estimatedTransmittedPowerVariance = null;
                }

                if (pathLossEstimationEnabled) {
                    // path-loss exponent estimation enabled
                    estimatedPathLossExponent = rssiInnerEstimator.getEstimatedPathLossExponent();
                    estimatedPathLossExponentVariance = rssiInnerEstimator.getEstimatedPathLossExponentVariance();
                } else {
                    // path-loss exponent estimation disabled
                    estimatedPathLossExponent = initialPathLossExponent;
                    estimatedPathLossExponentVariance = null;
                }

                // build covariance matrix
                if (rssiPositionEnabled) {
                    // if only RSSI estimation is done, we use directly the available estimated covariance
                    estimatedCovariance = rssiInnerEstimator.getEstimatedCovariance();

                } else {
                    // if both ranging and RSSI data is used, we build covariance matrix by setting
                    // position covariance estimated by ranging estimator into top-left corner, and then
                    // adding covariance terms related to path-loss exponent and transmitted power
                    final var rssiCov = rssiInnerEstimator.getEstimatedCovariance();
                    if (estimatedPositionCovariance != null && rssiCov != null) {
                        final var dims = getNumberOfDimensions();
                        var n = dims;
                        if (transmittedPowerEstimationEnabled) {
                            n++;
                        }
                        if (pathLossEstimationEnabled) {
                            n++;
                        }

                        final var dimsMinus1 = dims - 1;
                        final var nMinus1 = n - 1;
                        estimatedCovariance = new Matrix(n, n);
                        estimatedCovariance.setSubmatrix(0, 0, dimsMinus1, dimsMinus1,
                                estimatedPositionCovariance);
                        estimatedCovariance.setSubmatrix(dims, dims, nMinus1, nMinus1, rssiCov);
                    } else {
                        estimatedCovariance = null;
                    }
                }

            } else {
                estimatedCovariance = estimatedPositionCovariance;
                estimatedTransmittedPowerdBm = initialTransmittedPowerdBm;
                estimatedTransmittedPowerVariance = null;

                estimatedPathLossExponent = initialPathLossExponent;
                estimatedPathLossExponentVariance = null;
            }

            if (listener != null) {
                listener.onEstimateEnd(this);
            }

        } catch (final AlgebraException e) {
            throw new RadioSourceEstimationException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Gets estimated transmitted power expressed in milli watts (mW) or null if
     * not available.
     *
     * @return estimated transmitted power expressed in milli watts or null.
     */
    public Double getEstimatedTransmittedPower() {
        return estimatedTransmittedPowerdBm != null ? Utils.dBmToPower(estimatedTransmittedPowerdBm) : null;
    }

    /**
     * Gets estimated transmitted power expressed in dBm's or null if not available.
     *
     * @return estimated transmitted power expressed in dBm's or null.
     */
    public Double getEstimatedTransmittedPowerdBm() {
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
     * {@link #DEFAULT_PATH_LOSS_EXPONENT}
     *
     * @return estimated path loss exponent.
     */
    public double getEstimatedPathLossExponent() {
        return estimatedPathLossExponent;
    }

    /**
     * Gets estimated transmitted power variance.
     * This value will only be available when transmitted power
     * estimation is enabled.
     *
     * @return estimated transmitted power variance or null.
     */
    public Double getEstimatedTransmittedPowerVariance() {
        return estimatedTransmittedPowerVariance;
    }

    /**
     * Gets estimated path loss exponent variance.
     * This value will only be available when path-loss
     * exponent estimation is enabled.
     *
     * @return estimated path loss exponent variance or null.
     */
    public Double getEstimatedPathLossExponentVariance() {
        return estimatedPathLossExponentVariance;
    }

    /**
     * Creates inner estimators if needed.
     */
    protected abstract void createInnerEstimatorsIfNeeded();

    /**
     * Creates a ranging reading from a ranging and RSSI reading.
     *
     * @param reading input reading to convert from.
     * @return a ranging reading containing only the ranging data of input reading.
     */
    private RangingReadingLocated<S, P> createRangingReading(final RangingAndRssiReadingLocated<S, P> reading) {
        return new RangingReadingLocated<>(reading.getSource(), reading.getDistance(), reading.getPosition(),
                reading.getDistanceStandardDeviation(), reading.getPositionCovariance());
    }

    /**
     * Creates an RSSI reading from a ranging and RSSI reading.
     *
     * @param reading input reading to convert from.
     * @return an RSSI reading containing only the RSSI data of input reading.
     */
    private RssiReadingLocated<S, P> createRssiReading(final RangingAndRssiReadingLocated<S, P> reading) {
        return new RssiReadingLocated<>(reading.getSource(), reading.getRssi(), reading.getPosition(),
                reading.getRssiStandardDeviation(), reading.getPositionCovariance());
    }

    /**
     * Checks number of available ranging readings and number of available RSSI readings. Also determines
     * whether position must be estimated using ranging data or RSSI data.
     *
     * @param readings readings to be checked.
     */
    private void checkReadings(final List<? extends ReadingLocated<P>> readings) {
        numRangingReadings = numRssiReadings = 0;

        if (readings == null) {
            return;
        }

        for (final var reading : readings) {
            if (reading instanceof RangingReadingLocated) {
                numRangingReadings++;

            } else if (reading instanceof RssiReadingLocated) {
                numRssiReadings++;

            } else if (reading instanceof RangingAndRssiReadingLocated) {
                numRangingReadings++;
                numRssiReadings++;
            }
        }

        rssiPositionEnabled = numRangingReadings < getMinRangingReadings();
    }
}
