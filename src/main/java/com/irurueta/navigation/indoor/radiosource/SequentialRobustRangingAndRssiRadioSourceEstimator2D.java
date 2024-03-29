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
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconWithPowerAndLocated2D;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceWithPowerAndLocated;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated2D;

import java.util.List;

/**
 * Robustly estimates 2D position, transmitted power and path-loss exponent of a radio
 * source (e.g. Wi-Fi access point or bluetooth beacon), by discarding
 * outliers and assuming that the ranging data is available to obtain position with
 * greater accuracy and that the radio source emits isotropically following the
 * expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antenna
 * Gr is the Gain of the receiver antenna
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the radio signal.
 * <p>
 * Implementations of this class sequentially estimate position and then remaining
 * parameters. First ranging data is used to robustly estimate position and then
 * remaining parameters are robustly estimated using former estimated position as
 * an initial guess.
 * <p>
 * Because usually information about the antenna of the radio source cannot be
 * retrieved (because many measurements are made on unknown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be assumed an RSSI standard deviation of 1 dB.
 * <p>
 * Implementations of this class might produce more stable positions of estimated
 * radio sources than implementations of RobustRangingAndRssiRadioSourceEstimator2D.
 *
 * @param <S> a {@link RadioSource} type.
 */
public class SequentialRobustRangingAndRssiRadioSourceEstimator2D<S extends RadioSource> extends
        SequentialRobustRangingAndRssiRadioSourceEstimator<S, Point2D> {

    /**
     * Constructor.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D() {
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings) {
        super(readings);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition) {
        super(readings, initialPosition);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final Point2D initialPosition) {
        super(initialPosition);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final Point2D initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialPosition, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructors.
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
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @throws IllegalArgumentException if quality scores is null, or length of
     *                                  quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores) {
        super(qualityScores);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     *                                  null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings) {
        super(qualityScores, readings);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param listener      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores quality scores corresponding to each provided sample.
     *                      The larger the score value the better the quality of
     *                      the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @param listener      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     *                                  null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores   quality scores corresponding to each provided sample.
     *                        The larger the score value the better the quality of
     *                        the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid, quality scores is
     *                                  null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition) {
        super(qualityScores, readings, initialPosition);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided sample.
     *                        The larger the score value the better the quality of
     *                        the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final Point2D initialPosition) {
        super(qualityScores, initialPosition);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided sample.
     *                        The larger the score value the better the quality of
     *                        the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final Point2D initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, initialPosition, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores   quality scores corresponding to each provided sample.
     *                        The larger the score value the better the quality of
     *                        the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, initialPosition, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, readings, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided sample.
     *                                   The larger the score value the better the quality of
     *                                   the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructors.
     * Sets signal readings belonging to the same radio source.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public SequentialRobustRangingAndRssiRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustRangingAndRssiRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm,
                initialPathLossExponent, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinReadings();
    }


    /**
     * Gets minimum required number of readings to estimate
     * power, position and path-loss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings.
     *
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        int minReadings = Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
        if (isTransmittedPowerEstimationEnabled()) {
            minReadings++;
        }
        if (isPathLossEstimationEnabled()) {
            minReadings++;
        }
        return ++minReadings;
    }

    /**
     * Gets number of dimensions of position points.
     *
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated located radio source with estimated transmitted power.
     *
     * @return estimated located radio source with estimated transmitted power or null.
     */
    @Override
    @SuppressWarnings({"unchecked", "DuplicatedCode"})
    public RadioSourceWithPowerAndLocated<Point2D> getEstimatedRadioSource() {
        final List<? extends RangingAndRssiReadingLocated<S, Point2D>> readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }
        final S source = readings.get(0).getSource();

        final Point2D estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        final Matrix estimatedPositionCovariance = getEstimatedPositionCovariance();

        final Double transmittedPowerVariance =
                getEstimatedTransmittedPowerVariance();
        final Double transmittedPowerStandardDeviation = transmittedPowerVariance != null ?
                Math.sqrt(transmittedPowerVariance) : null;

        final Double pathlossExponentVariance =
                getEstimatedPathLossExponentVariance();
        final Double pathlossExponentStandardDeviation = pathlossExponentVariance != null ?
                Math.sqrt(pathlossExponentVariance) : null;

        if (source instanceof WifiAccessPoint) {
            final WifiAccessPoint accessPoint = (WifiAccessPoint) source;
            return new WifiAccessPointWithPowerAndLocated2D(accessPoint.getBssid(),
                    source.getFrequency(), accessPoint.getSsid(),
                    getEstimatedTransmittedPowerdBm(),
                    transmittedPowerStandardDeviation,
                    getEstimatedPathLossExponent(),
                    pathlossExponentStandardDeviation,
                    estimatedPosition,
                    estimatedPositionCovariance);
        } else if (source instanceof Beacon) {
            final Beacon beacon = (Beacon) source;
            return new BeaconWithPowerAndLocated2D(beacon.getIdentifiers(),
                    getEstimatedTransmittedPowerdBm(), beacon.getFrequency(),
                    beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(),
                    beacon.getBluetoothName(),
                    getEstimatedPathLossExponent(),
                    transmittedPowerStandardDeviation,
                    pathlossExponentStandardDeviation,
                    estimatedPosition, estimatedPositionCovariance);
        } else {
            return null;
        }
    }

    /**
     * Builds ranging estimator.
     */
    @Override
    protected void buildRangingEstimatorIfNeeded() {
        if (mRangingEstimator == null || mRangingEstimator.getMethod() != mRangingRobustMethod) {
            mRangingEstimator = RobustRangingRadioSourceEstimator2D.create(mRangingRobustMethod);
        }
    }

    /**
     * build RSSI estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void buildRssiEstimatorIfNeeded() throws LockedException {
        if (mRssiEstimator == null || mRssiEstimator.getMethod() != mRssiRobustMethod) {
            mRssiEstimator = RobustRssiRadioSourceEstimator2D.create(mRssiRobustMethod);

            //rssi estimator will never need position estimator, but to
            //ensure it is ready we need to provide an initial position
            mRssiEstimator.setPositionEstimationEnabled(false);
            mRssiEstimator.setInitialPosition(Point2D.create());
        }
    }

    /**
     * Setups ranging estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void setupRangingEstimator() throws LockedException {
        super.setupRangingEstimator();

        switch (mRangingRobustMethod) {
            case RANSAC:
                ((RANSACRobustRangingRadioSourceEstimator2D<S>) mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                RANSACRobustRangingRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case LMEDS:
                ((LMedSRobustRangingRadioSourceEstimator2D<S>) mRangingEstimator).
                        setStopThreshold(mRangingThreshold != null ? mRangingThreshold :
                                LMedSRobustRangingRadioSourceEstimator2D.DEFAULT_STOP_THRESHOLD);
                break;
            case MSAC:
                ((MSACRobustRangingRadioSourceEstimator2D<S>) mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                MSACRobustRangingRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case PROSAC:
                ((PROSACRobustRangingRadioSourceEstimator2D<S>) mRangingEstimator).
                        setThreshold(mRangingThreshold != null ? mRangingThreshold :
                                PROSACRobustRangingRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case PROMEDS:
                ((PROMedSRobustRangingRadioSourceEstimator2D<S>) mRangingEstimator).
                        setStopThreshold(mRangingThreshold != null ? mRangingThreshold :
                                PROMedSRobustRangingRadioSourceEstimator2D.DEFAULT_STOP_THRESHOLD);
                break;
            default:
                break;
        }
    }

    /**
     * Setups RSSI estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void setupRssiEstimator() throws LockedException {
        super.setupRssiEstimator();

        switch (mRssiRobustMethod) {
            case RANSAC:
                ((RANSACRobustRssiRadioSourceEstimator2D<S>) mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                RANSACRobustRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case LMEDS:
                ((LMedSRobustRssiRadioSourceEstimator2D<S>) mRssiEstimator).
                        setStopThreshold(mRssiThreshold != null ? mRssiThreshold :
                                LMedSRobustRssiRadioSourceEstimator2D.DEFAULT_STOP_THRESHOLD);
                break;
            case MSAC:
                ((MSACRobustRssiRadioSourceEstimator2D<S>) mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                MSACRobustRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case PROSAC:
                ((PROSACRobustRssiRadioSourceEstimator2D<S>) mRssiEstimator).
                        setThreshold(mRssiThreshold != null ? mRssiThreshold :
                                PROSACRobustRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD);
                break;
            case PROMEDS:
                ((PROMedSRobustRssiRadioSourceEstimator2D<S>) mRssiEstimator).
                        setStopThreshold(mRssiThreshold != null ? mRssiThreshold :
                                PROMedSRobustRssiRadioSourceEstimator2D.DEFAULT_STOP_THRESHOLD);
                break;
            default:
                break;
        }
    }
}
