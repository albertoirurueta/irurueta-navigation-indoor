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

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * Robustly estimate 2D position, transmitted power and path-loss exponent of a radio
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
 * This implementation is like SequentialRobustRangingAndRssiRadioSourceEstimator but
 * allows mixing different kinds of located radio source readings (ranging, RSSI
 * and ranging+RSSI).
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("Duplicates")
public class SequentialRobustMixedRadioSourceEstimator2D<S extends RadioSource> extends
        SequentialRobustMixedRadioSourceEstimator<S, Point2D> {

    /**
     * Constructor.
     */
    public SequentialRobustMixedRadioSourceEstimator2D() {
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator2D(final List<? extends ReadingLocated<Point2D>> readings) {
        super(readings);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator2D(
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition) {
        super(readings, initialPosition);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public SequentialRobustMixedRadioSourceEstimator2D(final Point2D initialPosition) {
        super(initialPosition);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator2D(
            final Point2D initialPosition,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public SequentialRobustMixedRadioSourceEstimator2D(final Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public SequentialRobustMixedRadioSourceEstimator2D(
            final Double initialTransmittedPowerdBm,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialTransmittedPowerdBm, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Double initialTransmittedPowerdBm,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialPosition, initialTransmittedPowerdBm);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final Point2D initialPosition, final Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent) {
        super(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(final double[] qualityScores) {
        super(qualityScores);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings) {
        super(qualityScores, readings);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final Point2D initialPosition) {
        super(qualityScores, readings, initialPosition);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(final double[] qualityScores, final Point2D initialPosition) {
        super(qualityScores, initialPosition);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final Point2D initialPosition,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, initialPosition, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final Point2D initialPosition,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, initialPosition, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final Double initialTransmittedPowerdBm) {
        super(qualityScores, initialTransmittedPowerdBm);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final Double initialTransmittedPowerdBm) {
        super(qualityScores, readings, initialTransmittedPowerdBm);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final Double initialTransmittedPowerdBm,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, initialTransmittedPowerdBm, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final Double initialTransmittedPowerdBm,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, initialTransmittedPowerdBm, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final Point2D initialPosition, final Double initialTransmittedPowerdBm) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final Point2D initialPosition, final Double initialTransmittedPowerdBm) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
    public SequentialRobustMixedRadioSourceEstimator2D(
            final double[] qualityScores, final List<? extends ReadingLocated<Point2D>> readings,
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final SequentialRobustMixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(qualityScores, readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinReadings();
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
        var minReadings = Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
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
    @SuppressWarnings("unchecked")
    public RadioSourceLocated<Point2D> getEstimatedRadioSource() {
        final var readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }

        final S source;
        final var reading = readings.get(0);
        if (reading instanceof RangingReadingLocated) {
            source = ((RangingReadingLocated<S, Point2D>) reading).getSource();
        } else if (reading instanceof RssiReadingLocated) {
            source = ((RssiReadingLocated<S, Point2D>) reading).getSource();
        } else if (reading instanceof RangingAndRssiReadingLocated) {
            source = ((RangingAndRssiReadingLocated<S, Point2D>) reading).getSource();
        } else {
            return null;
        }

        final var estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        final var estimatedPositionCovariance = getEstimatedPositionCovariance();

        final var transmittedPowerdBm = getEstimatedTransmittedPowerdBm();

        final var transmittedPowerVariance = getEstimatedTransmittedPowerVariance();
        final var transmittedPowerStandardDeviation = transmittedPowerVariance != null
                ? Math.sqrt(transmittedPowerVariance) : null;

        final var pathlossExponentVariance = getEstimatedPathLossExponentVariance();
        final var pathlossExponentStandardDeviation = pathlossExponentVariance != null
                ? Math.sqrt(pathlossExponentVariance) : null;

        if (source instanceof WifiAccessPoint accessPoint) {
            if (transmittedPowerdBm != null) {
                return new WifiAccessPointWithPowerAndLocated2D(accessPoint.getBssid(), source.getFrequency(),
                        accessPoint.getSsid(), transmittedPowerdBm, transmittedPowerStandardDeviation,
                        getEstimatedPathLossExponent(), pathlossExponentStandardDeviation, estimatedPosition,
                        estimatedPositionCovariance);
            } else {
                return new WifiAccessPointLocated2D(accessPoint.getBssid(),
                        source.getFrequency(), accessPoint.getSsid(),
                        estimatedPosition, estimatedPositionCovariance);
            }
        } else if (source instanceof Beacon beacon) {
            return new BeaconWithPowerAndLocated2D(beacon.getIdentifiers(), beacon.getTransmittedPower(),
                    beacon.getFrequency(), beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(), beacon.getBluetoothName(),
                    getEstimatedPathLossExponent(), transmittedPowerStandardDeviation,
                    pathlossExponentStandardDeviation, estimatedPosition, estimatedPositionCovariance);
        } else {
            return null;
        }
    }

    /**
     * Builds ranging estimator.
     */
    @Override
    protected void buildRangingEstimatorIfNeeded() {
        if (rangingEstimator == null || rangingEstimator.getMethod() != rangingRobustMethod) {
            rangingEstimator = RobustRangingRadioSourceEstimator2D.create(rangingRobustMethod);
        }
    }

    /**
     * build RSSI estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void buildRssiEstimatorIfNeeded() throws LockedException {
        if (rssiEstimator == null || rssiEstimator.getMethod() != rssiRobustMethod) {
            rssiEstimator = RobustRssiRadioSourceEstimator2D.create(rssiRobustMethod);

            // rssi estimator will never need position estimator, but to
            // ensure it is ready we need to provide an initial position
            rssiEstimator.setPositionEstimationEnabled(false);
            rssiEstimator.setInitialPosition(Point2D.create());
        }
    }
}
