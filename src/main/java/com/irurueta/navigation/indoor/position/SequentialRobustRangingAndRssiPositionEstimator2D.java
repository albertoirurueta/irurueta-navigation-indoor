/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RangingAndRssiFingerprint;
import com.irurueta.navigation.indoor.RangingAndRssiReading;

import java.util.List;

/**
 * Robustly estimates 2D position, using RSSI readings first to obtain an initial coarse
 * position estimation, and then ranging readings to refine such estimation.
 */
public class SequentialRobustRangingAndRssiPositionEstimator2D extends
        SequentialRobustRangingAndRssiPositionEstimator<Point2D> {

    /**
     * Constructor.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D() {
        super();
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        super(sources);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        super(fingerprint);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        super(sources, fingerprint);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(sources, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(sources, fingerprint, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores) {
        super(sourceQualityScores, fingerprintReadingQualityScores);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        super(sourceQualityScores, fingerprintReadingQualityScores, sources);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing RSSI readings at an
     *                                        unknown location for provided located
     *                                        radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        super(sourceQualityScores, fingerprintReadingQualityScores, fingerprint);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param fingerprint                     fingerprint containing RSSI readings at an
     *                                        unknown location for provided located
     *                                        radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        super(sourceQualityScores, fingerprintReadingQualityScores, sources, fingerprint);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param listener                        listener in charge of handling events.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(sourceQualityScores, fingerprintReadingQualityScores, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(sourceQualityScores, fingerprintReadingQualityScores, sources, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing RSSI readings at an
     *                                        unknown location for provided located
     *                                        radio sources.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(sourceQualityScores, fingerprintReadingQualityScores, fingerprint, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param sources                         located radio sources used for
     *                                        lateration.
     * @param fingerprint                     fingerprint containing RSSI readings at an
     *                                        unknown location for provided located
     *                                        radio sources.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public SequentialRobustRangingAndRssiPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> listener) {
        super(sourceQualityScores, fingerprintReadingQualityScores, sources, fingerprint, listener);
        rangingPreliminarySubsetSize = rssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Gets number of dimensions of provided and estimated points.
     *
     * @return number of dimensions of provided and estimated points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets minimum required number of located radio sources to perform lateration.
     *
     * @return minimum required number of located radio sources to perform
     * lateration.
     */
    @Override
    public int getMinRequiredSources() {
        return Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Builds ranging and RSSI internal estimators.
     */
    @Override
    protected void buildEstimators() {
        rssiEstimator = RobustRssiPositionEstimator2D.create(rssiRobustMethod);
        rangingEstimator = RobustRangingPositionEstimator2D.create(rangingRobustMethod);
    }
}
