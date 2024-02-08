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
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.Reading;

import java.util.List;

/**
 * Robustly estimates 2D position, using RSSI readings first to obtain an initial coarse
 * position estimation, and then ranging readings to refine such estimation.
 * <p>
 * This implementation is like SequentialRobustRangingAndRssiPositionEstimator but
 * allows mixing different kinds of readings (ranging, RSSI or ranging+RSSI).
 */
public class SequentialRobustMixedPositionEstimator2D extends
        SequentialRobustMixedPositionEstimator<Point2D> {

    /**
     * Constructor.
     */
    public SequentialRobustMixedPositionEstimator2D() {
        super();
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of provided
     *                                  sources is less than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        super(sources);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super(fingerprint);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown location for provided
     *                    located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null or
     *                                  the number of provided sources is less than the
     *                                  required minimum.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super(sources, fingerprint);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final SequentialRobustMixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of provided
     *                                  sources is less than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final SequentialRobustMixedPositionEstimatorListener<Point2D> listener) {
        super(sources, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for provided
     *                    located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final SequentialRobustMixedPositionEstimatorListener<Point2D> listener) {
        super(fingerprint, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown location for provided
     *                    located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null or
     *                                  the number of provided sources is less than the
     *                                  required minimum.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final SequentialRobustMixedPositionEstimatorListener<Point2D> listener) {
        super(sources, fingerprint, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to each provided
     *                                        located radio source. The larger the score value
     *                                        the better the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings within
     *                                        provided fingerprint. The larger the score the
     *                                        better the quality of the reading.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores) {
        super(sourceQualityScores, fingerprintReadingQualityScores);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to each provided
     *                                        located radio source. The larger the score value
     *                                        the better the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings within
     *                                        provided fingerprint. The larger the score the
     *                                        better the quality of the reading.
     * @param sources                         located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of provided sources is less than the
     *                                  required minimum.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        super(sourceQualityScores, fingerprintReadingQualityScores, sources);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to each provided
     *                                        located radio source. The larger the score value
     *                                        the better the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings within
     *                                        provided fingerprint. The larger the score the
     *                                        better the quality of the reading.
     * @param fingerprint                     fingerprint containing readings at an unknown
     *                                        location for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super(sourceQualityScores, fingerprintReadingQualityScores, fingerprint);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to each provided
     *                                        located radio source. The larger the score value
     *                                        the better the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings within
     *                                        provided fingerprint. The larger the score the
     *                                        better the quality of the reading.
     * @param sources                         located radio sources used for lateration.
     * @param fingerprint                     fingerprint containing readings at an unknown
     *                                        location for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null or
     *                                  the number of provided sources is less than the
     *                                  required minimum.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super(sourceQualityScores, fingerprintReadingQualityScores, sources, fingerprint);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to each provided
     *                                        located radio source. The larger the score value
     *                                        the better the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings within
     *                                        provided fingerprint. The larger the score the
     *                                        better the quality of the reading.
     * @param listener                        listener in charge of handling events.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final SequentialRobustMixedPositionEstimatorListener<Point2D> listener) {
        super(sourceQualityScores, fingerprintReadingQualityScores, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to each provided
     *                                        located radio source. The larger the score value
     *                                        the better the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings within
     *                                        provided fingerprint. The larger the score the
     *                                        better the quality of the reading.
     * @param sources                         located radio sources used for lateration.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of provided
     *                                  sources is less than the required minimum.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final SequentialRobustMixedPositionEstimatorListener<Point2D> listener) {
        super(sourceQualityScores, fingerprintReadingQualityScores, sources, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to each provided
     *                                        located radio source. The larger the score value
     *                                        the better the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings within
     *                                        provided fingerprint. The larger the score the
     *                                        better the quality of the reading.
     * @param fingerprint                     fingerprint containing readings at an unknown
     *                                        location for provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final SequentialRobustMixedPositionEstimatorListener<Point2D> listener) {
        super(sourceQualityScores, fingerprintReadingQualityScores, fingerprint, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param sourceQualityScores             quality scores corresponding to each provided
     *                                        located radio source. The larger the score value
     *                                        the better the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings within
     *                                        provided fingerprint. The larger the score the
     *                                        better the quality of the reading.
     * @param sources                         located radio sources used for lateration.
     * @param fingerprint                     fingerprint containing readings at an unknown
     *                                        location for provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null or
     *                                  the number of provided sources is less than the
     *                                  required minimum.
     */
    public SequentialRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final SequentialRobustMixedPositionEstimatorListener<Point2D> listener) {
        super(sourceQualityScores, fingerprintReadingQualityScores, sources, fingerprint, listener);
        mRangingPreliminarySubsetSize = mRssiPreliminarySubsetSize = getMinRequiredSources();
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
     * @return minimum required number of located radio sources to perform lateration.
     */
    @Override
    public int getMinRequiredSources() {
        return Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Builds ranging internal estimator.
     */
    @Override
    protected void buildRangingEstimator() {
        mRangingEstimator = RobustRangingPositionEstimator2D.create(mRangingRobustMethod);
    }

    /**
     * Builds RSSI internal estimator.
     */
    @Override
    protected void buildRssiEstimator() {
        mRssiEstimator = RobustRssiPositionEstimator2D.create(mRssiRobustMethod);
    }

    /**
     * Setup ranging internal estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void setupRangingEstimator() throws LockedException {
        super.setupRangingEstimator();
        if (mRangingThreshold != null) {
            switch (mRangingRobustMethod) {
                case RANSAC:
                    ((RANSACRobustRangingPositionEstimator2D) mRangingEstimator).setThreshold(mRangingThreshold);
                    break;
                case LMEDS:
                    ((LMedSRobustRangingPositionEstimator2D) mRangingEstimator).setStopThreshold(mRangingThreshold);
                    break;
                case MSAC:
                    ((MSACRobustRangingPositionEstimator2D) mRangingEstimator).setThreshold(mRangingThreshold);
                    break;
                case PROSAC:
                    ((PROSACRobustRangingPositionEstimator2D) mRangingEstimator).setThreshold(mRangingThreshold);
                    break;
                case PROMEDS:
                default:
                    ((PROMedSRobustRangingPositionEstimator2D) mRangingEstimator).setStopThreshold(mRangingThreshold);
                    break;
            }
        }
    }

    /**
     * Setup RSSI internal estimator.
     *
     * @throws LockedException if estimator is locked.
     */
    @Override
    protected void setupRssiEstimator() throws LockedException {
        super.setupRssiEstimator();
        if (mRssiThreshold != null) {
            switch (mRssiRobustMethod) {
                case RANSAC:
                    ((RANSACRobustRssiPositionEstimator2D) mRssiEstimator).setThreshold(mRssiThreshold);
                    break;
                case LMEDS:
                    ((LMedSRobustRssiPositionEstimator2D) mRssiEstimator).setStopThreshold(mRssiThreshold);
                    break;
                case MSAC:
                    ((MSACRobustRssiPositionEstimator2D) mRssiEstimator).setThreshold(mRssiThreshold);
                    break;
                case PROSAC:
                    ((PROSACRobustRssiPositionEstimator2D) mRssiEstimator).setThreshold(mRssiThreshold);
                    break;
                case PROMEDS:
                default:
                    ((PROMedSRobustRssiPositionEstimator2D) mRssiEstimator).setStopThreshold(mRssiThreshold);
                    break;
            }
        }
    }
}
