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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Base class for robust ranging 2D position estimators using located radio sources and
 * their ranging readings at unknown locations.
 * These kind of estimators can be used to robustly determine the 2D position of a given
 * device by getting ranging readings at an unknown location of different radio sources
 * whose locations are known.
 * Implementations of this class should be able to detect and discard outliers in order
 * to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class RobustRangingPositionEstimator2D extends
        RobustRangingPositionEstimator<Point2D> {

    /**
     * Constructor.
     */
    protected RobustRangingPositionEstimator2D() {
        super();
        preliminarySubsetSize = getMinRequiredSources();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    protected RobustRangingPositionEstimator2D(final RobustRangingPositionEstimatorListener<Point2D> listener) {
        super(listener);
        preliminarySubsetSize = getMinRequiredSources();
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
     * Creates a robust 2D position estimator.
     *
     * @param method robust estimator method.
     * @return a robust 2D position estimator.
     */
    public static RobustRangingPositionEstimator2D create(final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D();
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D();
            case MSAC -> new MSACRobustRangingPositionEstimator2D();
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D();
            default -> new PROMedSRobustRangingPositionEstimator2D();
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sources located radio sources used for lateration.
     * @param method  robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final List<? extends RadioSourceLocated<Point2D>> sources, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(sources);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(sources);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(sources);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sources);
            default -> new PROMedSRobustRangingPositionEstimator2D(sources);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @param method      robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRangingPositionEstimator2D create(
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(fingerprint);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(fingerprint);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(fingerprint);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(fingerprint);
            default -> new PROMedSRobustRangingPositionEstimator2D(fingerprint);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @param method      robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(sources, fingerprint);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(sources, fingerprint);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(sources, fingerprint);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sources, fingerprint);
            default -> new PROMedSRobustRangingPositionEstimator2D(sources, fingerprint);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param listener listener in charge of handling events.
     * @param method   robust estimator method.
     * @return a robust 2D position estimator.
     */
    public static RobustRangingPositionEstimator2D create(
            final RobustRangingPositionEstimatorListener<Point2D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(listener);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(listener);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(listener);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(listener);
            default -> new PROMedSRobustRangingPositionEstimator2D(listener);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @param method   robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustRangingPositionEstimatorListener<Point2D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(sources, listener);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(sources, listener);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(sources, listener);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sources, listener);
            default -> new PROMedSRobustRangingPositionEstimator2D(sources, listener);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @param method      robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRangingPositionEstimator2D create(
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustRangingPositionEstimatorListener<Point2D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(fingerprint, listener);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(fingerprint, listener);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(fingerprint, listener);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(fingerprint, listener);
            default -> new PROMedSRobustRangingPositionEstimator2D(fingerprint, listener);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @param method      robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustRangingPositionEstimatorListener<Point2D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(sources, fingerprint, listener);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(sources, fingerprint, listener);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(sources, fingerprint, listener);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sources, fingerprint, listener);
            default -> new PROMedSRobustRangingPositionEstimator2D(sources, fingerprint, listener);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param method                          robust estimator method.
     * @return a robust 2D position estimator.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D();
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D();
            case MSAC -> new MSACRobustRangingPositionEstimator2D();
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores);
            default -> new PROMedSRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores);
        };
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @param method                          robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(sources);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(sources);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(sources);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores, sources);
            default -> new PROMedSRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingQualityScores,
                    sources);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing ranging
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param method                          robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(fingerprint);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(fingerprint);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(fingerprint);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores, fingerprint);
            default -> new PROMedSRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingQualityScores,
                    fingerprint);
        };
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @param fingerprint                     fingerprint containing ranging
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param method                          robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(sources, fingerprint);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(sources, fingerprint);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(sources, fingerprint);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores, sources, fingerprint);
            default -> new PROMedSRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingQualityScores,
                    sources, fingerprint);
        };
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @param method                          robust estimator method.
     * @return a robust 2D position estimator.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RobustRangingPositionEstimatorListener<Point2D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(listener);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(listener);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(listener);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores, listener);
            default -> new PROMedSRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingQualityScores,
                    listener);
        };
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @param method                          robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustRangingPositionEstimatorListener<Point2D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(sources, listener);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(sources, listener);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(sources, listener);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores, sources, listener);
            default -> new PROMedSRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingQualityScores,
                    sources, listener);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing ranging
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @param method                          robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores,
            final double[] fingerprintReadingQualityScores,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustRangingPositionEstimatorListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(fingerprint, listener);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(fingerprint, listener);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(fingerprint, listener);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores, fingerprint, listener);
            default -> new PROMedSRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingQualityScores,
                    fingerprint, listener);
        };
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @param fingerprint                     fingerprint containing ranging
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @param method                          robust estimator method.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustRangingPositionEstimatorListener<Point2D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingPositionEstimator2D(sources, fingerprint, listener);
            case LMEDS -> new LMedSRobustRangingPositionEstimator2D(sources, fingerprint, listener);
            case MSAC -> new MSACRobustRangingPositionEstimator2D(sources, fingerprint, listener);
            case PROSAC -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingQualityScores, sources, fingerprint, listener);
            default -> new PROMedSRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingQualityScores,
                    sources, fingerprint, listener);
        };
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @return a robust 2D position estimator.
     */
    public static RobustRangingPositionEstimator2D create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sources located radio sources used for lateration.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(final List<? extends RadioSourceLocated<Point2D>> sources) {
        return create(sources, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRangingPositionEstimator2D create(
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint) {
        return create(fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint) {
        return create(sources, fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     */
    public static RobustRangingPositionEstimator2D create(
            final RobustRangingPositionEstimatorListener<Point2D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustRangingPositionEstimatorListener<Point2D> listener) {
        return create(sources, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRangingPositionEstimator2D create(
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustRangingPositionEstimatorListener<Point2D> listener) {
        return create(fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustRangingPositionEstimatorListener<Point2D> listener) {
        return create(sources, fingerprint, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @return a robust 2D position estimator.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores) {
        return create(sourceQualityScores, fingerprintReadingQualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources) {
        return create(sourceQualityScores, fingerprintReadingQualityScores, sources, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing ranging
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint) {
        return create(sourceQualityScores, fingerprintReadingQualityScores, fingerprint, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @param fingerprint                     fingerprint containing ranging
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint) {
        return create(sourceQualityScores, fingerprintReadingQualityScores, sources, fingerprint,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @return a robust 2D position estimator.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RobustRangingPositionEstimatorListener<Point2D> listener) {
        return create(sourceQualityScores, fingerprintReadingQualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustRangingPositionEstimatorListener<Point2D> listener) {
        return create(sourceQualityScores, fingerprintReadingQualityScores, sources, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
     *
     * @param sourceQualityScores             quality scores corresponding to
     *                                        each provided located radio source.
     *                                        The larger the score value the better
     *                                        the quality of the radio source.
     * @param fingerprintReadingQualityScores quality scores corresponding to readings
     *                                        within provided fingerprint. The larger
     *                                        the score the better the quality of the
     *                                        reading.
     * @param fingerprint                     fingerprint containing ranging
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustRangingPositionEstimatorListener<Point2D> listener) {
        return create(sourceQualityScores, fingerprintReadingQualityScores, fingerprint, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 2D position estimator.
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
     * @param fingerprint                     fingerprint containing ranging
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @return a robust 2D position estimator.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public static RobustRangingPositionEstimator2D create(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RangingFingerprint<? extends RadioSource, ? extends RangingReading<?
                    extends RadioSource>> fingerprint,
            final RobustRangingPositionEstimatorListener<Point2D> listener) {
        return create(sourceQualityScores, fingerprintReadingQualityScores, sources, fingerprint, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal
     * lateration solver.
     *
     * @param positions                  positions to be set.
     * @param distances                  distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set.
     * @param distanceQualityScores      distance quality scores or null if not
     *                                   required.
     */
    @Override
    protected void setPositionsDistancesDistanceStandardDeviationsAndQualityScores(
            final List<Point2D> positions, List<Double> distances, final List<Double> distanceStandardDeviations,
            final List<Double> distanceQualityScores) {
        final var size = positions.size();
        Point2D[] positionsArray = new InhomogeneousPoint2D[size];
        positionsArray = positions.toArray(positionsArray);

        final var distancesArray = new double[size];
        final var distanceStandardDeviationsArray = new double[size];

        double[] qualityScoresArray = null;
        if (distanceQualityScores != null) {
            qualityScoresArray = new double[size];
        }

        for (var i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
            distanceStandardDeviationsArray[i] = distanceStandardDeviations.get(i);

            if (qualityScoresArray != null) {
                qualityScoresArray[i] = distanceQualityScores.get(i);
            }
        }

        try {
            laterationSolver.setPositionsDistancesAndStandardDeviations(positionsArray, distancesArray,
                    distanceStandardDeviationsArray);

            if (qualityScoresArray != null) {
                laterationSolver.setQualityScores(qualityScoresArray);
            }
        } catch (final LockedException e) {
            throw new IllegalArgumentException(e);
        }
    }
}
