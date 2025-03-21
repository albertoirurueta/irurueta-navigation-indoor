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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RangingAndRssiFingerprint;
import com.irurueta.navigation.indoor.RangingAndRssiReading;
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for robust position estimation, using RSSI readings first to obtain
 * an initial coarse position estimation, and then ranging readings to refine such
 * estimation.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class SequentialRobustRangingAndRssiPositionEstimator<P extends Point<?>> {

    /**
     * Default robust estimator method for robust position estimation using ranging
     * data when no robust method is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_RANGING_ROBUST_METHOD = RobustEstimatorMethod.PROMEDS;

    /**
     * Default robust method for coarse robust position estimation using RSSI
     * data when no robust method is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_RSSI_ROBUST_METHOD = RobustEstimatorMethod.PROMEDS;

    /**
     * Indicates that by default located radio source position covariance is taken
     * into account (if available) to determine distance standard deviation for ranging
     * measurements.
     */
    public static final boolean DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE = true;

    /**
     * Indicates that by default located radio source position covariance is taken
     * into account (if available) to determine distance standard deviation for RSSI
     * measurements.
     */
    public static final boolean DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE = true;

    /**
     * Indicates that by default readings are distributed evenly among radio sources
     * taking into account quality scores of both radio sources and ranging readings.
     */
    public static final boolean DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS = true;

    /**
     * Indicates that by default readings are distributed evenly among radio sources
     * taking into account quality scores of both radio sources and RSSI readings.
     */
    public static final boolean DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS = true;

    /**
     * Distance standard deviation assumed for provided distances as a fallback when
     * none can be determined.
     */
    public static final double FALLBACK_DISTANCE_STANDARD_DEVIATION =
            RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Default amount of progress variation before notifying a change in estimation progress.
     * By default, this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;

    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;

    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;

    /**
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen sub-samples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;

    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;

    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;

    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;

    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;

    /**
     * Indicates that result is refined by default using all found inliers.
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = true;

    /**
     * Indicates that by default a linear solver is used for preliminary solution
     * estimation using ranging measurements.
     * The result obtained on each preliminary solution might be later refined.
     */
    public static final boolean DEFAULT_USE_RANGING_LINEAR_SOLVER = true;

    /**
     * Indicates that by default a linear solver is used for preliminary solution
     * estimation using RSSI measurements.
     * The result obtained on each preliminary solution might be later refined.
     */
    public static final boolean DEFAULT_USE_RSSI_LINEAR_SOLVER = true;

    /**
     * Indicates that by default an homogeneous linear solver is used either to
     * estimate preliminary solutions or an initial solution for preliminary solutions
     * that will be later refined on the ranging fine estimation.
     */
    public static final boolean DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER = false;

    /**
     * Indicates that by default an homogeneous linear solver is used either to
     * estimate preliminary solutions or an initial solution for preliminary solutions
     * that will be later refined on the RSSI coarse estimation.
     */
    public static final boolean DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER = false;

    /**
     * Indicates that by default preliminary ranging solutions are refined.
     */
    public static final boolean DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS = true;

    /**
     * Indicates that by default preliminary RSSI solutions are refined.
     */
    public static final boolean DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS = true;

    /**
     * Internal robust estimator for position estimation using ranging readings.
     */
    protected RobustRangingPositionEstimator<P> rangingEstimator;

    /**
     * Internal robust estimator for coarse position estimation using RSSI readings.
     */
    protected RobustRssiPositionEstimator<P> rssiEstimator;

    /**
     * Robust method used for robust position estimation using ranging data.
     */
    protected RobustEstimatorMethod rangingRobustMethod = DEFAULT_RANGING_ROBUST_METHOD;

    /**
     * Robust method used for coarse robust position estimation using RSSI data.
     */
    protected RobustEstimatorMethod rssiRobustMethod = DEFAULT_RSSI_ROBUST_METHOD;

    /**
     * Size of subsets to be checked during robust estimation.
     */
    protected int rangingPreliminarySubsetSize;

    /**
     * Size of subsets to be checked during RSSI robust estimation.
     */
    protected int rssiPreliminarySubsetSize;

    /**
     * Indicates whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for ranging measurements.
     */
    private boolean useRangingRadioSourcePositionCovariance = DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE;

    /**
     * Indicates whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for RSSI measurements.
     */
    private boolean useRssiRadioSourcePositionCovariance = DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE;

    /**
     * Indicates whether ranging readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and ranging readings.
     */
    private boolean evenlyDistributeRangingReadings = DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS;

    /**
     * Indicates whether RSSI readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and RSSI readings.
     */
    private boolean evenlyDistributeRssiReadings = DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS;

    /**
     * Distance standard deviation fallback value to use when none can be determined
     * from provided RSSI measurements.
     */
    private double rssiFallbackDistanceStandardDeviation = FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Distance standard deviation fallback value to use when none can be determined
     * from provided ranging measurements.
     */
    private double rangingFallbackDistanceStandardDeviation = FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    private float progressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on ranging data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     */
    private double rangingConfidence = DEFAULT_CONFIDENCE;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on RSSI data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     */
    private double rssiConfidence = DEFAULT_CONFIDENCE;

    /**
     * Maximum allowed number of iterations for robust ranging position estimation.
     * When the maximum number of iterations is exceeded, an approximate result
     * might be available for retrieval.
     */
    private int rangingMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Maximum allowed number of iterations for robust RSSI position estimation.
     * When the maximum number of iterations is exceeded, an approximate result
     * might be available for retrieval.
     */
    private int rssiMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Indicates whether result is refined using all found inliers.
     */
    private boolean refineResult = DEFAULT_REFINE_RESULT;

    /**
     * Indicates that covariance is kept after refining result.
     */
    private boolean keepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Indicates that a linear solver is used for preliminary solution estimation
     * using ranging measurements.
     * The result obtained on each preliminary solution might be later refined.
     */
    private boolean useRangingLinearSolver = DEFAULT_USE_RANGING_LINEAR_SOLVER;

    /**
     * Indicates that a linear solver is used for preliminary solution estimation
     * using RSSI measurements.
     * The result obtained on each preliminary solution might be later refined.
     */
    private boolean useRssiLinearSolver = DEFAULT_USE_RSSI_LINEAR_SOLVER;

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the ranging fine estimation.
     */
    private boolean useRangingHomogeneousLinearSolver = DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the RSSI coarse estimation.
     */
    private boolean useRssiHomogeneousLinearSolver = DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Indicates whether preliminary ranging solutions are refined.
     */
    private boolean refineRangingPreliminarySolutions = DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS;

    /**
     * Indicates whether preliminary RSSI solutions are refined.
     */
    private boolean refineRssiPreliminarySolutions = DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS;

    /**
     * Threshold to determine when samples are inliers or not used during ranging
     * position estimation.
     * If not defined, default threshold will be used.
     */
    private Double rangingThreshold;

    /**
     * Threshold to determine when samples are inliers or not used during RSSI
     * position estimation.
     * If not defined, default threshold will be used.
     */
    private Double rssiThreshold;

    /**
     * Listener in charge of handling events.
     */
    private SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener;

    /**
     * Located radio sources used for lateration.
     */
    private List<? extends RadioSourceLocated<P>> sources;

    /**
     * Fingerprint containing readings at an unknown location for provided located
     * radio sources.
     */
    private RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
            extends RadioSource>> fingerprint;

    /**
     * Quality scores corresponding to each provided located radio source.
     * The larger the score value the better the quality of the radio source.
     */
    private double[] sourceQualityScores;

    /**
     * Quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     */
    private double[] fingerprintReadingsQualityScores;

    /**
     * An initial position to start the estimation from. This can be useful if we only
     * intend to refine a previously known estimation.
     */
    private P initialPosition;

    /**
     * Indicates if this instance is locked because estimation is being executed.
     */
    private boolean locked;

    /**
     * Constructor.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator() {
    }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(final List<? extends RadioSourceLocated<P>> sources) {
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing ranging+RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging+RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final List<? extends RadioSourceLocated<P>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final List<? extends RadioSourceLocated<P>> sources,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener) {
        this(sources);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing ranging+RSSI readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener) {
        this(fingerprint);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging+RSSI readings at an
     *                    unknown location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final List<? extends RadioSourceLocated<P>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener) {
        this(sources, fingerprint);
        this.listener = listener;
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
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores) {
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
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
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<P>> sources) {
        this(sources);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
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
     * @param fingerprint                     fingerprint containing ranging+RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        this(fingerprint);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
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
     * @param fingerprint                     fingerprint containing ranging+RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<P>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        this(sources, fingerprint);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
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
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener) {
        this(sourceQualityScores, fingerprintReadingQualityScores);
        this.listener = listener;
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
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<P>> sources,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener) {
        this(sourceQualityScores, fingerprintReadingQualityScores, sources);
        this.listener = listener;
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
     * @param fingerprint                     fingerprint containing ranging+RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener) {
        this(sourceQualityScores, fingerprintReadingQualityScores, fingerprint);
        this.listener = listener;
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
     * @param fingerprint                     fingerprint containing ranging+RSSI
     *                                        readings at an unknown location for
     *                                        provided located radio sources.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    protected SequentialRobustRangingAndRssiPositionEstimator(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<P>> sources,
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint,
            final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener) {
        this(sourceQualityScores, fingerprintReadingQualityScores, sources, fingerprint);
        this.listener = listener;
    }

    /**
     * Gets robust method used for robust position estimation using ranging data.
     *
     * @return robust method used for robust position estimation using ranging data.
     */
    public RobustEstimatorMethod getRangingRobustMethod() {
        return rangingRobustMethod;
    }

    /**
     * Sets robust method for robust position estimation using ranging data.
     *
     * @param rangingRobustMethod robust method used for robust position estimation
     *                            using ranging data.
     * @throws LockedException if this instance is locked.
     */
    public void setRangingRobustMethod(final RobustEstimatorMethod rangingRobustMethod) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.rangingRobustMethod = rangingRobustMethod;
    }

    /**
     * Gets robust method used for coarse robust position estimation using RSSI data.
     *
     * @return robust method used for coarse robust position estimation using RSSI data.
     */
    public RobustEstimatorMethod getRssiRobustMethod() {
        return rssiRobustMethod;
    }

    /**
     * Sets robust method used for coarse robust position estimation using RSSI data.
     *
     * @param rssiRobustMethod robust method used for coarse robust position estimation
     *                         using RSSI data.
     * @throws LockedException if this instance is locked.
     */
    public void setRssiRobustMethod(final RobustEstimatorMethod rssiRobustMethod) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.rssiRobustMethod = rssiRobustMethod;
    }

    /**
     * Indicates whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for ranging measurements.
     *
     * @return true to take into account radio source position covariance during
     * ranging position estimation, false otherwise.
     */
    public boolean isRangingRadioSourcePositionCovarianceUsed() {
        return useRangingRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether located radio source position covariance is taken into account
     * (if available) to determine distance standard deviation for ranging measurements.
     *
     * @param useRangingRadioSourcePositionCovariance true to take into account radio
     *                                                source position covariance during
     *                                                ranging position estimation, false
     *                                                otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setRangingRadioSourcePositionCovarianceUsed(final boolean useRangingRadioSourcePositionCovariance)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.useRangingRadioSourcePositionCovariance = useRangingRadioSourcePositionCovariance;
    }

    /**
     * Indicates whether located radio source position covariance is taken into
     * account (if available) to determine distance standard deviation for RSSI
     * measurements.
     *
     * @return true to take into account radio source position covariance during
     * RSSI position estimation, false otherwise.
     */
    public boolean isRssiRadioSourcePositionCovarianceUsed() {
        return useRssiRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether located radio source position covariance is taken into
     * account (if available) to determine distance standard deviation for RSSI
     * measurements.
     *
     * @param useRssiRadioSourcePositionCovariance true to take into account radio
     *                                             source position covariance during
     *                                             RSSI position estimation, false
     *                                             otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setRssiRadioSourcePositionCovarianceUsed(final boolean useRssiRadioSourcePositionCovariance)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.useRssiRadioSourcePositionCovariance = useRssiRadioSourcePositionCovariance;
    }

    /**
     * Indicates whether ranging readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and ranging readings.
     *
     * @return true if ranging readings are evenly distributed among radio sources,
     * false otherwise.
     */
    public boolean isRangingReadingsEvenlyDistributed() {
        return evenlyDistributeRangingReadings;
    }

    /**
     * Specifies whether ranging readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and ranging readings.
     *
     * @param evenlyDistributeRangingReadings true if ranging readings are evenly
     *                                        distributed among radio sources, false
     *                                        otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setRangingReadingsEvenlyDistributed(final boolean evenlyDistributeRangingReadings)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.evenlyDistributeRangingReadings = evenlyDistributeRangingReadings;
    }

    /**
     * Gets distance standard deviation fallback value to use when none can be
     * determined from provided RSSI measurements.
     *
     * @return distance standard deviation fallback value to use when none can be
     * determined from provided RSSI measurements.
     */
    public double getRssiFallbackDistanceStandardDeviation() {
        return rssiFallbackDistanceStandardDeviation;
    }

    /**
     * Sets distance standard deviation fallback value to use when none can be
     * determined from provided RSSI measurements.
     *
     * @param rssiFallbackDistanceStandardDeviation distance standard deviation
     *                                              fallback value to use when none can
     *                                              be determined from provided RSSI
     *                                              measurements.
     * @throws LockedException if this instance is locked.
     */
    public void setRssiFallbackDistanceStandardDeviation(final double rssiFallbackDistanceStandardDeviation)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.rssiFallbackDistanceStandardDeviation = rssiFallbackDistanceStandardDeviation;
    }

    /**
     * Gets distance standard deviation fallback value to use when none can be
     * determined from provided ranging measurements.
     *
     * @return distance standard deviation fallback value to use when none can be
     * determined from provided ranging measurements.
     */
    public double getRangingFallbackDistanceStandardDeviation() {
        return rangingFallbackDistanceStandardDeviation;
    }

    /**
     * Sets distance standard deviation fallback value to use when none can be
     * determined from provided ranging measurements.
     *
     * @param rangingFallbackDistanceStandardDeviation distance standard deviation
     *                                                 fallback value to use when none can
     *                                                 be determined from provided ranging
     *                                                 measurements.
     * @throws LockedException if this instance is locked.
     */
    public void setRangingFallbackDistanceStandardDeviation(final double rangingFallbackDistanceStandardDeviation)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.rangingFallbackDistanceStandardDeviation = rangingFallbackDistanceStandardDeviation;
    }

    /**
     * Indicates whether RSSI readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and RSSI readings.
     *
     * @return true if RSSI readings are evenly distributed among radio sources,
     * false otherwise.
     */
    public boolean isRssiReadingsEvenlyDistributed() {
        return evenlyDistributeRssiReadings;
    }

    /**
     * Specifies whether RSSI readings are evenly distributed among radio sources
     * taking into account quality scores of both radio sources and RSSI readings.
     *
     * @param evenlyDistributeRssiReadings true if RSSI readings are evenly distributed
     *                                     among radio sources, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setRssiReadingsEvenlyDistributed(final boolean evenlyDistributeRssiReadings) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.evenlyDistributeRssiReadings = evenlyDistributeRssiReadings;
    }

    /**
     * Gets amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @return amount of progress variation before notifying a progress change during
     * estimation.
     */
    public float getProgressDelta() {
        return progressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * estimation.
     *
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException          if this instance is locked.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        this.progressDelta = progressDelta;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on ranging data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence for robust position estimation as a value between
     * 0.0 and 1.0.
     */
    public double getRangingConfidence() {
        return rangingConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on ranging data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @param rangingConfidence confidence to be set for robust position estimation
     *                          as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if estimator is locked.
     */
    public void setRangingConfidence(final double rangingConfidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rangingConfidence < MIN_CONFIDENCE || rangingConfidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        this.rangingConfidence = rangingConfidence;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on RSSI data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence for robust position estimation as a value between
     * 0.0 and 1.0.
     */
    public double getRssiConfidence() {
        return rssiConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%) for robust position estimation on RSSI data. The amount
     * of confidence indicates the probability that the estimated result is correct.
     * Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @param rssiConfidence amount of confidence for robust position estimation as a
     *                       value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if estimator is locked.
     */
    public void setRssiConfidence(final double rssiConfidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rssiConfidence < MIN_CONFIDENCE || rssiConfidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        this.rssiConfidence = rssiConfidence;
    }

    /**
     * Gets maximum allowed number of iterations for robust ranging position estimation.
     * When the maximum number of iterations is exceeded, an approximate result might
     * be available for retrieval.
     *
     * @return maximum allowed number of iterations for position estimation.
     */
    public int getRangingMaxIterations() {
        return rangingMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations for robust ranging position
     * estimation.
     * When the maximum number of iterations is exceeded, an approximate result might
     * be available for retrieval.
     *
     * @param rangingMaxIterations maximum allowed number of iterations to be set for
     *                             position estimation.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if estimator is locked.
     */
    public void setRangingMaxIterations(final int rangingMaxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rangingMaxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        this.rangingMaxIterations = rangingMaxIterations;
    }

    /**
     * Gets maximum allowed number of iterations for robust RSSI position estimation.
     * When the maximum number of iterations is exceeded, an approximate result might
     * be available for retrieval.
     *
     * @return maximum allowed number of iterations for position estimation.
     */
    public int getRssiMaxIterations() {
        return rssiMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations for robust RSSI position estimation.
     * When the maximum number of iterations is exceeded, an approximate result might
     * be available for retrieval.
     *
     * @param rssiMaxIterations maximum allowed number of iterations to be set for
     *                          position estimation.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if estimator is locked.
     */
    public void setRssiMaxIterations(final int rssiMaxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rssiMaxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        this.rssiMaxIterations = rssiMaxIterations;
    }

    /**
     * Indicates whether result is refined using all found inliers.
     *
     * @return true if result is refined, false otherwise.
     */
    public boolean isResultRefined() {
        return refineResult;
    }

    /**
     * Specifies whether result is refined using all found inliers.
     *
     * @param refineResult true if result is refined, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setResultRefined(final boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.refineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return keepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining result,
     *                       false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(final boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.keepCovariance = keepCovariance;
    }

    /**
     * Indicates whether a linear solver is used for preliminary solution estimation
     * using ranging measurements.
     * The result obtained on each preliminary solution might be later refined.
     *
     * @return true if a linear solver is used for preliminary solution estimation on
     * ranging readings.
     */
    public boolean isRangingLinearSolverUsed() {
        return useRangingLinearSolver;
    }

    /**
     * Specifies whether a linear solver is used for preliminary solution estimation
     * using ranging measurements.
     * The result obtained on each preliminary solution might be later refined.
     *
     * @param useRangingLinearSolver true if a linear solver is used for preliminary
     *                               solution estimation on ranging readings.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingLinearSolverUsed(final boolean useRangingLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.useRangingLinearSolver = useRangingLinearSolver;
    }

    /**
     * Indicates whether a linear solver is used for preliminary solution estimation
     * using RSSI measurements.
     * The result obtained on each preliminary solution might be later refined.
     *
     * @return true if a linear solver is used for preliminary solution estimation on
     * RSSI readings.
     */
    public boolean isRssiLinearSolverUsed() {
        return useRssiLinearSolver;
    }

    /**
     * Specifies whether a linear solver is used for preliminary solution estimation
     * using RSSI measurements.
     * The result obtained on each preliminary solution might be later refined.
     *
     * @param useRssiLinearSolver true if a linear solver is used for preliminary
     *                            solution estimation on RSSI readings.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiLinearSolverUsed(final boolean useRssiLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.useRssiLinearSolver = useRssiLinearSolver;
    }

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the ranging fine estimation.
     *
     * @return true to use an homogeneous linear solver for preliminary solutions
     * during ranging fine position estimation.
     */
    public boolean isRangingHomogeneousLinearSolverUsed() {
        return useRangingHomogeneousLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the ranging fine estimation.
     *
     * @param useRangingHomogeneousLinearSolver true to use an homogeneous linear
     *                                          solver for preliminary solutions during
     *                                          ranging fine position estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingHomogeneousLinearSolverUsed(final boolean useRangingHomogeneousLinearSolver)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.useRangingHomogeneousLinearSolver = useRangingHomogeneousLinearSolver;
    }

    /**
     * Indicates whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the RSSI coarse estimation.
     *
     * @return true to use an homogeneous linear solver for preliminary solutions
     * during RSSI coarse position estimation.
     */
    public boolean isRssiHomogeneousLinearSolverUsed() {
        return useRssiHomogeneousLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used either to estimate
     * preliminary solutions or an initial solution for preliminary solutions that
     * will be later refined on the RSSI coarse estimation.
     *
     * @param useRssiHomogeneousLinearSolver true to use an homogeneous linear solver
     *                                       for preliminary solutions during RSSI fine
     *                                       position estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiHomogeneousLinearSolverUsed(final boolean useRssiHomogeneousLinearSolver)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.useRssiHomogeneousLinearSolver = useRssiHomogeneousLinearSolver;
    }

    /**
     * Indicates whether preliminary ranging solutions are refined after an initial
     * linear solution is found.
     * If no initial preliminary solution is found using a linear solver, a non-linear
     * solver will be used regardless of this value using an average solution
     * as the initial value to be refined.
     *
     * @return true if preliminary ranging solutions must be refined after an initial
     * linear solution, false otherwise.
     */
    public boolean isRangingPreliminarySolutionRefined() {
        return refineRangingPreliminarySolutions;
    }

    /**
     * Specifies whether preliminary ranging solutions are refined after an initial
     * linear solution is found.
     * If no initial preliminary solution is found using a linear solver, a non-linear
     * solver will be used regardless of this value using an average solution
     * as the initial value to be refined.
     *
     * @param refineRangingPreliminarySolutions true if preliminary ranging solutions
     *                                          must be refined after an initial linear
     *                                          solution, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingPreliminarySolutionRefined(final boolean refineRangingPreliminarySolutions)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refineRangingPreliminarySolutions = refineRangingPreliminarySolutions;
    }

    /**
     * Indicates whether preliminary RSSI solutions are refined after an initial
     * linear solution is found.
     * If no initial preliminary solution is found using a linear solver, a non-linear
     * solver will be used regardless of this value using an average solution
     * as the initial value to be refined.
     *
     * @return true if preliminary RSSI solutions must be refined after an initial
     * linear solution, false otherwise.
     */
    public boolean isRssiPreliminarySolutionRefined() {
        return refineRssiPreliminarySolutions;
    }

    /**
     * Specifies whether preliminary RSSI solutions are refined after an initial
     * linear solution is found.
     * If no initial preliminary solution is found using a linear solver, a non-linear
     * solver will be used regardless of this value using an average solution
     * as the initial value to be refined.
     *
     * @param refineRssiPreliminarySolutions true if preliminary RSSI solutions must
     *                                       be refined after an initial linear
     *                                       solution, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiPreliminarySolutionRefined(final boolean refineRssiPreliminarySolutions) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refineRssiPreliminarySolutions = refineRssiPreliminarySolutions;
    }

    /**
     * Gets size of subsets to be checked during ranging robust estimation.
     *
     * @return size of subsets to be checked during ranging robust estimation.
     */
    public int getRangingPreliminarySubsetSize() {
        return rangingPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during ranging robust estimation.
     *
     * @param rangingPreliminarySubsetSize size of subsets to be checked during
     *                                     ranging robust estimation.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinRequiredSources()}.
     */
    public void setRangingPreliminarySubsetSize(final int rangingPreliminarySubsetSize) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rangingPreliminarySubsetSize < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        this.rangingPreliminarySubsetSize = rangingPreliminarySubsetSize;
    }

    /**
     * Gets size of subsets to be checked during RSSI robust estimation.
     *
     * @return size of subsets to be checked during RSSI robust estimation.
     */
    public int getRssiPreliminarySubsetSize() {
        return rssiPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during RSSI robust estimation.
     *
     * @param rssiPreliminarySubsetSize size of subsets to be checked during
     *                                  RSSI robust estimation.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinRequiredSources()}.
     */
    public void setRssiPreliminarySubsetSize(final int rssiPreliminarySubsetSize) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rssiPreliminarySubsetSize < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        this.rssiPreliminarySubsetSize = rssiPreliminarySubsetSize;
    }

    /**
     * Gets threshold to determine when samples are inliers or not, used during robust
     * fine ranging position estimation.
     * If not defined, default threshold will be used.
     *
     * @return threshold for ranging estimation or null.
     */
    public Double getRangingThreshold() {
        return rangingThreshold;
    }

    /**
     * Sets threshold to determine when samples are inliers or not, used during robust
     * fine ranging position estimation.
     * If not defined, default threshold will be used.
     *
     * @param rangingThreshold threshold for ranging estimation or null.
     * @throws LockedException if estimator is locked.
     */
    public void setRangingThreshold(final Double rangingThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.rangingThreshold = rangingThreshold;
    }

    /**
     * Gets threshold to determine when samples are inliers or not, used during robust
     * coarse RSSI position estimation.
     * If not defined, default threshold will be used.
     *
     * @return threshold for RSSI estimation or null.
     */
    public Double getRssiThreshold() {
        return rssiThreshold;
    }

    /**
     * Sets threshold to determine when samples are inliers or not, used during robust
     * coarse RSSI position estimation.
     * If not defined, default threshold will be used.
     *
     * @param rssiThreshold threshold for RSSI estimation or null.
     * @throws LockedException if estimator is locked.
     */
    public void setRssiThreshold(final Double rssiThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.rssiThreshold = rssiThreshold;
    }

    /**
     * Gets located radio sources used for lateration.
     *
     * @return located radio sources used for lateration.
     */
    public List<RadioSourceLocated<P>> getSources() {
        //noinspection unchecked
        return (List<RadioSourceLocated<P>>) sources;
    }

    /**
     * Sets located radio sources used for lateration.
     *
     * @param sources located radio sources used for lateration.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public void setSources(final List<? extends RadioSourceLocated<P>> sources) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetSources(sources);
    }

    /**
     * Gets fingerprint containing ranging+RSSI readings at an unknown location for
     * provided located radio sources.
     *
     * @return fingerprint containing readings at an unknown location for provided
     * located radio sources.
     */
    public RangingAndRssiFingerprint<RadioSource, RangingAndRssiReading<RadioSource>> getFingerprint() {
        //noinspection unchecked
        return (RangingAndRssiFingerprint<RadioSource, RangingAndRssiReading<RadioSource>>) fingerprint;
    }

    /**
     * Sets fingerprint containing ranging+RSSI readings at an unknown location for
     * provided located radio sources.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprint(
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetFingerprint(fingerprint);
    }

    /**
     * Returns quality scores corresponding to each radio source.
     * The larger the score value the better the quality of radio source.
     *
     * @return quality scores corresponding to each radio source.
     */
    public double[] getSourceQualityScores() {
        return sourceQualityScores;
    }

    /**
     * Sets quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the radio source.
     *
     * @param sourceQualityScores quality scores corresponding to each radio source.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     *                                  than minimum required samples.
     */
    public void setSourceQualityScores(final double[] sourceQualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSourceQualityScores(sourceQualityScores);
    }

    /**
     * Gets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     *
     * @return quality scores corresponding to each reading within provided
     * fingerprint.
     */
    public double[] getFingerprintReadingsQualityScores() {
        return fingerprintReadingsQualityScores;
    }

    /**
     * Sets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     *
     * @param fingerprintReadingsQualityScores quality scores corresponding to each
     *                                         reading within provided fingerprint.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     *                                  than minimum required samples.
     */
    public void setFingerprintReadingsQualityScores(final double[] fingerprintReadingsQualityScores)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     *
     * @return listener to be notified of events raised by this instance.
     */
    public SequentialRobustRangingAndRssiPositionEstimatorListener<P> getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     *
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(final SequentialRobustRangingAndRssiPositionEstimatorListener<P> listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Gets initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     *
     * @return an initial position.
     */
    public P getInitialPosition() {
        return initialPosition;
    }

    /**
     * Sets initial position to use as a starting point to find a new solution.
     * This is optional, but if provided, when no linear solvers are used, this is
     * taken into account. If linear solvers are used, this is ignored.
     *
     * @param initialPosition an initial position.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(final P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.initialPosition = initialPosition;
    }

    /**
     * Returns boolean indicating if estimator is locked because estimation is
     * under progress.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    public boolean isReady() {
        final var numSources = sources != null ? sources.size() : 0;
        final var numReadings = fingerprint != null && fingerprint.getReadings() != null
                ? fingerprint.getReadings().size() : 0;

        return numSources > getMinRequiredSources() && numReadings >= numSources;
    }

    /**
     * Estimates position based on provided located radio sources and readings of such
     * sources at an unknown location.
     *
     * @return estimated position.
     * @throws LockedException          if estimator is locked.
     * @throws NotReadyException        if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for some other reason.
     */
    public P estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }

        // create inner estimators
        buildEstimators();
        setupEstimators();

        if (!isReady() || !rssiEstimator.isReady() || !rangingEstimator.isReady()) {
            throw new NotReadyException();
        }

        locked = true;
        if (listener != null) {
            listener.onEstimateStart(this);
        }

        P coarsePosition;
        try {
            // estimate coarse position using RSSI data
            coarsePosition = rssiEstimator.estimate();
        } catch (final RobustEstimatorException e) {
            coarsePosition = null;
        }

        // use coarse position as initial position for ranging estimation
        if (coarsePosition != null) {
            rangingEstimator.setInitialPosition(coarsePosition);
        }

        try {
            final var result = rangingEstimator.estimate();

            if (listener != null) {
                listener.onEstimateEnd(this);
            }

            return result;
        } finally {
            locked = false;
        }
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return rangingEstimator != null ? rangingEstimator.getInliersData() : null;
    }

    /**
     * Gets known positions of radio sources used internally to solve lateration.
     *
     * @return known positions used internally.
     */
    public P[] getPositions() {
        return rangingEstimator != null ? rangingEstimator.getPositions() : null;
    }

    /**
     * Gets Euclidean distances from known located radio sources to the location of
     * provided readings in a fingerprint.
     * Distance values are used internally to solve lateration.
     *
     * @return Euclidean distances used internally.
     */
    public double[] getDistances() {
        return rangingEstimator != null ? rangingEstimator.getDistances() : null;
    }

    /**
     * Gets standard deviation distances from known located radio sources to the
     * location of provided readings in a fingerprint.
     * Distance standard deviations are used internally to solve lateration.
     *
     * @return standard deviations used internally.
     */
    public double[] getDistanceStandardDeviations() {
        return rangingEstimator != null ? rangingEstimator.getDistanceStandardDeviations() : null;
    }

    /**
     * Gets estimated covariance of estimated position if available.
     * This is only available when result has been refined and covariance is kept.
     *
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return rangingEstimator != null ? rangingEstimator.getCovariance() : null;
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public P getEstimatedPosition() {
        return rangingEstimator != null ? rangingEstimator.getEstimatedPosition() : null;
    }

    /**
     * Gets number of dimensions of provided points.
     *
     * @return number of dimensions of provided points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Gets minimum required number of located radio sources to perform lateration.
     *
     * @return minimum required number of located radio sources to perform
     * lateration.
     */
    public abstract int getMinRequiredSources();

    /**
     * Builds ranging and RSSI internal estimators.
     */
    protected abstract void buildEstimators();

    /**
     * Setups ranging and RSSI internal estimators.
     *
     * @throws LockedException if estimator is locked.
     */
    @SuppressWarnings("DuplicatedCode")
    private void setupEstimators() throws LockedException {
        if (fingerprint != null) {
            // builds separated RSSI and ranging readings
            final var readings = fingerprint.getReadings();

            final var rangingReadings = new ArrayList<RangingReading<RadioSource>>();
            final var rssiReadings = new ArrayList<RssiReading<RadioSource>>();

            for (final var reading : readings) {
                rangingReadings.add(createRangingReading(reading));
                rssiReadings.add(createRssiReading(reading));
            }

            final var rssiFingerprint = new RssiFingerprint<>(rssiReadings);
            final var rangingFingerprint = new RangingFingerprint<>(rangingReadings);

            // set data and configuration on both internal estimators
            rssiEstimator.setSources(sources);
            rssiEstimator.setFingerprint(rssiFingerprint);
            rssiEstimator.setRadioSourcePositionCovarianceUsed(useRssiRadioSourcePositionCovariance);
            rssiEstimator.setEvenlyDistributeReadings(evenlyDistributeRssiReadings);
            rssiEstimator.setFallbackDistanceStandardDeviation(rssiFallbackDistanceStandardDeviation);
            rssiEstimator.setProgressDelta(2.0f * progressDelta);
            rssiEstimator.setConfidence(rssiConfidence);
            rssiEstimator.setMaxIterations(rssiMaxIterations);
            rssiEstimator.setResultRefined(refineResult);
            rssiEstimator.setCovarianceKept(keepCovariance);
            rssiEstimator.setInitialPosition(initialPosition);
            rssiEstimator.setLinearSolverUsed(useRssiLinearSolver);
            rssiEstimator.setHomogeneousLinearSolverUsed(useRssiHomogeneousLinearSolver);
            rssiEstimator.setPreliminarySolutionRefined(refineRssiPreliminarySolutions);
            rssiEstimator.setSourceQualityScores(sourceQualityScores);
            rssiEstimator.setFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
            rssiEstimator.setListener(new RobustRssiPositionEstimatorListener<>() {
                @Override
                public void onEstimateStart(final RobustRssiPositionEstimator<P> estimator) {
                    // not used
                }

                @Override
                public void onEstimateEnd(final RobustRssiPositionEstimator<P> estimator) {
                    // not used
                }

                @Override
                public void onEstimateNextIteration(
                        final RobustRssiPositionEstimator<P> estimator, final int iteration) {
                    // not used
                }

                @Override
                public void onEstimateProgressChange(
                        final RobustRssiPositionEstimator<P> estimator, final float progress) {
                    if (listener != null) {
                        listener.onEstimateProgressChange(
                                SequentialRobustRangingAndRssiPositionEstimator.this,
                                0.5f * progress);
                    }
                }
            });

            rssiEstimator.setPreliminarySubsetSize(
                    Math.max(rssiPreliminarySubsetSize, rssiEstimator.getMinRequiredSources()));

            rangingEstimator.setSources(sources);
            rangingEstimator.setFingerprint(rangingFingerprint);
            rangingEstimator.setRadioSourcePositionCovarianceUsed(useRangingRadioSourcePositionCovariance);
            rangingEstimator.setEvenlyDistributeReadings(evenlyDistributeRangingReadings);
            rangingEstimator.setFallbackDistanceStandardDeviation(rangingFallbackDistanceStandardDeviation);
            rangingEstimator.setProgressDelta(2.0f * progressDelta);
            rangingEstimator.setConfidence(rangingConfidence);
            rangingEstimator.setMaxIterations(rangingMaxIterations);
            rangingEstimator.setCovarianceKept(keepCovariance);
            rangingEstimator.setInitialPosition(initialPosition);
            rangingEstimator.setLinearSolverUsed(useRangingLinearSolver);
            rangingEstimator.setHomogeneousLinearSolverUsed(useRangingHomogeneousLinearSolver);
            rangingEstimator.setPreliminarySolutionRefined(refineRangingPreliminarySolutions);
            rangingEstimator.setSourceQualityScores(sourceQualityScores);
            rangingEstimator.setFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
            rangingEstimator.setListener(new RobustRangingPositionEstimatorListener<P>() {
                @Override
                public void onEstimateStart(final RobustRangingPositionEstimator<P> estimator) {
                    // not used
                }

                @Override
                public void onEstimateEnd(final RobustRangingPositionEstimator<P> estimator) {
                    // not used
                }

                @Override
                public void onEstimateNextIteration(
                        final RobustRangingPositionEstimator<P> estimator, final int iteration) {
                    // not used
                }

                @Override
                public void onEstimateProgressChange(
                        final RobustRangingPositionEstimator<P> estimator, final float progress) {
                    if (listener != null) {
                        listener.onEstimateProgressChange(
                                SequentialRobustRangingAndRssiPositionEstimator.this,
                                0.5f + 0.5f * progress);
                    }
                }
            });

            rangingEstimator.setPreliminarySubsetSize(
                    Math.max(rangingPreliminarySubsetSize, rangingEstimator.getMinRequiredSources()));
        }
    }

    /**
     * Internally sets located radio sources used for lateration.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided value is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    private void internalSetSources(final List<? extends RadioSourceLocated<P>> sources) {
        if (sources == null) {
            throw new IllegalArgumentException();
        }

        if (sources.size() < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        this.sources = sources;
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for
     * provided located radio sources.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    private void internalSetFingerprint(
            final RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        if (fingerprint == null) {
            throw new IllegalArgumentException();
        }

        this.fingerprint = fingerprint;
    }

    /**
     * Sets quality scores corresponding to each provided located radio source.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param sourceQualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 3 samples for 2D or 4 samples for 3D.
     */
    private void internalSetSourceQualityScores(final double[] sourceQualityScores) {
        if (sourceQualityScores == null || sourceQualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        this.sourceQualityScores = sourceQualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided reading within provided
     * fingerprint.
     * This method is used internally and does not check whether instance is locked
     * or not.
     *
     * @param fingerprintReadingsQualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than 3 samples for 2D or 4 samples for 3D.
     */
    private void internalSetFingerprintReadingsQualityScores(final double[] fingerprintReadingsQualityScores) {
        if (fingerprintReadingsQualityScores == null
                || fingerprintReadingsQualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        this.fingerprintReadingsQualityScores = fingerprintReadingsQualityScores;
    }

    /**
     * Creates a ranging reading from a ranging and RSSI reading.
     *
     * @param reading input reading to convert from.
     * @return a ranging reading containing only the ranging data of input reading.
     */
    private RangingReading<RadioSource> createRangingReading(
            final RangingAndRssiReading<? extends RadioSource> reading) {
        return new RangingReading<>(reading.getSource(),
                reading.getDistance(),
                reading.getDistanceStandardDeviation(),
                reading.getNumAttemptedMeasurements(),
                reading.getNumSuccessfulMeasurements());
    }

    /**
     * Creates an RSSI reading from a ranging and RSSI reading.
     *
     * @param reading input reading to convert from.
     * @return an RSSI reading containing only the RSSI data of input reading.
     */
    private RssiReading<RadioSource> createRssiReading(final RangingAndRssiReading<? extends RadioSource> reading) {
        return new RssiReading<>(reading.getSource(), reading.getRssi(),
                reading.getRssiStandardDeviation());
    }
}
