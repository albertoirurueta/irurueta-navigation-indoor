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
import com.irurueta.navigation.lateration.PROSACRobustLateration2DSolver;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates 2D position using located radio sources and their readings at
 * unknown locations and using PROSAC algorithm to discard outliers.
 * This kind of estimator can be used to robustly determine the 2D position of a given
 * device by getting readings at an unknown location of different radio
 * sources whose 2D locations are known.
 */
public class PROSACRobustMixedPositionEstimator2D extends RobustMixedPositionEstimator2D {

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
     * Constructor.
     */
    public PROSACRobustMixedPositionEstimator2D() {
        super();
        init();
    }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public PROSACRobustMixedPositionEstimator2D(final List<? extends RadioSourceLocated<Point2D>> sources) {
        super();
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than the required minimum.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super();
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public PROSACRobustMixedPositionEstimator2D(final RobustMixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param sources  located radio sources used for lateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided location radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less than the required minimum.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        super(listener);
        init();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
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
    public PROSACRobustMixedPositionEstimator2D(
            double[] sourceQualityScores, final double[] fingerprintReadingQualityScores) {
        this();
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
     *                                  provided sources is less than the required minimum.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources) {
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
     * @param fingerprint                     fingerprint containing ranging+RSSI readings
     *                                        at an unknown location for provided located
     *                                        radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
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
    public PROSACRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
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
    public PROSACRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        this(listener);
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
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        this(sources, listener);
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
     *                                        provided location radio sources.
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        this(fingerprint, listener);
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
     * @param listener                        listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less than the required minimum.
     */
    public PROSACRobustMixedPositionEstimator2D(
            final double[] sourceQualityScores, final double[] fingerprintReadingQualityScores,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final RobustMixedPositionEstimatorListener<Point2D> listener) {
        this(sources, fingerprint, listener);
        internalSetSourceQualityScores(sourceQualityScores);
        internalSetFingerprintReadingsQualityScores(fingerprintReadingQualityScores);
    }

    /**
     * Returns quality scores corresponding to each radio source.
     * The larger the score value the better the quality of the sample.
     *
     * @return quality scores corresponding to each radio source.
     */
    @Override
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
    @Override
    public void setSourceQualityScores(final double[] sourceQualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSourceQualityScores(sourceQualityScores);
    }

    /**
     * Gets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @return quality scores corresponding to each reading within provided
     * fingerprint.
     */
    @Override
    public double[] getFingerprintReadingsQualityScores() {
        return fingerprintReadingsQualityScores;
    }

    /**
     * Sets quality scores corresponding to each reading within provided fingerprint.
     * The larger the score value the better the quality of the reading.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @param fingerprintReadingsQualityScores quality scores corresponding to each
     *                                         reading within provided fingerprint.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided quality scores length is smaller
     *                                  than minimum required samples.
     */
    @Override
    public void setFingerprintReadingsQualityScores(final double[] fingerprintReadingsQualityScores)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetFingerprintReadingsQualityScores(fingerprintReadingsQualityScores);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return ((PROSACRobustLateration2DSolver) laterationSolver).getThreshold();
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     *
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException          if this solver is locked.
     */
    public void setThreshold(final double threshold) throws LockedException {
        ((PROSACRobustLateration2DSolver) laterationSolver).setThreshold(threshold);
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return ((PROSACRobustLateration2DSolver) laterationSolver).isComputeAndKeepInliersEnabled();
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if this solver is locked.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers) throws LockedException {
        ((PROSACRobustLateration2DSolver) laterationSolver).setComputeAndKeepInliersEnabled(computeAndKeepInliers);
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResiduals() {
        return ((PROSACRobustLateration2DSolver) laterationSolver).isComputeAndKeepResiduals();
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if this solver is locked.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals) throws LockedException {
        ((PROSACRobustLateration2DSolver) laterationSolver).setComputeAndKeepResidualsEnabled(computeAndKeepResiduals);
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Initializes robust lateration solver.
     */
    private void init() {
        laterationSolver = new PROSACRobustLateration2DSolver(trilaterationSolverListener);
    }

    /**
     * Sets quality scores corresponding to each provided located radio source.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param sourceQualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 3 samples.
     */
    private void internalSetSourceQualityScores(final double[] sourceQualityScores) {
        if (sourceQualityScores == null || sourceQualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        this.sourceQualityScores = sourceQualityScores;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }

    /**
     * Sets quality scores corresponding to each provided reading within provided
     * fingerprint.
     * This method is used internally and does not check whether instance is locked
     * or not.
     *
     * @param fingerprintReadingsQualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than 3 samples.
     */
    private void internalSetFingerprintReadingsQualityScores(final double[] fingerprintReadingsQualityScores) {
        if (fingerprintReadingsQualityScores == null || fingerprintReadingsQualityScores.length < getMinRequiredSources()) {
            throw new IllegalArgumentException();
        }

        this.fingerprintReadingsQualityScores = fingerprintReadingsQualityScores;

        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();
    }
}
