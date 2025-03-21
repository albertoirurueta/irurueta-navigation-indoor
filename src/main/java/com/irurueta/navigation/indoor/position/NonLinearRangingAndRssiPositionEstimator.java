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
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RangingAndRssiReading;
import com.irurueta.navigation.lateration.LaterationException;
import com.irurueta.navigation.lateration.LaterationSolver;
import com.irurueta.navigation.lateration.LaterationSolverListener;
import com.irurueta.navigation.lateration.NonLinearLeastSquaresLaterationSolver;

import java.util.ArrayList;
import java.util.List;

/**
 * Estimates position non-linearly using located radio sources and their ranging+RSSI
 * readings at unknown locations.
 * These kind of estimators can be used to determine the position of a given device by
 * getting ranging+RSSI readings at an unknown location of different radio sources whose
 * locations are known.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class NonLinearRangingAndRssiPositionEstimator<P extends Point<?>> extends
        RangingAndRssiPositionEstimator<P> {

    /**
     * Distance standard deviation assumed for provided distances as a fallback when
     * none can be determined.
     */
    public static final double FALLBACK_DISTANCE_STANDARD_DEVIATION =
            NonLinearLeastSquaresLaterationSolver.DEFAULT_DISTANCE_STANDARD_DEVIATION;

    /**
     * A non-linear lateration solver to solve position.
     */
    protected NonLinearLeastSquaresLaterationSolver<P> trilaterationSolver;

    /**
     * Listener for the lateration solver.
     */
    protected LaterationSolverListener<P> laterationSolverListener;

    /**
     * Initial position to start position estimation.
     * If not defined, centroid of provided located sources will be used.
     */
    private P initialPosition;

    /**
     * Indicates whether located radio source position covariance must be taken
     * into account (if available) to determine distance standard deviation.
     */
    private boolean useRadioSourcePositionCovariance;

    /**
     * Distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     */
    private double fallbackDistanceStandardDeviation = FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Constructor.
     */
    protected NonLinearRangingAndRssiPositionEstimator() {
        super();
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    protected NonLinearRangingAndRssiPositionEstimator(final RangingAndRssiPositionEstimatorListener<P> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start position estimation.
     */
    protected NonLinearRangingAndRssiPositionEstimator(final P initialPosition) {
        this();
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start position estimation.
     * @param listener        listener in charge of handling events.
     */
    protected NonLinearRangingAndRssiPositionEstimator(
            final P initialPosition, final RangingAndRssiPositionEstimatorListener<P> listener) {
        this(listener);
        this.initialPosition = initialPosition;
    }

    /**
     * Gets initial position to start position estimation.
     * If not defined, centroid of located sources position will be used to start
     * the estimation.
     *
     * @return initial position to start position estimation.
     */
    public P getInitialPosition() {
        return initialPosition;
    }

    /**
     * Sets initial position to start position estimation.
     * If not defined, centroid of located sources position will be used to start
     * the estimation.
     *
     * @param initialPosition initial position to start position estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(final P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.initialPosition = initialPosition;
    }

    /**
     * Indicates whether located radio source position covariance must be taken into
     * account (if available) to determine distance standard deviation.
     *
     * @return true to take radio source position covariance into account, false
     * otherwise.
     */
    public boolean isRadioSourcePositionCovarianceUsed() {
        return useRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether located radio source position covariance must be taken into
     * account (if available) to determine distance standard deviation.
     *
     * @param useRadioSourcePositionCovariance true to take radio source position
     *                                         covariance into account, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRadioSourcePositionCovarianceUsed(final boolean useRadioSourcePositionCovariance)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.useRadioSourcePositionCovariance = useRadioSourcePositionCovariance;
    }

    /**
     * Gets distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     *
     * @return distance standard deviation to use as fallback.
     */
    public double getFallbackDistanceStandardDeviation() {
        return fallbackDistanceStandardDeviation;
    }

    /**
     * Sets distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     *
     * @param fallbackDistanceStandardDeviation distance standard deviation to use
     *                                          as fallback.
     * @throws LockedException if estimator is locked.
     */
    public void setFallbackDistanceStandardDeviation(final double fallbackDistanceStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.fallbackDistanceStandardDeviation = fallbackDistanceStandardDeviation;
    }

    /**
     * Gets minimum required number of located radio sources to perform lateration.
     *
     * @return minimum required number of located radio sources to perform lateration.
     */
    @Override
    public int getMinRequiredSources() {
        return trilaterationSolver.getMinRequiredPositionsAndDistances();
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return trilaterationSolver.isReady();
    }

    /**
     * Returns boolean indicating whether this estimator is locked because an estimation
     * is already in progress.
     *
     * @return true if estimator is locked, false otherwise.
     */
    @Override
    public boolean isLocked() {
        return trilaterationSolver.isLocked();
    }

    /**
     * Gets standard deviations of distances from known located radio sources to the
     * location of provided readings in a fingerprint.
     * Distance standard deviations are used internally to solve lateration.
     *
     * @return standard deviations used internally.
     */
    public double[] getDistanceStandardDeviations() {
        return trilaterationSolver.getDistanceStandardDeviations();
    }

    /**
     * Estimates position based on provided located radio sources and readings of such
     * radio sources at an unknown location.
     *
     * @throws LockedException             if estimator is locked.
     * @throws NotReadyException           if estimator is not ready.
     * @throws PositionEstimationException if estimation fails for some other reason.
     */
    @Override
    public void estimate() throws LockedException, NotReadyException, PositionEstimationException {
        try {
            trilaterationSolver.setInitialPosition(initialPosition);

            trilaterationSolver.solve();
            estimatedPositionCoordinates = trilaterationSolver.getEstimatedPositionCoordinates();
        } catch (final LaterationException e) {
            throw new PositionEstimationException(e);
        }
    }

    /**
     * Gets known positions of radio sources used internally to solve lateration.
     *
     * @return known positions used internally.
     */
    @Override
    public P[] getPositions() {
        return trilaterationSolver.getPositions();
    }

    /**
     * Gets Euclidean distances from known located radio sources to the location of
     * provided readings in a fingerprint.
     * Distance values are used internally to solve lateration.
     *
     * @return Euclidean distances used internally.
     */
    @Override
    public double[] getDistances() {
        return trilaterationSolver.getDistances();
    }

    /**
     * Gets estimated covariance matrix for estimated position.
     *
     * @return estimated covariance matrix for estimated position.
     */
    public Matrix getCovariance() {
        return trilaterationSolver.getCovariance();
    }

    /**
     * Internally sets located radio sources used for lateration.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided value is null or the number of
     *                                  provided sources is less than the required minimum.
     */
    @Override
    protected void internalSetSources(final List<? extends RadioSourceLocated<P>> sources) {
        super.internalSetSources(sources);
        buildPositionsDistancesAndDistanceStandardDeviations();
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for provided
     * located radio sources.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    @Override
    protected void internalSetFingerprint(
            final Fingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                    extends RadioSource>> fingerprint) {
        super.internalSetFingerprint(fingerprint);
        buildPositionsDistancesAndDistanceStandardDeviations();
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal
     * lateration solver.
     *
     * @param positions                  positions to be set.
     * @param distances                  distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set.
     */
    protected abstract void setPositionsDistancesAndDistanceStandardDeviations(
            final List<P> positions, List<Double> distances, final List<Double> distanceStandardDeviations);

    /**
     * Initializes lateration solver listener.
     */
    @SuppressWarnings("Duplicates")
    private void init() {
        laterationSolverListener = new LaterationSolverListener<P>() {
            @Override
            public void onSolveStart(final LaterationSolver<P> solver) {
                if (listener != null) {
                    listener.onEstimateStart(NonLinearRangingAndRssiPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(final LaterationSolver<P> solver) {
                if (listener != null) {
                    listener.onEstimateEnd(NonLinearRangingAndRssiPositionEstimator.this);
                }
            }
        };
    }

    /**
     * Builds positions, distances and standard deviation of distances for the internal
     * lateration solver.
     */
    @SuppressWarnings("Duplicates")
    private void buildPositionsDistancesAndDistanceStandardDeviations() {
        if (trilaterationSolver == null) {
            return;
        }

        final int min = getMinRequiredSources();
        if (sources == null || fingerprint == null || sources.size() < min || fingerprint.getReadings() == null
                || fingerprint.getReadings().size() < min) {
            return;
        }

        final var positions = new ArrayList<P>();
        final var distances = new ArrayList<Double>();
        final var distanceStandardDeviations = new ArrayList<Double>();
        PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                sources, fingerprint, useRadioSourcePositionCovariance, fallbackDistanceStandardDeviation, positions,
                distances, distanceStandardDeviations);

        setPositionsDistancesAndDistanceStandardDeviations(positions, distances, distanceStandardDeviations);
    }
}
