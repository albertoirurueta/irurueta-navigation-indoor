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

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RangingAndRssiReading;
import com.irurueta.navigation.lateration.HomogeneousLinearLeastSquaresLaterationSolver;
import com.irurueta.navigation.lateration.InhomogeneousLinearLeastSquaresLaterationSolver;
import com.irurueta.navigation.lateration.LaterationException;
import com.irurueta.navigation.lateration.LaterationSolver;
import com.irurueta.navigation.lateration.LaterationSolverListener;

import java.util.ArrayList;
import java.util.List;

/**
 * Linearly estimates position using located radio sources and their RSSI+ranging readings
 * at unknown locations.
 * These kind of estimators can be used to determine the position of a given device by
 * getting RSSI+ranging readings at an unknown location of different radio sources whose
 * locations are known.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class LinearRangingAndRssiPositionEstimator<P extends Point<P>> extends
        RangingAndRssiPositionEstimator<P> {

    /**
     * Indicates that by default an homogeneous linear solver is used to estimate
     * position.
     */
    public static final boolean DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER = true;

    /**
     * An homogeneous linear lateration solver to solve position.
     */
    protected HomogeneousLinearLeastSquaresLaterationSolver<P> homogeneousTrilaterationSolver;

    /**
     * An inhomogeneous linear lateration solver to solve position.
     */
    protected InhomogeneousLinearLeastSquaresLaterationSolver<P> inhomogeneousTrilaterationSolver;

    /**
     * Listener for the lateration solver.
     */
    protected LaterationSolverListener<P> laterationSolverListener;

    /**
     * Indicates whether an homogeneous linear solver is used to estimate position.
     */
    protected boolean useHomogeneousLinearSolver = DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Constructor.
     */
    protected LinearRangingAndRssiPositionEstimator() {
        super();
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    protected LinearRangingAndRssiPositionEstimator(final RangingAndRssiPositionEstimatorListener<P> listener) {
        super(listener);
        init();
    }

    /**
     * Indicates whether an homogeneous linear solver is used to estimate position.
     *
     * @return true if homogeneous linear solver is used, false if an inhomogeneous
     * linear one is used instead.
     */
    public boolean isHomogeneousLinearSolverUsed() {
        return useHomogeneousLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used to estimate position.
     *
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used, false
     *                                   if an inhomogeneous linear one is used instead.
     * @throws LockedException if estimator is locked.
     */
    public void setHomogeneousLinearSolverUsed(final boolean useHomogeneousLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.useHomogeneousLinearSolver = useHomogeneousLinearSolver;
    }

    /**
     * Gets minimum required number of located radio sources to perform lateration.
     *
     * @return minimum required number of located radio sources to perform lateration.
     */
    @Override
    public int getMinRequiredSources() {
        return useHomogeneousLinearSolver ? homogeneousTrilaterationSolver.getMinRequiredPositionsAndDistances()
                : inhomogeneousTrilaterationSolver.getMinRequiredPositionsAndDistances();
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return (!useHomogeneousLinearSolver && inhomogeneousTrilaterationSolver.isReady())
                || (useHomogeneousLinearSolver && homogeneousTrilaterationSolver.isReady());
    }

    /**
     * Returns boolean indicating whether this estimator is locked because an estimation is already in progress.
     *
     * @return true if estimator is locked, false otherwise.
     */
    @Override
    public boolean isLocked() {
        return inhomogeneousTrilaterationSolver.isLocked() || homogeneousTrilaterationSolver.isLocked();
    }

    /**
     * Estimates position based on provided located radio sources and RSSI readings of
     * such radio sources at an unknown location.
     *
     * @throws LockedException             if estimator is locked.
     * @throws NotReadyException           if estimator is not ready.
     * @throws PositionEstimationException if estimation fails for some other reason.
     */
    @SuppressWarnings("Duplicates")
    @Override
    public void estimate() throws LockedException, NotReadyException, PositionEstimationException {
        try {
            if (useHomogeneousLinearSolver) {
                homogeneousTrilaterationSolver.solve();
                estimatedPositionCoordinates = homogeneousTrilaterationSolver.getEstimatedPositionCoordinates();
            } else {
                inhomogeneousTrilaterationSolver.solve();
                estimatedPositionCoordinates = inhomogeneousTrilaterationSolver.getEstimatedPositionCoordinates();
            }
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
        return useHomogeneousLinearSolver ? homogeneousTrilaterationSolver.getPositions()
                : inhomogeneousTrilaterationSolver.getPositions();
    }

    /**
     * Gets Euclidean distances from known located radio sources to
     * the location of provided readings in a fingerprint.
     * Distance values are used internally to solve lateration.
     *
     * @return Euclidean distances used internally.
     */
    @Override
    public double[] getDistances() {
        return useHomogeneousLinearSolver ? homogeneousTrilaterationSolver.getDistances()
                : inhomogeneousTrilaterationSolver.getDistances();
    }

    /**
     * Internally sets located radio sources used for lateration.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided value is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    @Override
    protected void internalSetSources(final List<? extends RadioSourceLocated<P>> sources) {
        super.internalSetSources(sources);
        buildPositionsAndDistances();
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
            final Fingerprint<? extends RadioSource, ? extends RangingAndRssiReading<? extends RadioSource>> fingerprint) {
        super.internalSetFingerprint(fingerprint);
        buildPositionsAndDistances();
    }

    /**
     * Sets positions and distances on internal lateration solver.
     *
     * @param positions positions to be set.
     * @param distances distances to be set.
     */
    protected abstract void setPositionsAndDistances(final List<P> positions, final List<Double> distances);

    /**
     * Initializes lateration solver listener.
     */
    @SuppressWarnings("Duplicates")
    private void init() {
        laterationSolverListener = new LaterationSolverListener<>() {
            @Override
            public void onSolveStart(final LaterationSolver<P> solver) {
                if (listener != null) {
                    listener.onEstimateStart(LinearRangingAndRssiPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(final LaterationSolver<P> solver) {
                if (listener != null) {
                    listener.onEstimateEnd(LinearRangingAndRssiPositionEstimator.this);
                }
            }
        };
    }

    /**
     * Builds positions and distances for the internal lateration solver.
     */
    @SuppressWarnings("Duplicates")
    private void buildPositionsAndDistances() {
        if ((useHomogeneousLinearSolver && homogeneousTrilaterationSolver == null)
                || (!useHomogeneousLinearSolver && inhomogeneousTrilaterationSolver == null)) {
            return;
        }

        final int min = getMinRequiredSources();
        if (sources == null || fingerprint == null || sources.size() < min || fingerprint.getReadings() == null
                || fingerprint.getReadings().size() < min) {
            return;
        }

        final var positions = new ArrayList<P>();
        final var distances = new ArrayList<Double>();
        PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint, positions, distances);

        setPositionsAndDistances(positions, distances);
    }
}
