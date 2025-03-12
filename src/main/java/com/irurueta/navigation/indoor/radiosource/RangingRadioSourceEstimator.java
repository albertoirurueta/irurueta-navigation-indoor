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

import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.Accuracy;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RangingReadingLocated;
import com.irurueta.navigation.lateration.HomogeneousLinearLeastSquaresLaterationSolver;
import com.irurueta.navigation.lateration.InhomogeneousLinearLeastSquaresLaterationSolver;
import com.irurueta.navigation.lateration.LaterationException;
import com.irurueta.navigation.lateration.NonLinearLeastSquaresLaterationSolver;

import java.util.ArrayList;
import java.util.List;

/**
 * Estimates position of a radio source (e.g. Wi-Fi access point or bluetooth beacon)
 * by using ranging measurements.
 * Ranging measurements can be obtained by protocols such as ieee 802.11mc (Wi-Fi RTT) which
 * measures travel time of signal and converts the result into distances by taking into
 * account the speed of light as the propagation speed.
 */
public abstract class RangingRadioSourceEstimator<S extends RadioSource, P extends Point<P>>
        extends RadioSourceEstimator<P, RangingReadingLocated<S, P>, RangingRadioSourceEstimatorListener<S, P>> {

    /**
     * Indicates that by default position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    public static final boolean DEFAULT_USE_READING_POSITION_COVARIANCES = true;

    /**
     * Indicates that by default an homogeneous linear solver is used to estimate an
     * initial position.
     */
    public static final boolean DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER = true;

    /**
     * Internal homogeneous linear solver to find radio source position when no initial
     * position is provided.
     */
    protected HomogeneousLinearLeastSquaresLaterationSolver<P> homogeneousLinearSolver;

    /**
     * Internal inhomogeneous linear solver to find radio source position when no initial
     * position is provided.
     */
    protected InhomogeneousLinearLeastSquaresLaterationSolver<P> inhomogeneousLinearSolver;

    /**
     * Internal non-linear solver to estimate radio source position and covariance
     * for an initial provided or estimated position.
     */
    protected NonLinearLeastSquaresLaterationSolver<P> nonLinearSolver;

    /**
     * Contains accuracy of a reading position.
     * This is used internally to compute additional distance standard deviation due to position
     * accuracy.
     */
    protected Accuracy accuracy;

    /**
     * Initial position to start the estimation of radio source position.
     */
    protected P initialPosition;

    /**
     * Indicates whether non-linear solver is enabled.
     * If disabled a linear solver is always used, initial position ignored and
     * covariance is not computed.
     */
    protected boolean nonLinearSolverEnabled = true;

    /**
     * Indicates whether an homogeneous linear solver is used to estimate an initial
     * position.
     */
    protected boolean useHomogeneousLinearSolver = DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    protected boolean useReadingPositionCovariances = DEFAULT_USE_READING_POSITION_COVARIANCES;

    /**
     * Constructor.
     */
    protected RangingRadioSourceEstimator() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param readings radio signal ranging readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RangingRadioSourceEstimator(final List<? extends RangingReadingLocated<S, P>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    protected RangingRadioSourceEstimator(final RangingRadioSourceEstimatorListener<S, P> listener) {
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
    protected RangingRadioSourceEstimator(
            final List<? extends RangingReadingLocated<S, P>> readings,
            final RangingRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    protected RangingRadioSourceEstimator(final P initialPosition) {
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
    protected RangingRadioSourceEstimator(
            final List<? extends RangingReadingLocated<S, P>> readings, final P initialPosition) {
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
    protected RangingRadioSourceEstimator(
            final P initialPosition, final RangingRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param readings        radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio source
     *                        position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RangingRadioSourceEstimator(
            final List<? extends RangingReadingLocated<S, P>> readings, final P initialPosition,
            final RangingRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        this.initialPosition = initialPosition;
    }

    /**
     * Gets initial position to start the non-linear estimation of radio source position.
     * If not defined, a linear solution is found instead.
     *
     * @return initial position.
     */
    public P getInitialPosition() {
        return initialPosition;
    }

    /**
     * Sets initial position to start the non-linear estimation of radio source position.
     * If not defined, a linear solution is found instead.
     *
     * @param initialPosition initial position to start the estimation of radio source
     *                        position or null.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(final P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.initialPosition = initialPosition;
    }

    /**
     * Indicates whether non-linear solver is enabled.
     * If disabled a linear solver is always used, initial position ignored and
     * covariance is not computed.
     *
     * @return true if non-linear solver is enabled, false otherwise.
     */
    public boolean isNonLinearSolverEnabled() {
        return nonLinearSolverEnabled;
    }

    /**
     * Specifies whether non-linear solver is enabled.
     * If disabled a linear solver is always used, initial position ignored and
     * covariance is not computed.
     *
     * @param nonLinearSolverEnabled true if non-linear solver is enabled,
     *                               false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setNonLinearSolverEnabled(final boolean nonLinearSolverEnabled) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.nonLinearSolverEnabled = nonLinearSolverEnabled;
    }

    /**
     * Indicates whether an homogeneous linear solver is used to estimate an initial
     * position.
     *
     * @return true if homogeneous linear solver is used, false if an inhomogeneous linear
     * one is used instead.
     */
    public boolean isHomogeneousLinearSolverUsed() {
        return useHomogeneousLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used to estimate an initial
     * position.
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
     * Indicates whether this instance is ready to start the estimation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        // readings must be valid
        return areValidReadings(readings);
    }

    /**
     * Estimate position of radio source.
     *
     * @throws RadioSourceEstimationException if estimation fails.
     * @throws NotReadyException              if estimator is not ready.
     * @throws LockedException                if estimator is locked.
     */
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

            buildSolversIfNeeded();
            buildPositionsDistancesAndDistanceStandardDeviations();

            if (((useHomogeneousLinearSolver && homogeneousLinearSolver != null)
                    || (!useHomogeneousLinearSolver && inhomogeneousLinearSolver != null))
                    && (initialPosition == null || !nonLinearSolverEnabled)) {
                // if no initial position is provided, use linear solver to estimate one
                if (useHomogeneousLinearSolver) {
                    homogeneousLinearSolver.solve();
                    initialPosition = homogeneousLinearSolver.getEstimatedPosition();
                } else {
                    inhomogeneousLinearSolver.solve();
                    initialPosition = inhomogeneousLinearSolver.getEstimatedPosition();
                }
            }

            if (nonLinearSolver != null && nonLinearSolverEnabled) {
                nonLinearSolver.setInitialPosition(initialPosition);
                nonLinearSolver.solve();

                // get position and covariance
                estimatedPositionCoordinates = nonLinearSolver.getEstimatedPositionCoordinates();
                estimatedPositionCovariance = estimatedCovariance = nonLinearSolver.getCovariance();
            } else {
                // non-linear solver disabled
                if (useHomogeneousLinearSolver) {
                    estimatedPositionCoordinates = homogeneousLinearSolver != null
                            ? homogeneousLinearSolver.getEstimatedPositionCoordinates() : null;
                } else {
                    estimatedPositionCoordinates = inhomogeneousLinearSolver != null
                            ? inhomogeneousLinearSolver.getEstimatedPositionCoordinates() : null;
                }
                estimatedPositionCovariance = estimatedCovariance = null;
            }

            if (listener != null) {
                listener.onEstimateEnd(this);
            }

        } catch (final LaterationException e) {
            throw new RadioSourceEstimationException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Builds an instance of a linear lateration solver if needed.
     */
    protected abstract void buildLinearSolverIfNeeded();

    /**
     * Builds an instance of a non-linear lateration solver if needed.
     */
    protected abstract void buildNonLinearSolverIfNeeded();

    /**
     * Build an instance of accuracy if needed.
     */
    protected abstract void buildAccuracyIfNeeded();

    /**
     * Sets positions, distances and standard deviations of distances on internal
     * lateration solver.
     *
     * @param positions                  positions to be set.
     * @param distances                  distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set or
     *                                   null.
     * @throws LockedException if solvers are locked.
     */
    protected abstract void setPositionsDistancesAndDistanceStandardDeviations(
            final List<P> positions, List<Double> distances, final List<Double> distanceStandardDeviations)
            throws LockedException;

    /**
     * Build instances of lateration solvers if needed.
     */
    private void buildSolversIfNeeded() {
        buildLinearSolverIfNeeded();
        buildNonLinearSolverIfNeeded();
        buildAccuracyIfNeeded();
    }

    /**
     * Builds positions, distances and standard deviations of distances for the
     * internal lateration solver.
     *
     * @throws LockedException if solvers are locked.
     */
    private void buildPositionsDistancesAndDistanceStandardDeviations() throws LockedException {
        final var min = getMinReadings();
        if (readings == null || readings.size() < min) {
            return;
        }

        final var positions = new ArrayList<P>();
        final var distances = new ArrayList<Double>();
        final var distanceStandardDeviations = new ArrayList<Double>();

        for (final var reading : readings) {
            final var position = reading.getPosition();
            if (position == null) {
                return;
            }

            var positionDistanceStandardDeviation = 0.0;
            if (useReadingPositionCovariances && accuracy != null && reading.getPositionCovariance() != null) {
                try {
                    accuracy.setCovarianceMatrix(reading.getPositionCovariance());
                    positionDistanceStandardDeviation = accuracy.getAverageAccuracy();
                } catch (final NonSymmetricPositiveDefiniteMatrixException e) {
                    positionDistanceStandardDeviation = 0.0;
                }
            }

            final var distance = reading.getDistance();
            var distanceStandardDeviation = reading.getDistanceStandardDeviation();
            if (distanceStandardDeviation == null) {
                distanceStandardDeviation = NonLinearLeastSquaresLaterationSolver.DEFAULT_DISTANCE_STANDARD_DEVIATION;
            }

            if (useReadingPositionCovariances) {
                // assuming that ranging measure and position measure are statistically independent, the
                // resulting variance would be the sum of their variances, when the resulting standard
                // deviation is the square root of the resulting variance (which is the sum of the square of
                // the standard deviations)
                distanceStandardDeviation = Math.sqrt(Math.pow(distanceStandardDeviation, 2.0)
                        + Math.pow(positionDistanceStandardDeviation, 2.0));
            }

            positions.add(position);
            distances.add(distance);
            distanceStandardDeviations.add(distanceStandardDeviation);
        }

        setPositionsDistancesAndDistanceStandardDeviations(positions, distances, distanceStandardDeviations);
    }
}
