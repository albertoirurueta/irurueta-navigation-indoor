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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.lateration.NonLinearLeastSquaresLateration3DSolver;

import java.util.List;

/**
 * Estimates 3D position non-linearly using located radio sources and their ranging
 * readings at unknown locations.
 * These kind of estimators can be used to determine the position of a given device by
 * getting ranging readings at an unknown location of different radio sources whose
 * locations are known.
 */
public class NonLinearRangingPositionEstimator3D extends
        NonLinearRangingPositionEstimator<Point3D> {

    /**
     * Constructor.
     */
    public NonLinearRangingPositionEstimator3D() {
        super();
        initialize();
    }

    /**
     * Constructor
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public NonLinearRangingPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources) {
        super();
        initialize();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing RSSI readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public NonLinearRangingPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint) {
        super();
        initialize();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public NonLinearRangingPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint) {
        super();
        initialize();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public NonLinearRangingPositionEstimator3D(
            final RangingPositionEstimatorListener<Point3D> listener) {
        super(listener);
        initialize();
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
    public NonLinearRangingPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final RangingPositionEstimatorListener<Point3D> listener) {
        super(listener);
        initialize();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public NonLinearRangingPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint,
            final RangingPositionEstimatorListener<Point3D> listener) {
        super(listener);
        initialize();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public NonLinearRangingPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint,
            final RangingPositionEstimatorListener<Point3D> listener) {
        super(listener);
        initialize();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start position estimation.
     */
    public NonLinearRangingPositionEstimator3D(final Point3D initialPosition) {
        super(initialPosition);
        initialize();
    }

    /**
     * Constructor.
     *
     * @param sources         located radio sources used for lateration.
     * @param initialPosition initial position to start position estimation.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public NonLinearRangingPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Point3D initialPosition) {
        super(initialPosition);
        initialize();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint     fingerprint containing ranging readings at an unknown
     *                        location for provided located radio sources.
     * @param initialPosition initial position to start position estimation.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public NonLinearRangingPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint,
            final Point3D initialPosition) {
        super(initialPosition);
        initialize();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources         located radio sources used for lateration.
     * @param fingerprint     fingerprint containing readings at an unknown location
     *                        for provided located radio sources.
     * @param initialPosition initial position to start position estimation
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public NonLinearRangingPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint,
            final Point3D initialPosition) {
        super(initialPosition);
        initialize();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start position estimation.
     * @param listener        listener in charge of handling events.
     */
    public NonLinearRangingPositionEstimator3D(
            final Point3D initialPosition,
            final RangingPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, listener);
        initialize();
    }

    /**
     * Constructor.
     *
     * @param sources         located radio sources used for lateration.
     * @param initialPosition initial position to start position estimation.
     * @param listener        listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public NonLinearRangingPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Point3D initialPosition,
            final RangingPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, listener);
        initialize();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint     fingerprint containing readings at an unknown location
     *                        for provided located radio sources.
     * @param initialPosition initial position to start position estimation.
     * @param listener        listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public NonLinearRangingPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint,
            final Point3D initialPosition,
            final RangingPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, listener);
        initialize();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources         located radio sources used for lateration.
     * @param fingerprint     fingerprint containing readings at an unknown location
     *                        for provided located radio sources.
     * @param initialPosition initial position to start position estimation.
     * @param listener        listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public NonLinearRangingPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends RangingReading<? extends RadioSource>> fingerprint,
            final Point3D initialPosition,
            final RangingPositionEstimatorListener<Point3D> listener) {
        super(initialPosition, listener);
        initialize();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public Point3D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        final InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal
     * lateration solver.
     *
     * @param positions                  positions to be set.
     * @param distances                  distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set.
     */
    @Override
    @SuppressWarnings("Duplicates")
    protected void setPositionsDistancesAndDistanceStandardDeviations(
            final List<Point3D> positions, final List<Double> distances,
            final List<Double> distanceStandardDeviations) {

        final int size = positions.size();
        Point3D[] positionsArray = new InhomogeneousPoint3D[size];
        positionsArray = positions.toArray(positionsArray);

        final double[] distancesArray = new double[size];
        final double[] distanceStandardDeviationsArray = new double[size];
        for (int i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
            distanceStandardDeviationsArray[i] = distanceStandardDeviations.get(i);
        }

        try {
            mTrilaterationSolver.setPositionsDistancesAndStandardDeviations(
                    positionsArray, distancesArray, distanceStandardDeviationsArray);
        } catch (final LockedException e) {
            throw new IllegalArgumentException(e);
        }
    }

    /**
     * Initializes lateration solver.
     */
    private void initialize() {
        mTrilaterationSolver = new NonLinearLeastSquaresLateration3DSolver(
                mLaterationSolverListener);
    }
}