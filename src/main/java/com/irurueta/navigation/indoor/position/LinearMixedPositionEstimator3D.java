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
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.lateration.HomogeneousLinearLeastSquaresLateration3DSolver;
import com.irurueta.navigation.lateration.InhomogeneousLinearLeastSquaresLateration3DSolver;

import java.util.List;

/**
 * Linearly estimates 3D position using located radio sources and their readings at
 * unknown locations.
 * These kind of estimators can be used to determine the 3D position of a given device by
 * getting ranging readings at an unknown location of different radio sources whose
 * locations are known.
 */
public class LinearMixedPositionEstimator3D extends LinearMixedPositionEstimator<Point3D> {

    /**
     * Constructor.
     */
    public LinearMixedPositionEstimator3D() {
        super();
        initialize();
    }

    /**
     * Constructor.
     *
     * @param sources located radio sources used for lateration.
     * @throws IllegalArgumentException if provided sources is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    public LinearMixedPositionEstimator3D(final List<? extends RadioSourceLocated<Point3D>> sources) {
        super();
        initialize();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing ranging readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public LinearMixedPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
        super();
        initialize();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging readings at an unknown location
     *                    for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null
     *                                  or the number of provided sources is less than
     *                                  the required minimum.
     */
    public LinearMixedPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint) {
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
    public LinearMixedPositionEstimator3D(final MixedPositionEstimatorListener<Point3D> listener) {
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
    public LinearMixedPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final MixedPositionEstimatorListener<Point3D> listener) {
        super(listener);
        initialize();
        internalSetSources(sources);
    }

    /**
     * Constructor.
     *
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided location radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public LinearMixedPositionEstimator3D(
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final MixedPositionEstimatorListener<Point3D> listener) {
        super(listener);
        initialize();
        internalSetFingerprint(fingerprint);
    }

    /**
     * Constructor.
     *
     * @param sources     located radio sources used for lateration.
     * @param fingerprint fingerprint containing ranging readings at an unknown
     *                    location for provided located radio sources.
     * @param listener    listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is
     *                                  null or the number of provided sources is less
     *                                  than the required minimum.
     */
    public LinearMixedPositionEstimator3D(
            final List<? extends RadioSourceLocated<Point3D>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final MixedPositionEstimatorListener<Point3D> listener) {
        super(listener);
        initialize();
        internalSetSources(sources);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    @Override
    public Point3D getEstimatedPosition() {
        if (estimatedPositionCoordinates == null) {
            return null;
        }

        final var result = new InhomogeneousPoint3D();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Sets positions and distances on internal lateration solver.
     *
     * @param positions positions to be set.
     * @param distances distances to be set.
     * @throws IllegalArgumentException if something fails.
     */
    @Override
    @SuppressWarnings("Duplicates")
    protected void setPositionsAndDistances(
            final List<Point3D> positions, final List<Double> distances) {
        final var size = positions.size();
        Point3D[] positionsArray = new InhomogeneousPoint3D[size];
        positionsArray = positions.toArray(positionsArray);

        final var distancesArray = new double[size];
        for (var i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
        }

        try {
            homogeneousTrilaterationSolver.setPositionsAndDistances(positionsArray, distancesArray);
            inhomogeneousTrilaterationSolver.setPositionsAndDistances(positionsArray, distancesArray);
        } catch (final LockedException e) {
            throw new IllegalArgumentException(e);
        }
    }

    /**
     * Initializes lateration solver.
     */
    private void initialize() {
        homogeneousTrilaterationSolver = new HomogeneousLinearLeastSquaresLateration3DSolver(laterationSolverListener);
        inhomogeneousTrilaterationSolver = new InhomogeneousLinearLeastSquaresLateration3DSolver(
                laterationSolverListener);
    }
}
