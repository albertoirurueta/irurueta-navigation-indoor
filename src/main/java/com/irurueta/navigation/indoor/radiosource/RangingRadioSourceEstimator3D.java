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

import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconLocated3D;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RangingReadingLocated;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.navigation.lateration.HomogeneousLinearLeastSquaresLateration3DSolver;
import com.irurueta.navigation.lateration.InhomogeneousLinearLeastSquaresLateration3DSolver;
import com.irurueta.navigation.lateration.NonLinearLeastSquaresLateration3DSolver;

import java.util.List;

/**
 * Estimates position of a radio source (e.g. Wi-Fi access point or bluetooth beacon)
 * by using ranging measurements.
 * Ranging measurements can be obtained by protocols such as ieee 802.11mc (Wi-Fi RTT) which
 * measures travel time of signal and converts the result into distances by taking into
 * account the speed of light as the propagation speed.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("DuplicatedCode")
public class RangingRadioSourceEstimator3D<S extends RadioSource> extends RangingRadioSourceEstimator<S, Point3D> {

    /**
     * Constructor.
     */
    public RangingRadioSourceEstimator3D() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RangingRadioSourceEstimator3D(final List<? extends RangingReadingLocated<S, Point3D>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RangingRadioSourceEstimator3D(final RangingRadioSourceEstimatorListener<S, Point3D> listener) {
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
    public RangingRadioSourceEstimator3D(
            final List<? extends RangingReadingLocated<S, Point3D>> readings,
            final RangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    public RangingRadioSourceEstimator3D(final Point3D initialPosition) {
        super(initialPosition);
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
    public RangingRadioSourceEstimator3D(
            final List<? extends RangingReadingLocated<S, Point3D>> readings, final Point3D initialPosition) {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public RangingRadioSourceEstimator3D(
            final Point3D initialPosition, final RangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
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
    public RangingRadioSourceEstimator3D(
            final List<? extends RangingReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
    }

    /**
     * Gets minimum required number of readings to estimate position of radio source,
     * which is 4 readings.
     *
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets number of dimensions of position points.
     * This is always 3.
     *
     * @return number of dimensions of position points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated radio source 3D position.
     *
     * @return estimated radio source 3D position.
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
     * Gets estimated located radio source.
     *
     * @return estimated located radio source or null.
     */
    @Override
    @SuppressWarnings("unchecked")
    public RadioSourceLocated<Point3D> getEstimatedRadioSource() {
        final var readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }
        final var source = readings.get(0).getSource();

        final var estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        final var estimatedPositionCovariance = getEstimatedPositionCovariance();

        if (source instanceof WifiAccessPoint accessPoint) {
            return new WifiAccessPointLocated3D(accessPoint.getBssid(), accessPoint.getFrequency(),
                    accessPoint.getSsid(), estimatedPosition, estimatedPositionCovariance);
        } else if (source instanceof Beacon beacon) {
            return new BeaconLocated3D(beacon.getIdentifiers(), beacon.getTransmittedPower(), beacon.getFrequency(),
                    beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(), beacon.getManufacturer(),
                    beacon.getServiceUuid(), beacon.getBluetoothName(), estimatedPosition, estimatedPositionCovariance);
        } else {
            return null;
        }
    }

    /**
     * Builds an instance of a linear lateration solver if needed.
     */
    @Override
    protected void buildLinearSolverIfNeeded() {
        if (initialPosition == null || !nonLinearSolverEnabled) {
            if (useHomogeneousLinearSolver && homogeneousLinearSolver == null) {
                homogeneousLinearSolver = new HomogeneousLinearLeastSquaresLateration3DSolver();
            }
            if (!useHomogeneousLinearSolver && inhomogeneousLinearSolver == null) {
                inhomogeneousLinearSolver = new InhomogeneousLinearLeastSquaresLateration3DSolver();
            }
        }
    }

    /**
     * Builds an instance of a non-linear lateration solver if needed.
     */
    @Override
    protected void buildNonLinearSolverIfNeeded() {
        if (nonLinearSolver == null && nonLinearSolverEnabled) {
            nonLinearSolver = new NonLinearLeastSquaresLateration3DSolver();
        }
    }

    /**
     * Build an instance of accuracy if needed.
     */
    @Override
    protected void buildAccuracyIfNeeded() {
        if (accuracy == null && useReadingPositionCovariances) {
            accuracy = new Accuracy3D();

            // to work with standard deviations, we need a unitary factor
            accuracy.setStandardDeviationFactor(1.0);
        }
    }

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
    @Override
    protected void setPositionsDistancesAndDistanceStandardDeviations(
            final List<Point3D> positions, final List<Double> distances, final List<Double> distanceStandardDeviations)
            throws LockedException {

        final var size = positions.size();
        Point3D[] positionsArray = new InhomogeneousPoint3D[size];
        positionsArray = positions.toArray(positionsArray);

        final var distancesArray = new double[size];
        final var distanceStandardDeviationsArray = new double[size];
        for (var i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
            distanceStandardDeviationsArray[i] = distanceStandardDeviations.get(i);
        }

        if (initialPosition == null || !nonLinearSolverEnabled) {
            if (homogeneousLinearSolver != null) {
                homogeneousLinearSolver.setPositionsAndDistances(positionsArray, distancesArray);
            }
            if (inhomogeneousLinearSolver != null) {
                inhomogeneousLinearSolver.setPositionsAndDistances(positionsArray, distancesArray);
            }
        }

        if (nonLinearSolver != null && nonLinearSolverEnabled) {
            nonLinearSolver.setPositionsDistancesAndStandardDeviations(positionsArray, distancesArray,
                    distanceStandardDeviationsArray);
        }
    }
}
