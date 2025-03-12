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

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconLocated3D;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RangingReadingLocated;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * This is an abstract class to robustly estimate 3D position of a radio source (e.g. Wi-Fi
 * access point or bluetooth beacon), by discarding outliers.
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class RobustRangingRadioSourceEstimator3D<S extends RadioSource> extends
        RobustRangingRadioSourceEstimator<S, Point3D> {

    /**
     * Radio source estimator used internally.
     */
    protected final RangingRadioSourceEstimator3D<S> innerEstimator = new RangingRadioSourceEstimator3D<>();

    /**
     * Subset of readings used by inner estimator.
     */
    private final List<RangingReadingLocated<S, Point3D>> innerReadings = new ArrayList<>();

    /**
     * Constructor.
     */
    protected RobustRangingRadioSourceEstimator3D() {
        super();
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param readings radio signal ranging readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingRadioSourceEstimator3D(final List<? extends RangingReadingLocated<S, Point3D>> readings) {
        super(readings);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    protected RobustRangingRadioSourceEstimator3D(
            final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingRadioSourceEstimator3D(
            final List<? extends RangingReadingLocated<S, Point3D>> readings,
            final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    protected RobustRangingRadioSourceEstimator3D(final Point3D initialPosition) {
        super(initialPosition);
        preliminarySubsetSize = getMinReadings();
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
    protected RobustRangingRadioSourceEstimator3D(
            final List<? extends RangingReadingLocated<S, Point3D>> readings, final Point3D initialPosition) {
        super(readings, initialPosition);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    protected RobustRangingRadioSourceEstimator3D(
            final Point3D initialPosition, final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
        preliminarySubsetSize = getMinReadings();
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
    protected RobustRangingRadioSourceEstimator3D(
            final List<? extends RangingReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param method robust estimator method.
     * @param <S>    a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>();
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>();
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>();
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>();
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>();
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param method   robust estimator method.
     * @param <S>      a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final List<? extends RangingReadingLocated<S, Point3D>> readings, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(readings);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(readings);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(readings);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(readings);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(readings);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param listener listener in charge of attending events raised by this instance.
     * @param method   robust estimator method.
     * @param <S>      a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(listener);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(listener);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(listener);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(listener);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method   robust estimator method.
     * @param <S>      a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final List<? extends RangingReadingLocated<S, Point3D>> readings,
            final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(readings, listener);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(readings, listener);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(readings, listener);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(readings, listener);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(readings, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(initialPosition);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings        radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final List<? extends RangingReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition, listener);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition, listener);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(initialPosition, listener);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(initialPosition, listener);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(initialPosition, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings        radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final List<? extends RangingReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, listener);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param method        robust estimator method.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>();
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>();
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>();
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param readings      radio signal ranging readings belonging to the same radio source.
     * @param method        robust estimator method.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingReadingLocated<S, Point3D>> readings,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(readings);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(readings);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(readings);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores, readings);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.*
     * @param listener      listener in charge of attending events raised by this instance.
     * @param method        robust estimator method.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(listener);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(listener);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(listener);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, listener);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.**
     * @param readings      radio signal ranging readings belonging to the same radio source.
     * @param listener      listener in charge of attending events raised by this instance.
     * @param method        robust estimator method.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingReadingLocated<S, Point3D>> readings,
            final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(readings, listener);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(readings, listener);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(readings, listener);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, listener);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.*
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(initialPosition);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, initialPosition);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores, initialPosition);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.*
     * @param readings        radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, initialPosition);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, initialPosition);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.*
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition,
            final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition, listener);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(initialPosition, listener);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(initialPosition, listener);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, initialPosition, listener);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores, initialPosition, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.*
     * @param readings        radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final RobustRangingRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case LMEDS -> new LMedSRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case MSAC -> new MSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case PROSAC -> new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, initialPosition,
                    listener);
            default -> new PROMedSRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, initialPosition,
                    listener);
        };
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
     * Gets estimated located radio source.
     *
     * @return estimated located radio source or null.
     */
    @SuppressWarnings("unchecked")
    @Override
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
     * Indicates whether an homogeneous linear solver is used to estimate an initial
     * position.
     *
     * @return true if homogeneous linear solver is used, false if an inhomogeneous linear
     * one is used instead.
     */
    @Override
    public boolean isHomogeneousLinearSolverUsed() {
        return innerEstimator.isHomogeneousLinearSolverUsed();
    }

    /**
     * Specifies whether an homogeneous linear solver is used to estimate an initial
     * position.
     *
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used, false
     *                                   if an inhomogeneous linear one is used instead.
     * @throws LockedException if estimator is locked.
     */
    @Override
    public void setHomogeneousLinearSolverUsed(final boolean useHomogeneousLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        innerEstimator.setHomogeneousLinearSolverUsed(useHomogeneousLinearSolver);
    }

    /**
     * Solves preliminary solution for a subset of samples.
     *
     * @param samplesIndices indices of subset samples.
     * @param solutions      instance where solution will be stored.
     */
    @Override
    protected void solvePreliminarySolutions(final int[] samplesIndices, final List<Solution<Point3D>> solutions) {
        try {
            innerReadings.clear();
            for (final var samplesIndex : samplesIndices) {
                innerReadings.add(readings.get(samplesIndex));
            }

            // initial position might or might not be available
            innerEstimator.setInitialPosition(initialPosition);

            innerEstimator.setReadings(innerReadings);

            // for preliminary solutions, non-linear solver is not needed, and if no
            // initial position is used, we can obtain faster solutions disabling
            // non-linear solver and using a linear one only (because covariance is not
            // required)
            innerEstimator.setNonLinearSolverEnabled(initialPosition != null);

            // indicates whether readings position covariances must be taken into account
            innerEstimator.setUseReadingPositionCovariances(useReadingPositionCovariances);

            innerEstimator.estimate();

            final var estimatedPosition = innerEstimator.getEstimatedPosition();
            solutions.add(new Solution<>(estimatedPosition));
        } catch (final NavigationException ignore) {
            // if anything fails, no solution is added
        }
    }

    /**
     * Attempts to refine estimated position and transmitted power contained in
     * provided solution if refinement is requested.
     * This method sets a refined result and transmitted power or provided input
     * result if refinement is not requested or has failed.
     * If refinement is enabled, and it is requested to keep covariance, this method
     * will also keep covariance of refined result.
     * solution if not requested or refinement failed.
     *
     * @param result result to be refined.
     */
    protected void attemptRefine(final Solution<Point3D> result) {
        final var initialPosition = result.getEstimatedPosition();

        if (refineResult && inliersData != null) {
            final var inliers = inliersData.getInliers();
            final var nSamples = readings.size();

            innerReadings.clear();

            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    innerReadings.add(readings.get(i));
                }
            }

            try {
                innerEstimator.setInitialPosition(initialPosition);
                innerEstimator.setReadings(innerReadings);

                innerEstimator.setNonLinearSolverEnabled(true);
                innerEstimator.setUseReadingPositionCovariances(useReadingPositionCovariances);
                innerEstimator.estimate();

                final var cov = innerEstimator.getEstimatedCovariance();
                if (keepCovariance && cov != null) {
                    // keep covariance
                    estimatedPositionCovariance = covariance = cov;

                } else {
                    covariance = null;
                    estimatedPositionCovariance = null;
                }

                estimatedPosition = innerEstimator.getEstimatedPosition();
            } catch (final Exception e) {
                // refinement failed, so we return input value, and covariance
                // becomes unavailable
                covariance = null;
                estimatedPositionCovariance = null;

                estimatedPosition = initialPosition;
            }
        } else {
            covariance = null;
            estimatedPositionCovariance = null;

            estimatedPosition = initialPosition;
        }
    }
}

