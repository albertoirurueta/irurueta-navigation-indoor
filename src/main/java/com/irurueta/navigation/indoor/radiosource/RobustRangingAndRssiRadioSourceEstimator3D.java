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
package com.irurueta.navigation.indoor.radiosource;

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconWithPowerAndLocated3D;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceWithPowerAndLocated;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * This is an abstract class to robustly estimate 3D position, transmitted power and path-loss
 * exponent of a radio source (e.g. Wi-Fi access point or bluetooth beacon), by discarding
 * outliers and assuming that the ranging data is available to obtain position with
 * greater accuracy and that the radio source emits isotropically following the
 * expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antenna
 * Gr is the Gain of the receiver antenna
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the radio signal.
 * Because usually information about the antenna of the radio source cannot be
 * retrieved (because many measurements are made on unknown devices where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be assumed an RSSI standard deviation of 1 dB.
 * <p>
 * Although RobustRssiRadioSourceEstimator can estimate the same parameters of a radio
 * source, when ranging measures are available along with RSSI measurements,
 * implementations of this class should be preferred instead as they can provide
 * greater accuracy.
 *
 * @param <S> a {@link RadioSource} type.
 */
public abstract class RobustRangingAndRssiRadioSourceEstimator3D<S extends RadioSource> extends
        RobustRangingAndRssiRadioSourceEstimator<S, Point3D> {

    /**
     * Radio source estimator used internally.
     */
    protected final RangingAndRssiRadioSourceEstimator3D<S> innerEstimator =
            new RangingAndRssiRadioSourceEstimator3D<>();

    /**
     * Subset of readings used by inner estimator.
     */
    private final List<RangingAndRssiReadingLocated<S, Point3D>> innerReadings = new ArrayList<>();

    /**
     * Constructor.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D() {
        super();
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings) {
        super(readings);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition) {
        super(readings, initialPosition);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(final Point3D initialPosition) {
        super(initialPosition);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(final Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   Wi-Fi signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialTransmittedPowerdBm, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   Wi-Fi signal readings belonging to the radio source point.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialPosition, initialTransmittedPowerdBm);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent) {
        super(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Constructor.
     * Sets signal readings belonging to the same radio source.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    protected RobustRangingAndRssiRadioSourceEstimator3D(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
        preliminarySubsetSize = getMinReadings();
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param method robust estimator method.
     * @param <S>    a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>();
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>();
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>();
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>();
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>();
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param method   robust estimator method.
     * @param <S>      a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings);
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
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @param method   robust estimator method.
     * @param <S>      a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Double initialTransmittedPowerdBm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D radio source power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm,
                    listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm,
                    listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm,
                    listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm,
                    listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm,
                    listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D radio source power and position estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
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
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>();
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>();
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>();
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @param method        robust estimator method.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener in charge of attending events raised by this instance.
     * @param method        robust estimator method.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @param listener      listener in charge of attending events raised by this instance.
     * @param method        robust estimator method.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance
     * @param method          robust estimator method.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's).
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Double initialTransmittedPowerdBm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    initialTransmittedPowerdBm);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    initialTransmittedPowerdBm);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialTransmittedPowerdBm);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialTransmittedPowerdBm);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialTransmittedPowerdBm, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    initialTransmittedPowerdBm, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    initialTransmittedPowerdBm, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm,
                    listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm,
                    listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialTransmittedPowerdBm,
                    listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialTransmittedPowerdBm, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialTransmittedPowerdBm, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition, initialTransmittedPowerdBm);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, initialPosition,
                    initialTransmittedPowerdBm);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    initialTransmittedPowerdBm);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    initialTransmittedPowerdBm);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    initialTransmittedPowerdBm, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    initialTransmittedPowerdBm, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition, initialTransmittedPowerdBm, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition, initialTransmittedPowerdBm, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param method                     robust estimator method.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case LMEDS -> new LMedSRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case MSAC -> new MSACRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, initialPathLossExponent, listener);
            case PROSAC -> new PROSACRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
            default -> new PROMedSRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                    initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
        };
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param <S> a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param <S>      a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings) {
        return create(readings, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param listener listener in charge of attending events raised by this instance.
     * @param <S>      a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @param <S>      a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(readings, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition) {
        return create(readings, initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition) {
        return create(initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(readings, initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Double initialTransmittedPowerdBm) {
        return create(initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm) {
        return create(readings, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(readings, initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        return create(readings, initialPosition, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm) {
        return create(initialPosition, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(initialPosition, initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(readings, initialPosition, initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent) {
        return create(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        return create(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings) {
        return create(qualityScores, readings, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener in charge of attending events raised by this instance.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings      signal readings belonging to the same radio source.
     * @param listener      listener in charge of attending events raised by this instance.
     * @param <S>           a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, readings, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition) {
        return create(qualityScores, readings, initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition) {
        return create(qualityScores, initialPosition, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param readings        signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance
     * @param <S>             a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, readings, initialPosition, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Double initialTransmittedPowerdBm) {
        return create(qualityScores, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm) {
        return create(qualityScores, readings, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, readings, initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm) {
        return create(qualityScores, readings, initialPosition, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final Double initialTransmittedPowerdBm) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, readings, initialPosition, initialTransmittedPowerdBm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        return create(qualityScores, readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust 3D position radio source estimator using
     * default method.
     *
     * @param qualityScores              quality scores corresponding to each provided
     *                                   sample. The larger the score value the better
     *                                   the quality of the sample.
     * @param readings                   signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @param <S>                        a {@link RadioSource} type.
     * @return a new robust 3D position radio source estimator.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public static <S extends RadioSource> RobustRangingAndRssiRadioSourceEstimator3D<S> create(
            final double[] qualityScores, final List<? extends RangingAndRssiReadingLocated<S, Point3D>> readings,
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent,
            final RobustRangingAndRssiRadioSourceEstimatorListener<S, Point3D> listener) {
        return create(qualityScores, readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Indicates whether an homogeneous linear solver is used to estimate an initial
     * position for the internal ranging radio source estimator.
     *
     * @return true if homogeneous linear solver is used, false if an inhomogeneous linear
     * one is used instead.
     */
    @Override
    public boolean isHomogeneousRangingLinearSolverUsed() {
        return innerEstimator.isHomogeneousRangingLinearSolverUsed();
    }

    /**
     * Specifies whether an homogeneous linear solver is used to estimate an initial
     * position for the internal ranging radio source estimator.
     *
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used, false
     *                                   if an inhomogeneous linear one is used instead.
     * @throws LockedException if estimator is locked.
     */
    @Override
    public void setHomogeneousRangingLinearSolverUsed(final boolean useHomogeneousLinearSolver) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        innerEstimator.setHomogeneousRangingLinearSolverUsed(useHomogeneousLinearSolver);
    }

    /**
     * Gets minimum required number of readings to estimate
     * power, position and path-loss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 3
     * readings.
     *
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        var minReadings = Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
        if (isTransmittedPowerEstimationEnabled()) {
            minReadings++;
        }
        if (isPathLossEstimationEnabled()) {
            minReadings++;
        }
        return ++minReadings;
    }

    /**
     * Gets number of dimensions of position points.
     *
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return innerEstimator.getNumberOfDimensions();
    }

    /**
     * Gets estimated located radio source with estimated transmitted power.
     *
     * @return estimated located radio source with estimated transmitted power or null.
     */
    @Override
    @SuppressWarnings("unchecked")
    public RadioSourceWithPowerAndLocated<Point3D> getEstimatedRadioSource() {
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

        final var transmittedPowerVariance = getEstimatedTransmittedPowerVariance();
        final var transmittedPowerStandardDeviation = transmittedPowerVariance != null
                ? Math.sqrt(transmittedPowerVariance) : null;

        final var pathlossExponentVariance = getEstimatedPathLossExponentVariance();
        final var pathlossExponentStandardDeviation = pathlossExponentVariance != null
                ? Math.sqrt(pathlossExponentVariance) : null;

        if (source instanceof WifiAccessPoint accessPoint) {
            return new WifiAccessPointWithPowerAndLocated3D(accessPoint.getBssid(), source.getFrequency(),
                    accessPoint.getSsid(), getEstimatedTransmittedPowerdBm(), transmittedPowerStandardDeviation,
                    getEstimatedPathLossExponent(), pathlossExponentStandardDeviation, estimatedPosition,
                    estimatedPositionCovariance);
        } else if (source instanceof Beacon beacon) {
            return new BeaconWithPowerAndLocated3D(beacon.getIdentifiers(), getEstimatedTransmittedPowerdBm(),
                    beacon.getFrequency(), beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(), beacon.getBluetoothName(),
                    getEstimatedPathLossExponent(), transmittedPowerStandardDeviation,
                    pathlossExponentStandardDeviation, estimatedPosition, estimatedPositionCovariance);
        } else {
            return null;
        }
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

            // initial transmitted power and position might or might not be available
            innerEstimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            innerEstimator.setInitialPosition(initialPosition);
            innerEstimator.setInitialPathLossExponent(initialPathLossExponent);

            innerEstimator.setTransmittedPowerEstimationEnabled(transmittedPowerEstimationEnabled);
            innerEstimator.setPathLossEstimationEnabled(pathLossEstimationEnabled);

            innerEstimator.setReadings(innerReadings);

            // indicates whether readings position covariances must be taken into account
            innerEstimator.setUseReadingPositionCovariances(useReadingPositionCovariances);

            innerEstimator.estimate();

            final var estimatedPosition = innerEstimator.getEstimatedPosition();
            final var estimatedTransmittedPowerdBm = innerEstimator.getEstimatedTransmittedPowerdBm();
            final var estimatedPathLossExponent = innerEstimator.getEstimatedPathLossExponent();
            solutions.add(new Solution<>(estimatedPosition, estimatedTransmittedPowerdBm, estimatedPathLossExponent));
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
        final var initialTransmittedPowerdBm = result.getEstimatedTransmittedPowerdBm();
        final var initialPathLossExponent = result.getEstimatedPathLossExponent();

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
                innerEstimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
                innerEstimator.setInitialPathLossExponent(initialPathLossExponent);
                innerEstimator.setTransmittedPowerEstimationEnabled(transmittedPowerEstimationEnabled);
                innerEstimator.setPathLossEstimationEnabled(pathLossEstimationEnabled);
                innerEstimator.setReadings(innerReadings);
                innerEstimator.setUseReadingPositionCovariances(useReadingPositionCovariances);

                innerEstimator.estimate();

                final var cov = innerEstimator.getEstimatedCovariance();
                if (keepCovariance && cov != null) {
                    // keep covariance
                    covariance = cov;

                    final var dims = getNumberOfDimensions();
                    var pos = 0;

                    final var d = dims - 1;
                    if (estimatedPositionCovariance == null) {
                        estimatedPositionCovariance = covariance.getSubmatrix(0, 0, d, d);
                    } else {
                        covariance.getSubmatrix(0, 0, d, d, estimatedPositionCovariance);
                    }
                    pos += dims;

                    if (transmittedPowerEstimationEnabled) {
                        // transmitted power estimation enabled
                        estimatedTransmittedPowerVariance = covariance.getElementAt(pos, pos);
                        pos++;
                    } else {
                        // transmitted power estimation disabled
                        estimatedTransmittedPowerVariance = null;
                    }

                    if (pathLossEstimationEnabled) {
                        // path-loss exponent estimation enabled
                        estimatedPathLossExponentVariance = covariance.getElementAt(pos, pos);
                    } else {
                        // path-loss exponent estimation disabled
                        estimatedPathLossExponentVariance = null;
                    }
                } else {
                    covariance = null;
                    estimatedPositionCovariance = null;
                    estimatedTransmittedPowerVariance = null;
                    estimatedPathLossExponentVariance = null;
                }

                estimatedPosition = innerEstimator.getEstimatedPosition();
                estimatedTransmittedPowerdBm = innerEstimator.getEstimatedTransmittedPowerdBm();
                estimatedPathLossExponent = innerEstimator.getEstimatedPathLossExponent();
            } catch (final Exception e) {
                // refinement failed, so we return input value, and covariance
                // becomes unavailable
                covariance = null;
                estimatedPositionCovariance = null;
                estimatedTransmittedPowerVariance = null;
                estimatedPathLossExponentVariance = null;

                estimatedPosition = initialPosition;
                estimatedTransmittedPowerdBm = initialTransmittedPowerdBm;
                estimatedPathLossExponent = initialPathLossExponent;
            }
        } else {
            covariance = null;
            estimatedPositionCovariance = null;
            estimatedTransmittedPowerVariance = null;
            estimatedPathLossExponentVariance = null;

            estimatedPosition = initialPosition;
            estimatedTransmittedPowerdBm = initialTransmittedPowerdBm;
            estimatedPathLossExponent = initialPathLossExponent;
        }
    }
}
