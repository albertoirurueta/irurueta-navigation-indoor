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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconWithPowerAndLocated3D;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceWithPowerAndLocated;
import com.irurueta.navigation.indoor.RssiReadingLocated;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated3D;

import java.util.List;

/**
 * Estimates 3D position and transmitted power of a radio source assuming that the
 * radio source emits isotropically following the expression below:
 * Pr = Pt*Gt*Gr*lambda^2 / (4*pi*d)^2,
 * where Pr is the received power (expressed in mW),
 * Gt is the Gain of the transmission antenna
 * Gr is the Gain of the receiver antenna
 * d is the distance between emitter and receiver
 * and lambda is the wavelength and is equal to: lambda = c / f,
 * where c is the speed of light
 * and f is the carrier frequency of the Wi-Fi signal.
 * Because usually information about the antenna of the radio source cannot be
 * retrieved (because many measurements are made on unknown radio sources where
 * physical access is not possible), this implementation will estimate the
 * equivalent transmitted power as: Pte = Pt * Gt * Gr.
 * If Readings contain RSSI standard deviations, those values will be used,
 * otherwise it will be assumed an RSSI standard deviation of 1 dB.
 * <p>
 * IMPORTANT: Implementations of this class can choose to estimate a
 * combination of radio source position, transmitted power and path loss
 * exponent. However enabling all three estimations usually achieves
 * inaccurate results. When using this class, estimation must be of at least
 * one parameter (position, transmitted power or path loss exponent) when
 * initial values are provided for the other two, and at most it should consist
 * of two parameters (either position and transmitted power, position and
 * path loss exponent or transmitted power and path loss exponent), providing an
 * initial value for the remaining parameter.
 *
 * @param <S> a {@link RadioSource} type.
 */
public class RssiRadioSourceEstimator3D<S extends RadioSource> extends RssiRadioSourceEstimator<S, Point3D> {

    /**
     * Constructor.
     */
    public RssiRadioSourceEstimator3D() {
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(final List<? extends RssiReadingLocated<S, Point3D>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator3D(final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same
     *                 radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings,
            final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public RssiRadioSourceEstimator3D(final Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings        radio signal readings belonging to the same
     *                        radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition) {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator3D(
            final Point3D initialPosition, final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings        radio signal readings belonging to the same
     *                        radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     */
    public RssiRadioSourceEstimator3D(final Double initialTransmittedPowerdBm) {
        super(initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same
     *                                   radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings, final Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator3D(
            final Double initialTransmittedPowerdBm, final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same
     *                                   radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings, final Double initialTransmittedPowerdBm,
            final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same
     *                                   radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of access point transmitted power
     *                                   (expressed in dBm's)
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm) {
        super(readings, initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     */
    public RssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same
     *                                   radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same
     *                                   radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent) {
        super(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     */
    public RssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent);
    }

    /**
     * Constructor.
     *
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public RssiRadioSourceEstimator3D(
            final Point3D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent, final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same
     *                                   radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's)
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RssiRadioSourceEstimator3D(
            final List<? extends RssiReadingLocated<S, Point3D>> readings, final Point3D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final RssiRadioSourceEstimatorListener<S, Point3D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
    }

    /**
     * Gets minimum required number of readings to estimate
     * power, position and path-loss exponent.
     * This value depends on the number of parameters to
     * be estimated, but for position only, this is 4
     * readings for 3D.
     *
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        var minReadings = 0;
        if (isPositionEstimationEnabled()) {
            minReadings += Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
        }
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
     * Gets estimated located radio source with estimated transmitted power.
     *
     * @return estimated located radio source with estimated transmitted power or null.
     */
    @Override
    @SuppressWarnings({"unchecked", "DuplicatedCode"})
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
            return new WifiAccessPointWithPowerAndLocated3D(accessPoint.getBssid(), accessPoint.getFrequency(),
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
}
