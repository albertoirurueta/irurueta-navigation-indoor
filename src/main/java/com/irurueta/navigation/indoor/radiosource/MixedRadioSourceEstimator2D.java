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

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * Estimates 2D position, transmitted power and path loss exponent of a
 * radio source (e.g. Wi-Fi access point or bluetooth beacon) assuming
 * that the ranging data is available to obtain position with greater
 * accuracy and that the radio source emits isotropically following the
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
 * This implementation is like RangingAndRssiRadioSourceEstimator2D but allows mixing
 * different kinds of located radio source readings (ranging, RSSI and ranging+RSSI).
 *
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("Duplicates")
public class MixedRadioSourceEstimator2D<S extends RadioSource> extends MixedRadioSourceEstimator<S, Point2D> {

    /**
     * Constructor.
     */
    public MixedRadioSourceEstimator2D() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same
     *                 radio sources.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator2D(final List<? extends ReadingLocated<Point2D>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public MixedRadioSourceEstimator2D(final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
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
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings,
            final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     */
    public MixedRadioSourceEstimator2D(final Point2D initialPosition) {
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
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition) {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public MixedRadioSourceEstimator2D(
            final Point2D initialPosition, final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings        radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
            final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerDbm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     */
    public MixedRadioSourceEstimator2D(final Double initialTransmittedPowerDbm) {
        super(initialTransmittedPowerDbm);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Double initialTransmittedPowerdBm) {
        super(readings, initialTransmittedPowerdBm);
    }

    /**
     * Constructor.
     *
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public MixedRadioSourceEstimator2D(
            final Double initialTransmittedPowerdBm, final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Double initialTransmittedPowerdBm,
            final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
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
     *                                   (expressed in dBm's).
     */
    public MixedRadioSourceEstimator2D(final Point2D initialPosition, final Double initialTransmittedPowerdBm) {
        super(initialPosition, initialTransmittedPowerdBm);
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
    public MixedRadioSourceEstimator2D(
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
            final Double initialTransmittedPowerdBm, final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
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
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     */
    public MixedRadioSourceEstimator2D(
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
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
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     */
    public MixedRadioSourceEstimator2D(
            final Point2D initialPosition, final Double initialTransmittedPowerdBm,
            final double initialPathLossExponent, final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings                   radio signal readings belonging to the same radio source.
     * @param initialPosition            initial position to start the estimation of radio
     *                                   source position.
     * @param initialTransmittedPowerdBm initial transmitted power to start the
     *                                   estimation of radio source transmitted power
     *                                   (expressed in dBm's).
     * @param initialPathLossExponent    initial path loss exponent. A typical value is 2.0.
     * @param listener                   listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public MixedRadioSourceEstimator2D(
            final List<? extends ReadingLocated<Point2D>> readings, final Point2D initialPosition,
            final Double initialTransmittedPowerdBm, final double initialPathLossExponent,
            final MixedRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, initialTransmittedPowerdBm, initialPathLossExponent, listener);
    }

    /**
     * Gets number of dimensions of position points.
     *
     * @return number of dimensions of position points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated located radio source.
     *
     * @return estimated located radio source or null.
     */
    @SuppressWarnings("unchecked")
    @Override
    public RadioSourceLocated<Point2D> getEstimatedRadioSource() {
        final var readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }

        final S source;
        final var reading = readings.get(0);
        if (reading instanceof RangingReadingLocated) {
            source = ((RangingReadingLocated<S, Point2D>) reading).getSource();
        } else if (reading instanceof RssiReadingLocated) {
            source = ((RssiReadingLocated<S, Point2D>) reading).getSource();
        } else if (reading instanceof RangingAndRssiReadingLocated) {
            source = ((RangingAndRssiReadingLocated<S, Point2D>) reading).getSource();
        } else {
            return null;
        }

        final var estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        final var estimatedPositionCovariance = getEstimatedPositionCovariance();

        final var transmittedPowerdBm = getEstimatedTransmittedPowerdBm();

        final var transmittedPowerVariance = getEstimatedTransmittedPowerVariance();
        final var transmittedPowerStandardDeviation = transmittedPowerVariance != null
                ? Math.sqrt(transmittedPowerVariance) : null;

        final var pathLossExponent = getEstimatedPathLossExponent();

        final var pathLossExponentVariance = getEstimatedPathLossExponentVariance();
        final var pathLossExponentStandardDeviation = pathLossExponentVariance != null
                ? Math.sqrt(pathLossExponentVariance) : null;

        if (source instanceof WifiAccessPoint accessPoint) {
            if (transmittedPowerdBm != null) {
                return new WifiAccessPointWithPowerAndLocated2D(accessPoint.getBssid(), accessPoint.getFrequency(),
                        accessPoint.getSsid(), transmittedPowerdBm, transmittedPowerStandardDeviation, pathLossExponent,
                        pathLossExponentStandardDeviation, estimatedPosition, estimatedPositionCovariance);
            } else {
                return new WifiAccessPointLocated2D(accessPoint.getBssid(), accessPoint.getFrequency(),
                        accessPoint.getSsid(), estimatedPosition, estimatedPositionCovariance);
            }
        } else if (source instanceof Beacon beacon) {
            // transmitted power does not need to be estimated for beacons because
            // they broadcast such information
            return new BeaconWithPowerAndLocated2D(beacon.getIdentifiers(), beacon.getTransmittedPower(),
                    beacon.getFrequency(), beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(), beacon.getBluetoothName(), pathLossExponent,
                    transmittedPowerStandardDeviation, pathLossExponentStandardDeviation, estimatedPosition,
                    estimatedPositionCovariance);
        } else {
            return null;
        }
    }

    /**
     * Creates inner estimators if needed.
     */
    @Override
    protected void createInnerEstimatorsIfNeeded() {
        if (rangingInnerEstimator == null) {
            rangingInnerEstimator = new RangingRadioSourceEstimator2D<>();
        }

        if (rssiInnerEstimator == null && (transmittedPowerEstimationEnabled || pathLossEstimationEnabled)) {
            rssiInnerEstimator = new RssiRadioSourceEstimator2D<>();
        }
    }
}
