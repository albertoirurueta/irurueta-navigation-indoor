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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class NonLinearFingerprintPositionEstimator2DTest implements FingerprintPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(NonLinearFingerprintPositionEstimator2DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final int MIN_FINGERPRINTS = 100;
    private static final int MAX_FINGERPRINTS = 1000;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 100;

    @Test
    void testCreate() {
        // test create empty

        // first order
        var estimator = NonLinearFingerprintPositionEstimator2D.create(
                NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator2D.create(
                NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator2D.create(
                NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator2D.create();

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // test create with listener

        // first order
        estimator = NonLinearFingerprintPositionEstimator2D.create(this,
                NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator2D.create(this,
                NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator2D.create(this,
                NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator2D.create(this);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // test create with located fingerprints, fingerprint and sources
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RssiReading<RadioSource>>();
        for (var i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final var accessPoint = new WifiAccessPoint("bssid" + i, FREQUENCY);
            final var rssi = randomizer.nextDouble();

            final var reading = new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        final var locatedFingerprint = new RssiFingerprintLocated2D<>(readings, Point2D.create());

        final var locatedFingerprints =
                new ArrayList<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>>();
        locatedFingerprints.add(locatedFingerprint);

        final var fingerprint = new RssiFingerprint<>(readings);

        final var sources = new ArrayList<RadioSourceLocated<Point2D>>();
        for (var i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final var source = new WifiAccessPointLocated2D("bssid" + 1, FREQUENCY, Point2D.create());
            sources.add(source);
        }

        // first order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // test create with located fingerprints, fingerprint, sources and listener

        // first order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                this, NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                this, NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                this, NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                this);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // test create with located fingerprints, fingerprint, sources and initial position
        final var initialPosition = Point2D.create();

        // first order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                initialPosition, NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                initialPosition, NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                initialPosition, NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                initialPosition);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // test create with located fingerprints, fingerprint, sources, initial position
        // and listener

        // first order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                initialPosition, this, NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                initialPosition, this, NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                initialPosition, this, NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints, fingerprint, sources,
                initialPosition, this);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
    }

    @Test
    void testEstimateOrderComparison() throws LockedException, NotReadyException, FingerprintEstimationException {
        var avgRssiErrorFirstOrder = 0.0;
        var avgRssiErrorSecondOrder = 0.0;
        var avgRssiErrorThirdOrder = 0.0;

        var avgPositionErrorFirstOrder = 0.0;
        var avgPositionErrorSecondOrder = 0.0;
        var avgPositionErrorThirdOrder = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final var sources = new ArrayList<RadioSourceLocated<Point2D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint2D(x, y);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated2D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint2D(x, y);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated2D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint2D(x, y);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance
                ));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // find the closest fingerprint based on RSSI without mean
            final var fingerprintFinder = new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            final var nearestFingerprint = fingerprintFinder.findNearestTo(fingerprint);
            final var nearestFingerprintPosition = nearestFingerprint.getPosition();

            final var firstOrderEstimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints,
                    fingerprint, sources, this, NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);
            final var secondOrderEstimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints,
                    fingerprint, sources, this, NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);
            final var thirdOrderEstimator = NonLinearFingerprintPositionEstimator2D.create(locatedFingerprints,
                    fingerprint, sources, this, NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

            // compute average RSSI error for each order
            final var params = new double[2];
            params[0] = x;
            params[1] = y;

            final var point = new double[6];
            final var derivatives = new double[2];
            for (var i = 0; i < numSources; i++) {
                final var source = sources.get(i);
                final var reading = readings.get(i);

                final var sourcePosition = source.getPosition();

                final var readingRssi = reading.getRssi();

                point[0] = readingRssi;

                point[1] = nearestFingerprintPosition.getInhomX();
                point[2] = nearestFingerprintPosition.getInhomY();

                point[3] = sourcePosition.getInhomX();
                point[4] = sourcePosition.getInhomY();

                point[5] = LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT;

                final var estimatedRssiFirstOrder = firstOrderEstimator.evaluate(
                        0, point, params, derivatives);
                final var estimatedRssiSecondOrder = secondOrderEstimator.evaluate(
                        0, point, params, derivatives);
                final var estimatedRssiThirdOrder = thirdOrderEstimator.evaluate(
                        0, point, params, derivatives);

                final var rssiErrorFirstOrder = Math.abs(estimatedRssiFirstOrder - readingRssi);
                final var rssiErrorSecondOrder = Math.abs(estimatedRssiSecondOrder - readingRssi);
                final var rssiErrorThirdOrder = Math.abs(estimatedRssiThirdOrder - readingRssi);

                avgRssiErrorFirstOrder += rssiErrorFirstOrder / TIMES;
                avgRssiErrorSecondOrder += rssiErrorSecondOrder / TIMES;
                avgRssiErrorThirdOrder += rssiErrorThirdOrder / TIMES;

                firstOrderEstimator.estimate();
                secondOrderEstimator.estimate();
                thirdOrderEstimator.estimate();

                final var estimatedPositionFirstOrder = firstOrderEstimator.getEstimatedPosition();
                final var estimatedPositionSecondOrder = secondOrderEstimator.getEstimatedPosition();
                final var estimatedPositionThirdOrder = thirdOrderEstimator.getEstimatedPosition();

                final var positionErrorFirstOrder = estimatedPositionFirstOrder.distanceTo(position);
                final var positionErrorSecondOrder = estimatedPositionSecondOrder.distanceTo(position);
                final var positionErrorThirdOrder = estimatedPositionThirdOrder.distanceTo(position);

                avgPositionErrorFirstOrder += positionErrorFirstOrder / TIMES;
                avgPositionErrorSecondOrder += positionErrorSecondOrder / TIMES;
                avgPositionErrorThirdOrder += positionErrorThirdOrder / TIMES;
            }
        }

        LOGGER.log(Level.INFO, "Avg rssi error 1st order: {0} dBm", avgRssiErrorFirstOrder);
        LOGGER.log(Level.INFO, "Avg rssi error 2nd order: {0} dBm", avgRssiErrorSecondOrder);
        LOGGER.log(Level.INFO, "Avg rssi error 3rd order: {0} dBm", avgRssiErrorThirdOrder);

        LOGGER.log(Level.INFO, "Avg position error 1st order: {0} m", avgPositionErrorFirstOrder);
        LOGGER.log(Level.INFO, "Avg position error 2nd order: {0} m", avgPositionErrorSecondOrder);
        LOGGER.log(Level.INFO, "Avg position error 3rd order: {0} m", avgPositionErrorThirdOrder);

        assertTrue(avgPositionErrorFirstOrder >= avgPositionErrorSecondOrder);
        assertTrue(avgPositionErrorFirstOrder >= avgPositionErrorThirdOrder);
    }

    @Override
    public void onEstimateStart(final FingerprintPositionEstimator<Point2D> estimator) {
        checkLocked((NonLinearFingerprintPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(final FingerprintPositionEstimator<Point2D> estimator) {
        checkLocked((NonLinearFingerprintPositionEstimator2D) estimator);
    }

    private static double receivedPower(final double equivalentTransmittedPower, final double distance) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final var k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY),
                BaseFingerprintPositionEstimator.DEFAULT_PATH_LOSS_EXPONENT);
        return equivalentTransmittedPower * k /
                Math.pow(distance, BaseFingerprintPositionEstimator.DEFAULT_PATH_LOSS_EXPONENT);
    }

    private void checkLocked(final NonLinearFingerprintPositionEstimator2D estimator) {
        assertThrows(LockedException.class, () -> estimator.setLocatedFingerprints(null));
        assertThrows(LockedException.class, () -> estimator.setFingerprint(null));
        assertThrows(LockedException.class, () -> estimator.setMinMaxNearestFingerprints(1, 1));
        assertThrows(LockedException.class, () -> estimator.setPathLossExponent(2.0));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.setSources(null));
        assertThrows(LockedException.class, () -> estimator.setUseSourcesPathLossExponentWhenAvailable(false));
        assertThrows(LockedException.class, () -> estimator.setUseNoMeanNearestFingerprintFinder(false));
        assertThrows(LockedException.class, () -> estimator.setMeansFromFingerprintReadingsRemoved(false));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, () -> estimator.setFallbackRssiStandardDeviation(1.0));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
