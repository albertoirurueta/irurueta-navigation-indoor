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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class LinearRangingAndRssiPositionEstimator2DTest implements RangingAndRssiPositionEstimatorListener<Point2D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final double ERROR_STD = 1e-3;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;

    @Test
    void testConstructor() {
        // empty constructor
        var estimator = new LinearRangingAndRssiPositionEstimator2D();

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(3, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // constructor with sources
        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        for (var i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY, new InhomogeneousPoint2D()));
        }
        estimator = new LinearRangingAndRssiPositionEstimator2D(sources);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(3, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new LinearRangingAndRssiPositionEstimator2D((List<WifiAccessPointLocated2D>) null));
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(
                new ArrayList<WifiAccessPointLocated2D>()));

        // constructor with fingerprint
        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        estimator = new LinearRangingAndRssiPositionEstimator2D(fingerprint);

        // check default value
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(3, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null));

        // constructor with sources and fingerprint
        estimator = new LinearRangingAndRssiPositionEstimator2D(sources, fingerprint);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(3, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(null,
                fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(
                new ArrayList<WifiAccessPointLocated2D>(), fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(sources,
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null));

        // constructor with listener
        estimator = new LinearRangingAndRssiPositionEstimator2D(this);

        // check
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(3, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // constructor with sources and listener
        estimator = new LinearRangingAndRssiPositionEstimator2D(sources, this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(3, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(
                (List<WifiAccessPointLocated2D>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(
                new ArrayList<WifiAccessPointLocated2D>(), this));

        // constructor with fingerprint and listener
        estimator = new LinearRangingAndRssiPositionEstimator2D(fingerprint, this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(3, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                this));

        // constructor with sources, fingerprint and listener
        estimator = new LinearRangingAndRssiPositionEstimator2D(sources, fingerprint, this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(3, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(null,
                fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(
                new ArrayList<WifiAccessPointLocated2D>(), fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new LinearRangingAndRssiPositionEstimator2D(sources,
                null, this));
    }

    @Test
    void testGetSetSources() throws LockedException {
        final var estimator = new LinearRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        for (var i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY, new InhomogeneousPoint2D()));
        }

        estimator.setSources(sources);

        // check
        assertSame(sources, estimator.getSources());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setSources(null));
        assertThrows(IllegalArgumentException.class,
                () -> estimator.setSources(new ArrayList<WifiAccessPointLocated2D>()));
    }

    @Test
    void testGetSetFingerprint() throws LockedException {
        final var estimator = new LinearRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final var fingerprint = new RangingAndRssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(fingerprint, estimator.getFingerprint());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setFingerprint(null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new LinearRangingAndRssiPositionEstimator2D();

        // check default size
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testEstimateNoErrorHomogeneous() throws LockedException, NotReadyException, PositionEstimationException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
            }

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new LinearRangingAndRssiPositionEstimator2D(sources, fingerprint, this);

            reset();

            // check initial state
            assertTrue(estimator.isHomogeneousLinearSolverUsed());
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            var estimatedPosition = estimator.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }

        // force NotReadyException
        final var estimator = new LinearRangingAndRssiPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithErrorHomogeneous() throws LockedException, NotReadyException, PositionEstimationException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var error = errorRandomizer.nextDouble();
                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent)) + error;

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
            }

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new LinearRangingAndRssiPositionEstimator2D(sources, fingerprint, this);

            reset();

            // check initial state
            assertTrue(estimator.isHomogeneousLinearSolverUsed());
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var distance = position.distanceTo(estimatedPosition);
            if (distance >= LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            numValid++;
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        final var estimator = new LinearRangingAndRssiPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoErrorInhomogeneous() throws LockedException, NotReadyException, PositionEstimationException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
            }

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new LinearRangingAndRssiPositionEstimator2D(sources, fingerprint, this);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();

            // check initial state
            assertFalse(estimator.isHomogeneousLinearSolverUsed());
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }

        // force NotReadyException
        final var estimator = new LinearRangingAndRssiPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithErrorInhomogeneous() throws LockedException, NotReadyException, PositionEstimationException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var error = errorRandomizer.nextDouble();
                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent)) + error;

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
            }

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new LinearRangingAndRssiPositionEstimator2D(sources, fingerprint, this);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();

            // check initial state
            assertFalse(estimator.isHomogeneousLinearSolverUsed());
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var distance = position.distanceTo(estimatedPosition);
            if (distance >= LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            numValid++;
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        final var estimator = new LinearRangingAndRssiPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(final RangingAndRssiPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((LinearRangingAndRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(final RangingAndRssiPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((LinearRangingAndRssiPositionEstimator2D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private static double receivedPower(final double equivalentTransmittedPower, final double distance,
                                        final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final var k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k / Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(final LinearRangingAndRssiPositionEstimator2D estimators) {
        assertThrows(LockedException.class, () -> estimators.setHomogeneousLinearSolverUsed(false));
        assertThrows(LockedException.class, () -> estimators.setSources(null));
        assertThrows(LockedException.class, () -> estimators.setFingerprint(null));
        assertThrows(LockedException.class, () -> estimators.setListener(null));
        assertThrows(LockedException.class, estimators::estimate);
    }
}
