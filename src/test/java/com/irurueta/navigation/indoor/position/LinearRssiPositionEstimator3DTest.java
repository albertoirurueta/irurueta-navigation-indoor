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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated3D;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.Test;

public class LinearRssiPositionEstimator3DTest implements
        RssiPositionEstimatorListener<Point3D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final int MIN_SOURCES = 4;
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
    public void testConstructor() {
        // empty constructor
        LinearRssiPositionEstimator3D estimator = new LinearRssiPositionEstimator3D();

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // constructor with sources
        final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY, new InhomogeneousPoint3D()));
        }
        estimator = new LinearRssiPositionEstimator3D(sources);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRssiPositionEstimator3D((List<WifiAccessPointLocated3D>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearRssiPositionEstimator3D(new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // constructor with fingerprint
        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        estimator = new LinearRssiPositionEstimator3D(fingerprint);

        // check default value
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRssiPositionEstimator3D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // constructor with sources and fingerprint
        estimator = new LinearRssiPositionEstimator3D(sources, fingerprint);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRssiPositionEstimator3D(null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearRssiPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearRssiPositionEstimator3D(sources,
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // constructor with listener
        estimator = new LinearRssiPositionEstimator3D(this);

        // check
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // constructor with sources and listener
        estimator = new LinearRssiPositionEstimator3D(sources, this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRssiPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearRssiPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // constructor with fingerprint and listener
        estimator = new LinearRssiPositionEstimator3D(fingerprint, this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRssiPositionEstimator3D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // constructor with sources, fingerprint and listener
        estimator = new LinearRssiPositionEstimator3D(sources, fingerprint, this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRssiPositionEstimator3D(null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearRssiPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearRssiPositionEstimator3D(sources, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetSources() throws LockedException {
        final LinearRssiPositionEstimator3D estimator = new LinearRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY, new InhomogeneousPoint3D()));
        }

        estimator.setSources(sources);

        // check
        assertSame(sources, estimator.getSources());

        // force IllegalArgumentException
        try {
            estimator.setSources(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setSources(new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        final LinearRssiPositionEstimator3D estimator = new LinearRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(fingerprint, estimator.getFingerprint());

        // force IllegalArgumentException
        try {
            estimator.setFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final LinearRssiPositionEstimator3D estimator = new LinearRssiPositionEstimator3D();

        // check default size
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testEstimateNoErrorHomogeneous() throws LockedException, NotReadyException,
            PositionEstimationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final LinearRssiPositionEstimator3D estimator =
                    new LinearRssiPositionEstimator3D(sources, fingerprint, this);

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

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, 5.0 * ABSOLUTE_ERROR));
        }

        // force NotReadyException
        final LinearRssiPositionEstimator3D estimator = new LinearRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithErrorHomogeneous() throws LockedException, NotReadyException,
            PositionEstimationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                0.0, ERROR_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double error = errorRandomizer.nextDouble();
                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent)) + error;

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final LinearRssiPositionEstimator3D estimator =
                    new LinearRssiPositionEstimator3D(sources, fingerprint, this);

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

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            final double distance = position.distanceTo(estimatedPosition);
            if (distance >= LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            numValid++;
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        final LinearRssiPositionEstimator3D estimator = new LinearRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoErrorInhomogeneous() throws LockedException, NotReadyException,
            PositionEstimationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final LinearRssiPositionEstimator3D estimator =
                    new LinearRssiPositionEstimator3D(sources, fingerprint, this);
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

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }

        // force NotReadyException
        final LinearRssiPositionEstimator3D estimator = new LinearRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithErrorInhomogeneous() throws LockedException, NotReadyException,
            PositionEstimationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                0.0, ERROR_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double error = errorRandomizer.nextDouble();
                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent)) + error;

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final LinearRssiPositionEstimator3D estimator =
                    new LinearRssiPositionEstimator3D(sources, fingerprint, this);
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

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            final double distance = position.distanceTo(estimatedPosition);
            if (distance >= LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            numValid++;
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        final LinearRssiPositionEstimator3D estimator = new LinearRssiPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(final RssiPositionEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((LinearRssiPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateEnd(final RssiPositionEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((LinearRssiPositionEstimator3D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private double receivedPower(final double equivalentTransmittedPower,
                                 final double distance, final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(final LinearRssiPositionEstimator3D estimators) {
        try {
            estimators.setHomogeneousLinearSolverUsed(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimators.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimators.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimators.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimators.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
