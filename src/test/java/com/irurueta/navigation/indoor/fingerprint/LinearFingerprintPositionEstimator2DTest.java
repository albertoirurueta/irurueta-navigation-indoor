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
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class LinearFingerprintPositionEstimator2DTest implements FingerprintPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            LinearFingerprintPositionEstimator2DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final int MIN_FINGERPRINTS = 100;
    private static final int MAX_FINGERPRINTS = 1000;

    private static final double RSSI_BIAS = 1.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double ERROR_STD = 1.0e-1;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    @Test
    public void testConstructor() {
        // test empty constructor
        LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertFalse(estimator.isMeansFromFingerprintReadingsRemoved());

        // test constructor with listener
        estimator = new LinearFingerprintPositionEstimator2D(this);

        // check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertFalse(estimator.isMeansFromFingerprintReadingsRemoved());

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RssiReading<RadioSource>> readings = new ArrayList<>();
        for (int i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i, FREQUENCY);
            final double rssi = randomizer.nextDouble();

            final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                new RssiFingerprintLocated2D<>(readings, Point2D.create());

        final List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                new ArrayList<>();
        locatedFingerprints.add(locatedFingerprint);

        final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);

        final List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
        for (int i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final WifiAccessPointLocated2D source = new WifiAccessPointLocated2D("bssid" + 1,
                    FREQUENCY, Point2D.create());
            sources.add(source);
        }

        // test constructor with located fingerprints, fingerprint and sources
        estimator = new LinearFingerprintPositionEstimator2D(locatedFingerprints, fingerprint, sources);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertFalse(estimator.isMeansFromFingerprintReadingsRemoved());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearFingerprintPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearFingerprintPositionEstimator2D(null,
                    fingerprint, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearFingerprintPositionEstimator2D(locatedFingerprints,
                    null, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearFingerprintPositionEstimator2D(locatedFingerprints,
                    fingerprint, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with located fingerprints, fingerprint, sources and listener
        estimator = new LinearFingerprintPositionEstimator2D(locatedFingerprints,
                fingerprint, sources, this);

        // check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(sources, estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertFalse(estimator.isMeansFromFingerprintReadingsRemoved());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearFingerprintPositionEstimator2D(
                    new ArrayList<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>>(),
                    fingerprint, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearFingerprintPositionEstimator2D(null,
                    fingerprint, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearFingerprintPositionEstimator2D(locatedFingerprints,
                    null, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LinearFingerprintPositionEstimator2D(locatedFingerprints,
                    fingerprint, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetLocatedFingerprints() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default value
        assertNull(estimator.getLocatedFingerprints());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RssiReading<RadioSource>> readings = new ArrayList<>();
        for (int i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i, FREQUENCY);
            final double rssi = randomizer.nextDouble();

            final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                new RssiFingerprintLocated2D<>(readings, Point2D.create());

        final List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                new ArrayList<>();
        locatedFingerprints.add(locatedFingerprint);

        estimator.setLocatedFingerprints(locatedFingerprints);

        // check correctness
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());

        // force IllegalArgumentException
        try {
            estimator.setLocatedFingerprints(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setLocatedFingerprints(
                    new ArrayList<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RssiReading<RadioSource>> readings = new ArrayList<>();
        for (int i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i, FREQUENCY);
            final double rssi = randomizer.nextDouble();

            final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);
        estimator.setFingerprint(fingerprint);

        // check correctness
        assertSame(fingerprint, estimator.getFingerprint());

        // force IllegalArgumentException
        try {
            estimator.setFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMinMaxNearestFingerprints() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default values
        assertEquals(1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());

        // set new values
        estimator.setMinMaxNearestFingerprints(2, 3);

        // check
        assertEquals(2, estimator.getMinNearestFingerprints());
        assertEquals(3, estimator.getMaxNearestFingerprints());

        // force IllegalArgumentException
        try {
            estimator.setMinMaxNearestFingerprints(0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setMinMaxNearestFingerprints(2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPathLossExponent() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default value
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pathLossExponent = randomizer.nextDouble();

        estimator.setPathLossExponent(pathLossExponent);

        // check correctness
        assertEquals(pathLossExponent, estimator.getPathLossExponent(), 0.0);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetSources() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default values
        assertNull(estimator.getSources());

        // set new value
        final List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
        estimator.setSources(sources);

        // check
        assertSame(estimator.getSources(), sources);

        // force IllegalArgumentException
        try {
            estimator.setSources(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetUseSourcesPathLossExponentWhenAvailable() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default value
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        // set new value
        estimator.setUseSourcesPathLossExponentWhenAvailable(false);

        // check
        assertFalse(estimator.getUseSourcesPathLossExponentWhenAvailable());
    }

    @Test
    public void testGetSetUseNoMeanNearestFingerprintFinder() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default value
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());

        // set new value
        estimator.setUseNoMeanNearestFingerprintFinder(false);

        // check
        assertFalse(estimator.getUseNoMeanNearestFingerprintFinder());
    }

    @Test
    public void testIsSetMeansFromFingerprintReadingsRemoved() throws LockedException {
        final LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D();

        // check default value
        assertFalse(estimator.isMeansFromFingerprintReadingsRemoved());

        // set new value
        estimator.setMeansFromFingerprintReadingsRemoved(true);

        // check
        assertTrue(estimator.isMeansFromFingerprintReadingsRemoved());
    }

    @Test
    public void testEstimateWithoutErrorAndWithoutBias() throws LockedException, NotReadyException,
            FingerprintEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // build sources
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final WifiAccessPointWithPowerAndLocated2D accessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (final RadioSourceLocated<Point2D> source : sources) {
                    final double distance = source.getPosition().distanceTo(position);
                    final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                            getTransmittedPower();

                    final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                    final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated2D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

            final List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (final RadioSourceLocated<Point2D> source : sources) {
                final double distance = source.getPosition().distanceTo(position);
                final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                        getTransmittedPower();

                final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find real closest fingerprint based on location
            RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point2D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                final Point2D fingerprintPosition = locatedFingerprint.getPosition();
                final double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            final double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;

            // find the closest fingerprint based on RSSI without mean
            final RadioSourceNoMeanKNearestFinder<Point2D, RadioSource> noMeanFinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprintNoMean =
                    noMeanFinder.findNearestTo(fingerprint);
            final Point2D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            final double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;

            // find the closest fingerprint based on RSSI
            final RadioSourceKNearestFinder<Point2D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            final Point2D rssiClosestPosition = nearestFingerprint.getPosition();

            final double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;

            // create estimator with means removed on finder and fingerprints
            LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on finder
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on readings
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means not removed
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            final double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            // find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }

        }

        LOGGER.log(Level.INFO, "Results without error and without bias");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double) numBestIsNoMeanRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double) numBestIsRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double) numBestIsNoMeansEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double) numBestIsNoMeanFinderEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double) numBestIsNoMeanReadingsEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double) numBestIsEstimatedPosition / (double) TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m", avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m", avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m", avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m", avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        final int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        // check that best result is obtained when means are removed to find
        // the closest fingerprints, but not removed for readings
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertEquals(bestNum, numBestIsNoMeanReadingsEstimatedPosition);
        assertEquals(4, bestPos);
    }

    @Test
    public void testEstimateWithBias() throws LockedException,
            NotReadyException, FingerprintEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // build sources
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final WifiAccessPointWithPowerAndLocated2D accessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (final RadioSourceLocated<Point2D> source : sources) {
                    final double distance = source.getPosition().distanceTo(position);
                    final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                            getTransmittedPower();

                    final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                    final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated2D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

            final List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (final RadioSourceLocated<Point2D> source : sources) {
                final double distance = source.getPosition().distanceTo(position);
                final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                        getTransmittedPower();

                final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT)) +
                        RSSI_BIAS;
                final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find real closest fingerprint based on location
            RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point2D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                final Point2D fingerprintPosition = locatedFingerprint.getPosition();
                final double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            final double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;

            // find the closest fingerprint based on RSSI without mean
            final RadioSourceNoMeanKNearestFinder<Point2D, RadioSource> noMeanFinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprintNoMean =
                    noMeanFinder.findNearestTo(fingerprint);
            final Point2D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            final double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;

            // find the closest fingerprint based on RSSI
            final RadioSourceKNearestFinder<Point2D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            final Point2D rssiClosestPosition = nearestFingerprint.getPosition();

            final double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;

            // create estimator with means removed on finder and fingerprints
            LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on finder
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on readings
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means not removed
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            final double[] errors = new double[]{
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            // find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }
        }

        LOGGER.log(Level.INFO, "Results with bias");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double) numBestIsNoMeanRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double) numBestIsRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double) numBestIsNoMeansEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double) numBestIsNoMeanFinderEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double) numBestIsNoMeanReadingsEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double) numBestIsEstimatedPosition / (double) TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m", avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m", avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m", avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m", avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        final int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        // check that best result is obtained when means are removed from both
        // fingerprint finder and readings to account for possible biases between devices
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertEquals(bestNum, numBestIsNoMeansEstimatedPosition);
        assertEquals(2, bestPos);
    }

    @Test
    public void testEstimateWithError() throws LockedException,
            NotReadyException, FingerprintEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            // build sources
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final WifiAccessPointWithPowerAndLocated2D accessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (final RadioSourceLocated<Point2D> source : sources) {
                    final double distance = source.getPosition().distanceTo(position);
                    final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                            getTransmittedPower();

                    final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                    final double rssiError = errorRandomizer.nextDouble();
                    final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi + rssiError);
                    readings.add(reading);
                }

                final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated2D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

            final List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (final RadioSourceLocated<Point2D> source : sources) {
                final double distance = source.getPosition().distanceTo(position);
                final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                        getTransmittedPower();

                final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                final double rssiError = errorRandomizer.nextDouble();
                final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi + rssiError);
                readings.add(reading);
            }

            final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find real closest fingerprint based on location
            RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point2D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                final Point2D fingerprintPosition = locatedFingerprint.getPosition();
                final double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            final double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;

            // find the closest fingerprint based on RSSI without mean
            final RadioSourceNoMeanKNearestFinder<Point2D, RadioSource> noMeanFinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprintNoMean =
                    noMeanFinder.findNearestTo(fingerprint);
            final Point2D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            final double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;

            // find the closest fingerprint based on RSSI
            final RadioSourceKNearestFinder<Point2D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            final Point2D rssiClosestPosition = nearestFingerprint.getPosition();

            final double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;

            // create estimator with means removed on finder and fingerprints
            LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on finder
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on readings
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means not removed
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            final double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            // find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }
        }

        LOGGER.log(Level.INFO, "Results with error bias");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double) numBestIsNoMeanRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double) numBestIsRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double) numBestIsNoMeansEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double) numBestIsNoMeanFinderEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double) numBestIsNoMeanReadingsEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double) numBestIsEstimatedPosition / (double) TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m", avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m", avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m", avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m", avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        final int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        // check that best result is obtained when means are removed from both
        // fingerprint finder and readings to account for possible biases between devices
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertEquals(bestNum, numBestIsNoMeanReadingsEstimatedPosition);
        assertEquals(4, bestPos);
    }

    @Test
    public void testEstimateWithErrorAndWithBias() throws LockedException,
            NotReadyException, FingerprintEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            // build sources
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final WifiAccessPointWithPowerAndLocated2D accessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (final RadioSourceLocated<Point2D> source : sources) {
                    final double distance = source.getPosition().distanceTo(position);
                    final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                            getTransmittedPower();

                    final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                    final double rssiError = errorRandomizer.nextDouble();
                    final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi + rssiError);
                    readings.add(reading);
                }

                final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated2D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

            final List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (final RadioSourceLocated<Point2D> source : sources) {
                final double distance = source.getPosition().distanceTo(position);
                final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                        getTransmittedPower();

                final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                final double rssiError = errorRandomizer.nextDouble();
                final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi + rssiError + RSSI_BIAS);
                readings.add(reading);
            }

            final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find real closest fingerprint based on location
            RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point2D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                final Point2D fingerprintPosition = locatedFingerprint.getPosition();
                final double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            final double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;

            // find the closest fingerprint based on RSSI without mean
            final RadioSourceNoMeanKNearestFinder<Point2D, RadioSource> noMeanFinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprintNoMean =
                    noMeanFinder.findNearestTo(fingerprint);
            final Point2D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            final double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;

            // find the closest fingerprint based on RSSI
            final RadioSourceKNearestFinder<Point2D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            final Point2D rssiClosestPosition = nearestFingerprint.getPosition();

            final double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;

            // create estimator with means removed on finder and fingerprints
            LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on finder
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on readings
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means not removed
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            final double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            // find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }
        }

        LOGGER.log(Level.INFO, "Results with error and bias");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double) numBestIsNoMeanRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double) numBestIsRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double) numBestIsNoMeansEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double) numBestIsNoMeanFinderEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double) numBestIsNoMeanReadingsEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double) numBestIsEstimatedPosition / (double) TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m", avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m", avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m", avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m", avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        final int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        // check that best result is obtained when means are removed from both
        // fingerprint finder and readings to account for possible biases between devices
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertTrue(bestNum == numBestIsNoMeanRssiPosition ||
                bestNum == numBestIsNoMeanFinderEstimatedPosition);
        assertTrue(bestPos == 0 || bestPos == 3);

    }

    @Test
    public void testEstimateWithOtherPathloss() throws LockedException,
            NotReadyException, FingerprintEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            // build sources
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);

                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final WifiAccessPointWithPowerAndLocated2D accessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm,
                                position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (final RadioSourceLocated<Point2D> source : sources) {
                    final double distance = source.getPosition().distanceTo(position);
                    final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                            getTransmittedPower();

                    final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, pathLossExponent));
                    final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated2D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

            final List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (final RadioSourceLocated<Point2D> source : sources) {
                final double distance = source.getPosition().distanceTo(position);
                final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                        getTransmittedPower();

                final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, pathLossExponent));
                final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find real closest fingerprint based on location
            RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point2D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                final Point2D fingerprintPosition = locatedFingerprint.getPosition();
                final double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            final double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;

            // find the closest fingerprint based on RSSI without mean
            final RadioSourceNoMeanKNearestFinder<Point2D, RadioSource> noMeanFinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprintNoMean =
                    noMeanFinder.findNearestTo(fingerprint);
            final Point2D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            final double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;

            // find the closest fingerprint based on RSSI
            final RadioSourceKNearestFinder<Point2D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            final Point2D rssiClosestPosition = nearestFingerprint.getPosition();

            final double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;

            // create estimator with means removed on finder and fingerprints
            LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);
            estimator.setPathLossExponent(pathLossExponent);
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on finder
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);
            estimator.setPathLossExponent(pathLossExponent);
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on readings
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);
            estimator.setPathLossExponent(pathLossExponent);
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            // create estimator with means not removed
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);
            estimator.setPathLossExponent(pathLossExponent);
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(1, estimator.getNearestFingerprints().size());

            final double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            // find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }

        }

        LOGGER.log(Level.INFO, "Results for different path loss exponents");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double) numBestIsNoMeanRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double) numBestIsRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double) numBestIsNoMeansEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double) numBestIsNoMeanFinderEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double) numBestIsNoMeanReadingsEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double) numBestIsEstimatedPosition / (double) TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m", avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m", avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m", avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m", avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        final int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        // check that best result is obtained when means are removed to find
        // the closest fingerprints, but not removed for readings
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertEquals(bestNum, numBestIsNoMeanReadingsEstimatedPosition);
        assertEquals(4, bestPos);
    }

    @Test
    public void testEstimateWithoutErrorAndWithoutBiasOneRadioSource() throws LockedException,
            NotReadyException, FingerprintEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // build sources
            final int numSources = 1;
            final List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final WifiAccessPointWithPowerAndLocated2D accessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                final List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (final RadioSourceLocated<Point2D> source : sources) {
                    final double distance = source.getPosition().distanceTo(position);
                    final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                            getTransmittedPower();

                    final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                    final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated2D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

            final List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (final RadioSourceLocated<Point2D> source : sources) {
                final double distance = source.getPosition().distanceTo(position);
                final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                        getTransmittedPower();

                final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, LinearFingerprintPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT));
                final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find real closest fingerprint based on location
            RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point2D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (final RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                final Point2D fingerprintPosition = locatedFingerprint.getPosition();
                final double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            final double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;

            // find the closest fingerprint based on RSSI without mean
            final RadioSourceNoMeanKNearestFinder<Point2D, RadioSource> noMeanFinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprintNoMean =
                    noMeanFinder.findNearestTo(fingerprint);
            final Point2D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            final double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;

            // find the closest fingerprint based on RSSI
            final RadioSourceKNearestFinder<Point2D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            final Point2D rssiClosestPosition = nearestFingerprint.getPosition();

            final double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;

            // create estimator with means removed on finder and fingerprints
            LinearFingerprintPositionEstimator2D estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point2D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(2, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on finder
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(2, estimator.getNearestFingerprints().size());

            // create estimator with means removed only on readings
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(2, estimator.getNearestFingerprints().size());

            // create estimator with means not removed
            estimator = new LinearFingerprintPositionEstimator2D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            final double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(2, estimator.getNearestFingerprints().size());

            final double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            // find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }
        }

        LOGGER.log(Level.INFO, "Results without error and without bias with a single radio source");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double) numBestIsNoMeanRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double) numBestIsRssiPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double) numBestIsNoMeansEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double) numBestIsNoMeanFinderEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double) numBestIsNoMeanReadingsEstimatedPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double) numBestIsEstimatedPosition / (double) TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m", avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m", avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m", avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m", avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);
    }

    @Override
    public void onEstimateStart(final FingerprintPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((LinearFingerprintPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(final FingerprintPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((LinearFingerprintPositionEstimator2D) estimator);
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

    private void checkLocked(final LinearFingerprintPositionEstimator2D estimator) {
        try {
            estimator.setLocatedFingerprints(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMinMaxNearestFingerprints(1, 1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPathLossExponent(2.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMeansFromFingerprintReadingsRemoved(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
