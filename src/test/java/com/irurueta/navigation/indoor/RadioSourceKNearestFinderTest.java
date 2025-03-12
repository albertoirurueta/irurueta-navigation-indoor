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
package com.irurueta.navigation.indoor;

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.KDTree2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

import static com.irurueta.navigation.indoor.Utils.dBmToPower;
import static com.irurueta.navigation.indoor.Utils.powerTodBm;
import static org.junit.jupiter.api.Assertions.*;

class RadioSourceKNearestFinderTest {

    private static final Logger LOGGER = Logger.getLogger(RadioSourceKNearestFinderTest.class.getName());

    private static final int MIN_RSSI = -100;
    private static final int MAX_RSSI = -50;

    private static final int MIN_AP = 1;
    private static final int MAX_AP = 5;

    private static final int MIN_FINGERPRINTS = 50;
    private static final int MAX_FINGERPRINTS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double SEPARATION_POS = 1.0;

    private static final double ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final double SPEED_OF_LIGHT = 3e8; // (m/s)

    private static final int MAX_K = 20;

    @Test
    void testConstructorWifi2D() {
        final var fingerprints =
                new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
        final var finder = new RadioSourceKNearestFinder<>(fingerprints);

        // check
        assertSame(finder.getFingerprints(), fingerprints);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new RadioSourceKNearestFinder<Point2D, WifiAccessPoint>(null));
    }

    @Test
    void testConstructorWifi3D() {
        final var fingerprints =
                new ArrayList<RssiFingerprintLocated3D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
        final var finder = new RadioSourceKNearestFinder<>(fingerprints);

        // check
        assertSame(fingerprints, finder.getFingerprints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new RadioSourceKNearestFinder<Point3D, WifiAccessPoint>(null));
    }

    @Test
    void testConstructorBeacon2D() {
        final var fingerprints = new ArrayList<RssiFingerprintLocated2D<Beacon, RssiReading<Beacon>>>();
        final var finder = new RadioSourceKNearestFinder<>(fingerprints);

        // check
        assertSame(fingerprints, finder.getFingerprints());

        //Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new RadioSourceKNearestFinder<Point2D, Beacon>(null));
    }

    @Test
    void testConstructorBeacon3D() {
        final var fingerprints = new ArrayList<RssiFingerprintLocated3D<Beacon, RssiReading<Beacon>>>();
        final var finder = new RadioSourceKNearestFinder<>(fingerprints);

        // check
        assertSame(fingerprints, finder.getFingerprints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new RadioSourceKNearestFinder<Point3D, Beacon>(null));
    }

    @Test
    void testFindNearestTo() {
        var numValid = 0;
        var avgValidSignalDistance = 0.0;
        var avgValidDistance = 0.0;
        var avgSignalDistance = 0.0;
        var avgDistance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var fingerprintsPositions = new Point2D[numFingerprints];
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                for (var j = 0; j < numAccessPoints; j++) {
                    final var distance = fingerprintsPositions[i].distanceTo(accessPointPositions[j]);
                    final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                            accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final var tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            // find the closest fingerprint
            final var closestFingerprint1 = finder.findNearestTo(fingerprint);
            final var closestFingerprint2 = RadioSourceKNearestFinder.findNearestTo(fingerprint, fingerprints);

            final var closestFingerprints1 = finder.findKNearestTo(fingerprint, 1);
            final var closestFingerprints2 =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestSqrDistances = new ArrayList<Double>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints2, nearestSqrDistances);

            final var closestFingerprint3 = closestFingerprints1.get(0);
            final var closestFingerprint4 = closestFingerprints2.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint1.getPosition().distanceTo(position);

            final var nearestPosition = tree.nearestPoint(position);
            if (!nearestPosition.equals(closestFingerprint1.getPosition(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(nearestPosition.equals(closestFingerprint1.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(closestFingerprint2.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(closestFingerprint3.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(closestFingerprint4.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestSqrDistances.get(0) >= 0.0);

            avgValidSignalDistance += nearestSqrDistances.get(0);
            avgValidDistance += closestFingerprint1.getPosition().distanceTo(position);
            numValid++;

            // force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> RadioSourceKNearestFinder.findNearestTo(null,
                    fingerprints));
            assertThrows(IllegalArgumentException.class, () -> RadioSourceKNearestFinder.findNearestTo(fingerprint,
                    null));
        }

        assertTrue(numValid > 0);

        avgSignalDistance /= TIMES;
        avgDistance /= TIMES;
        avgValidSignalDistance /= numValid;
        avgValidDistance /= numValid;

        LOGGER.log(Level.INFO, "Average signal distance: {0}", avgSignalDistance);
        LOGGER.log(Level.INFO, "Average position distance: {0}", avgDistance);
        LOGGER.log(Level.INFO, "Average valid signal distance: {0}", avgValidSignalDistance);
        LOGGER.log(Level.INFO, "Average valid position distance: {0}", avgValidDistance);
    }

    @Test
    void testFindKNearestTo() {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var fingerprintsPositions = new Point2D[numFingerprints];
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                for (var j = 0; j < numAccessPoints; j++) {
                    final var distance = fingerprintsPositions[i].distanceTo(accessPointPositions[j]);
                    final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                            accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final var tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            // find the k-closest fingerprints
            final var k = randomizer.nextInt(2, numFingerprints);
            final var kClosestFingerprints1 = finder.findKNearestTo(fingerprint, k);
            final var kClosestFingerprints2 = RadioSourceKNearestFinder.findKNearestTo(fingerprint, fingerprints, k);

            final var kClosestFingerprints3 =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestSqrDistances3 = new ArrayList<Double>();
            finder.findKNearestTo(fingerprint, k, kClosestFingerprints3, nearestSqrDistances3);

            final var kClosestFingerprints4 =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestSqrDistances4 = new ArrayList<Double>();
            RadioSourceKNearestFinder.findKNearestTo(fingerprint, fingerprints, k, kClosestFingerprints4,
                    nearestSqrDistances4);

            // check
            assertEquals(kClosestFingerprints1.size(), k);
            assertEquals(kClosestFingerprints2.size(), k);
            assertEquals(kClosestFingerprints3.size(), k);
            assertEquals(kClosestFingerprints4.size(), k);
            assertEquals(nearestSqrDistances3.size(), k);
            assertEquals(nearestSqrDistances4.size(), k);

            for (var i = 1; i < k; i++) {
                assertTrue(nearestSqrDistances3.get(i - 1) <= nearestSqrDistances3.get(i));
                assertTrue(nearestSqrDistances4.get(i - 1) <= nearestSqrDistances4.get(i));
            }

            final var nearestPosition = tree.nearestPoint(position);

            // check that k nearest fingerprints contains closest one
            var found = false;
            for (var i = 0; i < k; i++) {
                final var fingerprint1 = kClosestFingerprints1.get(i);
                final var fingerprint2 = kClosestFingerprints2.get(i);
                final var fingerprint3 = kClosestFingerprints3.get(i);
                final var fingerprint4 = kClosestFingerprints4.get(i);

                assertEquals(fingerprint1.getPosition(), fingerprint2.getPosition());
                assertEquals(fingerprint2.getPosition(), fingerprint3.getPosition());
                assertEquals(fingerprint3.getPosition(), fingerprint4.getPosition());
                assertEquals(nearestSqrDistances3.get(i), nearestSqrDistances4.get(i));

                if (fingerprint1.getPosition().equals(nearestPosition, ABSOLUTE_ERROR)) {
                    found = true;
                    break;
                }
            }

            if (found) {
                numValid++;
            }

            // force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> RadioSourceKNearestFinder.findKNearestTo(null,
                    fingerprints, k, kClosestFingerprints4, nearestSqrDistances4));
            assertThrows(IllegalArgumentException.class, () -> RadioSourceKNearestFinder.findKNearestTo(fingerprint,
                    null, k, kClosestFingerprints4, nearestSqrDistances4));
            assertThrows(IllegalArgumentException.class, () -> RadioSourceKNearestFinder.findKNearestTo(fingerprint,
                    fingerprints, k, null, nearestSqrDistances4));
            assertThrows(IllegalArgumentException.class, () -> RadioSourceKNearestFinder.findKNearestTo(fingerprint,
                    fingerprints, k, kClosestFingerprints4, null));
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testFindKNearestToAll() {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var fingerprintsPositions = new Point2D[numFingerprints];
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                for (var j = 0; j < numAccessPoints; j++) {
                    final var distance = fingerprintsPositions[i].distanceTo(accessPointPositions[j]);
                    final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                            accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            final var closestFingerprints =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestSqrDistances = new ArrayList<Double>();
            RadioSourceKNearestFinder.findKNearestTo(fingerprint, fingerprints, numFingerprints, closestFingerprints,
                    nearestSqrDistances);

            // check
            for (var i = 1; i < numFingerprints; i++) {
                assertTrue(nearestSqrDistances.get(i - 1) <= nearestSqrDistances.get(i));
            }

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testFindNearestToWithError() {
        var numValid = 0;
        var avgValidSignalDistance = 0.0;
        var avgValidDistance = 0.0;
        var avgSignalDistance = 0.0;
        var avgDistance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var fingerprintsPositions = new Point2D[numFingerprints];
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                for (var j = 0; j < numAccessPoints; j++) {
                    final var distance = fingerprintsPositions[i].distanceTo(accessPointPositions[j]);
                    final var error = errorRandomizer.nextDouble();
                    final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                            accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final var tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var error = errorRandomizer.nextDouble();
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            // find the closest fingerprint
            final var closestFingerprint1 = finder.findNearestTo(fingerprint);
            final var closestFingerprint2 = RadioSourceKNearestFinder.findNearestTo(fingerprint, fingerprints);

            final var closestFingerprints1 = finder.findKNearestTo(fingerprint, 1);
            final var closestFingerprints2 =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestSqrDistances = new ArrayList<Double>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints2, nearestSqrDistances);

            final var closestFingerprint3 = closestFingerprints1.get(0);
            final var closestFingerprint4 = closestFingerprints2.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint1.getPosition().distanceTo(position);

            final var nearestPosition = tree.nearestPoint(position);
            if (!nearestPosition.equals(closestFingerprint1.getPosition(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(nearestPosition.equals(closestFingerprint1.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(closestFingerprint2.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(closestFingerprint3.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(closestFingerprint4.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestSqrDistances.get(0) >= 0.0);

            avgValidSignalDistance += nearestSqrDistances.get(0);
            avgValidDistance += closestFingerprint1.getPosition().distanceTo(position);
            numValid++;
        }

        assertTrue(numValid > 0);

        avgSignalDistance /= TIMES;
        avgDistance /= TIMES;
        avgValidSignalDistance /= numValid;
        avgValidDistance /= numValid;

        LOGGER.log(Level.INFO, "Average signal distance: {0}", avgSignalDistance);
        LOGGER.log(Level.INFO, "Average position distance: {0}", avgDistance);
        LOGGER.log(Level.INFO, "Average valid signal distance: {0}", avgValidSignalDistance);
        LOGGER.log(Level.INFO, "Average valid position distance: {0}", avgValidDistance);
    }

    @Test
    void testFindBestK() {
        var avgK = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var fingerprintsPositions = new Point2D[numFingerprints];
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                for (var j = 0; j < numAccessPoints; j++) {
                    final var distance = fingerprintsPositions[i].distanceTo(accessPointPositions[j]);
                    final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                            accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final var tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            final var nearestPosition = tree.nearestPoint(position);

            for (var k = 1; k < numFingerprints; k++) {
                final var nearestFingerprints = finder.findKNearestTo(fingerprint, k);
                var found = false;
                for (var i = 0; i < k; i++) {
                    if (nearestFingerprints.get(i).getPosition().equals(nearestPosition, ABSOLUTE_ERROR)) {
                        avgK += k;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }
        }

        assertTrue(avgK > 0.0);

        avgK /= TIMES;
        LOGGER.log(Level.INFO, "Average best k: {0}", avgK);
    }

    @Test
    void testFindBestKWithError() {
        var avgK = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var fingerprintsPositions = new Point2D[numFingerprints];
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                for (var j = 0; j < numAccessPoints; j++) {
                    final var distance = fingerprintsPositions[i].distanceTo(accessPointPositions[j]);
                    final var error = errorRandomizer.nextDouble();
                    final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                            accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final var tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var error = errorRandomizer.nextDouble();
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            final var nearestPosition = tree.nearestPoint(position);

            for (var k = 1; k < numFingerprints; k++) {
                final var nearestFingerprints = finder.findKNearestTo(fingerprint, k);
                var found = false;
                for (var i = 0; i < k; i++) {
                    if (nearestFingerprints.get(i).getPosition().equals(nearestPosition, ABSOLUTE_ERROR)) {
                        avgK += k;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }
        }

        assertTrue(avgK > 0.0);

        avgK /= TIMES;
        LOGGER.log(Level.INFO, "Average best k: {0}", avgK);
    }

    @Test
    void testFindNearestToUniformFingerprints() {
        var avgSignalDistance = 0.0;
        var avgDistance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (var y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    final var fingerprintPosition = new InhomogeneousPoint2D(x, y);

                    final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                    for (var j = 0; j < numAccessPoints; j++) {
                        final var distance = fingerprintPosition.distanceTo(accessPointPositions[j]);
                        final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                                accessPoints[j].getFrequency()));
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            // find the closest fingerprint
            final var closestFingerprints =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestSqrDistances = new ArrayList<Double>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints, nearestSqrDistances);

            final var closestFingerprint = closestFingerprints.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint.getPosition().distanceTo(position);
        }

        assertTrue(avgSignalDistance > 0.0);
        assertTrue(avgDistance > 0.0);

        avgSignalDistance /= TIMES;
        avgDistance /= TIMES;
        LOGGER.log(Level.INFO, "Average signal distance: {0}", avgSignalDistance);
        LOGGER.log(Level.INFO, "Average position distance: {0}", avgDistance);
    }

    @Test
    void testFindNearestToWithErrorUniformFingerprints() {
        var avgSignalDistance = 0.0;
        var avgDistance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (var y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    final var fingerprintPosition = new InhomogeneousPoint2D(x, y);

                    final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                    for (var j = 0; j < numAccessPoints; j++) {
                        final var distance = fingerprintPosition.distanceTo(accessPointPositions[j]);
                        final var error = errorRandomizer.nextDouble();
                        final var rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var error = errorRandomizer.nextDouble();
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            // find the closest fingerprint
            final var closestFingerprints =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestSqrDistances = new ArrayList<Double>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints, nearestSqrDistances);

            final var closestFingerprint = closestFingerprints.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint.getPosition().distanceTo(position);
        }

        assertTrue(avgSignalDistance > 0.0);
        assertTrue(avgDistance > 0.0);

        avgSignalDistance /= TIMES;
        avgDistance /= TIMES;
        LOGGER.log(Level.INFO, "Average signal distance: {0}", avgSignalDistance);
        LOGGER.log(Level.INFO, "Average position distance: {0}", avgDistance);
    }

    @Test
    void testFindBestKUniformFingerprints() {
        var avgK = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final var fingerprintsPositionsList = new ArrayList<Point2D>();
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (var y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    final var fingerprintPosition = new InhomogeneousPoint2D(x, y);
                    fingerprintsPositionsList.add(fingerprintPosition);

                    final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                    for (var j = 0; j < numAccessPoints; j++) {
                        final var distance = fingerprintPosition.distanceTo(accessPointPositions[j]);
                        final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                                accessPoints[j].getFrequency()));
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final var tree = new KDTree2D(fingerprintsPositionsList);

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            final var nearestPosition = tree.nearestPoint(position);

            final var numFingerprints = fingerprints.size();
            final var maxK = Math.min(numFingerprints, MAX_K);
            for (var k = 1; k < maxK; k++) {
                final var nearestFingerprints = finder.findKNearestTo(fingerprint, k);
                var found = false;
                for (var i = 0; i < k; i++) {
                    if (nearestFingerprints.get(i).getPosition().equals(nearestPosition, ABSOLUTE_ERROR)) {
                        avgK += k;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }
        }

        assertTrue(avgK > 0.0);

        avgK /= TIMES;
        LOGGER.log(Level.INFO, "Average best k: {0}", avgK);
    }

    @Test
    void testFindBestKWithErrorUniformFingerprints() {
        var avgK = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            final var randomizer = new UniformRandomizer();

            final var numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final var accessPointPositions = new Point2D[numAccessPoints];
            final var transmittedPower = new double[numAccessPoints];
            final var accessPoints = new WifiAccessPoint[numAccessPoints];
            for (var i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(dBmToPower(MIN_RSSI), dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final var fingerprintsPositionsList = new ArrayList<Point2D>();
            final var fingerprints =
                    new ArrayList<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>();
            for (var x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (var y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    final var fingerprintPosition = new InhomogeneousPoint2D(x, y);
                    fingerprintsPositionsList.add(fingerprintPosition);

                    final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
                    for (var j = 0; j < numAccessPoints; j++) {
                        final var distance = fingerprintPosition.distanceTo(accessPointPositions[j]);
                        final var error = errorRandomizer.nextDouble();
                        final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                                accessPoints[j].getFrequency())) + error;
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final var tree = new KDTree2D(fingerprintsPositionsList);

            // generate measurement at random position
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numAccessPoints; i++) {
                final var distance = position.distanceTo(accessPointPositions[i]);
                final var error = errorRandomizer.nextDouble();
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            final var nearestPosition = tree.nearestPoint(position);

            final var numFingerprints = fingerprints.size();
            final var maxK = Math.min(numFingerprints, MAX_K);
            for (var k = 1; k < maxK; k++) {
                final var nearestFingerprints = finder.findKNearestTo(fingerprint, k);
                var found = false;
                for (var i = 0; i < k; i++) {
                    if (nearestFingerprints.get(i).getPosition().equals(nearestPosition, ABSOLUTE_ERROR)) {
                        avgK += k;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }
        }

        assertTrue(avgK > 0.0);

        avgK /= TIMES;
        LOGGER.log(Level.INFO, "Average best k: {0}", avgK);
    }

    private static double receivedPower(final double equivalentTransmittedPower, final double distance,
                                        final double frequency) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final var k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), 2.0);
        return equivalentTransmittedPower * k / (distance * distance);
    }
}
