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
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
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

class WeightedKNearestNeighboursPositionSolver2DTest implements
        WeightedKNearestNeighboursPositionSolverListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            WeightedKNearestNeighboursPositionSolver2DTest.class.getName());

    private static final int MIN_RSSI = -100;
    private static final int MAX_RSSI = -50;

    private static final int MIN_AP = 1;
    private static final int MAX_AP = 5;

    private static final int MIN_FINGERPRINTS = 50;
    private static final int MAX_FINGERPRINTS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double SEPARATION_POS = 2.0;

    private static final double ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;
    private static final int SHORT_TIMES = 25;

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final double SPEED_OF_LIGHT = 3e8; // (m/s)

    private static final int MAX_K = 10;

    private int solveStart;
    private int solveEnd;

    @Test
    void testConstructor() {
        // test empty constructor
        var solver = new WeightedKNearestNeighboursPositionSolver2D();

        // check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNull(solver.getListener());
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, solver.getEpsilon(), 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());

        // test constructor with fingerprints and distances noinspection unchecked
        //noinspection unchecked
        final RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>[] fingerprints =
                new RssiFingerprintLocated2D[1];
        final var distances = new double[1];
        solver = new WeightedKNearestNeighboursPositionSolver2D(fingerprints, distances);

        // check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNull(solver.getListener());
        assertSame(fingerprints, solver.getFingerprints());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, solver.getEpsilon(), 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedKNearestNeighboursPositionSolver2D(
                null, distances));
        assertThrows(IllegalArgumentException.class, () -> new WeightedKNearestNeighboursPositionSolver2D(fingerprints,
                null));
        //noinspection unchecked
        assertThrows(IllegalArgumentException.class, () -> new WeightedKNearestNeighboursPositionSolver2D(
                new RssiFingerprintLocated2D[0], new double[0]));
        assertThrows(IllegalArgumentException.class, () -> new WeightedKNearestNeighboursPositionSolver2D(fingerprints,
                new double[2]));

        // test constructor with listener
        solver = new WeightedKNearestNeighboursPositionSolver2D(this);

        // check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertSame(this, solver.getListener());
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, solver.getEpsilon(), 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());

        // test constructor with fingerprints, distances and listener
        solver = new WeightedKNearestNeighboursPositionSolver2D(fingerprints, distances, this);

        // check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertSame(this, solver.getListener());
        assertSame(fingerprints, solver.getFingerprints());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, solver.getEpsilon(), 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedKNearestNeighboursPositionSolver2D(
                null, distances, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedKNearestNeighboursPositionSolver2D(fingerprints,
                null, this));
        //noinspection unchecked
        assertThrows(IllegalArgumentException.class, () -> new WeightedKNearestNeighboursPositionSolver2D(
                new RssiFingerprintLocated2D[0], new double[0], this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedKNearestNeighboursPositionSolver2D(fingerprints,
                new double[2], this));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var solver = new WeightedKNearestNeighboursPositionSolver2D();

        // check default value
        assertNull(solver.getListener());

        // set new value
        solver.setListener(this);

        // check
        assertSame(this, solver.getListener());
    }

    @Test
    void testSetFingerprintsAndDistances() throws LockedException {
        final var solver = new WeightedKNearestNeighboursPositionSolver2D();

        // check default values
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());

        // set new values
        //noinspection unchecked
        final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] fingerprints =
                new RssiFingerprintLocated[1];
        final var distances = new double[1];
        solver.setFingerprintsAndDistances(fingerprints, distances);

        // check
        assertSame(fingerprints, solver.getFingerprints());
        assertSame(distances, solver.getDistances());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setFingerprintsAndDistances(null,
                distances));
        assertThrows(IllegalArgumentException.class, () -> solver.setFingerprintsAndDistances(fingerprints,
                null));
        //noinspection unchecked
        assertThrows(IllegalArgumentException.class, () -> solver.setFingerprintsAndDistances(
                new RssiFingerprintLocated[0], new double[0]));
        assertThrows(IllegalArgumentException.class, () -> solver.setFingerprintsAndDistances(fingerprints,
                new double[2]));
    }

    @Test
    void testGetSetEpsilon() throws LockedException {
        final var solver = new WeightedKNearestNeighboursPositionSolver2D();

        // check default value
        assertEquals(WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, solver.getEpsilon(), 0.0);

        // set new value
        solver.setEpsilon(1.0);

        // check
        assertEquals(1.0, solver.getEpsilon(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setEpsilon(0.0));
    }

    @Test
    void testSolve1Fingerprint() throws NotReadyException, LockedException {
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
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            // find nearest fingerprint
            var k = 1;
            final var nearestFingerprintsList =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestDistancesList = new ArrayList<Double>();
            finder.findKNearestTo(fingerprint, k, nearestFingerprintsList, nearestDistancesList);

            //noinspection unchecked
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            final var nearestDistances = new double[k];
            for (var i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            final var solver = new WeightedKNearestNeighboursPositionSolver2D(nearestFingerprints, nearestDistances,
                    this);

            // solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);

            solver.solve();

            // check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);

            final var nearestPosition = tree.nearestPoint(position);
            final var estimatedPosition = solver.getEstimatedPosition();

            // estimated position is always equal to provided fingerprint when only one is provided
            assertEquals(estimatedPosition, nearestFingerprints[0].getPosition());

            if (nearestPosition.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                numValid++;
            }
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        final var solver = new WeightedKNearestNeighboursPositionSolver2D();
        assertThrows(NotReadyException.class, solver::solve);
    }

    @Test
    void testSolveKFingerprints() throws NotReadyException, LockedException {
        for (var k = 2; k < MAX_K; k++) {
            solveKFingerprints(k, 0.0);
        }
    }

    @Test
    void testSolveKFingerprintsWithError() throws NotReadyException, LockedException {
        for (var k = 2; k < MAX_K; k++) {
            solveKFingerprints(k, ERROR_STD);
        }
    }

    @Test
    void testFindBestK() throws NotReadyException, LockedException {
        findBestK(0.0);
    }

    @Test
    void testFindBestKWithError() throws NotReadyException, LockedException {
        findBestK(ERROR_STD);
    }

    @Test
    void testSolveKFingerprintsUniformFingerprints() throws NotReadyException, LockedException {
        for (var k = 2; k < MAX_K; k++) {
            solveKFingerprintsUniformFingerprints(k, 0.0);
        }
    }

    @Test
    void testSolveKFingerprintsWithErrorUniformFingerprints() throws NotReadyException, LockedException {
        for (var k = 2; k < MAX_K; k++) {
            solveKFingerprintsUniformFingerprints(k, ERROR_STD);
        }
    }

    @Test
    void testFindBestKUniformFingerprints() throws NotReadyException, LockedException {
        findBestKUniformFingerprints(0.0);
    }

    @Test
    void testFindBestKWithErrorUniformFingerprints() throws NotReadyException, LockedException {
        findBestKUniformFingerprints(ERROR_STD);
    }

    @Override
    public void onSolveStart(final WeightedKNearestNeighboursPositionSolver<Point2D> solver) {
        solveStart++;
        checkLocked((WeightedKNearestNeighboursPositionSolver2D) solver);
    }

    @Override
    public void onSolveEnd(final WeightedKNearestNeighboursPositionSolver<Point2D> solver) {
        solveEnd++;
        checkLocked((WeightedKNearestNeighboursPositionSolver2D) solver);
    }

    private void solveKFingerprints(final int k, final double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(0.0, errorStd);
        }

        var numValid = 0;
        var avgDistance = 0.0;
        var avgImprovedDistance = 0.0;
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
                    final var error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
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
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            // find nearest fingerprints
            final var nearestFingerprintsList =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestDistancesList = new ArrayList<Double>();
            finder.findKNearestTo(fingerprint, k, nearestFingerprintsList, nearestDistancesList);

            //noinspection unchecked
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            final var nearestDistances = new double[k];
            for (var i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            final var solver = new WeightedKNearestNeighboursPositionSolver2D(nearestFingerprints, nearestDistances,
                    this);

            // solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);

            solver.solve();

            // check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);

            final var nearestPosition = tree.nearestPoint(position);
            final var estimatedPosition = solver.getEstimatedPosition();

            // check if estimated position is closer to the actual position than nearest fingerprint
            final var distance = estimatedPosition.distanceTo(position);
            avgDistance += distance;

            if (distance <= nearestPosition.distanceTo(position)) {
                avgImprovedDistance += distance;
                numValid++;
            }
        }

        assertTrue(numValid > 0);

        avgDistance /= TIMES;
        avgImprovedDistance /= numValid;

        assertTrue(avgDistance > 0.0);
        assertTrue(avgImprovedDistance > 0.0);

        LOGGER.log(Level.INFO, "{0} of {1} estimated positions have improved with {2} neighbours",
                new Object[]{numValid, TIMES, k});
        LOGGER.log(Level.INFO, "Average error distance: {0} meters, Average improved error distance: {1} meters",
                new Object[]{avgDistance, avgImprovedDistance});
    }

    private void findBestK(final double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(0.0, errorStd);
        }

        var avgBestK = 0.0;
        var avgBestDistance = 0.0;
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
                    final var error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
                    final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                            accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final var finder = new RadioSourceKNearestFinder<>(fingerprints);

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

            var bestK = 0;
            var bestDistance = Double.MAX_VALUE;
            var maxK = Math.min(numFingerprints, MAX_K);
            for (var k = 1; k < maxK; k++) {
                final var nearestFingerprintsList =
                        new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
                final var nearestDistancesList = new ArrayList<Double>();
                finder.findKNearestTo(fingerprint, k, nearestFingerprintsList, nearestDistancesList);

                //noinspection unchecked
                final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[]
                        nearestFingerprints = new RssiFingerprintLocated[k];
                final var nearestDistances = new double[k];
                for (var i = 0; i < k; i++) {
                    nearestFingerprints[i] = nearestFingerprintsList.get(i);
                    nearestDistances[i] = nearestDistancesList.get(i);
                }

                final var solver = new WeightedKNearestNeighboursPositionSolver2D(nearestFingerprints,
                        nearestDistances);

                solver.solve();

                final var estimatedPosition = solver.getEstimatedPosition();

                final var distance = estimatedPosition.distanceTo(position);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestK = k;
                }
            }

            avgBestK += bestK;
            avgBestDistance += bestDistance;
        }

        avgBestK /= TIMES;
        avgBestDistance /= TIMES;

        assertTrue(avgBestK > 0.0);
        assertTrue(avgBestDistance > 0.0);

        LOGGER.log(Level.INFO, "Best number of neighbours: {0}, Average error distance: {1} meters",
                new Object[]{avgBestK, avgBestDistance});
    }

    private void solveKFingerprintsUniformFingerprints(int k, double errorStd) throws NotReadyException,
            LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(0.0, errorStd);
        }

        var numValid = 0;
        var avgDistance = 0.0;
        var avgImprovedDistance = 0.0;
        for (var t = 0; t < SHORT_TIMES; t++) {
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
                        final var error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
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
                final var rssi = powerTodBm(receivedPower(transmittedPower[i], distance,
                        accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final var fingerprint = new RssiFingerprint<>(readings);

            // find nearest fingerprints
            final var nearestFingerprintsList =
                    new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
            final var nearestDistancesList = new ArrayList<Double>();
            finder.findKNearestTo(fingerprint, k, nearestFingerprintsList, nearestDistancesList);

            //noinspection unchecked
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            final var nearestDistances = new double[k];
            for (var i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            final var solver = new WeightedKNearestNeighboursPositionSolver2D(nearestFingerprints, nearestDistances,
                    this);

            // solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);

            solver.solve();

            // check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);

            final var nearestPosition = tree.nearestPoint(position);
            final var estimatedPosition = solver.getEstimatedPosition();

            // check if estimated position is closer to the actual position than
            // nearest fingerprint
            final var distance = estimatedPosition.distanceTo(position);
            avgDistance += distance;

            if (distance <= nearestPosition.distanceTo(position)) {
                avgImprovedDistance += distance;
                numValid++;
            }
        }

        if (numValid > 0) {
            avgImprovedDistance /= numValid;
            avgDistance /= SHORT_TIMES;

            assertTrue(avgImprovedDistance > 0.0);
            assertTrue(avgDistance > 0.0);

            LOGGER.log(Level.INFO, "{0} of {1} estimated positions have improved with {2} neighbours",
                    new Object[]{numValid, SHORT_TIMES, k});
            LOGGER.log(Level.INFO,
                    "Average error distance: {0} meters, Average improved error distance: {1} meters",
                    new Object[]{avgDistance, avgImprovedDistance});
        }
    }

    private void findBestKUniformFingerprints(double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(0.0, errorStd);
        }

        var avgBestK = 0.0;
        var avgBestDistance = 0.0;
        for (var t = 0; t < SHORT_TIMES; t++) {
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
                        final var error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
                        final var rssi = powerTodBm(receivedPower(transmittedPower[j], distance,
                                accessPoints[j].getFrequency())) + error;
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

            var bestK = 0;
            var bestDistance = Double.MAX_VALUE;
            var numFingerprints = fingerprints.size();
            var maxK = Math.min(numFingerprints, MAX_K);
            for (var k = 1; k < maxK; k++) {
                final var nearestFingerprintsList =
                        new ArrayList<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>>();
                final var nearestDistancesList = new ArrayList<Double>();
                finder.findKNearestTo(fingerprint, k, nearestFingerprintsList, nearestDistancesList);

                // noinspection unchecked
                final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[]
                        nearestFingerprints = new RssiFingerprintLocated[k];
                final var nearestDistances = new double[k];
                for (var i = 0; i < k; i++) {
                    nearestFingerprints[i] = nearestFingerprintsList.get(i);
                    nearestDistances[i] = nearestDistancesList.get(i);
                }

                final var solver =
                        new WeightedKNearestNeighboursPositionSolver2D(nearestFingerprints, nearestDistances);

                solver.solve();

                final var estimatedPosition = solver.getEstimatedPosition();

                final var distance = estimatedPosition.distanceTo(position);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestK = k;
                }
            }

            avgBestK += bestK;
            avgBestDistance += bestDistance;
        }

        avgBestK /= SHORT_TIMES;
        avgBestDistance /= SHORT_TIMES;

        assertTrue(avgBestK > 0.0);
        assertTrue(avgBestDistance > 0.0);

        LOGGER.log(Level.INFO, "Best number of neighbours: {0}, Average error distance: {1} meters",
                new Object[]{avgBestK, avgBestDistance});
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }

    private static void checkLocked(final WeightedKNearestNeighboursPositionSolver2D solver) {
        assertThrows(LockedException.class, () -> solver.setListener(null));
        assertThrows(LockedException.class, () -> solver.setFingerprintsAndDistances(null, null));
        assertThrows(LockedException.class, () -> solver.setEpsilon(1.0));
        assertThrows(LockedException.class, solver::solve);
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
