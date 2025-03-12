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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Accuracy2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconIdentifier;
import com.irurueta.navigation.indoor.BeaconLocated2D;
import com.irurueta.navigation.indoor.IndoorException;
import com.irurueta.navigation.indoor.RangingReadingLocated2D;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated2D;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.UUID;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class RangingRadioSourceEstimator2DTest implements RangingRadioSourceEstimatorListener<WifiAccessPoint, Point2D> {

    private static final Logger LOGGER = Logger.getLogger(RangingRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; // (Hz)
    private static final double TRANSMITTED_POWER_DBM = -50.0;

    private static final int MIN_READINGS = 50;
    private static final int MAX_READINGS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double ERROR_STD = 0.2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    @Test
    void testConstructor() {
        final var randomizer = new UniformRandomizer();

        // test empty constructor
        var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();

        // check default values
        assertEquals(3, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isHomogeneousLinearSolverUsed());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // test constructor with readings
        final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated2D<>(accessPoint, 0.0, position));
        }

        estimator = new RangingRadioSourceEstimator2D<>(readings);

        // check default values
        assertEquals(3, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isHomogeneousLinearSolverUsed());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingRadioSourceEstimator2D<>(
                (List<RangingReadingLocated2D<WifiAccessPoint>>) null));
        final var wrongReadings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
        assertThrows(IllegalArgumentException.class, () -> new RangingRadioSourceEstimator2D<>(wrongReadings));

        // test constructor with listener
        estimator = new RangingRadioSourceEstimator2D<>(this);

        // check default values
        assertEquals(3, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isHomogeneousLinearSolverUsed());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // test constructor with readings and listener
        estimator = new RangingRadioSourceEstimator2D<>(readings, this);

        // check default values
        assertEquals(3, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isHomogeneousLinearSolverUsed());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingRadioSourceEstimator2D<>(
                (List<RangingReadingLocated2D<WifiAccessPoint>>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new RangingRadioSourceEstimator2D<>(wrongReadings,
                this));

        // test constructor with initial position
        final var initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new RangingRadioSourceEstimator2D<>(initialPosition);

        // check default values
        assertEquals(3, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isHomogeneousLinearSolverUsed());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // test constructor with readings and initial position
        estimator = new RangingRadioSourceEstimator2D<>(readings, initialPosition);

        // check default values
        assertEquals(3, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isHomogeneousLinearSolverUsed());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingRadioSourceEstimator2D<>(null,
                initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new RangingRadioSourceEstimator2D<>(wrongReadings,
                initialPosition));

        // test constructor with initial position and listener
        estimator = new RangingRadioSourceEstimator2D<>(initialPosition, this);

        // check default values
        assertEquals(3, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isHomogeneousLinearSolverUsed());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // test constructor with readings, initial position and listener
        estimator = new RangingRadioSourceEstimator2D<>(readings, initialPosition, this);

        // check default values
        assertEquals(3, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator.isNonLinearSolverEnabled());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isHomogeneousLinearSolverUsed());
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingRadioSourceEstimator2D<>(null,
                initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new RangingRadioSourceEstimator2D<>(wrongReadings,
                initialPosition, this));
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
    }

    @Test
    void testIsSetNonLinearSolverEnabled() throws LockedException {
        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertTrue(estimator.isNonLinearSolverEnabled());

        // set new value
        estimator.setNonLinearSolverEnabled(false);

        // check
        assertFalse(estimator.isNonLinearSolverEnabled());

        // set new value
        estimator.setNonLinearSolverEnabled(true);

        // check
        assertTrue(estimator.isNonLinearSolverEnabled());
    }

    @Test
    void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(false);

        // check
        assertFalse(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
    }

    @Test
    void testGetSetUseReadingPositionCovariance() throws LockedException {
        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());

        // set new value
        estimator.setUseReadingPositionCovariances(
                !RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // check
        assertEquals(!RangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
    }

    @Test
    void testAreValidReadings() {
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated2D<>(accessPoint, 0.0, position));
        }

        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(new ArrayList<>()));
    }

    @Test
    void testGetSetReadings() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated2D<>(accessPoint, 0.0, position));
        }

        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();

        // initial value
        assertNull(estimator.getReadings());
        assertFalse(estimator.isReady());

        // set new value
        estimator.setReadings(readings);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator.isReady());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setReadings(null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testEstimateWithoutInitialPositionAndWithoutError() throws LockedException, NotReadyException,
            AlgebraException, RadioSourceEstimationException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);


        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);

        // force NotReadyException
        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialPositionAndWithoutError() throws LockedException, NotReadyException, AlgebraException,
            RadioSourceEstimationException {

        var numValidPosition = 0;
        double positionError;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, accessPointPosition, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        final var estimator = new RangingRadioSourceEstimator2D<WifiAccessPoint>(new InhomogeneousPoint2D());
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithoutInitialPositionAndDistanceErrors() throws LockedException, NotReadyException,
            AlgebraException {

        var numValidPosition = 0;
        double positionError;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);
                final var error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);


        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));
    }

    @Test
    void testEstimateWithInitialPositionAndDistanceErrors() throws LockedException, NotReadyException,
            AlgebraException {

        var numValidPosition = 0;
        double positionError;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);
                final var error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, accessPointPosition, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);


        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));
    }

    @Test
    void testEstimateWithInitialPositionWithError() throws LockedException, NotReadyException, AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + randomizer.nextDouble(MIN_POS, MAX_POS),
                    accessPointPosition.getInhomY() + randomizer.nextDouble(MIN_POS, MAX_POS));
            final var estimator = new RangingRadioSourceEstimator2D<>(readings, initialPosition, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);


        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateWithInitialPositionWithPositionAndDistanceErrors() throws LockedException, NotReadyException,
            AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);
                final var error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + randomizer.nextDouble(MIN_POS, MAX_POS),
                    accessPointPosition.getInhomY() + randomizer.nextDouble(MIN_POS, MAX_POS));
            final var estimator = new RangingRadioSourceEstimator2D<>(readings, initialPosition, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
        }

        assertTrue(numValidPosition > 0);

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double) numValidPosition / (double) TIMES * 100.0);


        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateBeacon() throws LockedException, NotReadyException, AlgebraException,
            RadioSourceEstimationException {

        var numValidPosition = 0;
        double positionError;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final var beacon = new Beacon(Collections.singletonList(identifier), TRANSMITTED_POWER_DBM, FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<Beacon>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(beacon, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedBeacon = (BeaconLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(beacon.getIdentifiers(), estimatedBeacon.getIdentifiers());
            assertEquals(beacon.getFrequency(), estimatedBeacon.getFrequency(), 0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedBeacon.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedBeacon.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);

            break;
        }

        assertTrue(numValidPosition > 0);


        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        // force NotReadyException
        final var estimator = new RangingRadioSourceEstimator3D<Beacon>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithNonLinearSolverDisabledAndWithoutInitialPosition() throws LockedException, NotReadyException {

        var numValidPosition = 0;
        var positionError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);
            estimator.setNonLinearSolverEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertNull(estimatedAccessPoint.getPositionCovariance());

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateWithNonLinearSolverDisabledAndWithInitialPosition() throws LockedException, NotReadyException {

        var numValidPosition = 0;
        var positionError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, accessPointPosition, this);
            estimator.setNonLinearSolverEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertNull(estimatedAccessPoint.getPositionCovariance());

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateWithRangingStandardDeviations() throws LockedException, NotReadyException, AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);
                final var error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance + error, readingsPositions[i],
                        ERROR_STD));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);


        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateWithReadingPositionCovariances() throws LockedException, NotReadyException, AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readingsPositions[i].setInhomogeneousCoordinates(
                        readingsPositions[i].getInhomX() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomY() + errorRandomizer.nextDouble());

                final var positionCovariance = Matrix.diagonal(
                        new double[]{ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD});

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i],
                        positionCovariance));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);
            estimator.setUseReadingPositionCovariances(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateWithRangingStandardDeviationsAndReadingPositionCovariances() throws LockedException,
            NotReadyException, AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);
                final var error = Math.abs(errorRandomizer.nextDouble());

                readingsPositions[i].setInhomogeneousCoordinates(
                        readingsPositions[i].getInhomX() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomY() + errorRandomizer.nextDouble());

                final var positionCovariance = Matrix.diagonal(
                        new double[]{ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD});

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance + error,
                        readingsPositions[i], ERROR_STD, positionCovariance));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);
            estimator.setUseReadingPositionCovariances(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateWithPositionCovariancesDisabled() throws LockedException, NotReadyException, AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);
                final var error = Math.abs(errorRandomizer.nextDouble());

                readingsPositions[i].setInhomogeneousCoordinates(
                        readingsPositions[i].getInhomX() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomY() + errorRandomizer.nextDouble());

                final var positionCovariance = Matrix.diagonal(
                        new double[]{ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD});

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance + error, readingsPositions[i],
                        ERROR_STD, positionCovariance));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);
            estimator.setUseReadingPositionCovariances(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateInitialPositionHomogeneousSolver() throws LockedException, NotReadyException, AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);
            estimator.setNonLinearSolverEnabled(true);
            estimator.setHomogeneousLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateInitialPositionInhomogeneousSolver() throws LockedException, NotReadyException, AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);
            estimator.setNonLinearSolverEnabled(true);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateNonLinearDisabledAndHomogeneousSolver() throws LockedException, NotReadyException {

        var numValidPosition = 0;
        var positionError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);
            estimator.setNonLinearSolverEnabled(false);
            estimator.setHomogeneousLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Test
    void testEstimateNonLinearDisableAndInhomogeneousSolver() throws LockedException, NotReadyException {

        var numValidPosition = 0;
        var positionError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated2D<>(accessPoint, distance, readingsPositions[i]));
            }

            final var estimator = new RangingRadioSourceEstimator2D<>(readings, this);
            estimator.setNonLinearSolverEnabled(false);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            try {
                estimator.estimate();
            } catch (final IndoorException e) {
                continue;
            }

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getEstimatedCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
    }

    @Override
    public void onEstimateStart(final RangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final RangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private static void checkLocked(final RangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        assertThrows(LockedException.class, () -> estimator.setNonLinearSolverEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setHomogeneousLinearSolverUsed(false));
        assertThrows(LockedException.class, () -> estimator.setUseReadingPositionCovariances(false));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, () -> estimator.setReadings(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, estimator::estimate);
    }
}

