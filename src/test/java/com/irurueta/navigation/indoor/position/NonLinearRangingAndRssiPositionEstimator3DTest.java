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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class NonLinearRangingAndRssiPositionEstimator3DTest implements RangingAndRssiPositionEstimatorListener<Point3D> {

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

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    @Test
    void testConstructor() {
        // empty constructor
        var estimator = new NonLinearRangingAndRssiPositionEstimator3D();

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // constructor with sources
        final var sources = new ArrayList<WifiAccessPointLocated3D>();
        for (var i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY, new InhomogeneousPoint3D()));
        }
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                (List<WifiAccessPointLocated3D>) null));
        final var wrongSources = new ArrayList<WifiAccessPointLocated3D>();
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                wrongSources));

        // constructor with fingerprint
        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(fingerprint);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null));

        // constructor with sources and fingerprint
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, fingerprint);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(null,
                fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(wrongSources,
                fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(sources,
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null));

        // constructor with listener
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(this);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // constructor with sources and listener
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, this);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                (List<WifiAccessPointLocated3D>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(wrongSources,
                this));

        // constructor with fingerprint and listener
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(fingerprint, this);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                this));

        // constructor with sources, fingerprint and listener
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, fingerprint, this);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(null,
                fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(wrongSources,
                fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(sources,
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                this));

        // constructor with initial position
        final var initialPosition = new InhomogeneousPoint3D();
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(initialPosition);

        // check default values
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // constructor with sources and initial position
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, initialPosition);

        // check default values
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                (List<WifiAccessPointLocated3D>) null, initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(wrongSources,
                initialPosition));

        // constructor with fingerprint and initial position
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(fingerprint, initialPosition);

        // check default values
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                initialPosition));

        // constructor with sources, fingerprint and initial position
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, fingerprint, initialPosition);

        // check default values
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(null,
                fingerprint, initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(wrongSources,
                fingerprint, initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(sources,
                null, initialPosition));

        // constructor with initial position and listener
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(initialPosition, this);

        // check default values
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // constructor with sources, initial position and listener
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, initialPosition, this);

        // check default values
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                (List<WifiAccessPointLocated3D>) null, initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(wrongSources,
                initialPosition, this));

        // constructor with fingerprint, initial position and listener
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(fingerprint, initialPosition, this);

        // check default values
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                initialPosition, this));

        // constructor with sources, fingerprint, initial position and listener
        estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, fingerprint, initialPosition, this);

        // check default values
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(4, estimator.getMinRequiredSources());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(null,
                fingerprint, initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(wrongSources,
                fingerprint, initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearRangingAndRssiPositionEstimator3D(sources,
                null, initialPosition, this));
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var estimator = new NonLinearRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final var initialPosition = new InhomogeneousPoint3D();
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
    }

    @Test
    void testIsSetRadioSourcePositionCovarianceUsed() throws LockedException {
        final var estimator = new NonLinearRangingAndRssiPositionEstimator3D();

        // check default value
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRadioSourcePositionCovarianceUsed(true);

        // check
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
    }

    @Test
    void testGetSetFallbackDistanceStandardDeviation() throws LockedException {
        final var estimator = new NonLinearRangingAndRssiPositionEstimator3D();

        // check default value
        assertEquals(NonLinearRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);

        // set new value
        estimator.setFallbackDistanceStandardDeviation(5.0);

        // check
        assertEquals(5.0, estimator.getFallbackDistanceStandardDeviation(), 0.0);
    }

    @Test
    void testGetSetSources() throws LockedException {
        final var estimator = new NonLinearRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        final var sources = new ArrayList<WifiAccessPointLocated3D>();
        for (var i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY, new InhomogeneousPoint3D()));
        }

        estimator.setSources(sources);

        // check
        assertSame(sources, estimator.getSources());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setSources(null));
        final var wrongSources = new ArrayList<WifiAccessPointLocated3D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setSources(wrongSources));
    }

    @Test
    void testGetSetFingerprint() throws LockedException {
        final var estimator = new NonLinearRangingAndRssiPositionEstimator3D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(fingerprint, estimator.getFingerprint());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setFingerprint(null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new NonLinearRangingAndRssiPositionEstimator3D();

        // check default size
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testEstimateNoError() throws LockedException, NotReadyException, PositionEstimationException {
        final var randomizer = new UniformRandomizer();

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated3D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY,
                        transmittedPowerdBm, pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
            }

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);


            final var estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, fingerprint, this);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertNotNull(estimator.getDistanceStandardDeviations());
            assertNull(estimator.getCovariance());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            if (!position.equals(estimatedPosition, 10.0 * ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, 10.0 * ABSOLUTE_ERROR));
            assertNotNull(estimator.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        final var estimator = new NonLinearRangingAndRssiPositionEstimator3D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoErrorWithInitialPosition() throws LockedException, NotReadyException,
            PositionEstimationException {
        final var randomizer = new UniformRandomizer();

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated3D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY,
                        transmittedPowerdBm, pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
            }

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);


            final var estimator = new NonLinearRangingAndRssiPositionEstimator3D(sources, fingerprint, position,
                    this);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertNotNull(estimator.getDistanceStandardDeviations());
            assertNull(estimator.getCovariance());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(estimator.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        final var estimator = new NonLinearRangingAndRssiPositionEstimator3D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(final RangingAndRssiPositionEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((NonLinearRangingAndRssiPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateEnd(final RangingAndRssiPositionEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((NonLinearRangingAndRssiPositionEstimator3D) estimator);
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

    private static void checkLocked(final NonLinearRangingAndRssiPositionEstimator3D estimator) {
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, () -> estimator.setRadioSourcePositionCovarianceUsed(false));
        assertThrows(LockedException.class, () -> estimator.setSources(null));
        assertThrows(LockedException.class, () -> estimator.setFingerprint(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
