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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.Accuracy3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.UUID;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class NonLinearFingerprintPositionAndRadioSourceEstimator3DTest implements
        FingerprintPositionAndRadioSourceEstimatorListener<Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            NonLinearFingerprintPositionAndRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 10;
    private static final int MAX_SOURCES = 20;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final int MIN_FINGERPRINTS = 100;
    private static final int MAX_FINGERPRINTS = 1000;

    private static final double RSSI_BIAS = 1.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double MIN_RSSI_STANDARD_DEVIATION = 1e-2;
    private static final double MAX_RSSI_STANDARD_DEVIATION = 5e-1;

    private static final double MIN_PATH_LOSS_STANDARD_DEVIATION = 1e-2;
    private static final double MAX_PATH_LOSS_STANDARD_DEVIATION = 5e-2;

    private static final double MIN_POSITION_STANDARD_DEVIATION = 1e-1;
    private static final double MAX_POSITION_STANDARD_DEVIATION = 5e-1;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double INITIAL_POSITION_ERROR_STD = 1.0;

    private static final double RSSI_ERROR_STD = 1.0e-1;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    @Test
    void testConstructor() {
        // test empty constructor
        var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // test constructor with listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(this);

        // check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RssiReading<RadioSource>>();
        for (var i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final var accessPoint = new WifiAccessPoint("bssid" + i, FREQUENCY);
            final var rssi = randomizer.nextDouble();

            final var reading = new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, Point3D.create());

        final var locatedFingerprints = new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
        locatedFingerprints.add(locatedFingerprint);

        final var fingerprint = new RssiFingerprint<>(readings);

        // test constructor with located fingerprints and fingerprint
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints, fingerprint);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                null, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, null));

        // test constructor with located fingerprints, fingerprint and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints, fingerprint,
                this);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                null, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, null, this));

        // test constructor with located fingerprints, fingerprint and initial position
        final var initialPosition = Point3D.create();

        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints, fingerprint,
                initialPosition);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                null, fingerprint, initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, null, initialPosition));

        // test constructor with located fingerprints, fingerprint, initial position
        // and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints, fingerprint,
                initialPosition, this);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                null, fingerprint, initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, null, initialPosition));

        // test constructor with located fingerprints, fingerprint and
        // initial located sources
        final var initialLocatedSources = new ArrayList<RadioSourceLocated<Point3D>>();

        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints, fingerprint,
                initialLocatedSources);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertSame(initialLocatedSources, estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                null, fingerprint, initialLocatedSources));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, null, initialLocatedSources));

        // test constructor with located fingerprints, fingerprint and initial located sources
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints, fingerprint,
                initialLocatedSources, this);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertSame(initialLocatedSources, estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                null, fingerprint, initialLocatedSources));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, null, initialLocatedSources));

        // test constructor with located fingerprints, fingerprint, initial position and
        // initial located sources
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints, fingerprint,
                initialPosition, initialLocatedSources);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertSame(initialLocatedSources, estimator.getInitialLocatedSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                null, fingerprint, initialPosition, initialLocatedSources));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, null, initialPosition, initialLocatedSources));

        // test constructor with located fingerprints, fingerprint, initial position,
        // initial located sources and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints, fingerprint,
                initialPosition, initialLocatedSources, this);

        // check default values
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertSame(initialLocatedSources, estimator.getInitialLocatedSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(0.0, estimator.getChiSq(), 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                null, fingerprint, initialPosition, initialLocatedSources, this));
        assertThrows(IllegalArgumentException.class, () -> new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, null, initialPosition, initialLocatedSources, this));
    }

    @Test
    void testGetSetLocatedFingerprints() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertNull(estimator.getLocatedFingerprints());

        // set new value
        final var locatedFingerprints =
                new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
        estimator.setLocatedFingerprints(locatedFingerprints);

        // check
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
    }

    @Test
    void testGetSetFingerprint() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final var fingerprint = new RssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(fingerprint, estimator.getFingerprint());
    }

    @Test
    void testGetSetMinMaxNearestFingerprints() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default values
        assertEquals(-1, estimator.getMinNearestFingerprints());
        assertEquals(-1, estimator.getMaxNearestFingerprints());

        // set new values
        estimator.setMinMaxNearestFingerprints(2, 3);

        // check
        assertEquals(2, estimator.getMinNearestFingerprints());
        assertEquals(3, estimator.getMaxNearestFingerprints());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMinMaxNearestFingerprints(-1, 3));
        assertThrows(IllegalArgumentException.class, () -> estimator.setMinMaxNearestFingerprints(0, 3));
        assertThrows(IllegalArgumentException.class, () -> estimator.setMinMaxNearestFingerprints(2, 1));
    }

    @Test
    void testGetSetPathLossExponent() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertEquals(2.0, estimator.getPathLossExponent(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var pathLossExponent = randomizer.nextDouble();

        estimator.setPathLossExponent(pathLossExponent);

        // check correctness
        assertEquals(pathLossExponent, estimator.getPathLossExponent(), 0.0);
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetUseNoMeanNearestFingerprintFinder() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());

        // set new value
        estimator.setUseNoMeanNearestFingerprintFinder(false);

        // check
        assertFalse(estimator.getUseNoMeanNearestFingerprintFinder());
    }

    @Test
    void testGetSetInitialLocatedSources() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertNull(estimator.getInitialLocatedSources());

        // set new value
        final var initialLocatedSources = new ArrayList<RadioSourceLocated<Point3D>>();
        estimator.setInitialLocatedSources(initialLocatedSources);

        // check
        assertSame(initialLocatedSources, estimator.getInitialLocatedSources());
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final var initialPosition = Point3D.create();
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
    }

    @Test
    void testGetSetUseSourcesPathLossExponentWhenAvailable() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        // set new value
        estimator.setUseSourcesPathLossExponentWhenAvailable(false);

        // check
        assertFalse(estimator.getUseSourcesPathLossExponentWhenAvailable());
    }

    @Test
    void testGetSetFallbackRssiStandardDeviation() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertEquals(NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                estimator.getFallbackRssiStandardDeviation(), 0.0);

        // set new value
        estimator.setFallbackRssiStandardDeviation(0.5);

        // check
        assertEquals(0.5, estimator.getFallbackRssiStandardDeviation(), 0.0);
    }

    @Test
    void testIsSetFingerprintRssiStandardDeviationPropagated() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());

        // set new value
        estimator.setFingerprintRssiStandardDeviationPropagated(false);

        // check
        assertFalse(estimator.isFingerprintRssiStandardDeviationPropagated());
    }

    @Test
    void testIsSetPathlossExponentStandardDeviationPropagated() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());

        // set new value
        estimator.setPathlossExponentStandardDeviationPropagated(false);

        // check
        assertFalse(estimator.isPathlossExponentStandardDeviationPropagated());
    }

    @Test
    void testIsSetFingerprintPositionCovariancePropagated() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());

        // set new value
        estimator.setFingerprintPositionCovariancePropagated(false);

        // check
        assertFalse(estimator.isFingerprintPositionCovariancePropagated());
    }

    @Test
    void testIsSetRadioSourcePositionCovariancePropagated() throws LockedException {
        final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        // check default value
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        // set new value
        estimator.setRadioSourcePositionCovariancePropagated(false);

        // check
        assertFalse(estimator.isRadioSourcePositionCovariancePropagated());
    }

    @Test
    void testEstimateOneRadioSourceWithoutInitialValues() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check results
            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateOneRadioSourceWithExactInitialValues() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateOneRadioSourceWithApproximateInitialValues() throws LockedException,
            FingerprintEstimationException, NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var positionErrorRandomizer = new GaussianRandomizer(0.0, INITIAL_POSITION_ERROR_STD);

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            final var initialSources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var initialPosition = new InhomogeneousPoint3D(
                        x + positionErrorRandomizer.nextDouble(),
                        y + positionErrorRandomizer.nextDouble(),
                        z + positionErrorRandomizer.nextDouble());

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);

                final var initialAccessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, initialPosition);
                initialSources.add(initialAccessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var initialPosition = new InhomogeneousPoint3D(
                    x + positionErrorRandomizer.nextDouble(),
                    y + positionErrorRandomizer.nextDouble(),
                    z + positionErrorRandomizer.nextDouble());

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, initialPosition, initialSources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check results
            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;


        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateOneRadioSourceWithNonLocatedSources() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 1;
            final var locatedSources = new ArrayList<RadioSourceLocated<Point3D>>();
            final var sources = new ArrayList<RadioSource>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                locatedSources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint("bssid" + i, FREQUENCY);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                var k = 0;
                for (final var locatedSource : locatedSources) {
                    final var distance = locatedSource.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) locatedSource)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));

                    final var source = sources.get(k);
                    final var reading = new RssiReading<>(source, receivedRssi);
                    readings.add(reading);
                    k++;
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            var k = 0;
            for (final var locatedSource : locatedSources) {
                final var distance = locatedSource.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) locatedSource)
                        .getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));

                final var source = sources.get(k);
                final var reading = new RssiReading<>(source, receivedRssi);
                readings.add(reading);
                k++;
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, locatedSources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = locatedSources.indexOf(estimatedLocatedSource);
                final var source = locatedSources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateOneRadioSourceWithRssiError() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var rssiErrorRandomizer = new GaussianRandomizer(0.0, RSSI_ERROR_STD);

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var rssiError = rssiErrorRandomizer.nextDouble();

                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi + rssiError);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var rssiError = rssiErrorRandomizer.nextDouble();

                final var reading = new RssiReading<>((RadioSource) source, receivedRssi + rssiError);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateOneRadioSourceWithBias() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi + RSSI_BIAS);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi + RSSI_BIAS);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateOneRadioSourceWithRssiErrorAndBias() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var rssiErrorRandomizer = new GaussianRandomizer(0.0, RSSI_ERROR_STD);

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var rssiError = rssiErrorRandomizer.nextDouble();

                    final var reading = new RssiReading<>((RadioSource) source,
                            receivedRssi + rssiError + RSSI_BIAS);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var rssiError = rssiErrorRandomizer.nextDouble();

                final var reading = new RssiReading<>((RadioSource) source, receivedRssi + rssiError + RSSI_BIAS);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateOneRadioSourceWithVariancePropagation() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var positionStd = randomizer.nextDouble(MIN_POSITION_STANDARD_DEVIATION,
                        MAX_POSITION_STANDARD_DEVIATION);
                final var positionVariance = positionStd * positionStd;
                final var positionCovariance = Matrix.diagonal(
                        new double[]{positionVariance, positionVariance, positionVariance});
                final var pathLossStd = randomizer.nextDouble(MIN_PATH_LOSS_STANDARD_DEVIATION,
                        MAX_PATH_LOSS_STANDARD_DEVIATION);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, null,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, pathLossStd,
                        position, positionCovariance);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var positionStd = randomizer.nextDouble(MIN_POSITION_STANDARD_DEVIATION,
                        MAX_POSITION_STANDARD_DEVIATION);
                final var positionVariance = positionStd * positionStd;
                final var positionCovariance = Matrix.diagonal(
                        new double[]{positionVariance, positionVariance, positionVariance});

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var receivedRssiStd = randomizer.nextDouble(MIN_RSSI_STANDARD_DEVIATION,
                            MAX_RSSI_STANDARD_DEVIATION);

                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi, receivedRssiStd);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position, positionCovariance);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateBeacon() throws LockedException, FingerprintEstimationException, NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
                final var beacon = new BeaconLocated3D(Collections.singletonList(identifier), transmittedPowerdBm,
                        FREQUENCY, position);
                sources.add(beacon);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((BeaconLocated3D) source).getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((BeaconLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateOneRadioSourceWithOtherPathLossExponent() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            // build sources
            final var numSources = 1;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, pathLossExponent, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, pathLossExponent));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        pathLossExponent));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setPathLossExponent(pathLossExponent);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateTwoRadioSourcesWithoutInitialValues() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 2;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            // check results
            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(numSources, estimatedLocatedSources.size());
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;


        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateTwoRadioSourcesWithExactInitialValues() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 2;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(numSources, estimatedLocatedSources.size());
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateTwoRadioSourcesWithVariancePropagation() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 2;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var positionStd = randomizer.nextDouble(MIN_POSITION_STANDARD_DEVIATION,
                        MAX_POSITION_STANDARD_DEVIATION);
                final var positionVariance = positionStd * positionStd;
                final var positionCovariance = Matrix.diagonal(
                        new double[]{positionVariance, positionVariance, positionVariance});
                final var pathLossStd = randomizer.nextDouble(MIN_PATH_LOSS_STANDARD_DEVIATION,
                        MAX_PATH_LOSS_STANDARD_DEVIATION);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, null,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, pathLossStd,
                        position, positionCovariance);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var positionStd = randomizer.nextDouble(MIN_POSITION_STANDARD_DEVIATION,
                        MAX_POSITION_STANDARD_DEVIATION);
                final var positionVariance = positionStd * positionStd;
                final var positionCovariance = Matrix.diagonal(
                        new double[]{positionVariance, positionVariance, positionVariance});

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var receivedRssiStd = randomizer.nextDouble(MIN_RSSI_STANDARD_DEVIATION,
                            MAX_RSSI_STANDARD_DEVIATION);

                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi, receivedRssiStd);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position, positionCovariance);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertEquals(TIMES, numValid);
    }

    @Test
    void testEstimateWithoutEnoughFingerprintsForARadioSource() throws LockedException, NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = 2;
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {

                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // find closest located fingerprint to actual position and leave only
            // dims - 1 readings for last radio source
            final var minNearestFingerprints = 9;
            final var noMeanFinder = new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);
            final var nearestFingerprints = noMeanFinder.findKNearestTo(fingerprint, minNearestFingerprints);
            final var lastSource = sources.get(sources.size() - 1);
            var numReadingsLastSource = 0;
            locatedFingerprints.clear();

            for (final var nearFingerprint : nearestFingerprints) {
                final var nearReadings = nearFingerprint.getReadings();

                final var readingsToRemove = new ArrayList<RssiReading<RadioSource>>();
                for (final var nearReading : nearReadings) {
                    if (nearReading.getSource().equals(lastSource)) {
                        numReadingsLastSource++;

                        if (numReadingsLastSource >= Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
                            readingsToRemove.add(nearReading);
                        }
                    }
                }

                nearReadings.removeAll(readingsToRemove);

                locatedFingerprints.add(
                        (RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>) nearFingerprint);
            }


            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMinMaxNearestFingerprints(minNearestFingerprints, -1);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            try {
                estimator.estimate();
            } catch (FingerprintEstimationException e) {
                continue;
            }

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);

            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(numSources - 1, estimatedLocatedSources.size());
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        final var percentage = (double) numValid / (double) TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%", percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateMultipleRadioSourcesWithoutInitialValues() throws LockedException, FingerprintEstimationException,
            NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = Math.max(randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS),
                    2 * (Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1));
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(numSources, estimatedLocatedSources.size());
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;

            break;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateMultipleRadioSourcesWithExactInitialValues() throws LockedException,
            FingerprintEstimationException, NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // build sources
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final var numFingerprints = Math.max(randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS),
                    2 * (Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1));
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(numSources, estimatedLocatedSources.size());
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;

            break;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateMultipleRadioSourcesWithApproximateInitialValues() throws LockedException,
            FingerprintEstimationException, NotReadyException {

        final var accuracy = new Accuracy3D();

        var avgPositionError = 0.0;
        var avgSourcePositionError = 0.0;
        var avgPositionAccuracy = 0.0;
        var avgSourceAccuracy = 0.0;
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var positionErrorRandomizer = new GaussianRandomizer(0.0, INITIAL_POSITION_ERROR_STD);

            // build sources
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final var sources = new ArrayList<RadioSourceLocated<Point3D>>();
            final var initialSources = new ArrayList<RadioSourceLocated<Point3D>>();
            for (var i = 0; i < numSources; i++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var initialPosition = new InhomogeneousPoint3D(
                        x + positionErrorRandomizer.nextDouble(),
                        y + positionErrorRandomizer.nextDouble(),
                        z + positionErrorRandomizer.nextDouble());

                final var accessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, position);
                sources.add(accessPoint);

                final var initialAccessPoint = new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                        transmittedPowerdBm, initialPosition);
                initialSources.add(initialAccessPoint);
            }

            // build located fingerprints
            final var numFingerprints = Math.max(randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS),
                    2 * (Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1));
            final var locatedFingerprints =
                    new ArrayList<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>();
            for (var j = 0; j < numFingerprints; j++) {
                final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final var position = new InhomogeneousPoint3D(x, y, z);

                final var readings = new ArrayList<RssiReading<RadioSource>>();
                for (final var source : sources) {
                    final var distance = source.getPosition().distanceTo(position);
                    final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source)
                            .getTransmittedPower();

                    final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                    readings.add(reading);
                }

                final var locatedFingerprint = new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final var x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var position = new InhomogeneousPoint3D(x, y, z);

            final var initialPosition = new InhomogeneousPoint3D(
                    x + positionErrorRandomizer.nextDouble(),
                    y + positionErrorRandomizer.nextDouble(),
                    z + positionErrorRandomizer.nextDouble());

            final var readings = new ArrayList<RssiReading<RadioSource>>();
            for (final var source : sources) {
                final var distance = source.getPosition().distanceTo(position);
                final var transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).getTransmittedPower();

                final var receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                final var reading = new RssiReading<>((RadioSource) source, receivedRssi);
                readings.add(reading);
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            // create estimator
            final var estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(locatedFingerprints,
                    fingerprint, initialPosition, initialSources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            // check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            estimator.estimate();

            final var estimatedPosition = estimator.getEstimatedPosition();
            final var estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            final var estimatedLocatedSources = estimator.getEstimatedLocatedSources();
            final var estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(numSources, estimatedLocatedSources.size());
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            final var positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                final var positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            var avgSourcePositionErrorOneTime = 0.0;
            var avgSourceAccuracyOneTime = 0.0;
            var numValidSources = 0;
            for (final var estimatedLocatedSource : estimatedLocatedSources) {
                final var estimatedSourcePosition = estimatedLocatedSource.getPosition();
                final var estimatedSourcePositionCovariance = estimatedLocatedSource.getPositionCovariance();

                final var index = sources.indexOf(estimatedLocatedSource);
                final var source = sources.get(index);
                final var sourcePosition = source.getPosition();
                final var sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    final var sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;

            break;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        LOGGER.log(Level.INFO, "Avg. position error: {0} m", avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m", avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m", avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m", avgSourceAccuracy);

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final FingerprintPositionAndRadioSourceEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((NonLinearFingerprintPositionAndRadioSourceEstimator3D) estimator);
    }

    @Override
    public void onEstimateEnd(final FingerprintPositionAndRadioSourceEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((NonLinearFingerprintPositionAndRadioSourceEstimator3D) estimator);
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

    private void checkLocked(final NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator) {
        assertThrows(LockedException.class, () -> estimator.setLocatedFingerprints(null));
        assertThrows(LockedException.class, () -> estimator.setFingerprint(null));
        assertThrows(LockedException.class, () -> estimator.setMinMaxNearestFingerprints(-1, -1));
        assertThrows(LockedException.class, () -> estimator.setPathLossExponent(2.0));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.setUseNoMeanNearestFingerprintFinder(true));
        assertThrows(LockedException.class, () -> estimator.setInitialLocatedSources(null));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, () -> estimator.setUseSourcesPathLossExponentWhenAvailable(true));
        assertThrows(LockedException.class, () -> estimator.setFallbackRssiStandardDeviation(1.0));
        assertThrows(LockedException.class, () -> estimator.setFingerprintRssiStandardDeviationPropagated(true));
        assertThrows(LockedException.class, () -> estimator.setPathlossExponentStandardDeviationPropagated(true));
        assertThrows(LockedException.class, () -> estimator.setFingerprintPositionCovariancePropagated(true));
        assertThrows(LockedException.class, () -> estimator.setRadioSourcePositionCovariancePropagated(true));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
