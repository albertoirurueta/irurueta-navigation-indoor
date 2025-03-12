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
import com.irurueta.navigation.indoor.BeaconWithPowerAndLocated2D;
import com.irurueta.navigation.indoor.IndoorException;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated2D;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated2D;
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

import static com.irurueta.navigation.indoor.Utils.dBmToPower;
import static com.irurueta.navigation.indoor.Utils.powerTodBm;
import static org.junit.jupiter.api.Assertions.*;

class RangingAndRssiRadioSourceEstimator2DTest implements
        RangingAndRssiRadioSourceEstimatorListener<WifiAccessPoint, Point2D> {

    private static final Logger LOGGER = Logger.getLogger(RangingAndRssiRadioSourceEstimator2DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final int MIN_READINGS = 50;
    private static final int MAX_READINGS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;
    private static final double LARGE_POWER_ERROR = 0.5;
    private static final double PATH_LOSS_ERROR = 1.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    @Test
    void testConstants() {
        assertEquals(299792458.0, RangingAndRssiRadioSourceEstimator.SPEED_OF_LIGHT, 0.0);
        assertEquals(2.0, RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertFalse(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertTrue(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
    }

    @Test
    void testConstructor() {
        final var randomizer = new UniformRandomizer();

        // test empty constructor
        var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings
        final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 4; i++) {
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, 0.0, 0.0, position));
        }

        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null));
        final var wrongReadings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings));

        // test constructor with listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                this));

        // test constructor with initial position
        final var initialPosition = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(initialPosition);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and initial position
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(null,
                initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                initialPosition));

        // test constructor with initial position and listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(initialPosition, this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position and listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(null,
                initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                initialPosition, this));

        // test constructor with initial transmitted power
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(MAX_RSSI);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and initial transmitted power
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, MAX_RSSI);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                MAX_RSSI));

        // test constructor with initial transmitted power and listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(MAX_RSSI, this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial transmitted power and listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, MAX_RSSI, this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, MAX_RSSI, this));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                MAX_RSSI, this));

        // test constructor with readings, initial position and
        // initial transmitted power
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, MAX_RSSI);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(null,
                initialPosition, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                initialPosition, MAX_RSSI));

        // test constructor with initial position and initial transmitted power
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(initialPosition, MAX_RSSI);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power and listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(initialPosition, MAX_RSSI, this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power and listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, MAX_RSSI, this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(
                null, initialPosition, MAX_RSSI, this));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                initialPosition, MAX_RSSI, this));

        // test constructor with readings, initial position, initial
        // transmitted power and initial path loss exponent
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(null,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT));

        // test constructor with initial position and initial transmitted power
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power and
        // listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial
        // transmitted power and listener
        estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this);

        // check default values
        assertEquals(4, estimator.getMinReadings());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isLocked());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(null,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiRadioSourceEstimator2D<>(wrongReadings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this));
    }

    @Test
    void testGetMinReadings() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(4, estimator.getMinReadings());

        // position only
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(3, estimator.getMinReadings());

        // transmitted power and position
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(4, estimator.getMinReadings());

        // path-loss and position
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(4, estimator.getMinReadings());

        // position, transmitted power and path-loss
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(5, estimator.getMinReadings());
    }

    @Test
    void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialTransmittedPowerdBm());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        estimator.setInitialTransmittedPowerdBm(value);

        // check
        assertEquals(value, estimator.getInitialTransmittedPowerdBm(), 0.0);
    }

    @Test
    void testGetSetInitialTransmittedPower() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialTransmittedPower());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var value = Utils.dBmToPower(randomizer.nextDouble(MIN_RSSI, MAX_RSSI));
        estimator.setInitialTransmittedPower(value);

        // check
        assertEquals(value, estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setInitialTransmittedPower(-1.0));

        // set null value
        estimator.setInitialTransmittedPower(null);

        // check
        assertNull(estimator.getInitialTransmittedPower());
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
    }

    @Test
    void testGetSetInitialPathLossExponent() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
        estimator.setInitialPathLossExponent(value);

        // check
        assertEquals(estimator.getInitialPathLossExponent(), value, 0.0);
    }

    @Test
    void testIsSetTransmittedPowerEstimationEnabled() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());

        // set new value
        estimator.setTransmittedPowerEstimationEnabled(false);

        // check
        assertFalse(estimator.isTransmittedPowerEstimationEnabled());
    }

    @Test
    void testIsSetPathLossEstimationEnabled() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertFalse(estimator.isPathLossEstimationEnabled());

        // set new value
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertTrue(estimator.isPathLossEstimationEnabled());
    }

    @Test
    void testGetSetUseReadingPositionCovariance() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());

        // set new value
        estimator.setUseReadingPositionCovariances(
                !RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // check
        assertEquals(!RangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
    }

    @Test
    void testAreValidReadings() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, 0.0, 0.0, position));
        }

        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(new ArrayList<>()));
    }

    @Test
    void testGetSetReadings() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 3; i++) {
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, 0.0, 0.0, position));
        }

        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setInitialTransmittedPowerdBm(MAX_RSSI);
        estimator.setPathLossEstimationEnabled(false);
        estimator.setInitialPathLossExponent(MAX_PATH_LOSS_EXPONENT);

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
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testIsSetHomogeneousRangingLinearSolverUsed() throws LockedException {
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // set new value
        estimator.setHomogeneousRangingLinearSolverUsed(false);

        // check
        assertFalse(estimator.isHomogeneousRangingLinearSolverUsed());
    }

    @Test
    void testEstimateWithoutInitialPositionAndInitialTransmittedPowerAndWithoutError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialPositionWithoutError() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>(readings, initialPosition,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialTransmittedPowerWithoutError() throws LockedException, NotReadyException,
            IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            final var initialTransmittedPowerdBm = transmittedPowerdBm + errorRandomizer.nextDouble();

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialTransmittedPowerdBm,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialPositionAndInitialTransmittedPowerWithoutError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());
            final var initialTransmittedPowerdBm = transmittedPowerdBm + errorRandomizer.nextDouble();

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithoutInitialPositionAndInitialTransmittedPowerAndWithError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var error = errorRandomizer.nextDouble();
                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT)) + error;

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i],
                        ERROR_STD, ERROR_STD));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialPositionAndWithError() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var error = errorRandomizer.nextDouble();
                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT)) + error;

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i],
                        ERROR_STD, ERROR_STD));
            }

            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialTransmittedPowerAndWithError() throws LockedException, NotReadyException,
            IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var error = errorRandomizer.nextDouble();
                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT)) + error;

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i],
                        ERROR_STD, ERROR_STD));
            }

            final var initialTransmittedPowerdBm = transmittedPowerdBm + errorRandomizer.nextDouble();

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialTransmittedPowerdBm,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialPositionAndInitialTransmittedPowerAndWithError() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var error = errorRandomizer.nextDouble();
                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT)) + error;

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i],
                        ERROR_STD, ERROR_STD));
            }

            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + errorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + errorRandomizer.nextDouble());
            final var initialTransmittedPowerdBm = transmittedPowerdBm + errorRandomizer.nextDouble();

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition,
                    initialTransmittedPowerdBm, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionTransmittedPowerAndPathLossEstimationEnabled() throws LockedException, NotReadyException,
            IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var numValidPathLoss = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var pathLossError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        var pathLossStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);
            pathLossStd = Math.sqrt(pathLossVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() - pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValidPathLoss > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionTransmittedPowerAndPathLossEstimationEnabledWithInitialValues() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var numValidPathLoss = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var pathLossError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        var pathLossStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i],
                        ERROR_STD, ERROR_STD));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);
            pathLossStd = Math.sqrt(pathLossVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() - pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValidPathLoss > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialPathLoss() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(pathLossExponent, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateBeacon() throws LockedException, NotReadyException, IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var beaconPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);

            final var identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final var beacon = new Beacon(Collections.singletonList(identifier), transmittedPowerdBm, FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<Beacon>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(beaconPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, beacon.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(beacon, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final var estimatedBeacon = (BeaconWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(beacon.getIdentifiers(), estimatedBeacon.getIdentifiers());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedBeacon.getTransmittedPower(),
                    ABSOLUTE_ERROR);
            assertEquals(beacon.getFrequency(), estimatedBeacon.getFrequency(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedBeacon.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedBeacon.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedBeacon.getPositionCovariance());
            assertNull(estimatedBeacon.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(beaconPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(beaconPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionOnly() throws LockedException, NotReadyException, IndoorException, AlgebraException {

        var numValidPosition = 0;
        double positionError;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
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
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionOnlyWithInitialPosition() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i],
                        ERROR_STD, ERROR_STD));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
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
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionOnlyRepeated() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // repeat so that position covariance matrix is reused
            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
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
            assertEquals(2, estimateStart);
            assertEquals(2, estimateEnd);

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
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionAndPathloss() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPathLoss = 0;
        var positionError = 0.0;
        var pathLossError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        var pathLossStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimatedAccessPoint.getTransmittedPower(), 0.0);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            pathLossStd = Math.sqrt(pathLossVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() - pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPathLoss > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionAndPathlossWithInitialValues() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPathLoss = 0;
        var positionError = 0.0;
        var pathLossError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        var pathLossStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i],
                        ERROR_STD, ERROR_STD));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimatedAccessPoint.getTransmittedPower(), 0.0);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimatedAccessPoint.getPathLossExponent(), estimator.getEstimatedPathLossExponent(),
                    0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            pathLossStd = Math.sqrt(pathLossVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() - pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPathLoss > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithRangingStandardDeviations() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);
                final var error = Math.abs(errorRandomizer.nextDouble());

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance + error, rssi,
                        readingsPositions[i], ERROR_STD, null, null));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), LARGE_POWER_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithRSSIStandardDeviations() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);
                final var error = Math.abs(errorRandomizer.nextDouble());

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i], null, ERROR_STD, null));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), LARGE_POWER_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithPositionCovariance() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readingsPositions[i].setInhomogeneousCoordinates(
                        readingsPositions[i].getInhomX() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomY() + errorRandomizer.nextDouble());

                final var positionCovariance = Matrix.diagonal(
                        new double[]{ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD});

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i],
                        null, null, positionCovariance));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), LARGE_POWER_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithRangingAndRSSIStandardDeviations() throws LockedException, NotReadyException, IndoorException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                final var errorDistance = Math.abs(errorRandomizer.nextDouble());
                final var errorRssi = errorRandomizer.nextDouble();

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance + errorDistance,
                        rssi + errorRssi, readingsPositions[i], ERROR_STD, ERROR_STD, null));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), LARGE_POWER_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithRangingStandardDeviationsAndPositionCovariance() throws LockedException, NotReadyException,
            IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
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

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance + error, rssi,
                        readingsPositions[i], ERROR_STD, null, positionCovariance));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), LARGE_POWER_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithRSSIStandardDeviationsAndPositionCovariance() throws LockedException, NotReadyException,
            IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
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

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i], null, ERROR_STD, positionCovariance));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), LARGE_POSITION_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POSITION_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithRangingAndRSSIStandardDeviationsAndPositionCovariance() throws LockedException,
            NotReadyException, IndoorException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var errorRandomizer = new GaussianRandomizer(0.0, ERROR_STD);

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                readingsPositions[i].setInhomogeneousCoordinates(
                        readingsPositions[i].getInhomX() + errorRandomizer.nextDouble(),
                        readingsPositions[i].getInhomY() + errorRandomizer.nextDouble());

                final var positionCovariance = Matrix.diagonal(
                        new double[]{ERROR_STD * ERROR_STD, ERROR_STD * ERROR_STD});

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                final var errorDistance = Math.abs(errorRandomizer.nextDouble());
                final var errorRssi = errorRandomizer.nextDouble();

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance + errorDistance,
                        rssi + errorRssi, readingsPositions[i], ERROR_STD, ERROR_STD, positionCovariance));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), LARGE_POWER_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), LARGE_POWER_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new RangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithHomogeneousRangingLinearSolver() throws LockedException, NotReadyException, AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setHomogeneousRangingLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);
    }

    @Test
    void testEstimateWithInhomogeneousRangingLinearSolver() throws LockedException, NotReadyException,
            AlgebraException {

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi, readingsPositions[i]));
            }

            final var estimator = new RangingAndRssiRadioSourceEstimator2D<>(readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setHomogeneousRangingLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
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

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(), estimator.getEstimatedPositionCoordinates(),
                    0.0);
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        final var format = NumberFormat.getPercentInstance();
        var formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format("Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);
    }

    @Override
    public void onEstimateStart(final RangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final RangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private static double receivedPower(final double equivalentTransmittedPower, final double distance,
                                        final double frequency, final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final var k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), pathLossExponent);
        return equivalentTransmittedPower * k / Math.pow(distance, pathLossExponent);
    }

    private static void checkLocked(final RangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        assertThrows(LockedException.class, () -> estimator.setUseReadingPositionCovariances(false));
        assertThrows(LockedException.class, () -> estimator.setInitialTransmittedPowerdBm(null));
        assertThrows(LockedException.class, () -> estimator.setInitialTransmittedPower(null));
        assertThrows(LockedException.class, () -> estimator.setTransmittedPowerEstimationEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, () -> estimator.setInitialPathLossExponent(2.0));
        assertThrows(LockedException.class, () -> estimator.setPathLossEstimationEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setReadings(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setHomogeneousRangingLinearSolverUsed(false));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
