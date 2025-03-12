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
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconIdentifier;
import com.irurueta.navigation.indoor.BeaconWithPowerAndLocated3D;
import com.irurueta.navigation.indoor.RssiReadingLocated3D;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated3D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
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

class LMedSRobustRssiRadioSourceEstimator3DTest implements
        RobustRssiRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(LMedSRobustRssiRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final int MIN_READINGS = 100;
    private static final int MAX_READINGS = 500;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double INLIER_ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 1.0;
    private static final double LARGE_POWER_ERROR = 0.5;
    private static final double PATH_LOSS_ERROR = 1.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 50;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstructor() {
        final var randomizer = new UniformRandomizer();

        // test empty constructor
        var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with readings
        final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RssiReadingLocated3D<>(accessPoint, 0.0, position));
        }

        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(
                (List<RssiReadingLocated3D<WifiAccessPoint>>) null));
        final var wrongReadings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings));

        // test constructor with listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with readings and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(
                (List<RssiReadingLocated3D<WifiAccessPoint>>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                this));

        // test constructor with readings and initial position
        final var initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, initialPosition);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(null,
                initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                initialPosition));

        // test constructor with initial position
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(initialPosition);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with initial position and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(initialPosition, this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with readings, initial position and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, initialPosition, this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(null,
                initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                initialPosition, this));

        // test constructor with initial transmitter power
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(MAX_RSSI);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with readings and initial transmitted power
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, MAX_RSSI);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(
                (List<RssiReadingLocated3D<WifiAccessPoint>>) null, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                MAX_RSSI));

        // test constructor with initial transmitted power and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(MAX_RSSI, this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with readings, initial transmitted power and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, MAX_RSSI, this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertNull(estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(
                (List<RssiReadingLocated3D<WifiAccessPoint>>) null, MAX_RSSI, this));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                MAX_RSSI, this));

        // test constructor with readings, initial position and initial transmitted power
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, initialPosition, MAX_RSSI);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(null,
                initialPosition, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                initialPosition, MAX_RSSI));

        // test constructor with initial position and initial transmitted power
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(initialPosition, MAX_RSSI);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with initial position, initial transmitted power and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(initialPosition, MAX_RSSI, this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with readings, initial position, initial transmitted power
        // and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, initialPosition, MAX_RSSI, this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(
                null, initialPosition, MAX_RSSI, this));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                initialPosition, MAX_RSSI, this));

        // test constructor with readings, initial position and initial transmitted power
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(null,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT));

        // test constructor with initial position and initial transmitted power
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with initial position, initial transmitted power and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // test constructor with readings, initial position, initial transmitted power
        // and listener
        estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this);

        // check
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
        assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(),
                0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(null,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this));
        assertThrows(IllegalArgumentException.class, () -> new LMedSRobustRssiRadioSourceEstimator3D<>(wrongReadings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(LMedSRobustRssiRadioSourceEstimator3D.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check
        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setStopThreshold(0.0));
    }

    @Test
    void testGetMinReadings() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(5, estimator.getMinReadings());

        // position only
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(4, estimator.getMinReadings());

        // transmitted power only
        estimator.setPositionEstimationEnabled(false);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(2, estimator.getMinReadings());

        // path-loss only
        estimator.setPositionEstimationEnabled(false);
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(2, estimator.getMinReadings());

        // position and transmitted power
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(5, estimator.getMinReadings());

        // position and path-loss
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(5, estimator.getMinReadings());

        // transmitted power and path-loss
        estimator.setPositionEstimationEnabled(false);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(3, estimator.getMinReadings());

        // position, transmitted power and path-loss
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(6, estimator.getMinReadings());
    }

    @Test
    void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialTransmittedPowerdBm());

        // set new value
        estimator.setInitialTransmittedPowerdBm(MAX_RSSI);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
    }

    @Test
    void testGetSetInitialTransmittedPower() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialTransmittedPower());

        // set new value
        final var power = Utils.dBmToPower(MAX_RSSI);
        estimator.setInitialTransmittedPower(power);

        // check
        assertEquals(power, estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final var initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
    }

    @Test
    void testGetSetInitialPathLossExponent() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(),
                0.0);

        // set new value
        final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
        estimator.setInitialPathLossExponent(pathLossExponent);

        // check
        assertEquals(pathLossExponent, estimator.getInitialPathLossExponent(), 0.0);
    }

    @Test
    void testIsSetTransmittedPowerEstimationEnabled() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());

        // set new value
        estimator.setTransmittedPowerEstimationEnabled(false);

        // check
        assertFalse(estimator.isTransmittedPowerEstimationEnabled());
    }

    @Test
    void testIsSetPositionEstimationEnabled() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertTrue(estimator.isPositionEstimationEnabled());

        // set new value
        estimator.setPositionEstimationEnabled(false);

        // check
        assertFalse(estimator.isPositionEstimationEnabled());
    }

    @Test
    void testIsSetPathLossEstimationEnabled() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertFalse(estimator.isPathLossEstimationEnabled());

        // set new value
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertTrue(estimator.isPathLossEstimationEnabled());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check
        assertEquals(0.5, estimator.getConfidence(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check
        assertEquals(10, estimator.getMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(!RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(!RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(!RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        // check
        assertEquals(!RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
    }

    @Test
    void testAreValidReadings() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 6; i++) {
            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RssiReadingLocated3D<>(accessPoint, 0.0, position));
        }

        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(new ArrayList<>()));
    }

    @Test
    void testGetSetReadings() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        estimator.setPositionEstimationEnabled(true);
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setInitialTransmittedPowerdBm(MAX_RSSI);
        estimator.setPathLossEstimationEnabled(false);
        estimator.setInitialPathLossExponent(MAX_PATH_LOSS_EXPONENT);

        // check default value
        assertNull(estimator.getReadings());
        assertFalse(estimator.isReady());

        // set new value
        final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RssiReadingLocated3D<>(accessPoint, 0.0, position));
        }

        estimator.setReadings(readings);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator.isReady());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setReadings(null));
        final var wrongReadings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setReadings(wrongReadings));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        estimator.setQualityScores(new double[3]);

        // check
        assertNull(estimator.getQualityScores());
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(5, estimator.getPreliminarySubsetSize());

        // set new value
        estimator.setPreliminarySubsetSize(6);

        // check
        assertEquals(6, estimator.getPreliminarySubsetSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setPreliminarySubsetSize(4));
    }

    @Test
    void testEstimateNoInlierErrorNoRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);

        // force NotReadyException
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoInlierErrorWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInlierErrorWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);
        final var inlierErrorRandomizer = new GaussianRandomizer(0.0, INLIER_ERROR_STD);

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < 10 * TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                error += inlierErrorRandomizer.nextDouble();

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i],
                        INLIER_ERROR_STD));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoInlierErrorWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);
        final var inlierErrorRandomizer = new GaussianRandomizer(0.0, INLIER_ERROR_STD);

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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoInlierErrorWithInitialPower() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);
        final var inlierErrorRandomizer = new GaussianRandomizer(0.0, INLIER_ERROR_STD);

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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoInlierErrorWithInitialPositionAndPower() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);
        final var inlierErrorRandomizer = new GaussianRandomizer(0.0, INLIER_ERROR_STD);

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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());
            final var initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInlierErrorWithInitialPositionAndPower() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);
        final var inlierErrorRandomizer = new GaussianRandomizer(0.0, INLIER_ERROR_STD);

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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                error += inlierErrorRandomizer.nextDouble();

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i],
                        INLIER_ERROR_STD));
            }

            final var initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());
            final var initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionTransmittedPowerAndPathLossEstimationEnabled() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            if (pathLossVariance <= 0.0) {
                continue;
            }
            assertTrue(pathLossVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionTransmittedPowerAndPathLossEstimationEnabledWithInitialValues() throws LockedException,
            NotReadyException, RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            if (pathLossVariance <= 0.0) {
                continue;
            }
            assertTrue(pathLossVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInitialPathLoss() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPathLossExponent(pathLossExponent);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());

            if (estimator.getCovariance() == null) {
                continue;
            }

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateBeacon() throws LockedException, NotReadyException, RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

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
            final var beaconPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);

            final var identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final var beacon = new Beacon(Collections.singletonList(identifier), transmittedPowerdBm, FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<Beacon>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(beaconPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, beacon.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(beacon, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedBeacon = (BeaconWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedBeacon.getIdentifiers(), beacon.getIdentifiers());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedBeacon.getTransmittedPower(), 0.0);
            assertEquals(beacon.getFrequency(), estimatedBeacon.getFrequency(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedBeacon.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedBeacon.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedBeacon.getPositionCovariance());
            assertNull(estimatedBeacon.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionOnly() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionOnlyWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionOnlyRepeated() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // repeat so that position covariance matrix is reused
            estimator.estimate();

            // check
            assertEquals(2, estimateStart);
            assertEquals(2, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateTransmittedPowerOnly() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPower = 0;
        var powerError = 0.0;
        var powerStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(accessPointPosition, estimator.getEstimatedPosition());
            assertEquals(accessPointPosition, estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(pathLossExponent, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);

            powerStd = Math.sqrt(powerVariance);

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateTransmittedPowerOnlyWithInitialTransmittedPower() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPower = 0;
        var powerError = 0.0;
        var powerStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(accessPointPosition, estimator.getEstimatedPosition());
            assertEquals(accessPointPosition, estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(pathLossExponent, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);

            powerStd = Math.sqrt(powerVariance);

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() - transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(transmittedPower, estimator.getEstimatedTransmittedPower(), ABSOLUTE_ERROR);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);

        // force NotReadyException
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePathlossOnly() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPathLoss = 0;
        var pathLossError = 0.0;
        var pathLossStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(accessPointPosition, estimator.getEstimatedPosition());
            assertEquals(accessPointPosition, estimatedAccessPoint.getPosition());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(transmittedPowerdBm, estimatedAccessPoint.getTransmittedPower(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);

            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            pathLossStd = Math.sqrt(pathLossVariance);

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() - pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPathLoss > 0);

        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePathlossOnlyWithInitialPathloss() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPathLoss = 0;
        var pathLossError = 0.0;
        var pathLossStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(accessPointPosition, estimator.getEstimatedPosition());
            assertEquals(accessPointPosition, estimatedAccessPoint.getPosition());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(transmittedPowerdBm, estimatedAccessPoint.getTransmittedPower(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);

            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            pathLossStd = Math.sqrt(pathLossVariance);

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() - pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPathLoss > 0);

        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionAndPathloss() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimatePositionAndPathlossWithInitialValues() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final var accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateTransmittedPowerAndPathloss() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPower = 0;
        var numValidPathLoss = 0;
        var powerError = 0.0;
        var pathLossError = 0.0;
        var powerStd = 0.0;
        var pathLossStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(accessPointPosition, estimatedAccessPoint.getPosition());
            assertEquals(accessPointPosition, estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            powerStd = Math.sqrt(powerVariance);
            pathLossStd = Math.sqrt(pathLossVariance);

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPower > 0);
        assertTrue(numValidPathLoss > 0);

        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateTransmittedPowerAndPathlossWithInitialValues() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPower = 0;
        var numValidPathLoss = 0;
        var powerError = 0.0;
        var pathLossError = 0.0;
        var powerStd = 0.0;
        var pathLossStd = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(false);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(accessPointPosition, estimatedAccessPoint.getPosition());
            assertEquals(accessPointPosition, estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(), estimatedAccessPoint.getPathLossExponent(),
                    0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);
            assertNull(estimatedAccessPoint.getPositionCovariance());

            final var powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            final var pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            powerStd = Math.sqrt(powerVariance);
            pathLossStd = Math.sqrt(pathLossVariance);

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPower > 0);
        assertTrue(numValidPathLoss > 0);

        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB", powerStd);
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateLargerPreliminarySubsetSize() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RssiReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RssiReadingLocated3D<>(accessPoint, rssi + error, readingsPositions[i]));
            }

            final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<>(readings, this);
            estimator.setPositionEstimationEnabled(true);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            estimator.setPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB", powerError);

        // force NotReadyException
        final var estimator = new LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(final RobustRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateStart++;
        checkLocked((LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateEnd(final RobustRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateEnd++;
        checkLocked((LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateNextIteration(final RobustRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateProgressChange(final RobustRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
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

    private static void checkLocked(final LMedSRobustRssiRadioSourceEstimator3D<WifiAccessPoint> estimator) {
        assertThrows(LockedException.class, () -> estimator.setPreliminarySubsetSize(5));
        assertThrows(LockedException.class, () -> estimator.setPathLossEstimationEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setTransmittedPowerEstimationEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setPositionEstimationEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setInitialTransmittedPowerdBm(null));
        assertThrows(LockedException.class, () -> estimator.setInitialTransmittedPower(null));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.8));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setResultRefined(false));
        assertThrows(LockedException.class, () -> estimator.setCovarianceKept(false));
        assertThrows(LockedException.class, () -> estimator.setReadings(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
