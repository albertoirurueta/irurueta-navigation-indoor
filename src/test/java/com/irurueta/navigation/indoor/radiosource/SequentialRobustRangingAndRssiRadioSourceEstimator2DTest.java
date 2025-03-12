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
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated2D;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated2D;
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

class SequentialRobustRangingAndRssiRadioSourceEstimator2DTest implements
        SequentialRobustRangingAndRssiRadioSourceEstimatorListener<WifiAccessPoint, Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            SequentialRobustRangingAndRssiRadioSourceEstimator2DTest.class.getName());

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
    private static final double LARGE_POSITION_ERROR = 0.5;
    private static final double LARGE_POWER_ERROR = 0.5;
    private static final double PATH_LOSS_ERROR = 1.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 50;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimateProgressChange;

    @Test
    void testConstructor() throws LockedException {
        final var randomizer = new UniformRandomizer();

        // test empty constructor
        var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings
        final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 4; i++) {
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, 0.0, 0.0, position));
        }

        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null));
        final var wrongReadings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings));

        // test constructor with listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, this));

        // test constructor with readings and initial position
        final var initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, initialPosition));

        // test constructor with initial position
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(initialPosition);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(initialPosition, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, initialPosition, this));

        // test constructor with initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, MAX_RSSI));

        // test constructor with initial transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, MAX_RSSI, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, MAX_RSSI, this));

        // test constructor with readings, initial position and initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, initialPosition, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, initialPosition, MAX_RSSI));

        // test constructor with initial position and initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(initialPosition, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(initialPosition, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power
        // and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, MAX_RSSI,
                this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, initialPosition, MAX_RSSI, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, initialPosition, MAX_RSSI, this));

        // test constructor with readings, initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, MAX_RSSI,
                1.0);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, initialPosition, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, initialPosition, MAX_RSSI));

        // test constructor with initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(initialPosition, MAX_RSSI,
                1.0);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(initialPosition, MAX_RSSI,
                1.0, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(readings, initialPosition, MAX_RSSI,
                1.0, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, initialPosition, MAX_RSSI,
                1.0, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                wrongReadings, initialPosition, MAX_RSSI, 1.0, this));

        // test constructor with quality scores
        final var qualityScores = new double[4];

        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores and readings
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings));

        // test constructor with quality scores and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, this));

        // test constructor with quality scores, readings and initial position
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                initialPosition);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, null, initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, initialPosition));

        // test constructor with quality scores and initial position
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, initialPosition);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, initial position and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, initialPosition,
                this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings, initial position and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings, initialPosition,
                this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, null, initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, initialPosition, this));

        // test constructor with quality scores, initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings and initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, MAX_RSSI));

        // test constructor with quality scores, initial transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings, initial transmitted power
        // and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings, MAX_RSSI,
                this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, MAX_RSSI, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, MAX_RSSI, this));

        // test constructor with quality scores, readings, initial position and
        // initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings, initialPosition,
                MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, null, initialPosition, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, initialPosition, MAX_RSSI));

        // test constructor with quality scores, initial position and initial
        // transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, initialPosition,
                MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, initial position, initial
        // transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, initialPosition, MAX_RSSI,
                this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power
        // and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings, initialPosition,
                MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, null, initialPosition, MAX_RSSI, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, initialPosition, MAX_RSSI, this));

        // test constructor with readings, initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings, initialPosition,
                MAX_RSSI, 1.0);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, null, initialPosition, MAX_RSSI));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, initialPosition, MAX_RSSI));

        // test constructor with initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, initialPosition, MAX_RSSI,
                1.0);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, initialPosition, MAX_RSSI,
                1.0, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings, initialPosition,
                MAX_RSSI, 1.0, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, null, initialPosition, MAX_RSSI, 1.0, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(
                qualityScores, wrongReadings, initialPosition, MAX_RSSI, 1.0, this));
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetRangingRobustMethod() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());

        // set new value
        estimator.setRangingRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getRangingRobustMethod());
    }

    @Test
    void testGetSetRssiRobustMethod() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());

        // set new value
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getRssiRobustMethod());
    }

    @Test
    void testGetSetRangingThreshold() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getRangingThreshold());

        // set new value
        estimator.setRangingThreshold(INLIER_ERROR_STD);

        // check
        assertEquals(INLIER_ERROR_STD, estimator.getRangingThreshold(), 0.0);
    }

    @Test
    void testGetSetRssiThreshold() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getRssiThreshold());

        // set new value
        estimator.setRssiThreshold(INLIER_ERROR_STD);

        // check
        assertEquals(INLIER_ERROR_STD, estimator.getRssiThreshold(), 0.0);
    }

    @Test
    void testGetSetRangingConfidence() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);

        // set new value
        estimator.setRangingConfidence(0.5);

        // check
        assertEquals(0.5, estimator.getRangingConfidence(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setRangingConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setRangingConfidence(2.0));
    }

    @Test
    void testGetSetRssiConfidence() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);

        // set new value
        estimator.setRssiConfidence(0.5);

        // check
        assertEquals(0.5, estimator.getRssiConfidence(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setRssiConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setRssiConfidence(2.0));
    }

    @Test
    void testGetSetRangingMaxIterations() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());

        // set new value
        estimator.setRangingMaxIterations(50);

        // check
        assertEquals(50, estimator.getRangingMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setRangingMaxIterations(0));
    }

    @Test
    void testGetSetRssiMaxIterations() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());

        // set new value
        estimator.setRssiMaxIterations(50);

        // check
        assertEquals(50, estimator.getRssiMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setRssiMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(!SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(!SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(!SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        // check
        assertEquals(!SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
    }

    @Test
    void testGetSetReadings() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getReadings());

        // set new value
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 4; i++) {
            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, 0.0, 0.0, position));
        }

        estimator.setReadings(readings);

        // check correctness
        assertSame(readings, estimator.getReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setReadings(null));
        final var wrongReadings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setReadings(wrongReadings));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final var qualityScores = new double[4];
        estimator.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, estimator.getQualityScores());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setQualityScores(null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setQualityScores(new double[1]));
    }

    @Test
    void testGetSetRangingPreliminarySubsetSize() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());

        // set new value
        estimator.setRangingPreliminarySubsetSize(5);

        // check
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setRangingPreliminarySubsetSize(3));
    }

    @Test
    void testGetSetRssiPreliminarySubsetSize() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());

        // set new value
        estimator.setRssiPreliminarySubsetSize(5);

        // check
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setRssiPreliminarySubsetSize(3));
    }

    @Test
    void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialTransmittedPowerdBm());

        final var randomizer = new UniformRandomizer();

        final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);

        // check
        assertEquals(transmittedPowerdBm, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(transmittedPowerdBm), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetInitialTransmittedPower() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialTransmittedPower());

        final var randomizer = new UniformRandomizer();

        final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
        estimator.setInitialTransmittedPower(transmittedPower);

        // check
        assertEquals(transmittedPowerdBm, estimator.getInitialTransmittedPowerdBm(), ABSOLUTE_ERROR);
        assertEquals(transmittedPower, estimator.getInitialTransmittedPower(), 0.0);
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final var initialPosition = Point2D.create();
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
    }

    @Test
    void testGetSetInitialPathLossExponent() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);

        // set new value
        estimator.setInitialPathLossExponent(1.0);

        // check
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
    }

    @Test
    void testIsSetTransmittedPowerEstimationEnabled() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());

        // set new value
        estimator.setTransmittedPowerEstimationEnabled(
                !RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);

        // check
        assertEquals(!RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED,
                estimator.isTransmittedPowerEstimationEnabled());
    }

    @Test
    void testIsSetPathLossEstimationEnabled() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());

        // set new value
        estimator.setPathLossEstimationEnabled(!RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);

        // check
        assertEquals(!RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
    }

    @Test
    void testGetSetUseReadingPositionCovariance() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());

        // set new value
        estimator.setUseReadingPositionCovariances(
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // check
        assertEquals(!SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
    }

    @Test
    void testIsSetHomogeneousRangingLinearSolverUsed() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // set new value
        estimator.setHomogeneousRangingLinearSolverUsed(false);

        // check
        assertFalse(estimator.isHomogeneousRangingLinearSolverUsed());
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

        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RangingAndRssiReadingLocated<WifiAccessPoint, Point2D>>()));
    }

    @Test
    void testGetMinReadings() throws LockedException {
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();

        // check default value
        assertEquals(4, estimator.getMinReadings());

        // position only
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(3, estimator.getMinReadings());

        // position and transmitted power
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(4, estimator.getMinReadings());

        // position and path-loss
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
    void testEstimateNoInlierErrorNoRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoInlierErrorWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var numValid = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            numValid++;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>(
                    qualityScores, readings, this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());
            final var initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            final var initialPosition = new InhomogeneousPoint2D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());
            final var initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithPathLossEstimationEnabled() throws LockedException, NotReadyException,
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
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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
            final var beaconPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);

            final var identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final var beacon = new Beacon(Collections.singletonList(identifier), transmittedPowerdBm, FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<Beacon>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(beacon, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings);
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

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

            final var estimatedBeacon = (BeaconWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), estimator.getEstimatedPathLossExponent(),
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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);

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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(),estimatedAccessPoint.getPosition());
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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // repeat so that position covariance matrix is reused
            estimator.estimate();

            // check
            assertEquals(2, estimateStart);
            assertEquals(2, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
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

            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithStandardDeviationsAndPositionCovariance() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
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
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                final var positionCovariance = Matrix.diagonal(new double[]{
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD});

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD, positionCovariance));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setUseReadingPositionCovariances(true);

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
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWitHomogeneousRangingLinearSolver() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var numValid = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setHomogeneousRangingLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            numValid++;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

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

    @Test
    void testEstimateWitInhomogeneousRangingLinearSolver() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var numValid = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var powerStd = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setHomogeneousRangingLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

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
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            numValid++;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);

        final var format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
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
    void testEstimateLargerRangingPreliminarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            estimator.setRangingPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateLargerRssiPreliminarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            estimator.setRssiPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateLargerRangingAndRssiPreliminarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var numValidPower = 0;
        var positionError = 0.0;
        var powerError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point2D[numReadings];
            final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
            final var qualityScores = new double[numReadings];
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

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

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<>(qualityScores, readings,
                    this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            estimator.setRangingPreliminarySubsetSize(estimator.getMinReadings() + 1);
            estimator.setRssiPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(1.0, estimator.getEstimatedTransmittedPower(), 0.0);
            assertEquals(0.0, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateProgressChange);

            estimator.estimate();

            // check
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());

            final var estimatedAccessPoint = (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

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
        final var estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(
            final SequentialRobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateStart++;
        checkLocked((SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateEnd(
            final SequentialRobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateEnd++;
        checkLocked((SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final SequentialRobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator,
            final float progress) {
        estimateProgressChange++;
        checkLocked((SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateProgressChange = 0;
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

    private static void checkLocked(
            final SequentialRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator) {
        assertThrows(LockedException.class, () -> estimator.setRangingPreliminarySubsetSize(4));
        assertThrows(LockedException.class, () -> estimator.setRssiPreliminarySubsetSize(4));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setRangingRobustMethod(null));
        assertThrows(LockedException.class, () -> estimator.setRssiRobustMethod(null));
        assertThrows(LockedException.class, () -> estimator.setRangingThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setRssiThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setRangingConfidence(0.8));
        assertThrows(LockedException.class, () -> estimator.setRssiConfidence(0.8));
        assertThrows(LockedException.class, () -> estimator.setRangingMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setRssiMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setResultRefined(false));
        assertThrows(LockedException.class, () -> estimator.setCovarianceKept(false));
        assertThrows(LockedException.class, () -> estimator.setReadings(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setQualityScores(null));
        assertThrows(LockedException.class, () -> estimator.setInitialTransmittedPowerdBm(null));
        assertThrows(LockedException.class, () -> estimator.setInitialTransmittedPower(null));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, () -> estimator.setInitialPathLossExponent(2.0));
        assertThrows(LockedException.class, () -> estimator.setTransmittedPowerEstimationEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setPathLossEstimationEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setUseReadingPositionCovariances(true));
        assertThrows(LockedException.class, () -> estimator.setHomogeneousRangingLinearSolverUsed(false));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
