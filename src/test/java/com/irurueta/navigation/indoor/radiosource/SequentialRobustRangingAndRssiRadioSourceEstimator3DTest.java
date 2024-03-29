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
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconIdentifier;
import com.irurueta.navigation.indoor.BeaconWithPowerAndLocated3D;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated3D;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated3D;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.UUID;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class SequentialRobustRangingAndRssiRadioSourceEstimator3DTest implements
        SequentialRobustRangingAndRssiRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            SequentialRobustRangingAndRssiRadioSourceEstimator3DTest.class.getName());

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
    public void testConstructor() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // test empty constructor
        SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings
        final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint, 0.0, 0.0, position));
        }

        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings and initial position
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));

        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition);

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
        assertFalse(estimator.isReady());
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                readings, initialPosition, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(MAX_RSSI);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings, MAX_RSSI);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(MAX_RSSI, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings, MAX_RSSI, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null,
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(),
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings, initial position and initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings, initialPosition, MAX_RSSI);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position and initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(initialPosition, MAX_RSSI);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                initialPosition, MAX_RSSI, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power
        // and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings,
                initialPosition, MAX_RSSI, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null,
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings, initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings,
                initialPosition, MAX_RSSI, 1.0);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                initialPosition, MAX_RSSI, 1.0);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                initialPosition, MAX_RSSI, 1.0, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(readings,
                initialPosition, MAX_RSSI, 1.0, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null,
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[5];

        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores and readings
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, readings and initial position
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, readings, initialPosition);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    qualityScores, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and initial position
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, initial position and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, initialPosition, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings, initial position and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, readings, initialPosition, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    qualityScores, null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, MAX_RSSI);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings and initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, MAX_RSSI);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, initial transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, MAX_RSSI, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings, initial transmitted power
        // and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, readings, MAX_RSSI, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    (List<RangingAndRssiReadingLocated3D<WifiAccessPoint>>) null, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, readings, initial position and
        // initial transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, readings, initialPosition, MAX_RSSI);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    qualityScores, null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, initial position and initial
        // transmitted power
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, initialPosition, MAX_RSSI);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, initial position, initial
        // transmitted power and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, initialPosition, MAX_RSSI, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power
        // and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, readings, initialPosition, MAX_RSSI, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, null,
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings, initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings,
                initialPosition, MAX_RSSI, 1.0);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    qualityScores, null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(), initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, initialPosition,
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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                qualityScores, readings, initialPosition, MAX_RSSI, 1.0, this);

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
        assertEquals(5, estimator.getMinReadings());
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(
                    qualityScores, null, initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRangingRobustMethod() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());

        // set new value
        estimator.setRangingRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getRangingRobustMethod());
    }

    @Test
    public void testGetSetRssiRobustMethod() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());

        // set new value
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getRssiRobustMethod());
    }

    @Test
    public void testGetSetRangingThreshold() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getRangingThreshold());

        // set new value
        estimator.setRangingThreshold(INLIER_ERROR_STD);

        // check
        assertEquals(INLIER_ERROR_STD, estimator.getRangingThreshold(), 0.0);
    }

    @Test
    public void testGetSetRssiThreshold() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getRssiThreshold());

        // set new value
        estimator.setRssiThreshold(INLIER_ERROR_STD);

        // check
        assertEquals(INLIER_ERROR_STD, estimator.getRssiThreshold(), 0.0);
    }

    @Test
    public void testGetSetRangingConfidence() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);

        // set new value
        estimator.setRangingConfidence(0.5);

        // check
        assertEquals(0.5, estimator.getRangingConfidence(), 0.0);

        // force IllegalArgumentException
        try {
            estimator.setRangingConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setRangingConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRssiConfidence() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);

        // set new value
        estimator.setRssiConfidence(0.5);

        // check
        assertEquals(0.5, estimator.getRssiConfidence(), 0.0);

        // force IllegalArgumentException
        try {
            estimator.setRssiConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setRssiConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRangingMaxIterations() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());

        // set new value
        estimator.setRangingMaxIterations(50);

        // check
        assertEquals(50, estimator.getRangingMaxIterations());

        // force IllegalArgumentException
        try {
            estimator.setRangingMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRssiMaxIterations() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());

        // set new value
        estimator.setRssiMaxIterations(50);

        // check
        assertEquals(50, estimator.getRssiMaxIterations());

        // force IllegalArgumentException
        try {
            estimator.setRssiMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

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
    public void testIsSetCovarianceKept() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

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
    public void testGetSetReadings() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getReadings());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint, 0.0, 0.0, position));
        }

        estimator.setReadings(readings);

        // check correctness
        assertSame(readings, estimator.getReadings());

        // force IllegalArgumentException
        try {
            estimator.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setReadings(new ArrayList<RangingAndRssiReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[5];
        estimator.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, estimator.getQualityScores());

        // force IllegalArgumentException
        try {
            estimator.setQualityScores(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRangingPreliminarySubsetSize() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());

        // set new value
        estimator.setRangingPreliminarySubsetSize(6);

        // check
        assertEquals(6, estimator.getRangingPreliminarySubsetSize());

        // force IllegalArgumentException
        try {
            estimator.setRangingPreliminarySubsetSize(4);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRssiPreliminarySubsetSize() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());

        // set new value
        estimator.setRssiPreliminarySubsetSize(6);

        // check
        assertEquals(6, estimator.getRssiPreliminarySubsetSize());

        // force IllegalArgumentException
        try {
            estimator.setRssiPreliminarySubsetSize(4);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getInitialTransmittedPowerdBm());

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);

        // check
        assertEquals(transmittedPowerdBm, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(transmittedPowerdBm), estimator.getInitialTransmittedPower(), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetInitialTransmittedPower() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getInitialTransmittedPower());

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
        estimator.setInitialTransmittedPower(transmittedPower);

        // check
        assertEquals(transmittedPowerdBm, estimator.getInitialTransmittedPowerdBm(), ABSOLUTE_ERROR);
        assertEquals(transmittedPower, estimator.getInitialTransmittedPower(), 0.0);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final Point3D initialPosition = Point3D.create();
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
    }

    @Test
    public void testGetSetInitialPathLossExponent() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                estimator.getInitialPathLossExponent(), 0.0);

        // set new value
        estimator.setInitialPathLossExponent(1.0);

        // check
        assertEquals(1.0, estimator.getInitialPathLossExponent(), 0.0);
    }

    @Test
    public void testIsSetTransmittedPowerEstimationEnabled() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

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
    public void testIsSetPathLossEstimationEnabled() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());

        // set new value
        estimator.setPathLossEstimationEnabled(
                !RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);

        // check
        assertEquals(!RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED,
                estimator.isPathLossEstimationEnabled());
    }

    @Test
    public void testGetSetUseReadingPositionCovariance() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

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
    public void testIsSetHomogeneousRangingLinearSolverUsed() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // set new value
        estimator.setHomogeneousRangingLinearSolverUsed(false);

        // check
        assertFalse(estimator.isHomogeneousRangingLinearSolverUsed());
    }

    @Test
    public void testAreValidReadings() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint, 0.0, 0.0, position));
        }

        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RangingAndRssiReadingLocated<WifiAccessPoint, Point3D>>()));
    }

    @Test
    public void testGetMinReadings() throws LockedException {
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();

        // check default value
        assertEquals(5, estimator.getMinReadings());

        // position only
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(4, estimator.getMinReadings());

        // position and transmitted power
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(5, estimator.getMinReadings());

        // position and path-loss
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(5, estimator.getMinReadings());

        // position, transmitted power and path-loss
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(6, estimator.getMinReadings());
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInlierErrorWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

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

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    estimatedAccessPoint.getTransmittedPower(), 0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
            assertEquals(estimator.getEstimatedPositionCovariance(),
                    estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPower() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final double initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPositionAndPower() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());
            final double initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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
            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double) numValidPosition / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage valid power: {0} %",
                (double) numValidPower / (double) TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage both valid: {0} %",
                (double) numValid / (double) TIMES * 100.0);

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInlierErrorWithInitialPositionAndPower() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

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

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble(),
                    accessPointPosition.getInhomZ() + inlierErrorRandomizer.nextDouble());
            final double initialTransmittedPowerdBm = transmittedPowerdBm + inlierErrorRandomizer.nextDouble();

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithPathLossEstimationEnabled() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0, numValidPathLoss = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double pathLossError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        double pathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(),
                    estimatedAccessPoint.getPositionCovariance());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            if (powerVariance <= 0.0) {
                continue;
            }
            assertTrue(powerVariance > 0.0);
            final double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            if (pathLossVariance <= 0.0) {
                continue;
            }
            assertTrue(pathLossVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInitialPathLoss() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedAccessPoint.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateBeacon() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D beaconPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);

            final BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final Beacon beacon = new Beacon(Collections.singletonList(identifier), transmittedPowerdBm, FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<Beacon>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(beaconPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, beacon.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated3D<>(beacon, distance, rssi + error,
                        readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<Beacon> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings);
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

            final BeaconWithPowerAndLocated3D estimatedBeacon =
                    (BeaconWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(beacon.getIdentifiers(), estimatedBeacon.getIdentifiers());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedBeacon.getTransmittedPower(),
                    0.0);
            assertEquals(beacon.getFrequency(), estimatedBeacon.getFrequency(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedBeacon.getPathLossExponent(), 0.0);
            assertEquals(Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()),
                    estimatedBeacon.getTransmittedPowerStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedBeacon.getPositionCovariance());
            assertNull(estimatedBeacon.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimatePositionOnly() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimatePositionOnlyWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);

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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimatePositionOnlyRepeated() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), pathLossExponent));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(pathLossExponent, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimatePositionAndPathloss() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPathLoss = 0;
        double positionError = 0.0;
        double pathLossError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        double pathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimatedAccessPoint.getTransmittedPower(), 0.0);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimatePositionAndPathlossWithInitialValues() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPathLoss = 0;
        double positionError = 0.0;
        double pathLossError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        double pathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint, distance, rssi + error,
                        readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(transmittedPowerdBm, estimatedAccessPoint.getTransmittedPower(), 0.0);
            assertEquals(transmittedPowerdBm, estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(Math.sqrt(estimator.getEstimatedPathLossExponentVariance()),
                    estimatedAccessPoint.getPathLossExponentStandardDeviation(), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());

            final double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(positionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                positionStd, formattedConfidence));

        formattedConfidence = format.format(positionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                positionAccuracy, formattedConfidence));

        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);
        LOGGER.log(Level.INFO, "Path loss error: {0}", pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}", pathLossStd);

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithStandardDeviationsAndPositionCovariance() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                final Matrix positionCovariance = Matrix.diagonal(new double[]{
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD});

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i],
                        INLIER_ERROR_STD, INLIER_ERROR_STD, positionCovariance));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWitHomogeneousRangingLinearSolver() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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
    public void testEstimateWitInhomogeneousRangingLinearSolver() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), estimatedAccessPoint.getTransmittedPower(),
                    0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimator.getEstimatedPathLossExponent(), 0.0);
            assertEquals(MAX_PATH_LOSS_EXPONENT, estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
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

        final NumberFormat format = NumberFormat.getPercentInstance();
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
    public void testEstimateLargerRangingPreliminarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLargerRssiPreliminarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLargerRangingAndRssiPreliminarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        accessPoint.getFrequency(), MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>(qualityScores, readings, this);
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

            final WifiAccessPointWithPowerAndLocated3D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated3D) estimator.getEstimatedRadioSource();

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
        final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new SequentialRobustRangingAndRssiRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(
            final SequentialRobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateStart++;
        checkLocked((SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateEnd(
            final SequentialRobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateEnd++;
        checkLocked((SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final SequentialRobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
            final float progress) {
        estimateProgressChange++;
        checkLocked((SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateProgressChange = 0;
    }

    private double receivedPower(final double equivalentTransmittedPower, final double distance,
                                 final double frequency, final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), pathLossExponent);
        return equivalentTransmittedPower * k / Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(final SequentialRobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator) {
        try {
            estimator.setRangingPreliminarySubsetSize(5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiPreliminarySubsetSize(5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingRobustMethod(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiRobustMethod(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setResultRefined(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setCovarianceKept(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReadings(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialTransmittedPowerdBm(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialTransmittedPower(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialPathLossExponent(2.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTransmittedPowerEstimationEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPathLossEstimationEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setUseReadingPositionCovariances(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setHomogeneousRangingLinearSolverUsed(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
