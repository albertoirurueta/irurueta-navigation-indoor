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
import com.irurueta.navigation.indoor.*;
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

public class SequentialRobustMixedRadioSourceEstimator2DTest implements
        SequentialRobustMixedRadioSourceEstimatorListener<WifiAccessPoint, Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            SequentialRobustMixedRadioSourceEstimator2DTest.class.getName());

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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // test empty constructor
        SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings
        final List<ReadingLocated<Point2D>> readings =
                new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 4; i++) {
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                    0.0, 0.0, position));
        }

        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                readings, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings and initial position
        final InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));

        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings,
                initialPosition);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                initialPosition);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                initialPosition, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings,
                initialPosition, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings,
                MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(MAX_RSSI,
                this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings,
                MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings, initial position and initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings,
                initialPosition, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position and initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                initialPosition, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                initialPosition, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power
        // and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings,
                initialPosition, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings, initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings,
                initialPosition, MAX_RSSI, 1.0);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                initialPosition, MAX_RSSI, 1.0);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                initialPosition, MAX_RSSI, 1.0, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(readings,
                initialPosition, MAX_RSSI, 1.0, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    (List<ReadingLocated<Point2D>>) null,
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[4];

        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores and readings
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    (List<ReadingLocated<Point2D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    (List<ReadingLocated<Point2D>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>(),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, readings and initial position
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, initialPosition);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and initial position
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, initialPosition);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, initial position and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, initialPosition, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings, initial position and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, initialPosition, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores, null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings and initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    (List<ReadingLocated<Point2D>>) null,
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>(),
                    MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, initial transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, readings, initial transmitted power
        // and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    (List<ReadingLocated<Point2D>>) null,
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>(),
                    MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, readings, initial position and
        // initial transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, initialPosition, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores, null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, initial position and initial
        // transmitted power
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, initialPosition, MAX_RSSI);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with quality scores, initial position, initial
        // transmitted power and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, initialPosition, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power
        // and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, initialPosition, MAX_RSSI, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MixedRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores, null,
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings, initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, initialPosition, MAX_RSSI,
                1.0);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores, null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(),
                    initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position, initial transmitted power
        // and initial path-loss exponent
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, initialPosition, MAX_RSSI,
                1.0);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, initialPosition, MAX_RSSI,
                1.0, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power,
        // initial path-loss exponent and listener
        estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                qualityScores, readings, initialPosition, MAX_RSSI,
                1.0, this);

        // check default values
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertEquals(estimator.isResultRefined(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI),
                ABSOLUTE_ERROR);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
        assertEquals(estimator.isPathLossEstimationEnabled(),
                MixedRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustMixedRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isRssiPositionEnabled());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores, null, initialPosition, MAX_RSSI,
                    1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedRadioSourceEstimator2D<>(
                    qualityScores,
                    new ArrayList<ReadingLocated<Point2D>>(),
                    initialPosition, MAX_RSSI, 1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(estimator.getProgressDelta(), 0.5f, 0.0);

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
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getRangingRobustMethod(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_PANGING_ROBUST_METHOD);

        // set new value
        estimator.setRangingRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(estimator.getRangingRobustMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testGetSetRssiRobustMethod() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getRssiRobustMethod(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_RSSI_ROBUST_METHOD);

        // set new value
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(estimator.getRssiRobustMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testGetSetRangingThreshold() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getRangingThreshold());

        // set new value
        estimator.setRangingThreshold(INLIER_ERROR_STD);

        // check
        assertEquals(estimator.getRangingThreshold(), INLIER_ERROR_STD, 0.0);
    }

    @Test
    public void testGetSetRssiThreshold() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getRssiThreshold());

        // set new value
        estimator.setRssiThreshold(INLIER_ERROR_STD);

        // check
        assertEquals(estimator.getRssiThreshold(), INLIER_ERROR_STD, 0.0);
    }

    @Test
    public void testGetSetRangingConfidence() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getRangingConfidence(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);

        // set new value
        estimator.setRangingConfidence(0.5);

        // check
        assertEquals(estimator.getRangingConfidence(), 0.5, 0.0);

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
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getRssiConfidence(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);

        // set new value
        estimator.setRssiConfidence(0.5);

        // check
        assertEquals(estimator.getRssiConfidence(), 0.5, 0.0);

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
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getRangingMaxIterations(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setRangingMaxIterations(50);

        // check
        assertEquals(estimator.getRangingMaxIterations(), 50);

        // force IllegalArgumentException
        try {
            estimator.setRangingMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRssiMaxIterations() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getRssiMaxIterations(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setRssiMaxIterations(50);

        // check
        assertEquals(estimator.getRssiMaxIterations(), 50);

        // force IllegalArgumentException
        try {
            estimator.setRssiMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.isResultRefined(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        // set new value
        estimator.setResultRefined(
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(estimator.isResultRefined(),
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.isCovarianceKept(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        // set new value
        estimator.setCovarianceKept(
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        // check
        assertEquals(estimator.isCovarianceKept(),
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
    }

    @Test
    public void testGetSetReadings() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getReadings());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 4; i++) {
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                    0.0, 0.0, position));
        }

        estimator.setReadings(readings);

        // check correctness
        assertSame(estimator.getReadings(), readings);

        // force IllegalArgumentException
        try {
            estimator.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setReadings(new ArrayList<ReadingLocated<Point2D>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[4];
        estimator.setQualityScores(qualityScores);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);

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
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 4);

        // set new value
        estimator.setRangingPreliminarySubsetSize(5);

        // check
        assertEquals(estimator.getRangingPreliminarySubsetSize(), 5);

        // force IllegalArgumentException
        try {
            estimator.setRangingPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRssiPreliminarySubsetSize() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 4);

        // set new value
        estimator.setRssiPreliminarySubsetSize(5);

        // check
        assertEquals(estimator.getRssiPreliminarySubsetSize(), 5);

        // force IllegalArgumentException
        try {
            estimator.setRssiPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getInitialTransmittedPowerdBm());

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);

        // check
        assertEquals(estimator.getInitialTransmittedPowerdBm(),
                transmittedPowerdBm, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(transmittedPowerdBm), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetInitialTransmittedPower() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getInitialTransmittedPower());

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
        final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
        estimator.setInitialTransmittedPower(transmittedPower);

        // check
        assertEquals(estimator.getInitialTransmittedPowerdBm(),
                transmittedPowerdBm, ABSOLUTE_ERROR);
        assertEquals(estimator.getInitialTransmittedPower(),
                transmittedPower, 0.0);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final Point2D initialPosition = Point2D.create();
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testGetSetInitialPathLossExponent() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);

        // set new value
        estimator.setInitialPathLossExponent(1.0);

        // check
        assertEquals(estimator.getInitialPathLossExponent(), 1.0, 0.0);
    }

    @Test
    public void testIsSetTransmittedPowerEstimationEnabled() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);

        // set new value
        estimator.setTransmittedPowerEstimationEnabled(
                !RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);

        // check
        assertEquals(estimator.isTransmittedPowerEstimationEnabled(),
                !RangingAndRssiRadioSourceEstimator.DEFAULT_TRANSMITTED_POWER_ESTIMATION_ENABLED);
    }

    @Test
    public void testIsSetPathLossEstimationEnabled() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.isPathLossEstimationEnabled(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);

        // set new value
        estimator.setPathLossEstimationEnabled(
                !RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);

        // check
        assertEquals(estimator.isPathLossEstimationEnabled(),
                !RangingAndRssiRadioSourceEstimator.DEFAULT_PATHLOSS_ESTIMATION_ENABLED);
    }

    @Test
    public void testGetSetUseReadingPositionCovariance() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getUseReadingPositionCovariance(),
                SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // set new value
        estimator.setUseReadingPositionCovariances(
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // check
        assertEquals(estimator.getUseReadingPositionCovariance(),
                !SequentialRobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
    }

    @Test
    public void testIsSetHomogeneousRangingLinearSolverUsed() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

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

        final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                    0.0, 0.0, position));
        }

        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RangingAndRssiReadingLocated<WifiAccessPoint, Point2D>>()));
    }

    @Test
    public void testGetMinReadings() throws LockedException {
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getMinReadings(), 4);

        // position only
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(estimator.getMinReadings(), 3);

        // position and transmitted power
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        // check
        assertEquals(estimator.getMinReadings(), 4);

        // position and path-loss
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(estimator.getMinReadings(), 4);

        // position, transmitted power and path-loss
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertEquals(estimator.getMinReadings(), 5);
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinement()
            throws LockedException, NotReadyException, RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithRefinement()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInlierErrorWithRefinement()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, LARGE_POWER_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPosition()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final InhomogeneousPoint2D initialPosition =
                    new InhomogeneousPoint2D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPower()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPositionAndPower()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final InhomogeneousPoint2D initialPosition =
                    new InhomogeneousPoint2D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());
            final double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInlierErrorWithInitialPositionAndPower()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error,
                        readingsPositions[i], INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            final InhomogeneousPoint2D initialPosition =
                    new InhomogeneousPoint2D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());
            final double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    LARGE_POSITION_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > LARGE_POWER_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, LARGE_POWER_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithPathLossEstimationEnabled()
            throws LockedException, NotReadyException, RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0, numValidPathLoss = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double pathLossError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        double pathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
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
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);

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

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);
            pathLossStd = Math.sqrt(pathLossVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() -
                    pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);
        LOGGER.log(Level.INFO, "Path loss error: {0}",
                pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}",
                pathLossStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInitialPathLoss()
            throws LockedException, NotReadyException, RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPathLossExponent(pathLossExponent);
            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
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
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D beaconPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);

            final BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final Beacon beacon = new Beacon(Collections.singletonList(identifier),
                    transmittedPowerdBm, FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<Beacon>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        beaconPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        beacon.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(beacon,
                        distance, rssi + error,
                        readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<Beacon> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final BeaconWithPowerAndLocated2D estimatedBeacon =
                    (BeaconWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedBeacon.getIdentifiers(), beacon.getIdentifiers());
            assertEquals(estimatedBeacon.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), ABSOLUTE_ERROR);
            assertEquals(estimatedBeacon.getFrequency(), beacon.getFrequency(), 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedBeacon.getPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedBeacon.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedBeacon.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedBeacon.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(beaconPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(beaconPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
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
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
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
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
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
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);

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
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
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
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // repeat again so that position covariance matrix is reused
            estimator.estimate();

            // check
            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, 0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);

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

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
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
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPathLoss = 0;
        double positionError = 0.0;
        double pathLossError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        double pathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            pathLossStd = Math.sqrt(pathLossVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() -
                    pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Path loss error: {0}",
                pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}",
                pathLossStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
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
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPathLoss = 0;
        double positionError = 0.0;
        double pathLossError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        double pathLossStd = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error,
                        readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
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
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    estimatedAccessPoint.getPathLossExponent(), 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertEquals(estimatedAccessPoint.getPathLossExponentStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedPathLossExponentVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            final double pathLossVariance = estimator.getEstimatedPathLossExponentVariance();
            assertTrue(pathLossVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            pathLossStd = Math.sqrt(pathLossVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            pathLossError = Math.abs(estimator.getEstimatedPathLossExponent() -
                    pathLossExponent);
            if (pathLossError > PATH_LOSS_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedPathLossExponent(),
                    pathLossExponent, PATH_LOSS_ERROR);
            numValidPathLoss++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Avg. path loss error: {0}",
                pathLossError);

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Path loss error: {0}",
                pathLossError);
        LOGGER.log(Level.INFO, "Path loss standard deviation {0}",
                pathLossStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithStandardDeviationsAndPositionCovariance()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                final Matrix positionCovariance = Matrix.diagonal(new double[]{
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD});

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i],
                        INLIER_ERROR_STD, INLIER_ERROR_STD, positionCovariance));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setUseReadingPositionCovariances(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithMixedRangingAndRssiReadings()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<ReadingLocated<Point2D>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[3 * numReadings];
            int pos = 0;
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[pos] = 1.0;
                readings.add(new RangingReadingLocated2D<>(accessPoint, distance,
                        readingsPositions[i]));
                pos++;

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RssiReadingLocated2D<>(accessPoint, rssi + error,
                        readingsPositions[i]));
                pos++;

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
                pos++;
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithOnlyRssiReadings()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<ReadingLocated<Point2D>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            int pos = 0;
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RssiReadingLocated2D<>(accessPoint, rssi + error,
                        readingsPositions[i]));
                pos++;
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertTrue(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithOnlyRangingReadings()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<ReadingLocated<Point2D>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            int pos = 0;
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RangingReadingLocated2D<>(accessPoint,
                        distance + error, readingsPositions[i]));
                pos++;
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            // because transmitted power is required, estimator is not ready without
            // enough RSSI readings
            try {
                estimator.setReadings(readings);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            assertFalse(estimator.isReady());

            // if we enable only position estimation, then estimator becomes ready
            // when readings are set
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setReadings(readings);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointLocated2D estimatedAccessPoint =
                    (WifiAccessPointLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            assertNull(estimator.getEstimatedTransmittedPowerVariance());

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWitHomogeneousRangingLinearSolver()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<ReadingLocated<Point2D>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[3 * numReadings];
            int pos = 0;
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[pos] = 1.0;
                readings.add(new RangingReadingLocated2D<>(accessPoint, distance,
                        readingsPositions[i]));
                pos++;

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RssiReadingLocated2D<>(accessPoint, rssi + error,
                        readingsPositions[i]));
                pos++;

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
                pos++;
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setHomogeneousRangingLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);
    }

    @Test
    public void testEstimateWitInhomogeneousRangingLinearSolver()
            throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double powerStd = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<ReadingLocated<Point2D>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[3 * numReadings];
            int pos = 0;
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
                        MAX_PATH_LOSS_EXPONENT));

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[pos] = 1.0;
                readings.add(new RangingReadingLocated2D<>(accessPoint, distance,
                        readingsPositions[i]));
                pos++;

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RssiReadingLocated2D<>(accessPoint, rssi + error,
                        readingsPositions[i]));
                pos++;

                qualityScores[pos] = 1.0 / (1.0 + Math.abs(error));
                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
                pos++;
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setHomogeneousRangingLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getTransmittedPowerStandardDeviation(),
                    Math.sqrt(estimator.getEstimatedTransmittedPowerVariance()), 0.0);
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            final double powerVariance = estimator.getEstimatedTransmittedPowerVariance();
            assertTrue(powerVariance > 0.0);

            final Accuracy2D accuracyStd = new Accuracy2D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();
            powerStd = Math.sqrt(powerVariance);

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);
        LOGGER.log(Level.INFO, "Power standard deviation {0} dB",
                powerStd);
    }

    @Test
    public void testEstimateLargerRangingPreliminarySubsetSize()
            throws LockedException, NotReadyException, RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            estimator.setRangingPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLargerRssiPreliminarySubsetSize()
            throws LockedException, NotReadyException, RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            estimator.setRssiPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLargerRangingAndRssiPreliminarySubsetSize()
            throws LockedException, NotReadyException, RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0, numValidPower = 0;
        double positionError = 0.0;
        double powerError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint2D accessPointPosition =
                    new InhomogeneousPoint2D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(
                    MIN_READINGS, MAX_READINGS);
            final Point2D[] readingsPositions = new Point2D[numReadings];
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(
                        accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(
                        transmittedPower, distance,
                        accessPoint.getFrequency(),
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new SequentialRobustMixedRadioSourceEstimator2D<>(
                            qualityScores, readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);

            estimator.setRangingPreliminarySubsetSize(estimator.getMinReadings() + 1);
            estimator.setRssiPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedTransmittedPower());
            assertNull(estimator.getEstimatedTransmittedPowerdBm());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            assertNotNull(estimator.getInliersData());
            assertFalse(estimator.isRssiPositionEnabled());

            final WifiAccessPointWithPowerAndLocated2D estimatedAccessPoint =
                    (WifiAccessPointWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimatedAccessPoint.getPathLossExponent(), MAX_PATH_LOSS_EXPONENT, 0.0);
            assertNull(estimatedAccessPoint.getTransmittedPowerStandardDeviation());
            assertNull(estimatedAccessPoint.getPositionCovariance());
            assertNull(estimatedAccessPoint.getPathLossExponentStandardDeviation());

            positionError = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                    ABSOLUTE_ERROR));
            numValidPosition++;

            powerError = Math.abs(estimator.getEstimatedTransmittedPowerdBm() -
                    transmittedPowerdBm);
            if (powerError > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(estimator.getEstimatedTransmittedPower(), transmittedPower,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(),
                    transmittedPowerdBm, ABSOLUTE_ERROR);
            numValidPower++;

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            break;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);
        LOGGER.log(Level.INFO, "Power error: {0} dB",
                powerError);

        // force NotReadyException
        final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new SequentialRobustMixedRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(
            final SequentialRobustMixedRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateStart++;
        checkLocked((SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateEnd(
            final SequentialRobustMixedRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateEnd++;
        checkLocked((SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final SequentialRobustMixedRadioSourceEstimator<WifiAccessPoint, Point2D> estimator,
            final float progress) {
        estimateProgressChange++;
        checkLocked((SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateProgressChange = 0;
    }

    private double receivedPower(
            final double equivalentTransmittedPower, final double distance, final double frequency,
            final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(final SequentialRobustMixedRadioSourceEstimator2D<WifiAccessPoint> estimator) {
        try {
            estimator.setRangingPreliminarySubsetSize(4);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiPreliminarySubsetSize(4);
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
