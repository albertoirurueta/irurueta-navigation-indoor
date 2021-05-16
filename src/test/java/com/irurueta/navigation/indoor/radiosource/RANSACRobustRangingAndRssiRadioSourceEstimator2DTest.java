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
import org.junit.Assert;
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

public class RANSACRobustRangingAndRssiRadioSourceEstimator2DTest implements
        RobustRangingAndRssiRadioSourceEstimatorListener<WifiAccessPoint, Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            RANSACRobustRangingAndRssiRadioSourceEstimator2DTest.class.getName());

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
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // test empty constructor
        RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check
        assertEquals(estimator.getThreshold(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings
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

        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings);

        // check
        assertEquals(estimator.getThreshold(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(this);

        // check
        assertEquals(estimator.getThreshold(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, this);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings and initial position
        final InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, initialPosition);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                initialPosition);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                initialPosition, this);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, initialPosition, this);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertNull(estimator.getInitialTransmittedPowerdBm());
        assertNull(estimator.getInitialTransmittedPower());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial transmitted power
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                MAX_RSSI);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        Assert.assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings and initial transmitted power
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, MAX_RSSI);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial transmitted power and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                MAX_RSSI, this);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial transmitted power and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, MAX_RSSI, this);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertNull(estimator.getInitialPosition());
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    (List<RangingAndRssiReadingLocated2D<WifiAccessPoint>>) null, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings, initial position and initial transmitted power
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, initialPosition, MAX_RSSI);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    null, initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), initialPosition, MAX_RSSI);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position and initial transmitted power
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                initialPosition, MAX_RSSI);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                initialPosition, MAX_RSSI, this);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted
        // power and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, initialPosition, MAX_RSSI, this);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    null, initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with readings, initial position, initial transmitted power
        // and initial path loss exponent
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        // check
        assertEquals(estimator.getThreshold(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    null, initialPosition, MAX_RSSI,
                    MIN_PATH_LOSS_EXPONENT);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), initialPosition,
                    MAX_RSSI, MIN_PATH_LOSS_EXPONENT);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position, initial transmitted power and
        // initial path loss exponent
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        // check
        assertEquals(estimator.getThreshold(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with initial position, initial transmitted power,
        // initial path loss exponent and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this);

        // check
        assertEquals(estimator.getThreshold(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // test constructor with readings, initial position, initial transmitted power
        // initial path loss exponent and listener
        estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this);

        // check
        assertEquals(estimator.getThreshold(), RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getPreliminarySubsetSize(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(), Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertFalse(estimator.isPathLossEstimationEnabled());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertEquals(estimator.isResultRefined(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
        assertEquals(estimator.isCovarianceKept(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedTransmittedPowerVariance());
        assertEquals(estimator.getEstimatedTransmittedPower(), 1.0, 0.0);
        assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertEquals(estimator.getEstimatedPathLossExponent(),
                RangingAndRssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(estimator.getEstimatedPathLossExponentVariance());
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    null, initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                    new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>(), initialPosition, MAX_RSSI, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getThreshold(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_THRESHOLD,
                0.0);

        // set new value
        estimator.setThreshold(50.0);

        // check
        assertEquals(estimator.getThreshold(), 50.0, 0.0);

        // force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);

        // set new value
        estimator.setComputeAndKeepInliersEnabled(
                !RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);

        // check
        assertEquals(estimator.isComputeAndKeepInliersEnabled(),
                !RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(
                !RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);

        // check
        assertEquals(estimator.isComputeAndKeepResidualsEnabled(),
                !RANSACRobustRangingAndRssiRadioSourceEstimator2D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testGetMinReadings() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

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
    public void testGetSetInitialTransmittedPowerdBm() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getInitialTransmittedPowerdBm());

        // set new value
        estimator.setInitialTransmittedPowerdBm(MAX_RSSI);

        // check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
    }

    @Test
    public void testGetSetInitialTransmittedPower() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getInitialTransmittedPower());

        // set new value
        final double power = Utils.dBmToPower(MAX_RSSI);
        estimator.setInitialTransmittedPower(power);

        // check
        assertEquals(estimator.getInitialTransmittedPower(), power, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testGetSetInitialPathLossExponent() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getInitialPathLossExponent(),
                RssiRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                0.0);

        // set new value
        final double pathLossExponent = randomizer.nextDouble(
                MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
        estimator.setInitialPathLossExponent(pathLossExponent);

        // check
        assertEquals(estimator.getInitialPathLossExponent(),
                pathLossExponent, 0.0);
    }

    @Test
    public void testIsSetTransmittedPowerEstimationEnabled() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertTrue(estimator.isTransmittedPowerEstimationEnabled());

        // set new value
        estimator.setTransmittedPowerEstimationEnabled(false);

        // check
        assertFalse(estimator.isTransmittedPowerEstimationEnabled());
    }

    @Test
    public void testIsSetPathLossEstimationEnabled() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertFalse(estimator.isPathLossEstimationEnabled());

        // set new value
        estimator.setPathLossEstimationEnabled(true);

        // check
        assertTrue(estimator.isPathLossEstimationEnabled());
    }

    @Test
    public void testGetSetUseReadingPositionCovariance() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getUseReadingPositionCovariance(),
                RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // set new value
        estimator.setUseReadingPositionCovariances(
                !RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // check
        assertEquals(estimator.getUseReadingPositionCovariance(),
                !RobustRangingAndRssiRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                RobustRssiRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);

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
    public void testGetSetConfidence() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getConfidence(),
                RobustRssiRadioSourceEstimator.DEFAULT_CONFIDENCE,
                0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check
        assertEquals(estimator.getConfidence(), 0.5, 0.0);

        // force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                RobustRssiRadioSourceEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(10);

        // check
        assertEquals(estimator.getMaxIterations(), 10);

        // force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.isResultRefined(),
                RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        // set new value
        estimator.setResultRefined(
                !RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(estimator.isResultRefined(),
                !RobustRssiRadioSourceEstimator.DEFAULT_REFINE_RESULT);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.isCovarianceKept(),
                RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        // set new value
        estimator.setCovarianceKept(
                !RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        // check
        assertEquals(estimator.isCovarianceKept(),
                !RobustRssiRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);
    }

    @Test
    public void testAreValidReadings() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, 0.0, 0.0, position));
        }

        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        estimator.setTransmittedPowerEstimationEnabled(true);
        estimator.setPathLossEstimationEnabled(false);

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RangingAndRssiReadingLocated<WifiAccessPoint, Point2D>>()));
    }

    @Test
    public void testGetSetReadings() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        estimator.setTransmittedPowerEstimationEnabled(false);
        estimator.setInitialTransmittedPowerdBm(MAX_RSSI);
        estimator.setPathLossEstimationEnabled(false);
        estimator.setInitialPathLossExponent(MAX_PATH_LOSS_EXPONENT);

        // check default value
        assertNull(estimator.getReadings());
        assertFalse(estimator.isReady());

        // set new value
        final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 3; i++) {
            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, 0.0, 0.0, position));
        }

        // set new value
        estimator.setReadings(readings);

        // check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator.isReady());

        // force IllegalArgumentException
        try {
            estimator.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setReadings(new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        estimator.setQualityScores(new double[3]);

        // check
        assertNull(estimator.getQualityScores());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertEquals(estimator.getPreliminarySubsetSize(), 4);

        // set new value
        estimator.setPreliminarySubsetSize(5);

        // check
        assertEquals(estimator.getPreliminarySubsetSize(), 5);

        // force IllegalArgumentException
        try {
            estimator.setPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetHomogeneousRangingLinearSolverUsed() throws LockedException {
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();

        // check default value
        assertTrue(estimator.isHomogeneousRangingLinearSolverUsed());

        // set new value
        estimator.setHomogeneousRangingLinearSolverUsed(false);

        // check
        assertFalse(estimator.isHomogeneousRangingLinearSolverUsed());
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinementNoInlierDataAndNoResiduals()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinementNoInlierDataAndWithResiduals()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinementWithInlierDataAndNoResiduals()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinementWithInlierDataAndWithResiduals()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithRefinementNoInlierDataAndNoResiduals()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithRefinementNoInlierDataAndWithResiduals()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithRefinementWithInlierDataAndWithResiduals()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInlierErrorWithRefinementWithInlierDataAndWithResiduals()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i],
                        INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings = new ArrayList<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final InhomogeneousPoint2D initialPosition =
                    new InhomogeneousPoint2D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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
            final List<RangingAndRssiReadingLocated2D<WifiAccessPoint>> readings =
                    new ArrayList<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final InhomogeneousPoint2D initialPosition =
                    new InhomogeneousPoint2D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());
            final double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i],
                        INLIER_ERROR_STD, INLIER_ERROR_STD));
            }

            final InhomogeneousPoint2D initialPosition =
                    new InhomogeneousPoint2D(
                            accessPointPosition.getInhomX() + inlierErrorRandomizer.nextDouble(),
                            accessPointPosition.getInhomY() + inlierErrorRandomizer.nextDouble());
            final double initialTransmittedPowerdBm = transmittedPowerdBm +
                    inlierErrorRandomizer.nextDouble();

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPosition(initialPosition);
            estimator.setInitialTransmittedPowerdBm(initialTransmittedPowerdBm);
            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimatePositionTransmittedPowerAndPathLossEstimationEnabled()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimatePositionTransmittedPowerAndPathLossEstimationEnabledWithInitialValues()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setInitialPathLossExponent(pathLossExponent);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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

        int numValidPosition = 0, numValidPower = 0, numValid = 0;
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setInitialPathLossExponent(pathLossExponent);
            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
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

            numValid++;
        }

        assertTrue(numValidPosition > 0);
        assertTrue(numValidPower > 0);
        assertTrue(numValid > 0);


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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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
            final List<RangingAndRssiReadingLocated2D<Beacon>> readings = new ArrayList<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(beacon,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<Beacon> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

            final BeaconWithPowerAndLocated2D estimatedBeacon =
                    (BeaconWithPowerAndLocated2D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedBeacon.getIdentifiers(), beacon.getIdentifiers());
            assertEquals(estimatedBeacon.getTransmittedPower(),
                    estimator.getEstimatedTransmittedPowerdBm(), 0.0);
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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // repeat again so that position covariance matrix is reused
            estimator.estimate();

            // check
            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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

        LOGGER.log(Level.INFO, "Position error: {0} meters",
                positionError);

        // force NotReadyException
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimatePositionAndPathloss() throws LockedException, NotReadyException, RobustEstimatorException,
            AlgebraException {
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setInitialPosition(accessPointPosition);
            estimator.setTransmittedPowerEstimationEnabled(false);
            estimator.setInitialTransmittedPowerdBm(transmittedPowerdBm);
            estimator.setPathLossEstimationEnabled(true);
            estimator.setInitialPathLossExponent(pathLossExponent);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNotNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
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

                final Matrix positionCovariance = Matrix.diagonal(new double[]{
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD});

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i],
                        INLIER_ERROR_STD, INLIER_ERROR_STD, positionCovariance));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);
            estimator.setUseReadingPositionCovariances(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final MSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new MSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateHomogeneousRangingLinearSolverUsed()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setHomogeneousRangingLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
    public void testEstimateInhomogeneousRangingLinearSolverUsed()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setHomogeneousRangingLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());
            assertNotNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
    public void testEstimateLargerPreliminarySubsetSize()
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

                readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint,
                        distance, rssi + error, readingsPositions[i]));
            }

            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                    new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>(
                            readings, this);
            estimator.setTransmittedPowerEstimationEnabled(true);
            estimator.setPathLossEstimationEnabled(false);

            estimator.setResultRefined(false);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            estimator.setPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimator.getEstimatedTransmittedPower(), 1.0,
                    0.0);
            assertEquals(estimator.getEstimatedTransmittedPowerdBm(), 0.0,
                    0.0);
            assertEquals(estimator.getEstimatedPathLossExponent(),
                    MAX_PATH_LOSS_EXPONENT, 0.0);
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);

            estimator.estimate();

            // check
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());
            assertNull(estimator.getEstimatedTransmittedPowerVariance());
            assertNull(estimator.getEstimatedPathLossExponentVariance());

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
        final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator =
                new RANSACRobustRangingAndRssiRadioSourceEstimator2D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(
            final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateStart++;
        checkLocked((RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateEnd(
            final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        estimateEnd++;
        checkLocked((RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator,
            final int iteration) {
        estimateNextIteration++;
        checkLocked((RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator,
            final float progress) {
        estimateProgressChange++;
        checkLocked((RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint>) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
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

    private void checkLocked(
            final RANSACRobustRangingAndRssiRadioSourceEstimator2D<WifiAccessPoint> estimator) {
        try {
            estimator.setPreliminarySubsetSize(4);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPathLossEstimationEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTransmittedPowerEstimationEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setComputeAndKeepInliersEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setComputeAndKeepResidualsEnabled(false);
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
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(10);
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
