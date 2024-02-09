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
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Beacon;
import com.irurueta.navigation.indoor.BeaconIdentifier;
import com.irurueta.navigation.indoor.BeaconLocated3D;
import com.irurueta.navigation.indoor.RangingReadingLocated;
import com.irurueta.navigation.indoor.RangingReadingLocated3D;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
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

public class PROSACRobustRangingRadioSourceEstimator3DTest implements
        RobustRangingRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            PROSACRobustRangingRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; // (Hz)
    private static final double TRANSMITTED_POWER_DBM = -50.0;

    private static final int MIN_READINGS = 50;
    private static final int MAX_READINGS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double INLIER_ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;

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
        PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // test constructor with readings
        final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(readings);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated3D<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(this);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // test constructor with readings and listener
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(readings, this);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated3D<WifiAccessPoint>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(initialPosition);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // test constructor with readings and initial position
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated<WifiAccessPoint, Point3D>>) null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(), initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with initial position and listener
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(initialPosition, this);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // test constructor with readings, initial position and listener
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, this);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated<WifiAccessPoint, Point3D>>) null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(), initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[readings.size()];
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // test constructor with quality scores and readings
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                    (List<RangingReadingLocated3D<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(new double[1], readings);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and listener
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, this);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // test constructor with quality scores, readings and listener
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, this);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                    (List<RangingReadingLocated3D<WifiAccessPoint>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(new double[1], readings, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and initial position
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, initialPosition);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // test constructor with quality scores, readings and initial position
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, initialPosition);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertNull(estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(), initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(new double[1], readings, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, initial position and listener
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, initialPosition, this);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // test constructor with quality scores, readings, initial position and listener
        estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                readings, initialPosition, this);

        // check default values
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                    null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(), initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACRobustRangingRadioSourceEstimator3D<>(new double[1], readings,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check
        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(
                !PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);

        // check
        assertEquals(!PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(
                !PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);

        // check
        assertEquals(!PROSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
    }

    @Test
    public void testGetSetUseReadingPositionCovariance() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());

        // set new value
        estimator.setUseReadingPositionCovariances(
                !RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES);

        // check
        assertEquals(!RobustRangingRadioSourceEstimator.DEFAULT_USE_READING_POSITION_COVARIANCES,
                estimator.getUseReadingPositionCovariance());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);

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
    public void testGetSetConfidence() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(),
                0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check
        assertEquals(0.5, estimator.getConfidence(), 0.0);

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
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check
        assertEquals(10, estimator.getMaxIterations());

        // force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(!RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(!RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(!RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        // check
        assertEquals(!RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
    }

    @Test
    public void testAreValidReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(new ArrayList<RangingReadingLocated<WifiAccessPoint, Point3D>>()));
    }

    @Test
    public void testGetSetReadings() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // initial value
        assertNull(estimator.getReadings());
        assertFalse(estimator.isReady());

        // set new value
        estimator.setReadings(readings);

        // check
        assertSame(readings, estimator.getReadings());
        assertFalse(estimator.isReady());

        // set quality scores
        estimator.setQualityScores(new double[readings.size()]);

        // check
        assertTrue(estimator.isReady());

        // force IllegalArgumentException
        try {
            estimator.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setReadings(new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[4];
        estimator.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, estimator.getQualityScores());

        // force IllegalArgumentException
        try {
            estimator.setQualityScores(new double[3]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertEquals(4, estimator.getPreliminarySubsetSize());

        // set new value
        estimator.setPreliminarySubsetSize(5);

        // check
        assertEquals(5, estimator.getPreliminarySubsetSize());

        // force IllegalArgumentException
        try {
            estimator.setPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();

        // check default value
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(false);

        // check
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
    }

    @Test
    public void testEstimateNoInlierErrorNoRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, this);

            estimator.setResultRefined(false);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
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

            assertNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);


        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);

        // force NotReadyException
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, this);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
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

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInlierErrorWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < 10 * TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

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

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        Math.abs(distance + error), readingsPositions[i]));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, this);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
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

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
            if (positionError > LARGE_POSITION_ERROR) {
                continue;
            }

            assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition, LARGE_POSITION_ERROR));
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
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoInlierErrorWithInitialPositionAndRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores,
                            readings, accessPointPosition, this);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
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

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateBeacon() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final Beacon beacon = new Beacon(Collections.singletonList(identifier), TRANSMITTED_POWER_DBM, FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<Beacon>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingReadingLocated3D<>(beacon, distance + error, readingsPositions[i]));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<Beacon> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());

            estimator.estimate();

            // check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            final BeaconLocated3D estimatedBeacon = (BeaconLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(beacon.getIdentifiers(), estimatedBeacon.getIdentifiers());
            assertEquals(TRANSMITTED_POWER_DBM, estimatedBeacon.getTransmittedPower(), 0.0);
            assertEquals(beacon.getFrequency(), estimatedBeacon.getFrequency(), 0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedBeacon.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedBeacon.getPositionCovariance());

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
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithStandardDeviationsAndPositionCovariance() throws LockedException, NotReadyException,
            RobustEstimatorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                final Matrix positionCovariance = Matrix.diagonal(new double[]{
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD});

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error,
                        readingsPositions[i], INLIER_ERROR_STD, positionCovariance));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, this);
            estimator.setUseReadingPositionCovariances(true);

            estimator.setResultRefined(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
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

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double) numValidPosition / (double) TIMES * 100.0);


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
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateHomogeneousLinearSolver() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, this);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setHomogeneousLinearSolverUsed(true);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
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

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateInhomogeneousLinearSolver() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        double positionStd = 0.0, positionStdConfidence = 0.0;
        double positionAccuracy = 0.0, positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, this);

            estimator.setResultRefined(true);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
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

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLargerPreliminarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionError = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final Point3D[] readingsPositions = new Point3D[numReadings];
            final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] qualityScores = new double[numReadings];
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new PROSACRobustRangingRadioSourceEstimator3D<>(qualityScores, readings, this);

            estimator.setResultRefined(false);
            estimator.setComputeAndKeepInliersEnabled(false);
            estimator.setComputeAndKeepResidualsEnabled(false);

            estimator.setPreliminarySubsetSize(estimator.getMinReadings() + 1);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
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

            assertNull(estimator.getInliersData());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getEstimatedPositionCovariance());

            final WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

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

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            break;
        }

        assertTrue(numValidPosition > 0);


        LOGGER.log(Level.INFO, "Position error: {0} meters", positionError);

        // force NotReadyException
        final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new PROSACRobustRangingRadioSourceEstimator3D<>();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateStart++;
        checkLocked((PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateEnd(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateEnd++;
        checkLocked((PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateNextIteration(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateProgressChange(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private void checkLocked(final PROSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator) {
        try {
            estimator.setPreliminarySubsetSize(3);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setQualityScores(null);
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
            estimator.setHomogeneousLinearSolverUsed(false);
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
