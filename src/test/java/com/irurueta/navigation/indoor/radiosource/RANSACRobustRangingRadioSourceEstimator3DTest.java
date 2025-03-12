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
import com.irurueta.navigation.indoor.RangingReadingLocated3D;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
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

class RANSACRobustRangingRadioSourceEstimator3DTest implements
        RobustRangingRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            RANSACRobustRangingRadioSourceEstimator3DTest.class.getName());

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
    void testConstructor() {
        final var randomizer = new UniformRandomizer();

        // test empty constructor
        var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default values
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
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
        final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings);

        // check default values
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
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
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RANSACRobustRangingRadioSourceEstimator3D<>(
                (List<RangingReadingLocated3D<WifiAccessPoint>>) null));
        final var wrongReadings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
        assertThrows(IllegalArgumentException.class, () -> new RANSACRobustRangingRadioSourceEstimator3D<>(
                wrongReadings));

        // test constructor with listener
        estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(this);

        // check default values
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
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
        estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, this);

        // check default values
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
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
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RANSACRobustRangingRadioSourceEstimator3D<>(
                (List<RangingReadingLocated3D<WifiAccessPoint>>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRobustRangingRadioSourceEstimator3D<>(
                wrongReadings, this));

        // test constructor with initial position
        final var initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition);

        // check default values
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
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
        estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition);

        // check default values
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
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
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RANSACRobustRangingRadioSourceEstimator3D<>(
                null, initialPosition));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRobustRangingRadioSourceEstimator3D<>(
                wrongReadings, initialPosition));

        // test constructor with initial position and listener
        estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(initialPosition, this);

        // check default values
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
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
        estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, initialPosition, this);

        // check default values
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(),
                0.0);
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertEquals(4, estimator.getMinReadings());
        assertEquals(4, estimator.getPreliminarySubsetSize());
        assertEquals(3, estimator.getNumberOfDimensions());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
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
        assertNull(estimator.getQualityScores());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RANSACRobustRangingRadioSourceEstimator3D<>(
                null, initialPosition, this));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRobustRangingRadioSourceEstimator3D<>(
                wrongReadings, initialPosition, this));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check
        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(
                !RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS);

        // check
        assertEquals(!RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(
                !RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);

        // check
        assertEquals(!RANSACRobustRangingRadioSourceEstimator3D.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

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
    void testGetSetUseReadingPositionCovariance() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

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
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

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
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

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
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check
        assertEquals(10, estimator.getMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(!RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(!RobustRangingRadioSourceEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(!RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE);

        // check
        assertEquals(!RobustRangingRadioSourceEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
    }

    @Test
    void testAreValidReadings() {
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(new ArrayList<>()));
    }

    @Test
    void testGetSetReadings() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 5; i++) {
            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

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
        final var wrongReadings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setReadings(wrongReadings));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        estimator.setQualityScores(new double[3]);

        // check
        assertNull(estimator.getQualityScores());
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertEquals(4, estimator.getPreliminarySubsetSize());

        // set new value
        estimator.setPreliminarySubsetSize(5);

        // check
        assertEquals(5, estimator.getPreliminarySubsetSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setPreliminarySubsetSize(3));
    }

    @Test
    void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();

        // check default value
        assertTrue(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(false);

        // check
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
    }

    @Test
    void testEstimateNoInlierErrorNoRefinement() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, this);

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

            final var estimatedAccessPoint = (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

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
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoInlierErrorWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, this);

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

            final var estimatedAccessPoint = (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInlierErrorWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);
        final var inlierErrorRandomizer = new GaussianRandomizer(0.0, INLIER_ERROR_STD);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < 2 * TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }

                error += inlierErrorRandomizer.nextDouble();

                readings.add(new RangingReadingLocated3D<>(accessPoint, Math.abs(distance + error),
                        readingsPositions[i]));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, this);

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

            final var estimatedAccessPoint = (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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

        // force NotReadyException
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoInlierErrorWithInitialPositionAndRefinement() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, accessPointPosition,
                    this);

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

            final var estimatedAccessPoint = (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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

        // force NotReadyException
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateBeacon() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            final var beacon = new Beacon(Collections.singletonList(identifier), TRANSMITTED_POWER_DBM, FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<Beacon>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(beacon, distance + error, readingsPositions[i]));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings);

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

            final var estimatedBeacon = (BeaconLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(beacon.getIdentifiers(), estimatedBeacon.getIdentifiers());
            assertEquals(TRANSMITTED_POWER_DBM, estimatedBeacon.getTransmittedPower(), 0.0);
            assertEquals(beacon.getFrequency(), estimatedBeacon.getFrequency(), 0.0);
            assertEquals(estimator.getEstimatedPosition(), estimatedBeacon.getPosition());
            assertEquals(estimator.getEstimatedPositionCovariance(), estimatedBeacon.getPositionCovariance());

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
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithStandardDeviationsAndPositionCovariance() throws LockedException, NotReadyException,
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
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                final var positionCovariance = Matrix.diagonal(new double[]{
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD,
                        INLIER_ERROR_STD * INLIER_ERROR_STD});

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error, readingsPositions[i],
                        INLIER_ERROR_STD, positionCovariance));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, this);
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

            final var estimatedAccessPoint = (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
        final var estimator = new LMedSRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateHomogeneousLinearSolver() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, this);

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

            final var estimatedAccessPoint = (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
    }

    @Test
    void testEstimateInhomogeneousLinearSolver() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance + error,
                        readingsPositions[i]));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>(readings,
                    this);

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

            final var estimatedAccessPoint = (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals("bssid", estimatedAccessPoint.getBssid());
            assertEquals(FREQUENCY, estimatedAccessPoint.getFrequency(), 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimator.getEstimatedPosition(), estimatedAccessPoint.getPosition());
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
    }

    @Test
    void testEstimateLargerPreliminarySubsetSize() throws LockedException, NotReadyException, RobustEstimatorException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var accessPointPosition = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            final var readingsPositions = new Point3D[numReadings];
            final var readings = new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>();
            for (var i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var distance = readingsPositions[i].distanceTo(accessPointPosition);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    // inlier
                    error = 0.0;
                }

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<>(readings, this);

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

            final var estimatedAccessPoint = (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

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
        final var estimator = new RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateStart++;
        checkLocked((RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateEnd(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        estimateEnd++;
        checkLocked((RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateNextIteration(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                        final int iteration) {
        estimateNextIteration++;
        checkLocked((RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    @Override
    public void onEstimateProgressChange(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint>) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final RANSACRobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator) {
        assertThrows(LockedException.class, () -> estimator.setPreliminarySubsetSize(4));
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setComputeAndKeepInliersEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setComputeAndKeepResidualsEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.8));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setResultRefined(false));
        assertThrows(LockedException.class, () -> estimator.setCovarianceKept(false));
        assertThrows(LockedException.class, () -> estimator.setReadings(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setHomogeneousLinearSolverUsed(false));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
