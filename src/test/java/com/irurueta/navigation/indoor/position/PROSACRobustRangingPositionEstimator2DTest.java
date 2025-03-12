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

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.Accuracy2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated2D;
import com.irurueta.navigation.lateration.PROSACRobustLateration2DSolver;
import com.irurueta.navigation.lateration.RobustLaterationSolver;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class PROSACRobustRangingPositionEstimator2DTest implements RobustRangingPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(PROSACRobustRangingPositionEstimator2DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final int MIN_SOURCES = 100;
    private static final int MAX_SOURCES = 500;

    private static final int NUM_READINGS = 5;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 0.5;

    private static final int TIMES = 400;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private static final double INLIER_ERROR_STD = 0.1;

    private static final double RANGING_STD = 1.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstructor() {
        // empty constructor
        var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // constructor with sources
        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        for (var i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY, new InhomogeneousPoint2D()));
        }
        estimator = new PROSACRobustRangingPositionEstimator2D(sources);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustRangingPositionEstimator2D((List<WifiAccessPointLocated2D>) null));
        final var wrongSources = new ArrayList<WifiAccessPointLocated2D>();
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(wrongSources));

        // constructor with fingerprints
        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();
        estimator = new PROSACRobustRangingPositionEstimator2D(fingerprint);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>) null));

        // constructor with sources and fingerprint
        estimator = new PROSACRobustRangingPositionEstimator2D(sources, fingerprint);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(null,
                fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(wrongSources,
                fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(sources,
                (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>) null));

        // constructor with listener
        estimator = new PROSACRobustRangingPositionEstimator2D(this);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // constructor with sources and listener
        estimator = new PROSACRobustRangingPositionEstimator2D(sources, this);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                (List<WifiAccessPointLocated2D>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(wrongSources,
                this));

        // constructor with fingerprint and listener
        estimator = new PROSACRobustRangingPositionEstimator2D(fingerprint, this);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>) null, this));

        // constructor with sources, fingerprint and listener
        estimator = new PROSACRobustRangingPositionEstimator2D(sources, fingerprint, this);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(null,
                fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(wrongSources,
                fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(sources,
                null, this));

        // constructor with quality scores
        final var sourceQualityScores = new double[3];
        final var fingerprintReadingsQualityScores = new double[3];
        estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                null, fingerprintReadingsQualityScores));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, null));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, new double[1]));

        // constructor with quality scores and sources
        estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                null, fingerprintReadingsQualityScores, sources));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, null, sources));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, sources));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, new double[1], sources));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, (List<WifiAccessPointLocated2D>) null));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, wrongSources));

        // constructor with quality scores and fingerprints
        estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                fingerprint);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                null, fingerprintReadingsQualityScores, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, null, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, new double[1], fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>) null));

        // constructor with quality scores, sources and fingerprint
        estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, fingerprint);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                null, fingerprintReadingsQualityScores, sources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, null, sources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, sources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, new double[1], sources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, null, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, wrongSources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, sources,
                (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>) null));

        // constructor with quality scores and listener
        estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                this);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                null, fingerprintReadingsQualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, null, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                new double[1], this));

        // constructor with quality scores, sources and listener
        estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, this);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                null, fingerprintReadingsQualityScores, sources, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, null, sources, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, sources, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, new double[1], sources, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, (List<WifiAccessPointLocated2D>) null,
                this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, wrongSources, this));

        // constructor with quality scores, fingerprint and listener
        estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                fingerprint, this);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                null, fingerprintReadingsQualityScores, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, null, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, new double[1], fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>) null, this));

        // constructor with sources, fingerprint and listener
        estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, fingerprint, this);

        // check default values
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertEquals(3, estimator.getMinRequiredSources());
        assertEquals(3, estimator.getPreliminarySubsetSize());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);
        assertFalse(estimator.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getInitialPosition());
        assertTrue(estimator.isLinearSolverUsed());
        assertFalse(estimator.isHomogeneousLinearSolverUsed());
        assertTrue(estimator.isPreliminarySolutionRefined());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertFalse(estimator.isReady());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getCovariance());
        assertEquals(2, estimator.getNumberOfDimensions());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertTrue(estimator.getEvenlyDistributeReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                null, fingerprintReadingsQualityScores, sources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, null, sources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, sources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, new double[1], sources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, null, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, wrongSources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustRangingPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, sources, null, this));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(1.0);

        // check
        assertEquals(1.0, estimator.getThreshold(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    void testIsComputeAndKeepResidualsEnabled() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    void testGetSetSources() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        for (var i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY, new InhomogeneousPoint2D()));
        }

        estimator.setSources(sources);

        // check
        assertSame(sources, estimator.getSources());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setSources(null));
        final var wrongSources = new ArrayList<WifiAccessPointLocated2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setSources(wrongSources));
    }

    @Test
    void testGetSetFingerprint() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(fingerprint, estimator.getFingerprint());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setFingerprint(null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var solver = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertNull(solver.getInitialPosition());

        // set new value
        final var p = Point2D.create();
        solver.setInitialPosition(p);

        // check
        assertSame(p, solver.getInitialPosition());
    }

    @Test
    void testIsSetRadioSourcePositionCovarianceUsed() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
    }

    @Test
    void testGetSetFallbackDistanceStandardDeviation() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertEquals(RobustPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getFallbackDistanceStandardDeviation(), 0.0);

        // set new value
        estimator.setFallbackDistanceStandardDeviation(1.0);

        // check
        assertEquals(1.0, estimator.getFallbackDistanceStandardDeviation(), 0.0);
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.8);

        // check
        assertEquals(0.8, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(100);

        // check
        assertEquals(100, estimator.getMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check
        assertFalse(estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertTrue(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(false);

        // check
        assertFalse(estimator.isCovarianceKept());
    }

    @Test
    void testIsSetLinearSolverUsed() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertTrue(estimator.isLinearSolverUsed());

        // set new value
        estimator.setLinearSolverUsed(false);

        // check
        assertFalse(estimator.isLinearSolverUsed());
    }

    @Test
    void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertFalse(estimator.isHomogeneousLinearSolverUsed());

        // set new value
        estimator.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
    }

    @Test
    void testIsSetPreliminarySolutionRefined() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertTrue(estimator.isPreliminarySolutionRefined());

        // set new value
        estimator.setPreliminarySolutionRefined(false);

        // check
        assertFalse(estimator.isPreliminarySolutionRefined());
    }

    @Test
    void testGetSetSourceQualityScores() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertNull(estimator.getSourceQualityScores());

        // set new value
        final var qualityScores = new double[3];
        estimator.setSourceQualityScores(qualityScores);

        // check
        assertSame(qualityScores, estimator.getSourceQualityScores());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setSourceQualityScores(null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setSourceQualityScores(new double[1]));
    }

    @Test
    void testGetSetFingerprintReadingsQualityScores() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // set new value
        final var qualityScores = new double[3];
        estimator.setFingerprintReadingsQualityScores(qualityScores);

        // check
        assertSame(qualityScores, estimator.getFingerprintReadingsQualityScores());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setFingerprintReadingsQualityScores(null));
        assertThrows(IllegalArgumentException.class,
                () -> estimator.setFingerprintReadingsQualityScores(new double[1]));
    }

    @Test
    void testGetSetEvenlyDistributeReadings() throws LockedException {
        final var estimator = new PROSACRobustRangingPositionEstimator2D();

        // check default value
        assertTrue(estimator.getEvenlyDistributeReadings());

        // set new value
        estimator.setEvenlyDistributeReadings(false);

        // check
        assertFalse(estimator.getEvenlyDistributeReadings());
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var estimator = spy(new PROSACRobustRangingPositionEstimator2D());

        // check default value
        assertEquals(3, estimator.getPreliminarySubsetSize());

        // set new value
        estimator.setPreliminarySubsetSize(4);

        // check
        assertEquals(4, estimator.getPreliminarySubsetSize());
        verify(estimator, times(1))
                .buildPositionsDistancesDistanceStandardDeviationsAndQualityScores();

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setPreliminarySubsetSize(2));
    }

    @Test
    void testEstimate() throws LockedException, RobustEstimatorException, NotReadyException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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

        // force NotReadyException
        final var estimator = new PROSACRobustRangingPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateMultipleReadingsPerSource() throws LockedException, RobustEstimatorException, NotReadyException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                } else {
                    error1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                for (var j = 0; j < NUM_READINGS; j++) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        error2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        error2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] = 1.0 / (1.0 + Math.abs(error2));

                    readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2),
                            RANGING_STD));
                }
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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

        // force NotReadyException
        final var estimator = new PROSACRobustRangingPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateWithInlierError() throws LockedException, RobustEstimatorException, NotReadyException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);
        final var inlierErrorRandomizer = new GaussianRandomizer(0.0, INLIER_ERROR_STD);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            double inlierError;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                inlierError = inlierErrorRandomizer.nextDouble();

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2 + inlierError),
                        RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
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

        // force NotReadyException
        final var estimator = new PROSACRobustRangingPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateMultipleReadingsPerSourceWithInlierError() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);
        final var inlierErrorRandomizer = new GaussianRandomizer(0.0, INLIER_ERROR_STD);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double error1;
            double error2;
            double inlierError;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                } else {
                    error1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                for (var j = 0; j < NUM_READINGS; j++) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        error2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        error2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] = 1.0 / (1.0 + Math.abs(error2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RangingReading<>(accessPoint,
                            Math.max(0.0, distance + error1 + error2 + inlierError), RANGING_STD));
                }
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
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

        // force NotReadyException
        final var estimator = new PROSACRobustRangingPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException,
            RobustEstimatorException, NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(true);
            estimator.setHomogeneousLinearSolverUsed(true);
            estimator.setPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    void testEstimateLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            RobustEstimatorException, NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(true);
            estimator.setHomogeneousLinearSolverUsed(false);
            estimator.setPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    void testEstimatePreliminaryNotRefined() throws LockedException, RobustEstimatorException, NotReadyException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(true);
            estimator.setPreliminarySolutionRefined(false);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    void testEstimateLinearDisabled() throws LockedException, RobustEstimatorException, NotReadyException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(false);
            estimator.setPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    void testEstimateLinearDisabledAndNotPreliminaryRefined() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(false);
            estimator.setPreliminarySolutionRefined(false);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    void testEstimateLinearDisabledWithInitialPosition() throws LockedException, RobustEstimatorException,
            NotReadyException, NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setLinearSolverUsed(false);
            estimator.setPreliminarySolutionRefined(true);
            estimator.setInitialPosition(position);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    void testEstimateLargerPreliminarySubsetSize() throws LockedException, RobustEstimatorException, NotReadyException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double error1;
            double error2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error1 = errorRandomizer.nextDouble();
                    error2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error1 = 0.0;
                    error2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(error1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(error2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + error1 + error2), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new PROSACRobustRangingPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setPreliminarySubsetSize(4);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final var accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final var accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final var positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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

        // force NotReadyException
        final var estimator = new PROSACRobustRangingPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(final RobustRangingPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((PROSACRobustRangingPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(final RobustRangingPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((PROSACRobustRangingPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateNextIteration(final RobustRangingPositionEstimator<Point2D> estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACRobustRangingPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateProgressChange(final RobustRangingPositionEstimator<Point2D> estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACRobustRangingPositionEstimator2D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final PROSACRobustRangingPositionEstimator2D estimator) {
        assertThrows(LockedException.class, () -> estimator.setPreliminarySubsetSize(3));
        assertThrows(LockedException.class, () -> estimator.setSourceQualityScores(null));
        assertThrows(LockedException.class, () -> estimator.setFingerprintReadingsQualityScores(null));
        assertThrows(LockedException.class, () -> estimator.setEvenlyDistributeReadings(true));
        assertThrows(LockedException.class, () -> estimator.setRadioSourcePositionCovarianceUsed(true));
        assertThrows(LockedException.class, () -> estimator.setFallbackDistanceStandardDeviation(1.0));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.8));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(100));
        assertThrows(LockedException.class, () -> estimator.setResultRefined(true));
        assertThrows(LockedException.class, () -> estimator.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> estimator.setThreshold(1.0));
        assertThrows(LockedException.class, () -> estimator.setSources(null));
        assertThrows(LockedException.class, () -> estimator.setFingerprint(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
