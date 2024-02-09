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

import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
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
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class SequentialRobustRangingAndRssiPositionEstimator2DTest implements
        SequentialRobustRangingAndRssiPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            SequentialRobustRangingAndRssiPositionEstimator2DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final int MIN_SOURCES = 100;
    private static final int MAX_SOURCES = 500;

    private static final int NUM_READINGS = 5;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 0.5;

    private static final int TIMES = 400;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private static final double INLIER_ERROR_STD = 0.1;

    private static final double RANGING_STD = 1.0;

    private static final double TX_POWER_VARIANCE = 0.1;
    private static final double RX_POWER_VARIANCE = 0.5;
    private static final double PATH_LOSS_EXPONENT_VARIANCE = 0.001;

    private int estimateStart;
    private int estimateEnd;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // empty constructor
        SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // test constructor with sources
        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY, new InhomogeneousPoint2D()));
        }
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sources);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    (List<? extends RadioSourceLocated<Point2D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with fingerprint
        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(fingerprint);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with sources and fingerprint
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sources, fingerprint);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sources,
                    (RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<? extends RadioSource>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(this);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // test constructor with sources and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sources, this);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    (List<? extends RadioSourceLocated<Point2D>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with fingerprint and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(fingerprint, this);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with sources, fingerprint and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sources, fingerprint, this);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sources, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] sourceQualityScores = new double[3];
        final double[] fingerprintReadingsQualityScores = new double[3];
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    new double[1], fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and sources
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                fingerprintReadingsQualityScores, sources);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    new double[1], fingerprintReadingsQualityScores, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, new double[1], sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    (List<? extends RadioSourceLocated<Point2D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores,
                    new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and fingerprint
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, fingerprint);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(null,
                    fingerprintReadingsQualityScores, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(new double[1],
                    fingerprintReadingsQualityScores, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores, new double[1],
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, sources and fingerprint
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                fingerprintReadingsQualityScores, sources, fingerprint);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingsQualityScores);
        assertNull(estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    null, fingerprintReadingsQualityScores, sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    null, sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(new double[1],
                    fingerprintReadingsQualityScores, sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores, new double[1],
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores, null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, fingerprintReadingsQualityScores, new ArrayList<WifiAccessPointLocated2D>(),
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                fingerprintReadingsQualityScores, this);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(null,
                    fingerprintReadingsQualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(
                    sourceQualityScores, null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(new double[1],
                    fingerprintReadingsQualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores, new double[1],
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and sources and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                fingerprintReadingsQualityScores, sources, this);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(null,
                    fingerprintReadingsQualityScores, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    null, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(new double[1],
                    fingerprintReadingsQualityScores, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores, new double[1],
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, (List<? extends RadioSourceLocated<Point2D>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, new ArrayList<WifiAccessPointLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, fingerprint and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                fingerprintReadingsQualityScores, fingerprint, this);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertNull(estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(null,
                    fingerprintReadingsQualityScores, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(new double[1],
                    fingerprintReadingsQualityScores, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores, new double[1],
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, sources, fingerprint and listener
        estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                fingerprintReadingsQualityScores, sources, fingerprint, this);

        // check default values
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getNumberOfDimensions());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertNull(estimator.getRangingThreshold());
        assertNull(estimator.getRssiThreshold());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingsQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getInliersData());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPosition());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(null,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    null, sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(new double[1],
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores, new double[1],
                    sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetRangingRobustMethod() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());

        // set new value
        estimator.setRangingRobustMethod(RobustEstimatorMethod.PROSAC);

        // check
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getRangingRobustMethod());
    }

    @Test
    public void testGetSetRssiRobustMethod() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRssiRobustMethod());

        // set new value
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getRssiRobustMethod());
    }

    @Test
    public void testIsSetRangingRadioSourcePositionCovarianceUsed() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertTrue(estimator.isRangingRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRangingRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRangingRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testIsSetRssiRadioSourcePositionCovarianceUsed() throws LockedException {

        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertTrue(estimator.isRssiRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRssiRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRssiRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testIsSetRangingReadingEvenlyDistributed() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
                estimator.isRangingReadingsEvenlyDistributed());
        assertTrue(estimator.isRangingReadingsEvenlyDistributed());

        // set new value
        estimator.setRangingReadingsEvenlyDistributed(false);

        // check
        assertFalse(estimator.isRangingReadingsEvenlyDistributed());
    }

    @Test
    public void testGetSetRssiFallbackDistanceStandardDeviation() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRssiFallbackDistanceStandardDeviation(value);

        // check
        assertEquals(value, estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);
    }

    @Test
    public void testGetSetRangingFallbackDistanceStandardDeviation() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRangingFallbackDistanceStandardDeviation(value);

        // check
        assertEquals(value, estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);
    }

    @Test
    public void testIsSetRssiReadingsEvenlyDistributed() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_EVENLY_DISTRIBUTE_RSSI_READINGS,
                estimator.isRssiReadingsEvenlyDistributed());
        assertTrue(estimator.isRssiReadingsEvenlyDistributed());

        // set new value
        estimator.setRssiReadingsEvenlyDistributed(false);

        // check
        assertFalse(estimator.isRssiReadingsEvenlyDistributed());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);
    }

    @Test
    public void testGetSetRangingConfidence() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);

        // set new value
        estimator.setRangingConfidence(0.7);

        // check
        assertEquals(0.7, estimator.getRangingConfidence(), 0.0);
    }

    @Test
    public void testGetSetRssiConfidence() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRssiConfidence(), 0.0);

        // set new value
        estimator.setRssiConfidence(0.8);

        // check
        assertEquals(0.8, estimator.getRssiConfidence(), 0.0);
    }

    @Test
    public void testGetSetRangingMaxIterations() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());

        // set new value
        estimator.setRangingMaxIterations(100);

        // check
        assertEquals(100, estimator.getRangingMaxIterations());
    }

    @Test
    public void testGetSetRssiMaxIterations() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());

        // set new value
        estimator.setRssiMaxIterations(200);

        // check
        assertEquals(200, estimator.getRssiMaxIterations());
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
        assertTrue(estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(false);

        // check
        assertFalse(estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertTrue(estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(false);

        // check
        assertFalse(estimator.isCovarianceKept());
    }

    @Test
    public void testIsSetRangingLinearSolverUsed() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_LINEAR_SOLVER,
                estimator.isRangingLinearSolverUsed());
        assertTrue(estimator.isRangingLinearSolverUsed());

        // set new value
        estimator.setRangingLinearSolverUsed(false);

        // check
        assertFalse(estimator.isRangingLinearSolverUsed());
    }

    @Test
    public void testIsSetRssiLinearSolverUsed() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_LINEAR_SOLVER,
                estimator.isRssiLinearSolverUsed());
        assertTrue(estimator.isRssiLinearSolverUsed());

        // set new value
        estimator.setRssiLinearSolverUsed(false);

        // check
        assertFalse(estimator.isRssiLinearSolverUsed());
    }

    @Test
    public void testIsSetRangingHomogeneousLinearSolverUsed() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRangingHomogeneousLinearSolverUsed());
        assertFalse(estimator.isRangingHomogeneousLinearSolverUsed());

        // set new value
        estimator.setRangingHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isRangingHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetRssiHomogeneousLinearSolverUsed() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_HOMOGENEOUS_LINEAR_SOLVER,
                estimator.isRssiHomogeneousLinearSolverUsed());
        assertFalse(estimator.isRssiHomogeneousLinearSolverUsed());

        // set new value
        estimator.setRssiHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(estimator.isRssiHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetRangingPreliminarySolutionRefined() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RANGING_PRELIMINARY_SOLUTIONS,
                estimator.isRangingPreliminarySolutionRefined());
        assertTrue(estimator.isRangingPreliminarySolutionRefined());

        // set new value
        estimator.setRangingPreliminarySolutionRefined(false);

        // check
        assertFalse(estimator.isRangingPreliminarySolutionRefined());
    }

    @Test
    public void testIsSetRssiPreliminarySolutionRefined() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_REFINE_RSSI_PRELIMINARY_SOLUTIONS,
                estimator.isRssiPreliminarySolutionRefined());
        assertTrue(estimator.isRssiPreliminarySolutionRefined());

        // set new value
        estimator.setRssiPreliminarySolutionRefined(false);

        // check
        assertFalse(estimator.isRssiPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetRangingThreshold() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getRangingThreshold());

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);
    }

    @Test
    public void testGetSetRssiThreshold() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getRssiThreshold());

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);
    }

    @Test
    public void testGetSetSources() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY, new InhomogeneousPoint2D()));
        }
        estimator.setSources(sources);

        // check
        assertSame(sources, estimator.getSources());

        // force IllegalArgumentException
        try {
            estimator.setSources(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setSources(new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(fingerprint, estimator.getFingerprint());
    }

    @Test
    public void testGetSetSourceQualityScores() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getSourceQualityScores());

        // set new value
        final double[] value = new double[3];
        estimator.setSourceQualityScores(value);

        // check
        assertSame(value, estimator.getSourceQualityScores());

        // force IllegalArgumentException
        try {
            estimator.setSourceQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetFingerprintReadingsQualityScores() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // set new value
        final double[] value = new double[3];
        estimator.setFingerprintReadingsQualityScores(value);

        // check
        assertSame(value, estimator.getFingerprintReadingsQualityScores());

        // force IllegalArgumentException
        try {
            estimator.setFingerprintReadingsQualityScores(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final Point2D p = Point2D.create();
        estimator.setInitialPosition(p);

        // check
        assertSame(p, estimator.getInitialPosition());
    }

    @Test
    public void testEstimate() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateProgressChange);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateMultipleReadingsPerSource() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);
                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                for (int j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] =
                            1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    readings.add(new RangingAndRssiReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2),
                            rssi + errorRssi1 + errorRssi2, RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateWithInlierError() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                inlierError = inlierErrorRandomizer.nextDouble();

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError),
                        rssi + errorRssi1 + errorRssi2 + inlierError, RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateMultipleReadingsPerSourceWithInlierError() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);
        final GaussianRandomizer inlierErrorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, INLIER_ERROR_STD);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);
                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                for (int j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] =
                            1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RangingAndRssiReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError),
                            rssi + errorRssi1 + errorRssi2 + inlierError, RANGING_STD,
                            Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(true);
            estimator.setRangingLinearSolverUsed(true);
            estimator.setRssiHomogeneousLinearSolverUsed(true);
            estimator.setRangingHomogeneousLinearSolverUsed(true);
            estimator.setRssiPreliminarySolutionRefined(true);
            estimator.setRangingPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    public void testEstimateLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(true);
            estimator.setRangingLinearSolverUsed(true);
            estimator.setRssiHomogeneousLinearSolverUsed(false);
            estimator.setRangingHomogeneousLinearSolverUsed(false);
            estimator.setRssiPreliminarySolutionRefined(true);
            estimator.setRangingPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    public void testEstimatePreliminaryNotRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(true);
            estimator.setRangingLinearSolverUsed(true);
            estimator.setRssiPreliminarySolutionRefined(false);
            estimator.setRangingPreliminarySolutionRefined(false);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    public void testEstimateLinearDisabled() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(false);
            estimator.setRangingLinearSolverUsed(false);
            estimator.setRssiPreliminarySolutionRefined(true);
            estimator.setRangingPreliminarySolutionRefined(true);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    public void testEstimateLinearDisabledAndNotPreliminaryRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(false);
            estimator.setRangingLinearSolverUsed(false);
            estimator.setRssiPreliminarySolutionRefined(false);
            estimator.setRangingPreliminarySolutionRefined(false);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    public void testEstimateLinearDisabledWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRssiLinearSolverUsed(false);
            estimator.setRangingLinearSolverUsed(false);
            estimator.setRssiPreliminarySolutionRefined(true);
            estimator.setRangingPreliminarySolutionRefined(true);
            estimator.setInitialPosition(position);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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
    }

    @Test
    public void testEstimateLargerPreliminarySubsetSize() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                    errorRssi2 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                    new SequentialRobustRangingAndRssiPositionEstimator2D(sourceQualityScores,
                            fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRangingPreliminarySubsetSize(4);
            estimator.setRssiPreliminarySubsetSize(4);

            reset();

            // check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getCovariance());
            assertNull(estimator.getPositions());
            assertNull(estimator.getDistances());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateProgressChange);

            final Point2D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());


            final Point2D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy2D accuracyStd = new Accuracy2D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy2D accuracy = new Accuracy2D(estimator.getCovariance());
            accuracy.setConfidence(0.99);

            positionStd = accuracyStd.getAverageAccuracy();
            positionStdConfidence = accuracyStd.getConfidence();
            positionAccuracy = accuracy.getAverageAccuracy();
            positionAccuracyConfidence = accuracy.getConfidence();

            final double positionDistance = position.distanceTo(estimatedPosition);
            if (positionDistance > ABSOLUTE_ERROR) {
                continue;
            }

            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
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

        // force NotReadyException
        final SequentialRobustRangingAndRssiPositionEstimator2D estimator =
                new SequentialRobustRangingAndRssiPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(final SequentialRobustRangingAndRssiPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((SequentialRobustRangingAndRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(final SequentialRobustRangingAndRssiPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((SequentialRobustRangingAndRssiPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateProgressChange(final SequentialRobustRangingAndRssiPositionEstimator<Point2D> estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((SequentialRobustRangingAndRssiPositionEstimator2D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateProgressChange = 0;
    }

    private double receivedPower(final double equivalentTransmittedPower,
                                 final double distance, final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k / Math.pow(distance, pathLossExponent);
    }

    private void checkLocked(final SequentialRobustRangingAndRssiPositionEstimator2D estimator) {
        try {
            estimator.setRangingRobustMethod(RobustEstimatorMethod.PROMEDS);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiRobustMethod(RobustEstimatorMethod.PROMEDS);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingRadioSourcePositionCovarianceUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiRadioSourcePositionCovarianceUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingReadingsEvenlyDistributed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiFallbackDistanceStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingFallbackDistanceStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiReadingsEvenlyDistributed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingConfidence(0.9);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiConfidence(0.9);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setResultRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingHomogeneousLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiHomogeneousLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingPreliminarySolutionRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiPreliminarySolutionRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingPreliminarySubsetSize(3);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiPreliminarySubsetSize(3);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRangingThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setRssiThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSourceQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFingerprintReadingsQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
