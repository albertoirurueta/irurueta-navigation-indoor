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
import com.irurueta.navigation.lateration.LMedSRobustLateration2DSolver;
import com.irurueta.navigation.lateration.MSACRobustLateration2DSolver;
import com.irurueta.navigation.lateration.PROMedSRobustLateration2DSolver;
import com.irurueta.navigation.lateration.PROSACRobustLateration2DSolver;
import com.irurueta.navigation.lateration.RANSACRobustLateration2DSolver;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class SequentialRobustMixedPositionEstimator2DTest implements SequentialRobustMixedPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            SequentialRobustMixedPositionEstimator2DTest.class.getName());

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
    void testConstructor() {
        // empty constructor
        var estimator = new SequentialRobustMixedPositionEstimator2D();

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        for (var i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY, new InhomogeneousPoint2D()));
        }
        estimator = new SequentialRobustMixedPositionEstimator2D(sources);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                (List<? extends RadioSourceLocated<Point2D>>) null));
        final var wrongSources = new ArrayList<WifiAccessPointLocated2D>();
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(wrongSources));

        // test constructor with fingerprint
        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        estimator = new SequentialRobustMixedPositionEstimator2D(fingerprint);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null));

        // test constructor with sources and fingerprint
        estimator = new SequentialRobustMixedPositionEstimator2D(sources, fingerprint);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(null,
                fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(wrongSources,
                fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(sources,
                (RangingAndRssiFingerprint<? extends RadioSource,
                        ? extends RangingAndRssiReading<? extends RadioSource>>) null));

        // test constructor with listener
        estimator = new SequentialRobustMixedPositionEstimator2D(this);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        estimator = new SequentialRobustMixedPositionEstimator2D(sources, this);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                (List<? extends RadioSourceLocated<Point2D>>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(wrongSources,
                this));

        // test constructor with fingerprint and listener
        estimator = new SequentialRobustMixedPositionEstimator2D(fingerprint, this);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                this));

        // test constructor with sources, fingerprint and listener
        estimator = new SequentialRobustMixedPositionEstimator2D(sources, fingerprint, this);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(null,
                fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(wrongSources,
                fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(sources,
                null, this));

        // test constructor with quality scores
        final var sourceQualityScores = new double[3];
        final var fingerprintReadingsQualityScores = new double[3];
        estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                null, fingerprintReadingsQualityScores));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, null));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, new double[1]));

        // test constructor with quality scores and sources
        estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                null, fingerprintReadingsQualityScores, sources));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, null, sources));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, sources));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, new double[1], sources));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                (List<? extends RadioSourceLocated<Point2D>>) null));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, wrongSources));

        // test constructor with quality scores and fingerprint
        estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                fingerprint);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                null, fingerprintReadingsQualityScores, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, null, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, new double[1], fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null));

        // test constructor with quality scores, sources and fingerprint
        estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, fingerprint);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                null, fingerprintReadingsQualityScores, sources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, null, sources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, sources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, new double[1], sources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, null, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, wrongSources, fingerprint));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, sources,
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null));

        // test constructor with quality scores and listener
        estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                this);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                null, fingerprintReadingsQualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, null, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, new double[1], this));

        // test constructor with quality scores and sources and listener
        estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, this);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                null, fingerprintReadingsQualityScores, sources, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, null, sources, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, sources, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, new double[1], sources, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                (List<? extends RadioSourceLocated<Point2D>>) null, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, wrongSources, this));

        // test constructor with quality scores, fingerprint and listener
        estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                fingerprint, this);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                null, fingerprintReadingsQualityScores, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, null, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(new double[1],
                fingerprintReadingsQualityScores, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, new double[1], fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores,
                (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                this));

        // test constructor with quality scores, sources, fingerprint and listener
        estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, fingerprint, this);

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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
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
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                null, fingerprintReadingsQualityScores, sources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, null, sources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                new double[1], fingerprintReadingsQualityScores, sources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, new double[1], sources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, null, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, wrongSources, fingerprint, this));
        assertThrows(IllegalArgumentException.class, () -> new SequentialRobustMixedPositionEstimator2D(
                sourceQualityScores, fingerprintReadingsQualityScores, sources, null, this));
    }

    @Test
    void testGetSetRangingRobustMethod() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());

        // set new value
        estimator.setRangingRobustMethod(RobustEstimatorMethod.PROSAC);

        // check
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getRangingRobustMethod());
    }

    @Test
    void testGetSetRssiRobustMethod() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRssiRobustMethod());

        // set new value
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getRssiRobustMethod());
    }

    @Test
    void testIsSetRangingRadioSourcePositionCovarianceUsed() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator
                        .DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertTrue(estimator.isRangingRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRangingRadioSourcePositionCovarianceUsed(false);

        // check
        assertFalse(estimator.isRangingRadioSourcePositionCovarianceUsed());
    }

    @Test
    void testIsSetRssiRadioSourcePositionCovarianceUsed() throws LockedException {

        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testIsSetRangingReadingEvenlyDistributed() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testGetSetRssiFallbackDistanceStandardDeviation() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRssiFallbackDistanceStandardDeviation(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRssiFallbackDistanceStandardDeviation(value);

        // check
        assertEquals(estimator.getRssiFallbackDistanceStandardDeviation(), value, 0.0);
    }

    @Test
    void testGetSetRangingFallbackDistanceStandardDeviation() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                estimator.getRangingFallbackDistanceStandardDeviation(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRangingFallbackDistanceStandardDeviation(value);

        // check
        assertEquals(estimator.getRangingFallbackDistanceStandardDeviation(), value, 0.0);
    }

    @Test
    void testIsSetRssiReadingsEvenlyDistributed() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);
    }

    @Test
    void testGetSetRangingConfidence() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE,
                estimator.getRangingConfidence(), 0.0);

        // set new value
        estimator.setRangingConfidence(0.7);

        // check
        assertEquals(0.7, estimator.getRangingConfidence(), 0.0);
    }

    @Test
    void testGetSetRssiConfidence() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_CONFIDENCE, estimator.getRssiConfidence(),
                0.0);

        // set new value
        estimator.setRssiConfidence(0.8);

        // check
        assertEquals(0.8, estimator.getRssiConfidence(), 0.0);
    }

    @Test
    void testGetSetRangingMaxIterations() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRangingMaxIterations());

        // set new value
        estimator.setRangingMaxIterations(100);

        // check
        assertEquals(100, estimator.getRangingMaxIterations());
    }

    @Test
    void testGetSetRssiMaxIterations() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getRssiMaxIterations());

        // set new value
        estimator.setRssiMaxIterations(200);

        // check
        assertEquals(200, estimator.getRssiMaxIterations());
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testIsSetRangingLinearSolverUsed() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testIsSetRssiLinearSolverUsed() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testIsSetRangingHomogeneousLinearSolverUsed() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testIsSetRssiHomogeneousLinearSolverUsed() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testIsSetRangingPreliminarySolutionRefined() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testIsSetRssiPreliminarySolutionRefined() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
    void testGetSetRangingPreliminarySubsetSize() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());

        // set new value
        estimator.setRangingPreliminarySubsetSize(4);

        // check
        assertEquals(4, estimator.getRangingPreliminarySubsetSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setRangingPreliminarySubsetSize(2));
    }

    @Test
    void testGetSetRssiPreliminarySubsetSize() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        assertEquals(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());

        // set new value
        estimator.setRssiPreliminarySubsetSize(4);

        // check
        assertEquals(4, estimator.getRssiPreliminarySubsetSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setRssiPreliminarySubsetSize(2));
    }

    @Test
    void testGetSetRangingThreshold() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertNull(estimator.getRangingThreshold());

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);
    }

    @Test
    void testGetSetRangingThresholdRANSAC() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.RANSAC);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.RANSAC, estimator.rangingEstimator.getMethod());
        assertEquals(RANSACRobustLateration2DSolver.DEFAULT_THRESHOLD,
                ((RANSACRobustRangingPositionEstimator2D) estimator.rangingEstimator).getThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.RANSAC, estimator.rangingEstimator.getMethod());
        assertEquals(value, ((RANSACRobustRangingPositionEstimator2D) estimator.rangingEstimator).getThreshold(),
                0.0);
    }

    @Test
    void testGetSetRangingThresholdLMedS() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.LMEDS);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.LMEDS, estimator.rangingEstimator.getMethod());
        assertEquals(LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD,
                ((LMedSRobustRangingPositionEstimator2D) estimator.rangingEstimator).getStopThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.LMEDS, estimator.rangingEstimator.getMethod());
        assertEquals(value, ((LMedSRobustRangingPositionEstimator2D) estimator.rangingEstimator).getStopThreshold(),
                0.0);
    }

    @Test
    void testGetSetRangingThresholdMSAC() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.MSAC);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.MSAC, estimator.rangingEstimator.getMethod());
        assertEquals(MSACRobustLateration2DSolver.DEFAULT_THRESHOLD,
                ((MSACRobustRangingPositionEstimator2D) estimator.rangingEstimator).getThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.MSAC, estimator.rangingEstimator.getMethod());
        assertEquals(value, ((MSACRobustRangingPositionEstimator2D) estimator.rangingEstimator).getThreshold(),
                0.0);
    }

    @Test
    void testGetSetRangingThresholdPROMedS() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.PROMEDS);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.rangingEstimator.getMethod());
        assertEquals(PROMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD,
                ((PROMedSRobustRangingPositionEstimator2D) estimator.rangingEstimator).getStopThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.rangingEstimator.getMethod());
        assertEquals(value, ((PROMedSRobustRangingPositionEstimator2D) estimator.rangingEstimator).getStopThreshold(),
                0.0);
    }

    @Test
    void testGetSetRangingThresholdPROSAC() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.PROSAC);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.PROSAC, estimator.rangingEstimator.getMethod());
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD,
                ((PROSACRobustRangingPositionEstimator2D) estimator.rangingEstimator).getThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.PROSAC, estimator.rangingEstimator.getMethod());
        assertEquals(value, ((PROSACRobustRangingPositionEstimator2D) estimator.rangingEstimator).getThreshold(),
                0.0);
    }

    @Test
    void testGetSetRssiThreshold() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertNull(estimator.getRssiThreshold());

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);
    }

    @Test
    void testGetSetRssiThresholdRANSAC() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.RANSAC, estimator.rssiEstimator.getMethod());
        assertEquals(RANSACRobustLateration2DSolver.DEFAULT_THRESHOLD,
                ((RANSACRobustRssiPositionEstimator2D) estimator.rssiEstimator).getThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.RANSAC, estimator.rssiEstimator.getMethod());
        assertEquals(value, ((RANSACRobustRssiPositionEstimator2D) estimator.rssiEstimator).getThreshold(), 0.0);
    }

    @Test
    void testGetSetRssiThresholdLMedS() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.LMEDS);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.LMEDS, estimator.rssiEstimator.getMethod());
        assertEquals(LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD,
                ((LMedSRobustRssiPositionEstimator2D) estimator.rssiEstimator).getStopThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.LMEDS, estimator.rssiEstimator.getMethod());
        assertEquals(value, ((LMedSRobustRssiPositionEstimator2D) estimator.rssiEstimator).getStopThreshold(),
                0.0);
    }

    @Test
    void testGetSetRssiThresholdMSAC() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.MSAC);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.MSAC, estimator.rssiEstimator.getMethod());
        assertEquals(MSACRobustLateration2DSolver.DEFAULT_THRESHOLD,
                ((MSACRobustRssiPositionEstimator2D) estimator.rssiEstimator).getThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.MSAC, estimator.rssiEstimator.getMethod());
        assertEquals(value, ((MSACRobustRssiPositionEstimator2D) estimator.rssiEstimator).getThreshold(),
                0.0);
    }

    @Test
    void testGetSetRssiThresholdPROMedS() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.PROMEDS);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.rssiEstimator.getMethod());
        assertEquals(PROMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD,
                ((PROMedSRobustRssiPositionEstimator2D) estimator.rssiEstimator).getStopThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.rssiEstimator.getMethod());
        assertEquals(value, ((PROMedSRobustRssiPositionEstimator2D) estimator.rssiEstimator).getStopThreshold(),
                0.0);
    }

    @Test
    void testGetSetRssiThresholdPROSAC() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.PROSAC);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.PROSAC, estimator.rssiEstimator.getMethod());
        assertEquals(PROSACRobustLateration2DSolver.DEFAULT_THRESHOLD,
                ((PROSACRobustRssiPositionEstimator2D) estimator.rssiEstimator).getThreshold(), 0.0);

        // set new value
        final var value = new UniformRandomizer().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.PROSAC, estimator.rssiEstimator.getMethod());
        assertEquals(value, ((PROSACRobustRssiPositionEstimator2D) estimator.rssiEstimator).getThreshold(), 0.0);
    }

    @Test
    void testGetSetSources() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(fingerprint, estimator.getFingerprint());
    }

    @Test
    void testGetSetSourceQualityScores() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertNull(estimator.getSourceQualityScores());

        // set new value
        final var value = new double[3];
        estimator.setSourceQualityScores(value);

        // check
        assertSame(value, estimator.getSourceQualityScores());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setSourceQualityScores(new double[1]));
    }

    @Test
    void testGetSetFingerprintReadingsQualityScores() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // set new value
        final var value = new double[3];
        estimator.setFingerprintReadingsQualityScores(value);

        // check
        assertSame(value, estimator.getFingerprintReadingsQualityScores());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> estimator.setFingerprintReadingsQualityScores(new double[1]));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var estimator = new SequentialRobustMixedPositionEstimator2D();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final var p = Point2D.create();
        estimator.setInitialPosition(p);

        // check
        assertSame(p, estimator.getInitialPosition());
    }

    @Test
    void testEstimateRanging() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final var position = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final var sources = new ArrayList<WifiAccessPointLocated2D>();
            final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRanging1 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRssi() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final var position = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRssi2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRangingAndRssi() throws LockedException, NotReadyException, RobustEstimatorException,
            NonSymmetricPositiveDefiniteMatrixException {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValidPosition = 0;
        var positionStd = 0.0;
        var positionStdConfidence = 0.0;
        var positionAccuracy = 0.0;
        var positionAccuracyConfidence = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final var position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final var estimatedPosition = estimator.getEstimatedPosition();
            assertSame(estimatedPosition, p);
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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateMixed() throws LockedException, NotReadyException, RobustEstimatorException,
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
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRangingMultipleReadingsPerSource() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                for (var j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] = 1.0 / (1.0 + Math.abs(errorRanging2));

                    readings.add(new RangingReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2), RANGING_STD));
                }
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRssiMultipleReadingsPerSource() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRssi2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);
                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                for (var j = 0; j < NUM_READINGS; j++) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] = 1.0 / (1.0 + Math.abs(errorRssi2));

                    readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                            Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRangingAndRssiMultipleReadingsPerSource() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);
                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                for (var j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] = 1.0
                            / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    readings.add(new RangingAndRssiReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2),
                            rssi + errorRssi1 + errorRssi2, RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateMixedMultipleReadingsPerSource() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);
                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                for (var j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j)] = 1.0 / (1.0 + Math.abs(errorRssi2));
                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j) + 1] = 1.0
                            / (1.0 + Math.abs(errorRanging2));
                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j) + 2] = 1.0
                            / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                            Math.sqrt(RX_POWER_VARIANCE)));
                    readings.add(new RangingReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2), RANGING_STD));
                    readings.add(new RangingAndRssiReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2),
                            rssi + errorRssi1 + errorRssi2, RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRangingWithInlierError() throws LockedException, NotReadyException, RobustEstimatorException,
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRanging1 = 0.0;
                    errorRanging2 = 0.0;
                }

                inlierError = inlierErrorRandomizer.nextDouble();

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging2));

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError), RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRssiWithInlierError() throws LockedException, NotReadyException, RobustEstimatorException,
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            double inlierError;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRssi2 = 0.0;
                }

                inlierError = inlierErrorRandomizer.nextDouble();

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2 + inlierError,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRangingAndRssiWithInlierError() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateMixedWithInlierError() throws LockedException, NotReadyException, RobustEstimatorException,
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2 + inlierError,
                        Math.sqrt(RX_POWER_VARIANCE)));
                final var dist = Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError);
                readings.add(new RangingReading<>(accessPoint, dist, RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint, dist,
                        rssi + errorRssi1 + errorRssi2 + inlierError, RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRangingMultipleReadingsPerSourceWithInlierError() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                for (var j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] = 1.0 / (1.0 + Math.abs(errorRanging2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RangingReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError), RANGING_STD));
                }
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRssiMultipleReadingsPerSourceWithInlierError() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRssi2;
            double inlierError;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);
                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                for (var j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] = 1.0 / (1.0 + Math.abs(errorRssi2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2 + inlierError,
                            Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRangingAndRssiMultipleReadingsPerSourceWithInlierError() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);
                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                for (var j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[i * NUM_READINGS + j] = 1.0
                            / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RangingAndRssiReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError),
                            rssi + errorRssi1 + errorRssi2 + inlierError, RANGING_STD,
                            Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateMixedMultipleReadingsPerSourceWithInlierError() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);
                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1 + errorRanging1));
                for (var j = 0; j < NUM_READINGS; j++) {

                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        errorRssi2 = errorRandomizer.nextDouble();
                        errorRanging2 = errorRandomizer.nextDouble();
                    } else {
                        // inlier
                        errorRssi2 = 0.0;
                        errorRanging2 = 0.0;
                    }

                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j)] = 1.0 / (1.0 + Math.abs(errorRssi2));
                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j) + 1] = 1.0
                            / (1.0 + Math.abs(errorRanging2));
                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j) + 2] = 1.0
                            / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2 + inlierError,
                            Math.sqrt(RX_POWER_VARIANCE)));
                    final var dist = Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError);
                    readings.add(new RangingReading<>(accessPoint, dist, RANGING_STD));
                    readings.add(new RangingAndRssiReading<>(accessPoint, dist,
                            rssi + errorRssi1 + errorRssi2 + inlierError, RANGING_STD,
                            Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateRangingLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRanging1 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRssiLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRssi2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingAndRssiLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateMixedLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRanging1 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRssiLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRssi2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingAndRssiLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateMixedLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingPreliminaryNotRefined() throws LockedException, NotReadyException, RobustEstimatorException,
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRanging1 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRssiPreliminaryNotRefined() throws LockedException, NotReadyException, RobustEstimatorException,
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRssi2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingAndRssiPreliminaryNotRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateMixedPreliminaryNotRefined() throws LockedException, NotReadyException, RobustEstimatorException,
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingLinearDisabled() throws LockedException, NotReadyException, RobustEstimatorException,
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRanging1 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRssiLinearDisabled() throws LockedException, NotReadyException, RobustEstimatorException,
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRssi2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingAndRssiLinearDisabled() throws LockedException, NotReadyException, RobustEstimatorException,
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateMixedLinearDisabled() throws LockedException, NotReadyException, RobustEstimatorException,
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingLinearDisabledAndNotPreliminaryRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRanging1 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRssiLinearDisabledAndNotPreliminaryRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRssi2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingAndRssiLinearDisabledAndNotPreliminaryRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateMixedLinearDisabledAndNotPreliminaryRefined() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingLinearDisabledWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            double errorRanging1;
            double errorRanging2;
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
                    errorRanging1 = errorRandomizer.nextDouble();
                    errorRanging2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRanging1 = 0.0;
                    errorRanging2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging2));

                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
            }

            final var fingerprint = new RangingFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRssiLinearDisabledWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                    errorRssi2 = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    errorRssi1 = 0.0;
                    errorRssi2 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                fingerprintReadingsQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new RssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateRangingAndRssiLinearDisabledWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final var fingerprint = new RangingAndRssiFingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateMixedLinearDisabledWithInitialPosition() throws LockedException, NotReadyException,
            RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
    void testEstimateLargerPreliminarySubsetSize() throws LockedException, NotReadyException, RobustEstimatorException,
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
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var sources = new ArrayList<WifiAccessPointWithPowerAndLocated2D>();
            final var readings = new ArrayList<Reading<WifiAccessPoint>>();
            final var sourceQualityScores = new double[numSources];
            final var fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (var i = 0; i < numSources; i++) {
                final var accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS), randomizer.nextDouble(MIN_POS, MAX_POS));

                final var transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final var transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final var bssid = String.valueOf(i);

                final var locatedAccessPoint = new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY,
                        transmittedPowerdBm, Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                        Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final var accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final var distance = position.distanceTo(accessPointPosition);

                final var rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, Math.max(0.0, distance + errorRanging1 + errorRanging2),
                        RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final var fingerprint = new Fingerprint<>(readings);

            final var estimator = new SequentialRobustMixedPositionEstimator2D(sourceQualityScores,
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

            final var p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

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
        final var estimator = new SequentialRobustMixedPositionEstimator2D();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(final SequentialRobustMixedPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((SequentialRobustMixedPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(final SequentialRobustMixedPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((SequentialRobustMixedPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateProgressChange(final SequentialRobustMixedPositionEstimator<Point2D> estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((SequentialRobustMixedPositionEstimator2D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateProgressChange = 0;
    }

    private static double receivedPower(final double equivalentTransmittedPower, final double distance,
                                        final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final var k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k / Math.pow(distance, pathLossExponent);
    }

    private static void checkLocked(final SequentialRobustMixedPositionEstimator2D estimator) {
        assertThrows(LockedException.class, () -> estimator.setRangingRobustMethod(RobustEstimatorMethod.PROMEDS));
        assertThrows(LockedException.class, () -> estimator.setRssiRobustMethod(RobustEstimatorMethod.PROMEDS));
        assertThrows(LockedException.class, () -> estimator.setRangingRadioSourcePositionCovarianceUsed(true));
        assertThrows(LockedException.class, () -> estimator.setRssiRadioSourcePositionCovarianceUsed(true));
        assertThrows(LockedException.class, () -> estimator.setRangingReadingsEvenlyDistributed(true));
        assertThrows(LockedException.class, () -> estimator.setRssiFallbackDistanceStandardDeviation(1.0));
        assertThrows(LockedException.class, () -> estimator.setRangingFallbackDistanceStandardDeviation(1.0));
        assertThrows(LockedException.class, () -> estimator.setRssiReadingsEvenlyDistributed(true));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setRangingConfidence(0.9));
        assertThrows(LockedException.class, () -> estimator.setRssiConfidence(0.9));
        assertThrows(LockedException.class, () -> estimator.setRangingMaxIterations(100));
        assertThrows(LockedException.class, () -> estimator.setRssiMaxIterations(100));
        assertThrows(LockedException.class, () -> estimator.setResultRefined(true));
        assertThrows(LockedException.class, () -> estimator.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> estimator.setRangingLinearSolverUsed(true));
        assertThrows(LockedException.class, () -> estimator.setRssiLinearSolverUsed(true));
        assertThrows(LockedException.class, () -> estimator.setRangingHomogeneousLinearSolverUsed(true));
        assertThrows(LockedException.class, () -> estimator.setRssiHomogeneousLinearSolverUsed(true));
        assertThrows(LockedException.class, () -> estimator.setRangingPreliminarySolutionRefined(true));
        assertThrows(LockedException.class, () -> estimator.setRssiPreliminarySolutionRefined(true));
        assertThrows(LockedException.class, () -> estimator.setRangingPreliminarySubsetSize(3));
        assertThrows(LockedException.class, () -> estimator.setRssiPreliminarySubsetSize(3));
        assertThrows(LockedException.class, () -> estimator.setRangingThreshold(1.0));
        assertThrows(LockedException.class, () -> estimator.setRssiThreshold(1.0));
        assertThrows(LockedException.class, () -> estimator.setSources(null));
        assertThrows(LockedException.class, () -> estimator.setFingerprint(null));
        assertThrows(LockedException.class, () -> estimator.setSourceQualityScores(null));
        assertThrows(LockedException.class, () -> estimator.setFingerprintReadingsQualityScores(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setInitialPosition(null));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
