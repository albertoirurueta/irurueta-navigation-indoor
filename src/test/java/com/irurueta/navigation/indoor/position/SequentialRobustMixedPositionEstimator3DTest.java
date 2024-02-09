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
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.lateration.LMedSRobustLateration3DSolver;
import com.irurueta.navigation.lateration.MSACRobustLateration3DSolver;
import com.irurueta.navigation.lateration.PROMedSRobustLateration3DSolver;
import com.irurueta.navigation.lateration.PROSACRobustLateration3DSolver;
import com.irurueta.navigation.lateration.RANSACRobustLateration3DSolver;
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

public class SequentialRobustMixedPositionEstimator3DTest implements
        SequentialRobustMixedPositionEstimatorListener<Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            SequentialRobustMixedPositionEstimator3DTest.class.getName());

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
        SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRssiRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_EVENLY_DISTRIBUTE_RANGING_READINGS,
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
        final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }
        estimator = new SequentialRobustMixedPositionEstimator3D(sources);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getRssiPreliminarySubsetSize());
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
            estimator = new SequentialRobustMixedPositionEstimator3D(
                    (List<? extends RadioSourceLocated<Point3D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with fingerprint
        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        estimator = new SequentialRobustMixedPositionEstimator3D(fingerprint);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with sources and fingerprint
        estimator = new SequentialRobustMixedPositionEstimator3D(sources, fingerprint);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new ArrayList<WifiAccessPointLocated3D>(),
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sources,
                    (RangingAndRssiFingerprint<? extends RadioSource, ? extends RangingAndRssiReading<?
                            extends RadioSource>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new SequentialRobustMixedPositionEstimator3D(this);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
        estimator = new SequentialRobustMixedPositionEstimator3D(sources, this);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(
                    (List<? extends RadioSourceLocated<Point3D>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with fingerprint and listener
        estimator = new SequentialRobustMixedPositionEstimator3D(fingerprint, this);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with sources, fingerprint and listener
        estimator = new SequentialRobustMixedPositionEstimator3D(sources, fingerprint, this);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sources, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] sourceQualityScores = new double[4];
        final double[] fingerprintReadingsQualityScores = new double[4];
        estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                fingerprintReadingsQualityScores);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(
                    null, fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new double[1], fingerprintReadingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and sources
        estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RANGING_ROBUST_METHOD,
                estimator.getRangingRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_RSSI_ROBUST_METHOD,
                estimator.getRssiRobustMethod());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.DEFAULT_USE_RANGING_RADIO_SOURCE_POSITION_COVARIANCE,
                estimator.isRangingRadioSourcePositionCovarianceUsed());
        assertEquals(SequentialRobustRangingAndRssiPositionEstimator.
                        DEFAULT_USE_RSSI_RADIO_SOURCE_POSITION_COVARIANCE,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null,
                    fingerprintReadingsQualityScores, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    null, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new double[1], fingerprintReadingsQualityScores,
                    sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, new double[1], sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, (List<? extends RadioSourceLocated<Point3D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and fingerprint
        estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                fingerprint);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null,
                    fingerprintReadingsQualityScores, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new double[1], fingerprintReadingsQualityScores,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, new double[1], fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, sources and fingerprint
        estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, fingerprint);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null,
                    fingerprintReadingsQualityScores, sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    null, sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new double[1], fingerprintReadingsQualityScores,
                    sources, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, new double[1], sources,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, new ArrayList<WifiAccessPointLocated3D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and listener
        estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                this);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null,
                    fingerprintReadingsQualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new double[1], fingerprintReadingsQualityScores,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and sources and listener
        estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, this);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null,
                    fingerprintReadingsQualityScores, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    null, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new double[1], fingerprintReadingsQualityScores,
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, new double[1], sources,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, (List<? extends RadioSourceLocated<Point3D>>) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, fingerprint and listener
        estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                fingerprint, this);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null,
                    fingerprintReadingsQualityScores, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new double[1], fingerprintReadingsQualityScores,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, new double[1], fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores,
                    (RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>) null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores, sources, fingerprint and listener
        estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                sources, fingerprint, this);

        // check default values
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, estimator.getNumberOfDimensions());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1, estimator.getMinRequiredSources());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());
        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
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
            estimator = new SequentialRobustMixedPositionEstimator3D(null,
                    fingerprintReadingsQualityScores, sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    null, sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(new double[1], fingerprintReadingsQualityScores,
                    sources, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, new double[1], sources,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, new ArrayList<WifiAccessPointLocated3D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SequentialRobustMixedPositionEstimator3D(sourceQualityScores,
                    fingerprintReadingsQualityScores, sources, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetRangingRobustMethod() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator =
                new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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

        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
    public void testGetSetRangingPreliminarySubsetSize() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRangingPreliminarySubsetSize());

        // set new value
        estimator.setRangingPreliminarySubsetSize(5);

        // check
        assertEquals(5, estimator.getRangingPreliminarySubsetSize());

        // force IllegalArgumentException
        try {
            estimator.setRangingPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRssiPreliminarySubsetSize() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        assertEquals(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1,
                estimator.getRssiPreliminarySubsetSize());

        // set new value
        estimator.setRssiPreliminarySubsetSize(5);

        // check
        assertEquals(5, estimator.getRssiPreliminarySubsetSize());

        // force IllegalArgumentException
        try {
            estimator.setRssiPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRangingThreshold() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        // check default value
        assertNull(estimator.getRangingThreshold());

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);
    }

    @Test
    public void testGetSetRangingThresholdRANSAC() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.RANSAC);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.RANSAC, estimator.mRangingEstimator.getMethod());
        assertEquals(RANSACRobustLateration3DSolver.DEFAULT_THRESHOLD,
                ((RANSACRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.RANSAC, estimator.mRangingEstimator.getMethod());
        assertEquals(value, ((RANSACRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getThreshold(),
                0.0);
    }

    @Test
    public void testGetSetRangingThresholdLMedS() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.LMEDS);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.LMEDS, estimator.mRangingEstimator.getMethod());
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD,
                ((LMedSRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getStopThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.LMEDS, estimator.mRangingEstimator.getMethod());
        assertEquals(value, ((LMedSRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getStopThreshold(),
                0.0);
    }

    @Test
    public void testGetSetRangingThresholdMSAC() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.MSAC);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.MSAC, estimator.mRangingEstimator.getMethod());
        assertEquals(MSACRobustLateration3DSolver.DEFAULT_THRESHOLD,
                ((MSACRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.MSAC, estimator.mRangingEstimator.getMethod());
        assertEquals(value, ((MSACRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getThreshold(),
                0.0);
    }

    @Test
    public void testGetSetRangingThresholdPROMedS() throws LockedException {
        SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.PROMEDS);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.mRangingEstimator.getMethod());
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD,
                ((PROMedSRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getStopThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.mRangingEstimator.getMethod());
        assertEquals(value, ((PROMedSRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getStopThreshold(),
                0.0);
    }

    @Test
    public void testGetSetRangingThresholdPROSAC() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRangingRobustMethod(RobustEstimatorMethod.PROSAC);

        // check default value
        assertNull(estimator.getRangingThreshold());

        // check that internal ranging estimator has default value

        // setup ranging estimator
        estimator.buildRangingEstimator();
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.PROSAC, estimator.mRangingEstimator.getMethod());
        assertEquals(PROSACRobustLateration3DSolver.DEFAULT_THRESHOLD,
                ((PROSACRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRangingThreshold(value);

        // check
        assertEquals(value, estimator.getRangingThreshold(), 0.0);

        // check that internal ranging estimator also has new value

        // setup ranging estimator
        estimator.setupRangingEstimator();

        assertEquals(RobustEstimatorMethod.PROSAC, estimator.mRangingEstimator.getMethod());
        assertEquals(value, ((PROSACRobustRangingPositionEstimator3D) estimator.mRangingEstimator).getThreshold(),
                0.0);
    }

    @Test
    public void testGetSetRssiThreshold() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        // check default value
        assertNull(estimator.getRssiThreshold());

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);
    }

    @Test
    public void testGetSetRssiThresholdRANSAC() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.RANSAC);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.RANSAC, estimator.mRssiEstimator.getMethod());
        assertEquals(RANSACRobustLateration3DSolver.DEFAULT_THRESHOLD,
                ((RANSACRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.RANSAC, estimator.mRssiEstimator.getMethod());
        assertEquals(value, ((RANSACRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getThreshold(), 0.0);
    }

    @Test
    public void testGetSetRssiThresholdLMedS() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.LMEDS);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.LMEDS, estimator.mRssiEstimator.getMethod());
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD,
                ((LMedSRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getStopThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.LMEDS, estimator.mRssiEstimator.getMethod());
        assertEquals(((LMedSRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getStopThreshold(), value,
                0.0);
    }

    @Test
    public void testGetSetRssiThresholdMSAC() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.MSAC);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.MSAC, estimator.mRssiEstimator.getMethod());
        assertEquals(MSACRobustLateration3DSolver.DEFAULT_THRESHOLD,
                ((MSACRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.MSAC, estimator.mRssiEstimator.getMethod());
        assertEquals(value, ((MSACRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getThreshold(), 0.0);
    }

    @Test
    public void testGetSetRssiThresholdPROMedS() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.PROMEDS);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.mRssiEstimator.getMethod());
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD,
                ((PROMedSRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getStopThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.mRssiEstimator.getMethod());
        assertEquals(value, ((PROMedSRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getStopThreshold(),
                0.0);
    }

    @Test
    public void testGetSetRssiThresholdPROSAC() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        estimator.setRssiRobustMethod(RobustEstimatorMethod.PROSAC);

        // check default value
        assertNull(estimator.getRssiThreshold());

        // check that internal rssi estimator has default value

        // setup rssi estimator
        estimator.buildRssiEstimator();
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.PROSAC, estimator.mRssiEstimator.getMethod());
        assertEquals(PROSACRobustLateration3DSolver.DEFAULT_THRESHOLD,
                ((PROSACRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getThreshold(), 0.0);

        // set new value
        final double value = new Random().nextDouble();
        estimator.setRssiThreshold(value);

        // check
        assertEquals(value, estimator.getRssiThreshold(), 0.0);

        // check that internal rssi estimator also has new value

        // setup rssi estimator
        estimator.setupRssiEstimator();

        assertEquals(RobustEstimatorMethod.PROSAC, estimator.mRssiEstimator.getMethod());
        assertEquals(value, ((PROSACRobustRssiPositionEstimator3D) estimator.mRssiEstimator).getThreshold(),
                0.0);
    }

    @Test
    public void testGetSetSources() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY, new InhomogeneousPoint3D()));
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
            estimator.setSources(new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        // check default value
        assertNull(estimator.getSourceQualityScores());

        // set new value
        final double[] value = new double[4];
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        // check default value
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // set new value
        final double[] value = new double[4];
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        final Point3D p = Point3D.create();
        estimator.setInitialPosition(p);

        // check
        assertSame(p, estimator.getInitialPosition());
    }

    @Test
    public void testEstimateRanging() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRssi() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRangingAndRssi() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateMixed() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());


            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRangingMultipleReadingsPerSource() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRanging1;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                for (int j = 0; j < NUM_READINGS; j++) {

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRssiMultipleReadingsPerSource() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRssi2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);
                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                for (int j = 0; j < NUM_READINGS; j++) {

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRangingAndRssiMultipleReadingsPerSource() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator =
                new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateMixedMultipleReadingsPerSource() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j)] = 1.0 / (1.0 + Math.abs(errorRssi2));
                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j) + 1] =
                            1.0 / (1.0 + Math.abs(errorRanging2));
                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j) + 2] =
                            1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                            Math.sqrt(RX_POWER_VARIANCE)));
                    readings.add(new RangingReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2), RANGING_STD));
                    readings.add(new RangingAndRssiReading<>(accessPoint,
                            Math.max(0.0, distance + errorRanging1 + errorRanging2),
                            rssi + errorRssi1 + errorRssi2, RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRangingWithInlierError() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRssiWithInlierError() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRangingAndRssiWithInlierError() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateMixedWithInlierError() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

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
                final double tmp = Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError);
                readings.add(new RangingReading<>(accessPoint, tmp, RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint, tmp,
                        rssi + errorRssi1 + errorRssi2 + inlierError, RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator =
                    new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRangingMultipleReadingsPerSourceWithInlierError() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRanging1;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRanging1 = errorRandomizer.nextDouble();
                } else {
                    errorRanging1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRanging1));
                for (int j = 0; j < NUM_READINGS; j++) {

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator =
                    new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator =
                new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRssiMultipleReadingsPerSourceWithInlierError() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRssi2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);
                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    errorRssi1 = errorRandomizer.nextDouble();
                } else {
                    errorRssi1 = 0.0;
                }

                sourceQualityScores[i] = 1.0 / (1.0 + Math.abs(errorRssi1));
                for (int j = 0; j < NUM_READINGS; j++) {

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRangingAndRssiMultipleReadingsPerSourceWithInlierError() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateMixedMultipleReadingsPerSourceWithInlierError() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * NUM_READINGS * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            double inlierError;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j)] = 1.0 / (1.0 + Math.abs(errorRssi2));
                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j) + 1] =
                            1.0 / (1.0 + Math.abs(errorRanging2));
                    fingerprintReadingsQualityScores[3 * (i * NUM_READINGS + j) + 2] =
                            1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                    inlierError = inlierErrorRandomizer.nextDouble();

                    readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2 + inlierError,
                            Math.sqrt(RX_POWER_VARIANCE)));
                    final double tmp = Math.max(0.0, distance + errorRanging1 + errorRanging2 + inlierError);
                    readings.add(new RangingReading<>(accessPoint, tmp, RANGING_STD));
                    readings.add(new RangingAndRssiReading<>(accessPoint, tmp,
                            rssi + errorRssi1 + errorRssi2 + inlierError, RANGING_STD,
                            Math.sqrt(RX_POWER_VARIANCE)));
                }
            }

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator =
                    new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateRangingLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRssiLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingAndRssiLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateMixedLinearSolverUsedHomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator =
                    new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

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

                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), RANGING_STD));
            }

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRssiLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingAndRssiLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateMixedLinearSolverUsedInhomogeneousAndPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingPreliminaryNotRefined() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRssiPreliminaryNotRefined() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingAndRssiPreliminaryNotRefined() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateMixedPreliminaryNotRefined() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

                fingerprintReadingsQualityScores[3 * i] = 1.0 / (1.0 + Math.abs(errorRssi2));
                fingerprintReadingsQualityScores[3 * i + 1] = 1.0 / (1.0 + Math.abs(errorRanging2));
                fingerprintReadingsQualityScores[3 * i + 2] = 1.0 / (1.0 + Math.abs(errorRssi2 + errorRanging2));

                readings.add(new RssiReading<>(accessPoint, rssi + errorRssi1 + errorRssi2,
                        Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), RANGING_STD));
                readings.add(new RangingAndRssiReading<>(accessPoint,
                        Math.max(0.0, distance + errorRanging1 + errorRanging2), rssi + errorRssi1 + errorRssi2,
                        RANGING_STD, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingLinearDisabled() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRssiLinearDisabled() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingAndRssiLinearDisabled() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateMixedLinearDisabled() throws LockedException, NotReadyException, RobustEstimatorException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

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

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingLinearDisabledAndNotPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRssiLinearDisabledAndNotPreliminaryRefined() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingAndRssiLinearDisabledAndNotPreliminaryRefined() throws LockedException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator =
                    new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateMixedLinearDisabledAndNotPreliminaryRefined() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator =
                    new SequentialRobustMixedPositionEstimator3D(sourceQualityScores, fingerprintReadingsQualityScores,
                            sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingLinearDisabledWithInitialPosition() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRanging1;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated3D locatedAccessPoint = new WifiAccessPointLocated3D(bssid, FREQUENCY,
                        accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

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

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRssiLinearDisabledWithInitialPosition() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRssi2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance, pathLossExponent));

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

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateRangingAndRssiLinearDisabledWithInitialPosition() throws LockedException,
            NotReadyException, RobustEstimatorException, NonSymmetricPositiveDefiniteMatrixException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValidPosition = 0;
        double positionStd = 0.0;
        double positionStdConfidence = 0.0;
        double positionAccuracy = 0.0;
        double positionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
    public void testEstimateMixedLinearDisabledWithInitialPosition() throws LockedException, NotReadyException,
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());

            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourceQualityScores = new double[numSources];
            final double[] fingerprintReadingsQualityScores = new double[3 * numSources];
            double errorRssi1;
            double errorRanging1;
            double errorRssi2;
            double errorRanging2;
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid, FREQUENCY, transmittedPowerdBm,
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

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>(readings);

            final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D(
                    sourceQualityScores, fingerprintReadingsQualityScores, sources, fingerprint, this);
            estimator.setResultRefined(true);
            estimator.setRangingPreliminarySubsetSize(5);
            estimator.setRssiPreliminarySubsetSize(5);

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

            final Point3D p = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateProgressChange >= 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());


            final Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertSame(p, estimatedPosition);
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getCovariance());

            final Accuracy3D accuracyStd = new Accuracy3D(estimator.getCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            final Accuracy3D accuracy = new Accuracy3D(estimator.getCovariance());
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
        final SequentialRobustMixedPositionEstimator3D estimator = new SequentialRobustMixedPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(final SequentialRobustMixedPositionEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((SequentialRobustMixedPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateEnd(final SequentialRobustMixedPositionEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((SequentialRobustMixedPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateProgressChange(final SequentialRobustMixedPositionEstimator<Point3D> estimator,
                                         final float progress) {
        estimateProgressChange++;
        checkLocked((SequentialRobustMixedPositionEstimator3D) estimator);
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

    private void checkLocked(final SequentialRobustMixedPositionEstimator3D estimator) {
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
