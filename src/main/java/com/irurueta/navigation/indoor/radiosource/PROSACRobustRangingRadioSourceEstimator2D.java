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

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RangingReadingLocated;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimated 2D position of a radio source (e.g. Wi-Fi
 * access point or bluetooth beacon), by discarding outliers using PROSAC
 * algorithm.
 *
 * @param <S> a {@link RadioSource} type.
 */
public class PROSACRobustRangingRadioSourceEstimator2D<S extends RadioSource> extends
        RobustRangingRadioSourceEstimator2D<S> {

    /**
     * Constant defining default threshold to determine whether samples are inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 0.1;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;

    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;

    /**
     * Threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and
     * distances provided for each sample.
     */
    private double mThreshold = DEFAULT_THRESHOLD;

    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROSACRobustRangingRadioSourceEstimator2D() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param readings radio signal ranging readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final List<? extends RangingReadingLocated<S, Point2D>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of attending events raised by this instance.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final List<? extends RangingReadingLocated<S, Point2D>> readings,
            final RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final Point2D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param readings        radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final List<? extends RangingReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition) {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     *
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final Point2D initialPosition,
            final RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param readings        radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio source
     *                        position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final List<? extends RangingReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final double[] qualityScores) {
        super();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings      radio signal ranging readings belonging to the same
     *                      radio source.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingReadingLocated<S, Point2D>> readings) {
        super(readings);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final double[] qualityScores,
            final RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sample. The larger the score value the better
     *                      the quality of the sample.
     * @param readings      radio signal readings belonging to the same radio source.
     * @param listener      listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingReadingLocated<S, Point2D>> readings,
            final RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     * @throws IllegalArgumentException if quality scores is null, or length
     *                                  of quality scores is less than required minimum.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final double[] qualityScores,
            final Point2D initialPosition) {
        super(initialPosition);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param readings        radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition) {
        super(readings, initialPosition);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final double[] qualityScores,
            final Point2D initialPosition,
            final RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(initialPosition, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        sample. The larger the score value the better
     *                        the quality of the sample.
     * @param readings        radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio source
     *                        position.
     * @param listener        listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid, quality scores
     *                                  is null, or length of quality scores is less than required minimum.
     */
    public PROSACRobustRangingRadioSourceEstimator2D(
            final double[] qualityScores,
            final List<? extends RangingReadingLocated<S, Point2D>> readings,
            final Point2D initialPosition,
            final RobustRangingRadioSourceEstimatorListener<S, Point2D> listener) {
        super(readings, initialPosition, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and distances
     * provided for each sample.
     *
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException          if this solver is locked.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }

    /**
     * Returns quality scores corresponding to each pair of
     * positions and distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @return quality scores corresponding to each sample.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each pair of positions and
     * distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if robust solver is locked because an
     *                                  estimation is already in progress.
     */
    @Override
    public void setQualityScores(final double[] qualityScores)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mReadings.size();
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return mComputeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if this solver is locked.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mComputeAndKeepInliers = computeAndKeepInliers;
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResidualsEnabled() {
        return mComputeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if this solver is locked.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mComputeAndKeepResiduals = computeAndKeepResiduals;
    }

    /**
     * Robustly estimates position for a radio source.
     *
     * @throws LockedException          if instance is busy during estimation.
     * @throws NotReadyException        if estimator is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final PROSACRobustEstimator<Solution<Point2D>> innerEstimator =
                new PROSACRobustEstimator<>(
                        new PROSACRobustEstimatorListener<Solution<Point2D>>() {

                            @Override
                            public double[] getQualityScores() {
                                return mQualityScores;
                            }

                            @Override
                            public double getThreshold() {
                                return mThreshold;
                            }

                            @Override
                            public int getTotalSamples() {
                                return mReadings.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return Math.max(mPreliminarySubsetSize, getMinReadings());
                            }

                            @Override
                            public void estimatePreliminarSolutions(
                                    final int[] samplesIndices,
                                    final List<Solution<Point2D>> solutions) {
                                solvePreliminarySolutions(samplesIndices, solutions);
                            }

                            @Override
                            public double computeResidual(
                                    final Solution<Point2D> currentEstimation,
                                    final int i) {
                                return residual(currentEstimation, i);
                            }

                            @Override
                            public boolean isReady() {
                                return PROSACRobustRangingRadioSourceEstimator2D.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(
                                    final RobustEstimator<Solution<Point2D>> estimator) {
                                // no action needed
                            }

                            @Override
                            public void onEstimateEnd(
                                    final RobustEstimator<Solution<Point2D>> estimator) {
                                // no action needed
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<Solution<Point2D>> estimator,
                                    final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            PROSACRobustRangingRadioSourceEstimator2D.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<Solution<Point2D>> estimator,
                                    final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            PROSACRobustRangingRadioSourceEstimator2D.this,
                                            progress);
                                }
                            }
                        });

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            mInliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(
                    mComputeAndKeepInliers || mRefineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(
                    mComputeAndKeepResiduals || mRefineResult);
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            final Solution<Point2D> result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            attemptRefine(result);

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
    }


    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 3 samples.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null ||
                qualityScores.length < getMinReadings()) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }
}
