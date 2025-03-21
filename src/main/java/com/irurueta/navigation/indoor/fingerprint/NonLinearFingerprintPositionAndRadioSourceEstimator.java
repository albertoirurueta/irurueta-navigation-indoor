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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

/**
 * Base class for position and radio source estimators based only on located
 * fingerprints containing RSSI readings.
 * All implementations of this class estimate the position of a new fingerprint
 * and the position of all radio sources associated to fingerprints whose location
 * is known.
 * All implementations solve the problem in a non-linear way using Levenberg-Marquardt
 * algorithm.
 */
public abstract class NonLinearFingerprintPositionAndRadioSourceEstimator<P extends Point<?>> extends
        FingerprintPositionAndRadioSourceEstimator<P> {

    /**
     * Default RSSI standard deviation assumed for provided fingerprints as a fallback
     * when none can be determined.
     */
    public static final double FALLBACK_RSSI_STANDARD_DEVIATION = 1e-3;

    /**
     * Indicates that by default measured RSSI standard deviation of closest fingerprint
     * must be propagated into measured RSSI reading variance at unknown location.
     */
    public static final boolean DEFAULT_PROPAGATE_FINGERPRINT_RSSI_STANDARD_DEVIATION = true;

    /**
     * Indicates that by default path-loss exponent standard deviation of radio source
     * must be propagated into measured RSSI reading variance at unknown location.
     */
    public static final boolean DEFAULT_PROPAGATE_PATHLOSS_EXPONENT_STANDARD_DEVIATION = true;

    /**
     * Indicates that by default covariance of closest fingerprint position must be
     * propagated into measured RSSI reading variance at unknown location.
     */
    public static final boolean DEFAULT_PROPAGATE_FINGERPRINT_POSITION_COVARIANCE = true;

    /**
     * Indicates that by default covariance of radio source position must be propagated
     * into measured RSSI reading variance at unknown location.
     */
    public static final boolean DEFAULT_PROPAGATE_RADIO_SOURCE_POSITION_COVARIANCE = true;

    /**
     * Small value to be used as machine precision.
     */
    private static final double TINY = 1e-12;

    /**
     * Initial sources whose location is known.
     * If provided, their location will be used as initial values, but
     * after executing this estimator they will be refined.
     */
    protected List<? extends RadioSourceLocated<P>> mInitialLocatedSources;

    /**
     * Initial position to start the estimation.
     * This should be a value close to the expected solution.
     * If no value is provided, the average position among all selected nearest
     * located fingerprints will be used.
     */
    private P initialPosition;

    /**
     * Indicates whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     */
    protected boolean useSourcesPathLossExponentWhenAvailable = true;

    /**
     * RSSI standard deviation fallback value to use when none can be
     * determined from provided readings. This fallback value is only used if
     * no variance is propagated or the resulting value is too small to allow
     * convergence to a solution.
     */
    private double fallbackRssiStandardDeviation = FALLBACK_RSSI_STANDARD_DEVIATION;

    /**
     * Indicates whether measured RSSI standard deviation of closest fingerprint must
     * be propagated into measured RSSI reading variance at unknown location.
     */
    private boolean propagateFingerprintRssiStandardDeviation = DEFAULT_PROPAGATE_FINGERPRINT_RSSI_STANDARD_DEVIATION;

    /**
     * Indicates whether path-loss exponent standard deviation of radio source must
     * be propagated into measured RSSI reading variance at unknown location.
     */
    private boolean propagatePathlossExponentStandardDeviation = DEFAULT_PROPAGATE_PATHLOSS_EXPONENT_STANDARD_DEVIATION;

    /**
     * Indicates whether covariance of closest fingerprint position must be
     * propagated into measured RSSI reading variance at unknown location.
     */
    private boolean propagateFingerprintPositionCovariance = DEFAULT_PROPAGATE_FINGERPRINT_POSITION_COVARIANCE;

    /**
     * Indicates whether covariance of radio source position must be propagated
     * into measured RSSI reading variance at unknown location.
     */
    private boolean propagateRadioSourcePositionCovariance = DEFAULT_PROPAGATE_RADIO_SOURCE_POSITION_COVARIANCE;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter fitter = new LevenbergMarquardtMultiDimensionFitter();

    /**
     * Estimated covariance matrix for estimated non-located fingerprint position and
     * estimated located radio sources position.
     */
    private Matrix covariance;

    /**
     * Covariance of estimated position for non-located fingerprint.
     */
    private Matrix estimatedPositionCovariance;

    /**
     * Estimated chi square value.
     */
    private double chiSq;

    /**
     * Constructor.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @throws IllegalArgumentException if either non located fingerprint or located
     *                                  fingerprints are null.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint) {
        super(locatedFingerprints, fingerprint);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param listener            listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     *                                  fingerprints are null.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param initialPosition     initial position to be assumed on non located fingerprint or
     *                            null if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     *                                  fingerprints are null.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final P initialPosition) {
        super(locatedFingerprints, fingerprint);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param initialPosition     initial position to be assumed on non located fingerprint or
     *                            null if unknown.
     * @param listener            listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     *                                  fingerprints are null.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final P initialPosition,
            final FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
        this.initialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints   located fingerprints containing RSSI readings.
     * @param fingerprint           fingerprint containing readings at an unknown location
     *                              for provided located fingerprints.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     *                                  fingerprints are null.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<P>> initialLocatedSources) {
        super(locatedFingerprints, fingerprint);
        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints   located fingerprints containing RSSI readings.
     * @param fingerprint           fingerprint containing readings at an unknown location
     *                              for provided located fingerprints.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @param listener              listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     *                                  fingerprints are null.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<P>> initialLocatedSources,
            final FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints   located fingerprints containing RSSI readings.
     * @param fingerprint           fingerprint containing readings at an unknown location
     *                              for provided located fingerprints.
     * @param initialPosition       initial position to be assumed on non located fingerprint or
     *                              null if unknown.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     *                                  fingerprints are null.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final P initialPosition,
            final List<? extends RadioSourceLocated<P>> initialLocatedSources) {
        this(locatedFingerprints, fingerprint, initialPosition);
        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints   located fingerprints containing RSSI readings.
     * @param fingerprint           fingerprint containing readings at an unknown location
     *                              for provided located fingerprints.
     * @param initialPosition       initial position to be assumed on non located fingerprint or
     *                              null if unknown.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @param listener              listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     *                                  fingerprints are null.
     */
    protected NonLinearFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final P initialPosition,
            final List<? extends RadioSourceLocated<P>> initialLocatedSources,
            final FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        this(locatedFingerprints, fingerprint, initialPosition, listener);
        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Gets initial radio sources whose location is known.
     *
     * @return initial radio sources.
     */
    public List<RadioSourceLocated<P>> getInitialLocatedSources() {
        //noinspection unchecked
        return (List<RadioSourceLocated<P>>) mInitialLocatedSources;
    }

    /**
     * Sets initial radio sources whose location is known.
     *
     * @param initialLocatedSources initial radio sources.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialLocatedSources(final List<? extends RadioSourceLocated<P>> initialLocatedSources)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Gets initial position to start the solving algorithm.
     * This should be a value close to the expected solution.
     * If no value is provided, the average position among all selected nearest
     * located fingerprints will be used.
     *
     * @return initial position to start the solving algorithm or null.
     */
    public P getInitialPosition() {
        return initialPosition;
    }

    /**
     * Sets initial position to start the solving algorithm.
     * This should be a value close to the expected solution.
     * If no value is provided, the average position among all selected nearest
     * located fingerprints will be used.
     *
     * @param initialPosition initial position to start the solving algorithm or null.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(final P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.initialPosition = initialPosition;
    }

    /**
     * Gets estimated covariance matrix for estimated non-located fingerprint position
     * and estimated located radio sources position.
     *
     * @return estimated covariance matrix for estimated non-located fingerprint
     * position and estimated located radio sources position.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Gets covariance of estimated position for non-located fingerprint.
     *
     * @return covariance of estimated position for non-located fingerprint.
     */
    public Matrix getEstimatedPositionCovariance() {
        return estimatedPositionCovariance;
    }

    /**
     * Gets estimated chi square value.
     *
     * @return estimated chi square value.
     */
    public double getChiSq() {
        return chiSq;
    }

    /**
     * Indicates whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     *
     * @return true to use path loss exponent of provided sources when available,
     * false otherwise.
     */
    public boolean getUseSourcesPathLossExponentWhenAvailable() {
        return useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Specifies whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     *
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setUseSourcesPathLossExponentWhenAvailable(final boolean useSourcesPathLossExponentWhenAvailable)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.useSourcesPathLossExponentWhenAvailable = useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Gets RSSI standard deviation fallback value to use when none can be
     * determined from provided readings.
     *
     * @return RSSI standard deviation fallback.
     */
    public double getFallbackRssiStandardDeviation() {
        return fallbackRssiStandardDeviation;
    }

    /**
     * Sets RSSI standard deviation fallback value to use when none can be
     * determined from provided readings.
     *
     * @param fallbackRssiStandardDeviation RSSI standard deviation fallback
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is smaller than
     *                                  {@link #TINY}.
     */
    public void setFallbackRssiStandardDeviation(final double fallbackRssiStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (fallbackRssiStandardDeviation < TINY) {
            throw new IllegalArgumentException();
        }
        this.fallbackRssiStandardDeviation = fallbackRssiStandardDeviation;
    }

    /**
     * Indicates whether measured RSSI standard deviation of closest fingerprint must be
     * propagated into measured RSSI reading variance at unknown location.
     *
     * @return true to propagate RSSI standard deviation of closest fingerprint,
     * false otherwise.
     */
    public boolean isFingerprintRssiStandardDeviationPropagated() {
        return propagateFingerprintRssiStandardDeviation;
    }

    /**
     * Specifies whether measured RSSI standard deviation of closest fingerprint must be
     * propagated into measured RSSI reading variance at unknown location.
     *
     * @param propagateFingerprintRssiStandardDeviation true to propagate RSSI standard
     *                                                  deviation of closest fingerprint,
     *                                                  false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprintRssiStandardDeviationPropagated(final boolean propagateFingerprintRssiStandardDeviation)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.propagateFingerprintRssiStandardDeviation = propagateFingerprintRssiStandardDeviation;
    }

    /**
     * Indicates whether path-loss exponent standard deviation of radio source must be
     * propagated into measured RSSI reading variance at unknown location.
     *
     * @return true to propagate  path-loss exponent standard deviation of radio source,
     * false otherwise.
     */
    public boolean isPathlossExponentStandardDeviationPropagated() {
        return propagatePathlossExponentStandardDeviation;
    }

    /**
     * Specifies whether path-loss exponent standard deviation of radio source must be
     * propagated into measured RSSI reading variance at unknown location.
     *
     * @param propagatePathlossExponentStandardDeviation true to propagate path-loss
     *                                                   exponent standard deviation of
     *                                                   radio source, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPathlossExponentStandardDeviationPropagated(
            final boolean propagatePathlossExponentStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.propagatePathlossExponentStandardDeviation = propagatePathlossExponentStandardDeviation;
    }

    /**
     * Indicates whether covariance of closest fingerprint position must be propagated
     * into measured RSSI reading variance at unknown location.
     *
     * @return true to propagate fingerprint position covariance, false otherwise.
     */
    public boolean isFingerprintPositionCovariancePropagated() {
        return propagateFingerprintPositionCovariance;
    }

    /**
     * Specifies whether covariance of closest fingerprint position must be propagated
     * into measured RSSI reading variance at unknown location.
     *
     * @param propagateFingerprintPositionCovariance true to propagate fingerprint
     *                                               position covariance, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprintPositionCovariancePropagated(final boolean propagateFingerprintPositionCovariance)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.propagateFingerprintPositionCovariance = propagateFingerprintPositionCovariance;
    }

    /**
     * Indicates whether covariance of radio source position must be propagated into
     * measured RSSI reading variance at unknown location.
     *
     * @return true to propagate radio source position covariance, false otherwise.
     */
    public boolean isRadioSourcePositionCovariancePropagated() {
        return propagateRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether covariance of radio source position must be propagated into
     * measured RSSI reading variance at unknown location.
     *
     * @param propagateRadioSourcePositionCovariance true to propagate radio source
     *                                               position covariance, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRadioSourcePositionCovariancePropagated(final boolean propagateRadioSourcePositionCovariance)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.propagateRadioSourcePositionCovariance = propagateRadioSourcePositionCovariance;
    }

    /**
     * Estimates position and radio sources based on provided located radio sources and readings of
     * such radio sources at an unknown location.
     *
     * @throws LockedException                if estimator is locked.
     * @throws NotReadyException              if estimator is not ready.
     * @throws FingerprintEstimationException if estimation fails for some other reason.
     */
    @Override
    @SuppressWarnings("Duplicates")
    public void estimate() throws LockedException, NotReadyException, FingerprintEstimationException {

        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onEstimateStart(this);
            }

            RadioSourceNoMeanKNearestFinder<P, RadioSource> noMeanFinder = null;
            RadioSourceKNearestFinder<P, RadioSource> finder = null;
            if (useNoMeanNearestFingerprintFinder) {
                //noinspection unchecked
                noMeanFinder = new RadioSourceNoMeanKNearestFinder<>(
                        (Collection<RssiFingerprintLocated<RadioSource,
                                RssiReading<RadioSource>, P>>) locatedFingerprints);
            } else {
                //noinspection unchecked
                finder = new RadioSourceKNearestFinder<>(
                        (Collection<RssiFingerprintLocated<RadioSource,
                                RssiReading<RadioSource>, P>>) locatedFingerprints);
            }

            estimatedPositionCoordinates = null;
            covariance = null;
            estimatedPositionCovariance = null;
            nearestFingerprints = null;
            estimatedLocatedSources = null;

            final var min = Math.max(1, minNearestFingerprints);
            final var max = maxNearestFingerprints < 0
                    ? locatedFingerprints.size()
                    : Math.min(maxNearestFingerprints, locatedFingerprints.size());
            for (var k = min; k <= max; k++) {
                if (noMeanFinder != null) {
                    //noinspection unchecked
                    nearestFingerprints = noMeanFinder.findKNearestTo(
                            (RssiFingerprint<RadioSource, RssiReading<RadioSource>>) fingerprint, k);
                } else {
                    //noinspection unchecked
                    nearestFingerprints = finder.findKNearestTo(
                            (RssiFingerprint<RadioSource, RssiReading<RadioSource>>) fingerprint, k);
                }

                // Demonstration in 2D:
                // --------------------

                // The expression of received power expressed in dBm's is:
                // k = (c/(4*pi*f))
                // Pr = Pte*k^n / d^n

                // where c is the speed of light, pi is 3.14159..., f is the frequency of the radio source,
                // Pte is the equivalent transmitted power by the radio source, n is the path-loss exponent
                // (typically 2.0), and d is the distance from a point to the location of the radio source.

                // Hence:
                // Pr(dBm) = 10*log(Pte*k^n/d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d) =
                //           10*n*log(k) + 10*log(Pte) - 5*n*log(d^2)

                // where d^2 = dia^2 = (xi - xa)^2 + (yi - ya)^2 is the squared distance between
                // fingerprint and unknown point pi = (xi, yi) and
                // radio source a = a,b... M
                // 10*n*log(k) is constant for a given radio source "a", and
                // Pte is the equivalent transmitted power of radio source "a".

                // We assume that the 2 former terms are constant and known for a given radio source
                // K = 10*n*log(k) + 10*log(Pte), and the only unknown term is
                // the latter one depending on the distance of the
                // measuring point and the radio source.

                // Hence for a given radio source "a" at unknown location "i":
                // Pr(pi) = K - 5*n*log((xi - xa)^2 + (yi - ya)^2)

                // we assume that a known located fingerprint is located at p1 = (x1, y1),
                // Both readings Pr(pi) and Pr(p1) belong to the same radio source "a", hence
                // K term is the same.

                // Pr(p1) = K - 5*n*log((x1 - xa)^2 + (y1 - ya)^2)

                // where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 is the squared distance between
                // fingerprint 1 and radio source 2

                // To remove possible bias effects on readings, we consider the difference of received
                // power for fingerprint "1" and radio source "a" as:

                // Prdiff1a = Pr(pi) - Pr(p1) = (K - 5*n*log(dia^2)) - (K - 5*n*log(d1a^2)) =
                //   = 5*n*log(d1a^2) - 5*n*log(dia^2)

                // where both d1a^2 and dia^2 are unknown, because location pi=(xi,yi) and pa=(xa,ya) are unknown.
                // Now we have no dependencies on the amount of transmitted power of each radio source
                // contained on constant term K, and we only depend on squared distances d1a^2 and dia^2.

                // Consequently, the difference of received power for fingerprint "2" and radio source "a" is:
                // Prdiff2a = 5*n*log(d2a^2) - 5*n*log(dia^2)

                // the difference of received power for fingerprint "1" and radio source "b" is:
                // Prdiff1b = 5*n*log(d1b^2) - 5*n*log(dib^2)

                // and so on.

                // we want to find unknown location pi, and location of radio source pa, pb,... pM so that Prdiff
                // errors are minimized in LMSE (Least Mean Square Error) terms.

                // Assuming that we have M radio sources and N fingerprints, we have
                // y = [Prdiff1a Prdiff2a Prdiff1b Prdiff2b ... PrdiffNa PrdiffNb ... PrdiffNM]

                // and the unknowns to be found are:
                // x = [xi yi xa ya xb yb ... xM yM], which are the location of the unknown fingerprint
                // pi = (xi, yi) and the locations of the radio sources a, b ... M that we want to find pa = (xa, ya),
                // pb = (xb, yb) ... pM = (xM, yM)

                try {
                    final var sourcesToBeEstimated = setupFitter();
                    if (minNearestFingerprints < 0) {
                        // if no limit is set in minimum value, then a minimum of
                        // dims * (1 + numSources) is used
                        final var numSources = sourcesToBeEstimated.size();
                        final var dims = getNumberOfDimensions();
                        final var minNearest = dims * (1 + numSources);
                        if (k < minNearest) {
                            continue;
                        }
                    }

                    fitter.fit();

                    // estimated position
                    final var a = fitter.getA();
                    covariance = fitter.getCovar();
                    chiSq = fitter.getChisq();

                    final var dims = getNumberOfDimensions();

                    // obtain estimated position coordinates and covariance
                    estimatedPositionCoordinates = new double[dims];
                    System.arraycopy(a, 0, estimatedPositionCoordinates, 0, dims);

                    final var dimsMinusOne = dims - 1;
                    estimatedPositionCovariance = covariance.getSubmatrix(0, 0, dimsMinusOne,
                            dimsMinusOne);

                    // obtain radio sources estimated positions and covariance
                    final var totalSources = sourcesToBeEstimated.size();
                    estimatedLocatedSources = new ArrayList<>();
                    for (var j = 0; j < totalSources; j++) {
                        final var sourcePosition = createPoint();

                        final var start = dims * (1 + j);
                        final var end = start + dimsMinusOne;
                        for (var i = 0; i < dims; i++) {
                            sourcePosition.setInhomogeneousCoordinate(i, a[start + i]);
                        }

                        final var sourceCovariance = covariance.getSubmatrix(start, start, end, end);

                        final var source = sourcesToBeEstimated.get(j);
                        estimatedLocatedSources.add(createRadioSource(source, sourcePosition, sourceCovariance));
                    }

                    // a solution was found so we exit loop
                    break;
                } catch (final NumericalException e) {
                    // solution could not be found with current data
                    // Iterate to use additional nearby fingerprints
                    estimatedPositionCoordinates = null;
                    covariance = null;
                    estimatedPositionCovariance = null;
                    estimatedLocatedSources = null;
                }
            }

            if (estimatedPositionCoordinates == null || estimatedLocatedSources == null) {
                // no position could be estimated
                throw new FingerprintEstimationException();
            }

            if (listener != null) {
                listener.onEstimateEnd(this);
            }
        } finally {
            locked = false;
        }
    }

    /**
     * Propagates provided variances into RSSI differences.
     *
     * @param pathlossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest located fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non-linear algorithm.
     * @param pathlossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null if
     *                                      unknown.
     * @return variance of RSSI difference measured at non located fingerprint reading.
     */
    protected abstract Double propagateVariances(
            final double pathlossExponent, final P fingerprintPosition,
            final P radioSourcePosition, final P estimatedPosition,
            final Double pathlossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance);

    /**
     * Creates a located radio source from provided radio source, position and
     * covariance.
     *
     * @param source           radio source.
     * @param sourcePosition   radio source position.
     * @param sourceCovariance radio source position covariance.
     * @return located radio source.
     */
    private RadioSourceLocated<P> createRadioSource(
            final RadioSource source, final P sourcePosition, final Matrix sourceCovariance) {

        final var dims = getNumberOfDimensions();

        switch (source.getType()) {
            case BEACON:
                final var beacon = (Beacon) source;
                if (dims == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
                    // 2D

                    //noinspection unchecked
                    return (RadioSourceLocated<P>) new BeaconLocated2D(
                            beacon.getIdentifiers(), beacon.getTransmittedPower(),
                            beacon.getFrequency(), beacon.getBluetoothAddress(),
                            beacon.getBeaconTypeCode(), beacon.getManufacturer(),
                            beacon.getServiceUuid(), beacon.getBluetoothName(),
                            (Point2D) sourcePosition, sourceCovariance);
                } else {
                    // 3D

                    //noinspection unchecked
                    return (RadioSourceLocated<P>) new BeaconLocated3D(
                            beacon.getIdentifiers(), beacon.getTransmittedPower(),
                            beacon.getFrequency(), beacon.getBluetoothAddress(),
                            beacon.getBeaconTypeCode(), beacon.getManufacturer(),
                            beacon.getServiceUuid(), beacon.getBluetoothName(),
                            (Point3D) sourcePosition, sourceCovariance);
                }
            case WIFI_ACCESS_POINT:
            default:
                final var accessPoint = (WifiAccessPoint) source;
                if (dims == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
                    // 2D

                    //noinspection unchecked
                    return (RadioSourceLocated<P>) new WifiAccessPointLocated2D(
                            accessPoint.getBssid(), accessPoint.getFrequency(),
                            accessPoint.getSsid(), (Point2D) sourcePosition,
                            sourceCovariance);
                } else {
                    // 3D

                    //noinspection unchecked
                    return (RadioSourceLocated<P>) new WifiAccessPointLocated3D(
                            accessPoint.getBssid(), accessPoint.getFrequency(),
                            accessPoint.getSsid(), (Point3D) sourcePosition,
                            sourceCovariance);
                }
        }
    }

    /**
     * Builds data required to solve the problem.
     * This method takes into account current nearest fingerprints and discards those
     * readings belonging to radio sources not having enough data to be estimated.
     *
     * @param allPowerDiffs               list of received power differences of RSSI readings between a
     *                                    located fingerprint and an unknown fingerprint for a given radio
     *                                    source.
     * @param allFingerprintPositions     positions of all located fingerprints being taken into
     *                                    account.
     * @param allInitialSourcesPositions  initial positions of all radio sources to be taken
     *                                    into account. If initial located sources where provided,
     *                                    their positions will be used, otherwise the centroid of
     *                                    all located fingerprints associated to a radio source will
     *                                    be used as initial position.
     * @param allSourcesToBeEstimated     all radio sources that will be estimated.
     * @param allSourcesIndices           indices indicating the position radio source being used
     *                                    within the list of sources for current reading.
     * @param allPathLossExponents        list of path loss exponents.
     * @param allStandardDeviations       list of standard deviations for readings being used.
     * @param nearestFingerprintsCentroid centroid of nearest fingerprints being taken into account.
     */
    @SuppressWarnings("Duplicates")
    private void buildData(
            final List<Double> allPowerDiffs,
            final List<P> allFingerprintPositions,
            final List<P> allInitialSourcesPositions,
            final List<RadioSource> allSourcesToBeEstimated,
            final List<Integer> allSourcesIndices,
            final List<Double> allPathLossExponents,
            final List<Double> allStandardDeviations,
            final P nearestFingerprintsCentroid) {

        final var dims = getNumberOfDimensions();
        var num = 0;
        final var centroidCoords = new double[dims];
        for (final var fingerprint : nearestFingerprints) {
            final var position = fingerprint.getPosition();
            if (position == null) {
                continue;
            }

            for (var i = 0; i < dims; i++) {
                centroidCoords[i] += position.getInhomogeneousCoordinate(i);
            }
            num++;
        }

        if (num > 0) {
            for (var i = 0; i < dims; i++) {
                centroidCoords[i] /= num;
                nearestFingerprintsCentroid.setInhomogeneousCoordinate(i, centroidCoords[i]);
            }
        }

        // maps to keep cached in memory computed values to speed up computations
        final var numReadingsMap = new HashMap<RadioSource, Integer>();
        final var centroidsMap = new HashMap<RadioSource, P>();

        for (final var locatedFingerprint : nearestFingerprints) {

            final var locatedReadings = locatedFingerprint.getReadings();
            if (locatedReadings == null) {
                continue;
            }

            final var fingerprintPosition = locatedFingerprint.getPosition();
            final var fingerprintPositionCovariance = locatedFingerprint.getPositionCovariance();

            for (final var locatedReading : locatedReadings) {
                final var source = locatedReading.getSource();

                // obtain the total number of readings available for this source and
                // the centroid of all located fingerprints containing readings for
                // such source
                final int numReadings;
                if (!numReadingsMap.containsKey(source)) {
                    numReadings = totalReadingsForSource(source, nearestFingerprints, null);
                    numReadingsMap.put(source, numReadings);
                } else {
                    numReadings = numReadingsMap.get(source);
                }

                if (numReadings < dims) {
                    continue;
                }

                final P centroid;
                if (!centroidsMap.containsKey(source)) {
                    centroid = createPoint();

                    //noinspection unchecked
                    totalReadingsForSource(source,
                            (List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>>) locatedFingerprints,
                            centroid);

                    centroidsMap.put(source, centroid);
                } else {
                    centroid = centroidsMap.get(source);
                }

                // find within the list of located sources (if available) the source
                // of current located fingerprint

                //noinspection SuspiciousMethodCalls
                final var pos = mInitialLocatedSources != null ? mInitialLocatedSources.indexOf(source) : -1;

                var pathLossExponent = this.pathLossExponent;
                Double pathLossExponentVariance = null;
                if (useSourcesPathLossExponentWhenAvailable && source instanceof RadioSourceWithPower sourceWithPower) {
                    pathLossExponent = sourceWithPower.getPathLossExponent();
                    final var std = sourceWithPower.getPathLossExponentStandardDeviation();
                    pathLossExponentVariance = std != null ? std * std : null;
                }

                final P sourcePosition;
                Matrix sourcePositionCovariance = null;
                if (pos < 0) {
                    // located source is not available, so we use centroid
                    sourcePosition = centroid;
                } else {
                    final var locatedSource = mInitialLocatedSources.get(pos);
                    sourcePosition = locatedSource.getPosition();

                    if (useSourcesPathLossExponentWhenAvailable
                            && locatedSource instanceof RadioSourceWithPower locatedSourceWithPower
                            && pathLossExponentVariance == null) {
                        pathLossExponent = locatedSourceWithPower.getPathLossExponent();
                        final var std = locatedSourceWithPower.getPathLossExponentStandardDeviation();
                        pathLossExponentVariance = std != null ? std * std : null;
                    }

                    sourcePositionCovariance = locatedSource.getPositionCovariance();
                }

                final int sourceIndex;
                if (!allSourcesToBeEstimated.contains(source)) {
                    sourceIndex = allSourcesToBeEstimated.size();

                    allSourcesToBeEstimated.add(source);
                    allInitialSourcesPositions.add(sourcePosition);
                } else {
                    sourceIndex = allSourcesToBeEstimated.indexOf(source);
                }

                final var locatedRssi = locatedReading.getRssi();

                final var locatedRssiStd = locatedReading.getRssiStandardDeviation();
                final var locatedRssiVariance = locatedRssiStd != null ? locatedRssiStd * locatedRssiStd : null;

                final var readings = fingerprint.getReadings();
                for (final var reading : readings) {
                    if (reading.getSource() == null || !reading.getSource().equals(source)) {
                        continue;
                    }

                    // only take into account reading for matching sources on located
                    // and non-located readings
                    final var rssi = reading.getRssi();

                    final var powerDiff = rssi - locatedRssi;

                    Double standardDeviation = null;
                    if (propagatePathlossExponentStandardDeviation || propagateFingerprintPositionCovariance
                            || propagateRadioSourcePositionCovariance) {

                        // compute initial position
                        final var initialPosition = this.initialPosition != null
                                ? this.initialPosition : nearestFingerprintsCentroid;

                        final var variance = propagateVariances(pathLossExponent,
                                fingerprintPosition, sourcePosition, initialPosition,
                                propagatePathlossExponentStandardDeviation ? pathLossExponentVariance : null,
                                propagateFingerprintPositionCovariance ? fingerprintPositionCovariance : null,
                                propagateRadioSourcePositionCovariance ? sourcePositionCovariance : null);
                        if (variance != null) {
                            standardDeviation = Math.sqrt(variance);
                        }
                    }

                    if (standardDeviation == null) {
                        standardDeviation = reading.getRssiStandardDeviation();
                    }

                    if (propagateFingerprintRssiStandardDeviation) {
                        if (standardDeviation != null && reading.getRssiStandardDeviation() != null) {
                            // consider propagated variance and reading variance independent, so we
                            // sum them both
                            standardDeviation = standardDeviation * standardDeviation
                                    + reading.getRssiStandardDeviation() * reading.getRssiStandardDeviation();
                            standardDeviation = Math.sqrt(standardDeviation);
                        }

                        if (locatedRssiVariance != null && standardDeviation != null) {
                            // consider propagated variance and located reading variance
                            // independent, so we sum them both
                            standardDeviation = standardDeviation * standardDeviation + locatedRssiVariance;
                            standardDeviation = Math.sqrt(standardDeviation);
                        }
                    }

                    if (standardDeviation == null || standardDeviation < TINY) {
                        standardDeviation = fallbackRssiStandardDeviation;
                    }

                    allPowerDiffs.add(powerDiff);
                    allFingerprintPositions.add(fingerprintPosition);
                    allSourcesIndices.add(sourceIndex);
                    allPathLossExponents.add(pathLossExponent);
                    allStandardDeviations.add(standardDeviation);
                }
            }
        }
    }

    /**
     * Setups fitter to solve positions.
     *
     * @return list of radio sources whose location will be estimated.
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    @SuppressWarnings("Duplicates")
    private List<RadioSource> setupFitter() throws FittingException {
        // build lists of data
        final var allPowerDiffs = new ArrayList<Double>();
        final var allFingerprintPositions = new ArrayList<P>();
        final var allInitialSourcesPositions = new ArrayList<P>();
        final var allSourcesToBeEstimated = new ArrayList<RadioSource>();
        final var allSourcesIndices = new ArrayList<Integer>();
        final var allPathLossExponents = new ArrayList<Double>();
        final var allStandardDeviations = new ArrayList<Double>();
        final var nearestFingerprintsCentroid = createPoint();
        buildData(allPowerDiffs, allFingerprintPositions, allInitialSourcesPositions, allSourcesToBeEstimated,
                allSourcesIndices, allPathLossExponents, allStandardDeviations, nearestFingerprintsCentroid);

        final var totalReadings = allPowerDiffs.size();
        final var totalSources = allSourcesToBeEstimated.size();
        final var dims = getNumberOfDimensions();
        final var n = 1 + dims;

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return n;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[dims * (totalSources + 1)];

                if (initialPosition == null) {
                    // use centroid of nearest fingerprints as initial value
                    for (var i = 0; i < dims; i++) {
                        initial[i] = nearestFingerprintsCentroid.getInhomogeneousCoordinate(i);
                    }
                } else {
                    // use provided initial position
                    for (var i = 0; i < dims; i++) {
                        initial[i] = initialPosition.getInhomogeneousCoordinate(i);
                    }
                }

                var pos = dims;
                for (var j = 0; j < totalSources; j++) {
                    final var initialSourcePosition = allInitialSourcesPositions.get(j);
                    for (var i = 0; i < dims; i++) {
                        initial[pos] = initialSourcePosition.getInhomogeneousCoordinate(i);
                        pos++;
                    }
                }

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params, final double[] derivatives) {

                // For 2D:
                // -------

                // Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
                //   = 5*n*log((x1 - xa)^2 + (y1 - ya)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2)

                // derivatives respect parameters being estimated (xi,yi,xa,ya...,xM,yM)
                // for unknown point pi = (xi, yi)
                // diff(Prdiff1a)/diff(xi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2))*2*(xi - xa)
                //   = -10*n*(xi - xa)/(log(10)*((xi - xa)^2 + (yi - ya)^2))
                //   = -10*n*(xi - xa)/(log(10)*dia^2)

                // diff(Prdiff1a)/diff(yi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2))*2*(yi - ya)
                //   = -10*n*(yi - ya)/(log(10)*((xi - xa)^2 + (yi - ya)^2))
                //   = -10*n*(yi - ya)/(log(10)*dia^2)

                // for same radio source pa=(xa,ya)
                // diff(Prdiff1a)/diff(xa) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2))*-2*(x1 - xa) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2))*-2*(xi - xa) =
                //   = -10*n*(x1 - xa)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2)) + 10*n*(xi - xa)/(log(10)*((xi - xa)^2 + (yi - ya)^2))
                //   = 10*n*(-(x1 - xa)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2))

                // diff(Prdiff1a)/diff(ya) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2))*-2*(y1 - ya) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2))*-2*(yi - ya) =
                //   = -10*n*(y1 - ya)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2)) + 10*n*(yi - ya)/(log(10)*((xi - xa)^2 + (yi - ya)^2))
                //   = 10*n*(-(y1 - ya)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2))

                // for other radio source pb=(xb,yb)
                // diff(Prdiff1a)/diff(xb) = diff(Prdiff1a)/diff(yb) = 0

                // For 3D:
                // -------

                // Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
                //   = 5*n*log((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)

                // derivatives respect parameters being estimated (xi,yi,zi,xa,ya,za...,xM,yM,zM)
                // for unknown point pi = (xi, yi,zi)
                // diff(Prdiff1a)/diff(xi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(xi - xa)
                //   = -10*n*(xi - xa)/(log(10)*((xi - xa)^2 + (yi - ya)^2) + (zi - za)^2))
                //   = -10*n*(xi - xa)/(log(10)*dia^2)

                // diff(Prdiff1a)/diff(yi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(yi - ya)
                //   = -10*n*(yi - ya)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))
                //   = -10*n*(yi - ya)/(log(10)*dia^2)

                // diff(Prdiff1a)/diff(zi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(zi - za)
                //   = -10*n*(zi - za)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))
                //   = -10*n*(zi - za)/(log(10)*dia^2)

                // for same radio source pa=(xa,ya)
                // diff(Prdiff1a)/diff(xa) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(x1 - xa) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2*(xi - xa) =
                //   = -10*n*(x1 - xa)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(xi - xa)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)) =
                //   = 10*n*(-(x1 -xa)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2))

                // diff(Prdiff1a)/diff(ya) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(y1 - ya) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2*(yi - ya) =
                //   = -10*n*(y1 - ya)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(yi - ya)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)) =
                //   = 10*n(-(y1 - ya)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2))

                // diff(Prdiff1a)/diff(za) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(z1 - za) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2*(zi - za) =
                //   = -10*n*(z1 - za)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(zi - za)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)) =
                //   = 10*n(-(z1 - za)/(log(10)*d1a^2) + (zi - za)/(log(10)*dia^2))

                // for other radio source pb=(xb,yb)
                // diff(Prdiff1a)/diff(xb) = diff(Prdiff1a)/diff(yb) = 0

                final var dims = NonLinearFingerprintPositionAndRadioSourceEstimator.this.getNumberOfDimensions();

                // path loss exponent
                final var n = point[0];

                final var ln10 = Math.log(10.0);

                final var sourceIndex = allSourcesIndices.get(i);
                final var start = dims * (1 + sourceIndex);

                // d1a^2, d2a^2, ...
                var distanceFingerprint2 = 0.0;

                // dia^2, dib^2, ...
                var distancePoint2 = 0.0;
                for (var j = 0; j < dims; j++) {
                    // fingerprint coordinate p1=(x1,y1,z1), ...
                    final var fingerprintCoord = point[1 + j];

                    // unknown point "pi" coordinate
                    final var pointCoord = params[j];

                    // radio source coordinate pa=(xa,ya,za), ...
                    final var sourceCoord = params[start + j];

                    // x1 - xa, y1 - ya, ...
                    final var diffFingerprint = fingerprintCoord - sourceCoord;

                    // xi - xa, yi - ya, ...
                    final var diffPoint = pointCoord - sourceCoord;

                    final var diffFingerprint2 = diffFingerprint * diffFingerprint;
                    final var diffPoint2 = diffPoint * diffPoint;

                    distanceFingerprint2 += diffFingerprint2;
                    distancePoint2 += diffPoint2;
                }

                distanceFingerprint2 = Math.max(distanceFingerprint2, TINY);
                distancePoint2 = Math.max(distancePoint2, TINY);

                final var result = 5 * n * (Math.log10(distanceFingerprint2) - Math.log10(distancePoint2));


                // we clear derivatives array to ensure that derivatives respect other
                // radio sources are zero
                Arrays.fill(derivatives, 0.0);

                for (var j = 0; j < dims; j++) {
                    // fingerprint coordinate p1=(x1,y1,z1), ...
                    final var fingerprintCoord = point[1 + j];

                    // unknown point "pi" coordinate
                    final var pointCoord = params[j];

                    // radio source coordinate pa=(xa,ya,za), ...
                    final var sourceCoord = params[start + j];

                    // x1 - xa, y1 - ya, ...
                    final var diffFingerprint = fingerprintCoord - sourceCoord;

                    // xi - xa, yi - ya, ...
                    final var diffPoint = pointCoord - sourceCoord;

                    // Example: diff(Prdiff1a)/diff(xi) =  -10*n*(xi - xa)/(log(10)*dia^2)
                    final var derivativePointCoord = -10.0 * n * diffPoint / (ln10 * distancePoint2);

                    // Example: diff(Prdiff1a)/diff(xa) = 10*n*(-(x1 - xa)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2)) =
                    //   -10*n*(x1 - xa)/(log(10)*d1a^2) - diff(Prdiff1a)/diff(xi)
                    final var derivativeSameRadioSourceCoord =
                            -10.0 * n * diffFingerprint / (ln10 * distanceFingerprint2) - derivativePointCoord;

                    // derivatives respect point pi = (xi, yi, zi)
                    derivatives[j] = derivativePointCoord;

                    // derivatives respect same radio source pa = (xa, ya, za)
                    derivatives[dims * (1 + sourceIndex) + j] = derivativeSameRadioSourceCoord;
                }

                return result;
            }
        });

        try {
            // In 2D we know that for fingerprint "1" and radio source "a":
            // Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
            //   = 5*n*log((x1 - xa)^2 + (y1 - ya)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2)

            // Therefore x must have 1 + dims columns (for path-loss n and fingerprint position (x1,y1)

            final var x = new Matrix(totalReadings, n);
            final var y = new double[totalReadings];
            final var standardDeviations = new double[totalReadings];
            for (var i = 0; i < totalReadings; i++) {
                // path loss exponent
                x.setElementAt(i, 0, allPathLossExponents.get(i));

                final var fingerprintPosition = allFingerprintPositions.get(i);
                var col = 1;
                for (var j = 0; j < dims; j++) {
                    x.setElementAt(i, col, fingerprintPosition.getInhomogeneousCoordinate(j));
                    col++;
                }

                y[i] = allPowerDiffs.get(i);

                standardDeviations[i] = allStandardDeviations.get(i);
            }

            fitter.setInputData(x, y, standardDeviations);

            return allSourcesToBeEstimated;
        } catch (final AlgebraException e) {
            throw new FittingException(e);
        }
    }

    /**
     * Gets the total number of readings associated to provided radio source.
     * This method uses only current nearest fingerprints.
     *
     * @param source       radio source to be checked
     * @param centroid     centroid to be computed.
     * @param fingerprints fingerprints where search is made.
     * @return total number of readings associated to provided radio source.
     */
    private int totalReadingsForSource(
            final RadioSource source,
            final List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>> fingerprints,
            final P centroid) {
        if (source == null) {
            return 0;
        }

        final var dims = getNumberOfDimensions();

        var result = 0;
        final var centroidCoords = centroid != null ? new double[dims] : null;

        for (final var fingerprint : fingerprints) {
            final var readings = fingerprint.getReadings();
            if (readings == null) {
                continue;
            }

            final var fingerprintPosition = fingerprint.getPosition();

            for (final var reading : readings) {
                final var readingSource = reading.getSource();
                if (readingSource != null && readingSource.equals(source)) {
                    result++;

                    if (centroid != null) {
                        for (var i = 0; i < dims; i++) {
                            final var coord = fingerprintPosition.getInhomogeneousCoordinate(i);
                            centroidCoords[i] += coord;
                        }
                    }
                }
            }
        }

        if (centroid != null && result > 0) {
            for (var i = 0; i < dims; i++) {
                centroidCoords[i] /= result;
                centroid.setInhomogeneousCoordinate(i, centroidCoords[i]);
            }
        }

        return result;
    }
}
