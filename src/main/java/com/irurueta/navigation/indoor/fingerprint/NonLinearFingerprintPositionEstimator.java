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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceKNearestFinder;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RadioSourceNoMeanKNearestFinder;
import com.irurueta.navigation.indoor.RadioSourceWithPower;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiFingerprintLocated;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Base class for position estimators based on located fingerprints containing only
 * RSSI readings and having as well prior knowledge of the location of radio sources
 * associated to those readings.
 * This implementation uses a Taylor approximation over provided located
 * fingerprints to determine an approximate position for a non-located fingerprint using
 * a non-linear solving algorithm.
 * An initial position can be provided as a starting point to solve the position,
 * otherwise the average point of selected nearest fingerprints is used as a starting
 * point.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class NonLinearFingerprintPositionEstimator<P extends Point<?>> extends
        FingerprintPositionEstimator<P> {

    /**
     * Default RSSI standard deviation assumed for provided fingerprints as a fallback
     * when none can be determined.
     */
    public static final double FALLBACK_RSSI_STANDARD_DEVIATION = 1.0;

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
     * Default type to be used when none is provided.
     */
    public static final NonLinearFingerprintPositionEstimatorType DEFAULT_TYPE =
            NonLinearFingerprintPositionEstimatorType.THIRD_ORDER;

    /**
     * Small value to be used as the minimum allowed RSSI standard deviations. A value
     * larger than this must be provided to allow convergence to a solution
     */
    public static final double TINY_RSSI_STD = 1e-12;

    /**
     * Initial position to start the solving algorithm.
     * This should be a value close to the expected solution.
     * If no value is provided, the average position among all selected nearest
     * located fingerprints will be used.
     */
    private P mInitialPosition;

    /**
     * RSSI standard deviation fallback value to use when none can be
     * determined from provided readings. This fallback value is only used if
     * no variance is propagated or the resulting value is too small to allow
     * convergence to a solution.
     */
    private double mFallbackRssiStandardDeviation =
            FALLBACK_RSSI_STANDARD_DEVIATION;

    /**
     * Indicates whether measured RSSI standard deviation of closest fingerprint must
     * be propagated into measured RSSI reading variance at unknown location.
     */
    private boolean mPropagateFingerprintRssiStandardDeviation =
            DEFAULT_PROPAGATE_FINGERPRINT_RSSI_STANDARD_DEVIATION;

    /**
     * Indicates whether path-loss exponent standard deviation of radio source must
     * be propagated into measured RSSI reading variance at unknown location.
     */
    private boolean mPropagatePathlossExponentStandardDeviation =
            DEFAULT_PROPAGATE_PATHLOSS_EXPONENT_STANDARD_DEVIATION;

    /**
     * Indicates whether covariance of closest fingerprint position must be
     * propagated into measured RSSI reading variance at unknown location.
     */
    private boolean mPropagateFingerprintPositionCovariance =
            DEFAULT_PROPAGATE_FINGERPRINT_POSITION_COVARIANCE;

    /**
     * Indicates whether covariance of radio source position must be propagated
     * into measured RSSI reading variance at unknown location.
     */
    private boolean mPropagateRadioSourcePositionCovariance =
            DEFAULT_PROPAGATE_RADIO_SOURCE_POSITION_COVARIANCE;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter mFitter = new LevenbergMarquardtMultiDimensionFitter();

    /**
     * Estimated covariance matrix for estimated position.
     */
    private Matrix mCovariance;

    /**
     * Estimated chi square value.
     */
    private double mChiSq;

    /**
     * Constructor.
     */
    protected NonLinearFingerprintPositionEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    protected NonLinearFingerprintPositionEstimator(
            final FingerprintPositionEstimatorListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required. For 3D position
     *                                  estimation 3 located total readings are required among all fingerprints).
     */
    protected NonLinearFingerprintPositionEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<P>> sources) {
        super(locatedFingerprints, fingerprint, sources);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param listener            listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required. For 3D position
     *                                  estimation 3 located total readings are required among all fingerprints).
     */
    protected NonLinearFingerprintPositionEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<P>> sources,
            final FingerprintPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, sources, listener);
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param initialPosition     initial position to start the solving algorithm or null.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required. For 3D position
     *                                  estimation 3 located total readings are required among all fingerprints).
     */
    protected NonLinearFingerprintPositionEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<P>> sources, P initialPosition) {
        super(locatedFingerprints, fingerprint, sources);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint         fingerprint containing readings at an unknown location
     *                            for provided located fingerprints.
     * @param sources             located radio sources.
     * @param initialPosition     initial position to start the solving algorithm or null.
     * @param listener            listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     *                                  located fingerprints value is null or there are not enough fingerprints or
     *                                  readings within provided fingerprints (for 2D position estimation at least 2
     *                                  located total readings are required among all fingerprints, for example 2
     *                                  readings are required in a single fingerprint, or at least 2 fingerprints at
     *                                  different locations containing a single reading are required. For 3D position
     *                                  estimation 3 located total readings are required among all fingerprints).
     */
    protected NonLinearFingerprintPositionEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<P>> sources, P initialPosition,
            final FingerprintPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, sources, listener);
        mInitialPosition = initialPosition;
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
        return mInitialPosition;
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

        mInitialPosition = initialPosition;
    }

    /**
     * Gets RSSI standard deviation fallback value to use when none can be
     * determined from provided readings.
     *
     * @return RSSI standard deviation fallback.
     */
    public double getFallbackRssiStandardDeviation() {
        return mFallbackRssiStandardDeviation;
    }

    /**
     * Sets RSSI standard deviation fallback value to use when none can be
     * determined from provided readings.
     *
     * @param fallbackRssiStandardDeviation RSSI standard deviation fallback
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is smaller than
     *                                  {@link #TINY_RSSI_STD}.
     */
    public void setFallbackRssiStandardDeviation(
            final double fallbackRssiStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (fallbackRssiStandardDeviation < TINY_RSSI_STD) {
            throw new IllegalArgumentException();
        }
        mFallbackRssiStandardDeviation = fallbackRssiStandardDeviation;
    }

    /**
     * Indicates whether measured RSSI standard deviation of closest fingerprint must be
     * propagated into measured RSSI reading variance at unknown location.
     *
     * @return true to propagate RSSI standard deviation of closest fingerprint,
     * false otherwise.
     */
    public boolean isFingerprintRssiStandardDeviationPropagated() {
        return mPropagateFingerprintRssiStandardDeviation;
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
    public void setFingerprintRssiStandardDeviationPropagated(
            final boolean propagateFingerprintRssiStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPropagateFingerprintRssiStandardDeviation =
                propagateFingerprintRssiStandardDeviation;
    }

    /**
     * Indicates whether path-loss exponent standard deviation of radio source must be
     * propagated into measured RSSI reading variance at unknown location.
     *
     * @return true to propagate  path-loss exponent standard deviation of radio source,
     * false otherwise.
     */
    public boolean isPathlossExponentStandardDeviationPropagated() {
        return mPropagatePathlossExponentStandardDeviation;
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
        mPropagatePathlossExponentStandardDeviation =
                propagatePathlossExponentStandardDeviation;
    }

    /**
     * Indicates whether covariance of closest fingerprint position must be propagated
     * into measured RSSI reading variance at unknown location.
     *
     * @return true to propagate fingerprint position covariance, false otherwise.
     */
    public boolean isFingerprintPositionCovariancePropagated() {
        return mPropagateFingerprintPositionCovariance;
    }

    /**
     * Specifies whether covariance of closest fingerprint position must be propagated
     * into measured RSSI reading variance at unknown location.
     *
     * @param propagateFingerprintPositionCovariance true to propagate fingerprint
     *                                               position covariance, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprintPositionCovariancePropagated(
            final boolean propagateFingerprintPositionCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPropagateFingerprintPositionCovariance =
                propagateFingerprintPositionCovariance;
    }

    /**
     * Indicates whether covariance of radio source position must be propagated into
     * measured RSSI reading variance at unknown location.
     *
     * @return true to propagate radio source position covariance, false otherwise.
     */
    public boolean isRadioSourcePositionCovariancePropagated() {
        return mPropagateRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether covariance of radio source position must be propagated into
     * measured RSSI reading variance at unknown location.
     *
     * @param propagateRadioSourcePositionCovariance true to propagate radio source
     *                                               position covariance, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRadioSourcePositionCovariancePropagated(
            final boolean propagateRadioSourcePositionCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPropagateRadioSourcePositionCovariance =
                propagateRadioSourcePositionCovariance;
    }

    /**
     * Gets estimated covariance matrix for estimated position.
     *
     * @return estimated covariance matrix for estimated position.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Gets estimated chi square value.
     *
     * @return estimated chi square value.
     */
    public double getChiSq() {
        return mChiSq;
    }

    /**
     * Estimates position based on provided located radio sources and readings of such radio sources at
     * an unknown location.
     *
     * @throws LockedException                if estimator is locked.
     * @throws NotReadyException              if estimator is not ready.
     * @throws FingerprintEstimationException if estimation fails for some other reason.
     */
    @Override
    @SuppressWarnings("Duplicates")
    public void estimate() throws LockedException, NotReadyException,
            FingerprintEstimationException {

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
            mCovariance = null;
            nearestFingerprints = null;

            final int max = maxNearestFingerprints < 0 ?
                    locatedFingerprints.size() :
                    Math.min(maxNearestFingerprints, locatedFingerprints.size());
            for (int k = minNearestFingerprints; k <= max; k++) {
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
                // Taylor series expansion can be expressed as:
                // f(x) = f(a) + 1/1!*f'(a)*(x - a) + 1/2!*f''(a)*(x - a)^2 + ...

                // where f'(x) is the derivative of f respect x, which can also be expressed as:
                // f'(x) = diff(f(x))/diff(x)

                // and f'(a) is the derivative of f respect x evaluated at "a", which can be expressed
                // as f'(a) = diff(f(a))/diff(x)

                // consequently f''(a) is the second derivative respect x evaluated at "a", which can
                // be expressed as:
                // f''(x) = diff(f(x))/diff(x^2)

                // and:
                // f''(a) = diff(f(a))/diff(x^2)

                // Received power expressed in dBm is:
                // k = (c/(4*pi*f))
                // Pr = Pte*k^n / d^n

                // where c is the speed of light, pi is 3.14159..., f is the frequency of the radio source,
                // Pte is the equivalent transmitted power by the radio source, n is the path-loss exponent
                // (typically 2.0), and d is the distance from a point to the location of the radio source.

                // Hence:
                // Pr(dBm) = 10*log(Pte*k^n/d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d) =
                //           10*n*log(k) + 10*log(Pte) - 5*n*log(d^2)

                // The former 2 terms are constant, and only the last term depends on distance

                // Hence, assuming the constant K = 10*n*log(k) + Pte(dBm), where Pte(dBm) = 10*log(Pte),
                // assuming that transmitted power by the radio source Pte is known (so that K is also known),
                // and assuming that the location of the radio source is known, and it is located at pa = (xa, ya)
                // so that d^2 = (x - xa)^2 + (y - ya)^2 then the received power at an unknown point pi = (xi, yi) is:

                // Pr(pi) = Pr(xi,yi) = K - 5*n*log(d^2) = K - 5*n*log((xi - xa)^2 + (yi - ya)^2)

                // Suppose that received power at point p1=(x1,y1) is known on a located fingerprint
                // containing readings Pr(p1).

                // Then, for an unknown point pi=(xi,yi) close to fingerprint 1 located at p1 where we
                // have measured received power Pr(pi), we can get the following second-order Taylor
                // approximation:

                // Pr(pi) ~ Pr(p1) + JPr(p1)*(pi - p1) + 1/2*(pi - p1)^T*HPr(p1)*(pi - p1) + ...

                // where JPr(p1) is the Jacobian of Pr evaluated at p1. Since Pr is a multivariate function
                // with scalar result, the Jacobian has size 1x2 and is equal to the gradient.
                // HPr(p1) is the Hessian matrix evaluated at p1, which is a symmetric matrix of size 2x2,
                // and (pi-p1)^T is the transposed vector of (pi-p1)

                // Hence, the Jacobian at any point p=(x,y) is equal to:
                // JPr(p = (x,y)) = [diff(Pr(x,y))/diff(x)   diff(Pr(x,y))/diff(y)]

                // And the Hessian matrix is equal to
                // HPr(p = (x,y)) =  [diff(Pr(x,y))/diff(x^2)    diff(Pr(x,y))/diff(x*y)]
                //                   [diff(Pr(x,y))/diff(x*y)    diff(Pr(x,y))/diff(y^2)]

                // where the first order derivatives of Pr(p = (x,y)) are:
                // diff(Pr(x,y))/diff(x) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2)*2*(x - xa)
                // diff(Pr(x,y))/diff(x) = -10*n*(x - xa)/(ln(10)*((x - xa)^2 + (y - ya)^2))

                // diff(Pr(x,y))/diff(y) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2)*2*(y - ya)
                // diff(Pr(x,y))/diff(y) = -10*n*(y - ya)/(ln(10)*((x - xa)^2 + (y - ya)^2))

                // If we evaluate first order derivatives at p1 = (x1,y1), we get:
                // diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
                // diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                // where square distance from fingerprint 1 to radio source a can be expressed as:
                // d1a^2 = (x1 - xa)^2 + (y1 - ya)^2

                // where both the fingerprint and radio source positions are known, and hence d1a is known.

                // Then first order derivatives can be expressed as:
                // diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
                // diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*d1a^2)

                // To obtain second order derivatives we take into account that:
                // (f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2

                // hence, second order derivatives of Pr(p = (x,y)) are:
                // diff(Pr(x,y))/diff(x^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2) - (x - xa)*2*(x - xa)) / ((x - xa)^2 + (y - ya)^2)^2
                // diff(Pr(x,y))/diff(x^2) = -10*n*((y - ya)^2 - (x - xa)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2)^2)

                // diff(Pr(x,y))/diff(y^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2) - (y - ya)*2*(y - ya)) / ((x - xa)^2 + (y - ya)^2)^2
                // diff(Pr(x,y))/diff(y^2) = -10*n*((x - xa)^2 - (y - ya)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2)^2)

                // diff(Pr(x,y))/diff(x*y) = -10*n/ln(10)*(0*((x - xa)^2 + (y - ya)^2) - (x - xa)*2*(y - ya))/((x - xa)^2 + (y - ya)^2)^2
                // diff(Pr(x,y))/diff(x*y) = 20*n*((x - xa)*(y - ya))/(ln(10)*((x - xa)^2 + (y - ya)^2)^2)

                // If we evaluate second order derivatives at p1 = (x1,y1), we get:
                // diff(Pr(p1))/diff(x^2) = -10*n*((y1 - ya)^2 - (x1 - xa)^2))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
                // diff(Pr(p1))/diff(y^2) = -10*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
                // diff(Pr(p1))/diff(x*y) = 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)

                // and expressing the second order derivatives in terms of distance between
                // fingerprint 1 and radio source a d1a, we get:
                // diff(Pr(p1))/diff(x^2) = -10*n*((y1 - ya)^2 - (x1 - xa)^2))/(ln(10)*d1a^4)
                // diff(Pr(p1))/diff(y^2) = -10*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*d1a^4)
                // diff(Pr(p1))/diff(x*y) = 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)

                // Hence, second order Taylor expansion can be expressed as:
                // Pr(pi) = Pr(p1) + diff(Pr(p1))/diff(x)*(xi - x1) + diff(Pr(p1))/diff(y)*(yi - y1) +
                // 1/2*diff(Pr(p1))/diff(x^2)*(xi - x1)^2 + 1/2*diff(Pr(p1))/diff(y^2)*(yi - y1)^2 +
                // diff(Pr(p1))/diff(x*y)*(xi - x1)*(yi - y1)

                // Pr(pi) = Pr(p1) - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1) -10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                // - 5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2
                // - 5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*d1a^4)*(yi - y1)^2 +
                // 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4))*(xi - x1)*(yi - y1)

                // The equation above can be solved using a non-linear fitter such as Levenberg-Marquardt


                // Demonstration in 3D:
                // --------------------
                // Taylor series expansion can be expressed as:
                // f(x) = f(a) + 1/1!*f'(a)*(x - a) + 1/2!*f''(a)*(x - a)^2 + ...

                // where f'(x) is the derivative of f respect x, which can also be expressed as:
                // f'(x) = diff(f(x))/diff(x)

                // and f'(a) is the derivative of f respect x evaluated at "a", which can be expressed
                // as f'(a) = diff(f(a))/diff(x)

                // consequently f''(a) is the second derivative respect x evaluated at "a", which can
                // be expressed as:
                // f''(x) = diff(f(x))/diff(x^2)

                // and:
                // f''(a) = diff(f(a))/diff(x^2)

                // Received power expressed in dBm is:
                // k = (c/(4*pi*f))
                // Pr = Pte*k^n / d^n

                // where c is the speed of light, pi is 3.14159..., f is the frequency of the radio source,
                // Pte is the equivalent transmitted power by the radio source, n is the path-loss exponent
                // (typically 2.0), and d is the distance from a point to the location of the radio source.

                // Hence:
                // Pr(dBm) = 10*log(Pte*k^n/d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d) =
                //          10*n*log(k) + 10*log(Pte) - 5*n*log(d^2)

                // The former 2 terms are constant, and only the last term depends on distance

                // Hence, assuming the constant K = 10*n*log(k) + Pte(dBm), where Pte(dBm) = 10*log(Pte),
                // assuming that transmitted power by the radio source Pte is known (so that K is also known),
                // and assuming that the location of the radio source is known, and it is located at pa = (xa, ya, za)
                // so that d^2 = (x - xa)^2 + (y - ya)^2 + (z - za)^2 then the received power at an unknown point
                // pi = (xi, yi, zi) is:

                // Pr(pi) = Pr(xi,yi,zi) = K - 5*n*log(d^2) = K - 5*n*log((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)

                // Suppose that received power at point p1=(x1,y1,z1) is known on a located fingerprint
                // containing readings Pr(p1).

                // Then, for an unknown point pi=(xi,yi,zi) close to fingerprint 1 located at p1 where we
                // have measured received power Pr(pi), we can get the following second-order Taylor
                // approximation:

                // Pr(pi) ~ Pr(p1) + JPtr(p1)*(pi - p1) + 1/2*(pi - p1)^T*HPr(p1)*(pi - p1) + ...

                // where JPr(p1) is the Jacobian of Pr evaluated at p1. Since Pr is a multivariate function
                // with scalar result, the Jacobian has size 1x3 and is equal to the gradient.
                // HPtr(p1) is the Hessian matrix evaluated at p1, which is a symmetric matrix of size 3x3,
                // and (pi-p1)^T is the transposed vector of (pi-p1)

                // Hence, the Jacobian at any point p=(x,y,z) is equal to:
                // JPr(p = (x,y,z)) = [diff(Pr(x,y,z))/diff(x)     diff(Pr(x,y,z))/diff(y)     diff(Pr(x,y,z))/diff(z)]

                // And the Hessian matrix is equal to
                // HPr(p = (x,y,z)) = [diff(Pr(x,y,z))/diff(x^2)    diff(Pr(x,y,z))/diff(x*y)     diff(Pr(x,y,z))/diff(x*z)]
                //                    [diff(Pr(x,y,z))/diff(x*y)    diff(Pr(x,y,z))/diff(y^2)     diff(Pr(x,y,z))/diff(y*z)]
                //                    [diff(Pr(x,y,z))/diff(x*z)    diff(Pr(x,y,z))/diff(y*z)     diff(Pr(x,y,z))/diff(z^2)]

                // where the first order derivatives of Pr(p = (x,y)) are:
                // diff(Pr(x,y,z))/diff(x) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(x - xa)
                // diff(Pr(x,y,z))/diff(x) = -10*n*(x - xa)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

                // diff(Pr(x,y,z))/diff(y) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(y - ya)
                // diff(Pr(x,y,z))/diff(y) = -10*n*(y - ya)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

                // diff(Pr(x,y,z))/diff(z) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(z - za)
                // diff(Pr(x,y,z))/diff(z) = -10*n*(z - za)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

                // If we evaluate derivatives at p1 = (x1,y1,z1), we get:
                // diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                // diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                // diff(Pr(p1))/diff(z) = -10*n*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                // where square distance from fingerprint 1 to radio source a can be expressed as:
                // d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

                // where both the fingerprint and radio source positions are known, and hence d1a is known.

                // Then first order derivatives can be expressed as:
                // diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
                // diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*d1a^2)
                // diff(Pr(p1))/diff(z) = -10*n*(z1 - za)/(ln(10)*d1a^2)

                // To obtain second order derivatives we take into account that:
                // (f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2

                // hence, second order derivatives of Pr(p = (x,y,z)) are:
                // diff(Pr(x,y,z))/diff(x^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (x - xa)*2*(x - xa))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
                // diff(Pr(x,y,z))/diff(x^2) = -10*n*((y - ya)^2 + (z - za)^2 - (x - xa)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

                // diff(Pr(x,y,z))/diff(y^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (y - ya)*2*(y - ya))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
                // diff(Pr(x,y,z))/diff(y^2) = -10*n*((x - xa)^2 - (y - ya)^2 + (z - za)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

                // diff(Pr(x,y,z))/diff(z^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (z - za)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
                // diff(Pr(x,y,z))/diff(z^2) = -10*n*((x - xa)^2 + (y - ya)^2 - (z - za)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

                // diff(Pr(x,y,z))/diff(x*y) = -10*n/ln(10)*(0*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (x - xa)*2*(y - ya))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
                // diff(Pr(x,y,z))/diff(x*y) = 20*n*(x - xa)*(y - ya)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

                // diff(Pr(x,y,z))/diff(x*z) = -10*n/ln(10)*(0*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (x - xa)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
                // diff(Pr(x,y,z))/diff(x*z) = 20*n*(x - xa)*(z - za)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

                // diff(Pr(x,y,z))/diff(y*z) = -10*n/ln(10)*(0*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (y - ya)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
                // diff(Pr(x,y,z))/diff(y*z) = 20*n*(y - ya)*(z - za)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

                // If we evaluate second order derivatives at p1 = (x1,y1,z1), we get:
                // diff(Pr(p1))/diff(x^2) = -10*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
                // diff(Pr(p1))/diff(y^2) = -10*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
                // diff(Pr(p1))/diff(z^2) = -10*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
                // diff(Pr(p1))/diff(x*y) = 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
                // diff(Pr(p1))/diff(x*z) = 20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
                // diff(Pr(p1))/diff(y*z) = 20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)

                // and expressing the second order derivatives in terms of distance between
                // fingerprint 1 and radio source a d1a, we get:
                // diff(Pr(p1))/diff(x^2) = -10*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)
                // diff(Pr(p1))/diff(y^2) = -10*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*d1a^4)
                // diff(Pr(p1))/diff(z^2) = -10*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*d1a^4)
                // diff(Pr(p1))/diff(x*y) = 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)
                // diff(Pr(p1))/diff(x*z) = 20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)
                // diff(Pr(p1))/diff(y*z) = 20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)

                // Hence, second order Taylor expansion can be expressed as:
                // Pr(pi) = Pr(p1) + diff(Pr(p1))/diff(x)*(x - x1) +
                //       diff(Pr(p1))/diff(y)*(y - y1) +
                //       diff(Pr(p1))/diff(z)*(z - z1) +
                //       1/2*diff(Pr(p1))/diff(x^2)*(x - x1)^2 +
                //       1/2*diff(Pr(p1))/diff(y^2)*(y - y1)^2 +
                //	     1/2*diff(Pr(p1))/diff(z^2)*(z - z1)^2 +
                //	     diff(Pr(p1))/diff(x*y)*(x - x1)*(y - y1) +
                //	     diff(Pr(p1))/diff(y*z)*(y - y1)*(z - z1) +
                //	     diff(Pr(p1))/diff(x*z)*(x - x1)*(z - z1)

                // Pr(pi) = Pr(p1) - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi -x1)
                //       - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                //       - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)
                //       - 5*n*((y1 - ya)^2 + (z1 - za)^2) - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2
                //       - 5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2))/(ln(10)*d1a^4)*(yi - y1)^2
                //       - 5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2))/(ln(10)*d1a^4)*(zi - z1)^2
                //       + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1)
                //       + 20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)*(yi - y1)*(zi - z1)
                //       + 20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)*(xi - x1)*(zi - z1)

                // The equation above can be solved using a non-linear fitter such as Levenberg-Marquardt
                try {
                    setupFitter();

                    mFitter.fit();

                    // estimated position
                    estimatedPositionCoordinates = mFitter.getA();
                    mCovariance = mFitter.getCovar();
                    mChiSq = mFitter.getChisq();

                    // a solution was found so we exit loop
                    break;
                } catch (NumericalException e) {
                    // solution could not be found with current data
                    // Iterate to use additional nearby fingerprints
                    estimatedPositionCoordinates = null;
                    mCovariance = null;
                    nearestFingerprints = null;
                }
            }

            if (estimatedPositionCoordinates == null) {
                // no solution could be found
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
     * Gets type of position estimator.
     *
     * @return type of position estimator.
     */
    public abstract NonLinearFingerprintPositionEstimatorType getType();

    /**
     * Evaluates a non-linear multi dimension function at provided point using
     * provided parameters and returns its evaluation and derivatives of the
     * function respect the function parameters.
     *
     * @param i           number of sample being evaluated.
     * @param point       point where function will be evaluated.
     * @param params      initial parameters estimation to be tried. These will
     *                    change as the Levenberg-Marquardt algorithm iterates to the best solution.
     *                    These are used as input parameters along with point to evaluate function.
     * @param derivatives partial derivatives of the function respect to each
     *                    provided parameter.
     * @return function evaluation at provided point.
     */
    protected abstract double evaluate(
            final int i, final double[] point, final double[] params, final double[] derivatives);

    /**
     * Propagates provided variances into RSSI variance of non-located fingerprint
     * reading.
     *
     * @param fingerprintRssi               closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathlossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest located fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non-linear algorithm.
     * @param fingerprintRssiVariance       variance of fingerprint RSSI or null if unknown.
     * @param pathlossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null if
     *                                      unknown.
     * @return variance of RSSI measured at non located fingerprint reading.
     */
    protected abstract Double propagateVariances(
            final double fingerprintRssi, final double pathlossExponent, final P fingerprintPosition,
            final P radioSourcePosition, final P estimatedPosition, final Double fingerprintRssiVariance,
            final Double pathlossExponentVariance, final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance);

    /**
     * Builds data required to solve the problem.
     *
     * @param allReceivedPower        list of received powers for readings at unknown positions.
     * @param allFingerprintPower     list of power readings at fingerprint positions.
     * @param allFingerprintPositions list of fingerprint positions.
     * @param allSourcesPositions     list of radio sources positions.
     * @param allPathLossExponents    list of path loss exponents.
     * @param allStandardDeviations   list of standard deviations for readings being used.
     */
    @SuppressWarnings("Duplicates")
    private void buildData(
            final List<Double> allReceivedPower,
            final List<Double> allFingerprintPower,
            final List<P> allFingerprintPositions,
            final List<P> allSourcesPositions,
            final List<Double> allPathLossExponents,
            final List<Double> allStandardDeviations) {
        for (final var locatedFingerprint : nearestFingerprints) {

            final var locatedReadings = locatedFingerprint.getReadings();
            if (locatedReadings == null) {
                continue;
            }

            final var fingerprintPosition = locatedFingerprint.getPosition();
            final var fingerprintPositionCovariance = locatedFingerprint.getPositionCovariance();

            var locatedMeanRssi = 0.0;
            var meanRssi = 0.0;
            if (removeMeansFromFingerprintReadings) {
                locatedMeanRssi = locatedFingerprint.getMeanRssi();
            }

            for (final var locatedReading : locatedReadings) {
                final var source = locatedReading.getSource();

                // find within the list of located sources the source of
                // current located fingerprint reading.
                // Radio sources are compared by their id
                // regardless of them being located or not

                //noinspection SuspiciousMethodCalls
                final var pos = sources.indexOf(source);
                if (pos < 0) {
                    continue;
                }

                final var locatedSource = sources.get(pos);
                var pathLossExponent = this.pathLossExponent;
                Double pathLossExponentVariance = null;
                if (useSourcesPathLossExponentWhenAvailable
                        && locatedSource instanceof RadioSourceWithPower locatedSourceWithPower) {
                    pathLossExponent = locatedSourceWithPower.getPathLossExponent();
                    final var std = locatedSourceWithPower.getPathLossExponentStandardDeviation();
                    pathLossExponentVariance = std != null ? std * std : null;
                }

                final var sourcePosition = locatedSource.getPosition();
                final var sourcePositionCovariance = locatedSource.getPositionCovariance();
                var locatedRssi = locatedReading.getRssi();
                locatedRssi -= locatedMeanRssi;

                final var locatedRssiStd = locatedReading.getRssiStandardDeviation();
                final var locatedRssiVariance = locatedRssiStd != null ? locatedRssiStd * locatedRssiStd : null;
                if (removeMeansFromFingerprintReadings) {
                    meanRssi = fingerprint.getMeanRssi();
                }

                final var readings = fingerprint.getReadings();
                for (final var reading : readings) {
                    if (reading.getSource() == null || !reading.getSource().equals(locatedSource)) {
                        continue;
                    }

                    // only take into account reading for matching sources on located and
                    // non-located readings
                    var rssi = reading.getRssi();
                    rssi -= meanRssi;

                    Double standardDeviation = null;
                    if (mPropagateFingerprintRssiStandardDeviation || mPropagatePathlossExponentStandardDeviation
                            || mPropagateFingerprintPositionCovariance || mPropagateRadioSourcePositionCovariance) {

                        // compute initial position
                        final var initialPosition = mInitialPosition != null ? mInitialPosition : fingerprintPosition;

                        final var variance = propagateVariances(locatedRssi, pathLossExponent, fingerprintPosition,
                                sourcePosition, initialPosition,
                                mPropagateFingerprintRssiStandardDeviation ? locatedRssiVariance : null,
                                mPropagatePathlossExponentStandardDeviation ? pathLossExponentVariance : null,
                                mPropagateFingerprintPositionCovariance ? fingerprintPositionCovariance : null,
                                mPropagateRadioSourcePositionCovariance ? sourcePositionCovariance : null);
                        if (variance != null) {
                            standardDeviation = Math.sqrt(variance);
                        }
                    }

                    if (standardDeviation == null) {
                        standardDeviation = reading.getRssiStandardDeviation();
                    } else if (reading.getRssiStandardDeviation() != null) {
                        // consider propagated variance and reading variance independent, so we
                        // sum them both
                        standardDeviation = standardDeviation * standardDeviation +
                                reading.getRssiStandardDeviation() * reading.getRssiStandardDeviation();
                        standardDeviation = Math.sqrt(standardDeviation);
                    }

                    if (standardDeviation == null || standardDeviation < TINY_RSSI_STD) {
                        standardDeviation = mFallbackRssiStandardDeviation;
                    }

                    allReceivedPower.add(rssi);
                    allFingerprintPower.add(locatedRssi);
                    allFingerprintPositions.add(fingerprintPosition);
                    allSourcesPositions.add(sourcePosition);
                    allPathLossExponents.add(pathLossExponent);
                    allStandardDeviations.add(standardDeviation);
                }
            }
        }
    }

    /**
     * Setups fitter to solve position.
     *
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    @SuppressWarnings("Duplicates")
    private void setupFitter() throws FittingException {
        // build lists of data
        final var allReceivedPower = new ArrayList<Double>();
        final var allFingerprintPower = new ArrayList<Double>();
        final var allFingerprintPositions = new ArrayList<P>();
        final var allSourcesPosition = new ArrayList<P>();
        final var allPathLossExponents = new ArrayList<Double>();
        final var allStandardDeviations = new ArrayList<Double>();
        buildData(allReceivedPower, allFingerprintPower, allFingerprintPositions, allSourcesPosition,
                allPathLossExponents, allStandardDeviations);

        final var totalReadings = allReceivedPower.size();
        final var dims = getNumberOfDimensions();
        final var n = 2 + 2 * dims;

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return n;
            }

            @Override
            public double[] createInitialParametersArray() {

                final var initial = new double[dims];

                if (mInitialPosition == null) {
                    // use centroid of nearest fingerprints as initial value
                    var num = 0;
                    for (var fingerprint : nearestFingerprints) {
                        final var position = fingerprint.getPosition();
                        if (position == null) {
                            continue;
                        }

                        for (var i = 0; i < dims; i++) {
                            initial[i] += position.getInhomogeneousCoordinate(i);
                        }
                        num++;
                    }

                    if (num > 0) {
                        for (var i = 0; i < dims; i++) {
                            initial[i] /= num;
                        }
                    }
                } else {
                    // use provided initial position
                    for (var i = 0; i < dims; i++) {
                        initial[i] = mInitialPosition.getInhomogeneousCoordinate(i);
                    }
                }
                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params, final double[] derivatives) {
                return NonLinearFingerprintPositionEstimator.this.evaluate(i, point, params, derivatives);
            }
        });

        try {
            final var x = new Matrix(totalReadings, n);
            final var y = new double[totalReadings];
            final var standardDeviations = new double[totalReadings];
            for (var i = 0; i < totalReadings; i++) {
                // fingerprint power Pr(p1)
                x.setElementAt(i, 0, allFingerprintPower.get(i));
                for (var j = 0; j < dims; j++) {
                    x.setElementAt(i, j + 1, allFingerprintPositions.get(i).getInhomogeneousCoordinate(j));
                    x.setElementAt(i, j + 1 + dims, allSourcesPosition.get(i).getInhomogeneousCoordinate(j));
                }
                x.setElementAt(i, 1 + 2 * dims, allPathLossExponents.get(i));

                y[i] = allReceivedPower.get(i);

                standardDeviations[i] = allStandardDeviations.get(i);
            }

            mFitter.setInputData(x, y, standardDeviations);
        } catch (final AlgebraException e) {
            throw new FittingException(e);
        }
    }
}
