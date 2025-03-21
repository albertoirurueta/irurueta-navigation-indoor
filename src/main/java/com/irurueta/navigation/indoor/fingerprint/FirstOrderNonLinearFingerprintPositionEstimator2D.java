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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.IndoorException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiFingerprintLocated;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.navigation.indoor.Utils;

import java.util.List;

/**
 * 2D position estimator based on located fingerprints containing only RSSI readings and
 * having as well prior knowledge of the location of radio sources associated to those
 * readings.
 * This implementation uses a first-order Taylor approximation over provided located
 * fingerprints to determine an approximate position for a non-located fingerprint using
 * a non-linear solving algorithm.
 * An initial position can be provided as a starting point to solve the position,
 * otherwise the average point of selected nearest fingerprints is used as a starting
 * point.
 */
public class FirstOrderNonLinearFingerprintPositionEstimator2D extends NonLinearFingerprintPositionEstimator2D {

    /**
     * Constructor.
     */
    public FirstOrderNonLinearFingerprintPositionEstimator2D() {
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public FirstOrderNonLinearFingerprintPositionEstimator2D(
            final FingerprintPositionEstimatorListener<Point2D> listener) {
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
     *                                  different locations containing a single reading are required).
     */
    public FirstOrderNonLinearFingerprintPositionEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources) {
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
     *                                  different locations containing a single reading are required).
     */
    public FirstOrderNonLinearFingerprintPositionEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources,
            final FingerprintPositionEstimatorListener<Point2D> listener) {
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
     *                                  different locations containing a single reading are required).
     */
    public FirstOrderNonLinearFingerprintPositionEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources, Point2D initialPosition) {
        super(locatedFingerprints, fingerprint, sources, initialPosition);
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
     *                                  different locations containing a single reading are required).
     */
    public FirstOrderNonLinearFingerprintPositionEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> sources, Point2D initialPosition,
            final FingerprintPositionEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, sources, initialPosition, listener);
    }

    /**
     * Gets type of position estimator.
     *
     * @return type of position estimator.
     */
    @Override
    public NonLinearFingerprintPositionEstimatorType getType() {
        return NonLinearFingerprintPositionEstimatorType.FIRST_ORDER;
    }

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
    @Override
    @SuppressWarnings("Duplicates")
    protected double evaluate(
            final int i, final double[] point, final double[] params, final double[] derivatives) {
        // This method implements received power at point pi = (xi, yi) and its derivatives

        // Pr(pi) = Pr(p1)
        //   - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
        //   - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)

        final var xi = params[0];
        final var yi = params[1];

        // received power
        final var pr = point[0];

        // fingerprint coordinates
        final var x1 = point[1];
        final var y1 = point[2];

        // radio source coordinates
        final var xa = point[3];
        final var ya = point[4];

        // path loss exponent
        final var n = point[5];

        final var ln10 = Math.log(10.0);

        final var diffXi1 = xi - x1;
        final var diffYi1 = yi - y1;

        final var diffX1a = x1 - xa;
        final var diffY1a = y1 - ya;

        final var diffX1a2 = diffX1a * diffX1a;
        final var diffY1a2 = diffY1a * diffY1a;

        final var d1a2 = diffX1a2 + diffY1a2;

        final var value1 = -10.0 * n * diffX1a / (ln10 * d1a2);
        final var value2 = -10.0 * n * diffY1a / (ln10 * d1a2);

        final var result = pr + value1 * diffXi1 + value2 * diffYi1;

        // derivative respect xi
        // diff(Pr(pi))/diff(xi) = - 10*n*(x1 - xa)/(ln(10)*d1a^2)
        derivatives[0] = value1;

        // derivative respect yi
        // diff(Pr(pi))/diff(yi) = - 10*n*(y1 - ya)/(ln(10)*d1a^2)
        derivatives[1] = value2;

        return result;
    }

    /**
     * Propagates provided variances into RSSI variance of non-located fingerprint
     * reading.
     *
     * @param fingerprintRssi               closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathlossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
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
    @Override
    @SuppressWarnings("Duplicates")
    protected Double propagateVariances(
            final double fingerprintRssi, final double pathlossExponent, final Point2D fingerprintPosition,
            final Point2D radioSourcePosition, final Point2D estimatedPosition, final Double fingerprintRssiVariance,
            final Double pathlossExponentVariance, final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance) {
        try {
            final var dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(fingerprintRssi,
                    pathlossExponent, fingerprintPosition, radioSourcePosition, estimatedPosition,
                    fingerprintRssiVariance, pathlossExponentVariance, fingerprintPositionCovariance,
                    radioSourcePositionCovariance, null);
            if (dist == null) {
                return null;
            }

            final var covariance = dist.getCovariance();
            if (covariance == null) {
                return null;
            }

            return covariance.getElementAt(0, 0);

        } catch (final IndoorException e) {
            return null;
        }
    }
}
