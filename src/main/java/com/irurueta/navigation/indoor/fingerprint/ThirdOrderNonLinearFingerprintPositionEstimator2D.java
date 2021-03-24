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
import com.irurueta.statistics.MultivariateNormalDist;
import java.util.List;

/**
 * 2D position estimator based on located fingerprints containing only RSSI readings and
 * having as well prior knowledge of the location of radio sources associated to those
 * readings.
 * This implementation uses a third-order Taylor approximation over provided located
 * fingerprints to determine an approximate position for a non-located fingerprint using
 * a non-linear solving algorithm.
 * An initial position can be provided as a starting point to solve the position,
 * otherwise the average point of selected nearest fingerprints is used as a starting
 * point.
 */
@SuppressWarnings("WeakerAccess")
public class ThirdOrderNonLinearFingerprintPositionEstimator2D extends
        NonLinearFingerprintPositionEstimator2D {

    /**
     * Constructor.
     */
    public ThirdOrderNonLinearFingerprintPositionEstimator2D() {
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public ThirdOrderNonLinearFingerprintPositionEstimator2D(
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
    public ThirdOrderNonLinearFingerprintPositionEstimator2D(
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
    public ThirdOrderNonLinearFingerprintPositionEstimator2D(
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
    public ThirdOrderNonLinearFingerprintPositionEstimator2D(
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
    public ThirdOrderNonLinearFingerprintPositionEstimator2D(
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
        return NonLinearFingerprintPositionEstimatorType.THIRD_ORDER;
    }

    /**
     * Evaluates a non-linear multi dimension function at provided point using
     * provided parameters and returns its evaluation and derivatives of the
     * function respect the function parameters.
     *
     * @param i           number of sample being evaluated.
     * @param point       point where function will be evaluated.
     * @param params      initial parameters estimation to be tried. These will
     *                    change as the Levenberg-Marquard algorithm iterates to the best solution.
     *                    These are used as input parameters along with point to evaluate function.
     * @param derivatives partial derivatives of the function respect to each
     *                    provided parameter.
     * @return function evaluation at provided point.
     */
    @Override
    @SuppressWarnings("Duplicates")
    protected double evaluate(
            final int i, final double[] point, final double[] params,
            final double[] derivatives) {
        //Demonstration in 2D:
        //--------------------
        //Taylor series expansion can be expressed as:
        //f(x) = f(a) + 1/1!*f'(a)*(x - a) + 1/2!*f''(a)*(x - a)^2 + 1/3!*f'''(a)*(x - a)^3 ...

        //where f'(x) is the derivative of f respect x, which can also be expressed as:
        //f'(x) = diff(f(x))/diff(x)

        //and f'(a) is the derivative of f respect x evaluated at a, which can be expressed
        //as f'(a) = diff(f(a))/diff(x)

        //consequently f''(a) is the second derivative respect x evaluated at a, which can
        //be expressed as:
        //f''(x) = diff(f(x))/diff(x^2)

        //and:
        //f''(a) = diff(f(a))/diff(x^2)

        //and finally f'''(a) is the third derivative respect x evaluated at a, which can
        //be expressed as:
        //f'''(x) = diff(f(x))/diff(x^3)

        //and:
        //f'''(a) = diff(f(a))/diff(x^3)

        //Received power expressed in dBm is:
        //k = (c/(4*pi*f))
        //Pr = Pte*k^n / d^n

        //where c is the speed of light, pi is 3.14159..., f is the frequency of the radio source,
        //Pte is the equivalent transmitted power by the radio source, n is the path-loss exponent
        // (typically 2.0), and d is the distance from a point to the location of the radio source.

        //Hence:
        //Pr(dBm) = 10*log(Pte*k^n/d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d) =
        //          10*n*log(k) + 10*log(Pte) - 5*n*log(d^2)

        //The former 2 terms are constant, and only the last term depends on distance

        //Hence, assuming the constant K = 10*n*log(k) + Pte(dBm), where Pte(dBm) = 10*log(Pte),
        //assuming that transmitted power by the radio source Pte is known (so that K is also known),
        //and assuming that the location of the radio source is known and it is located at pa = (xa, ya)
        //so that d^2 = (x - xa)^2 + (y - ya)^2 then the received power at an unknown point pi = (xi, yi) is:

        //Pr(pi) = Pr(xi,yi) = K - 5*n*log(d^2) = K - 5*n*log((xi - xa)^2 + (yi - ya)^2)

        //Suppose that received power at point p1=(x1,y1) is known on a located fingerprint
        //containing readings Pr(p1).

        //Then, for an unknown point pi=(xi,yi) close to fingerprint 1 located at p1 where we
        //have measured received power Pr(pi), we can get the following third-order Taylor
        //approximation:

        //Pr(pi = (xi,yi)) = Pr(p1) +
        //  diff(Pr(p1))/diff(x)*(xi - x1) +
        //  diff(Pr(p1))/diff(y)*(yi - y1) +
        //  1/2*(diff(Pr(p1))/diff(x^2)*(xi - x1)^2 +
        //      diff(Pr(p1))/diff(y^2)*(yi - y1)^2 +
        //      2*diff(Pr(p1))/diff(x*y)*(xi - x1)*(yi - y1)) +
        //  1/6*(diff(Pr(p1))/diff(x^3)*(xi - x1)^3 +
        //      diff(Pr(p1))/diff(y^3)*(yi - y1)^3 +
        //      3*diff(Pr(p1))/diff(x^2*y)*(xi - x1)^2*(yi - y1) +
        //      3*diff(Pr(p1))/diff(x*y^2)*(xi - x1)*(yi - y1)^2

        //where the first order derivatives of Pr(p = (x,y)) are:
        //diff(Pr(x,y))/diff(x) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2)*2*(x - xa)
        //diff(Pr(x,y))/diff(x) = -10*n*(x - xa)/(ln(10)*((x - xa)^2 + (y - ya)^2))

        //diff(Pr(x,y))/diff(y) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2)*2*(y - ya)
        //diff(Pr(x,y))/diff(y) = -10*n*(y - ya)/(ln(10)*((x - xa)^2 + (y - ya)^2))

        //If we evaluate first order derivatives at p1 = (x1,y1), we get:
        //diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
        //diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

        //where square distance from fingerprint 1 to radio source a can be expressed as:
        //d1a^2 = (x1 - xa)^2 + (y1 - ya)^2

        //where both the fingerprint and radio source positions are known, and hence d1a is known.

        //Then first order derivatives can be expressed as:
        //diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
        //diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*d1a^2)

        //To obtain second order derivatives we take into account that:
        //(f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2

        //hence, second order derivatives of Pr(p = (x,y)) are:
        //diff(Pr(x,y))/diff(x^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2) - (x - xa)*2*(x - xa)) / ((x - xa)^2 + (y - ya)^2)^2
        //diff(Pr(x,y))/diff(x^2) = -10*n*((y - ya)^2 - (x - xa)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2)^2)

        //diff(Pr(x,y))/diff(y^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2) - (y - ya)*2*(y - ya)) / ((x - xa)^2 + (y - ya)^2)^2
        //diff(Pr(x,y))/diff(y^2) = -10*n*((x - xa)^2 - (y - ya)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2)^2)

        //diff(Pr(x,y))/diff(x*y) = -10*n/ln(10)*(0*((x - xa)^2 + (y - ya)^2) - (x - xa)*2*(y - ya))/((x - xa)^2 + (y - ya)^2)^2
        //diff(Pr(x,y))/diff(x*y) = 20*n*((x - xa)*(y - ya))/(ln(10)*((x - xa)^2 + (y - ya)^2)^2)

        //If we evaluate second order derivatives at p1 = (x1,y1), we get:
        //diff(Pr(p1))/diff(x^2) = -10*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
        //diff(Pr(p1))/diff(y^2) = -10*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
        //diff(Pr(p1))/diff(x*y) = 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)

        //and expressing the second order derivatives in terms of distance between
        //fingerprint 1 and radio source a d1a, we get:
        //diff(Pr(p1))/diff(x^2) = -10*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)
        //diff(Pr(p1))/diff(y^2) = -10*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*d1a^4)
        //diff(Pr(p1))/diff(x*y) = 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)

        //Finally, third order derivatives of Pr(p = (x,y)) are:
        //diff(Pr(x,y))/diff(x^3) = -10*n/ln(10)*(-2*(x - xa)*((x - xa)^2 + (y - ya)^2)^2 - ((y - ya)^2 - (x - xa)^2)*2*((x - xa)^2 + (y - ya)^2)*2*(x - xa))/((x - xa)^2 + (y - ya)^2)^4
        //diff(Pr(x,y))/diff(y^3) = -10*n/ln(10)*(-2*(y - ya)*((x - xa)^2 + (y - ya)^2)^2 - ((x - xa)^2 - (y - ya)^2)*2*((x - xa)^2 + (y - ya)^2)*2*(y - ya))/((x - xa)^2 + (y - ya)^2)^4
        //diff(Pr(x,y)/diff(x^2*y) = -10*n/ln(10)*(2*(y - ya)*((x - xa)^2 + (y - ya)^2)^2 - ((y - ya)^2 - (x - xa)^2)*2*((x - xa)^2 + (y - ya)^2)*2*(y - ya))/((x - xa)^2 + (y - ya)^2)^4
        //diff(Pr(x,y)/diff(x*y^2) = -10*n/ln(10)*(2*(x - xa)*((x - xa)^2 + (y - ya)^2)^2 - ((x - xa)^2 - (y - ya)^2)*2*((x - xa)^2 + (y - ya)^2)*2*(x - xa))/((x - xa)^2 + (y - ya)^2)^4

        //evaluating at p1 = (x1, y1), we get:
        //diff(Pr(p1))/diff(x^3) = -10*n/ln(10)*(-2*(x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((y1 - ya)^2 - (x1 - xa)^2)*2*((x1 - xa)^2 + (y1 - ya)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^4
        //diff(Pr(p1))/diff(y^3) = -10*n/ln(10)*(-2*(y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*2*((x1 - xa)^2 + (y1 - ya)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^4
        //diff(Pr(p1)/diff(x^2*y) = -10*n/ln(10)*(2*(y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((y1 - ya)^2 - (x1 - xa)^2)*2*((x1 - xa)^2 + (y1 - ya)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^4
        //diff(Pr(p1)/diff(x*y^2) = -10*n/ln(10)*(2*(x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*2*((x1 - xa)^2 + (y1 - ya)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^4

        //and substituting the distance between fingerprint and radio source d1a, we get:
        //diff(Pr(p1))/diff(x^3) = -10*n/ln(10)*(-2*(x1 - xa)*dia^4 - ((y1 - ya)^2 - (x1 - xa)^2)*4*d1a^2*(x1 - xa))/d1a^8
        //diff(Pr(p1))/diff(y^3) = -10*n/ln(10)*(-2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*4*d1a^2*(y1 - ya))/d1a^8
        //diff(Pr(p1)/diff(x^2*y) = -10*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((y1 - ya)^2 - (x1 - xa)^2)*4*d1a^2*(y1 - ya))/d1a^8
        //diff(Pr(p1)/diff(x*y^2) = -10*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*4*d1a^2*(x1 - xa))/d1a^8


        //Hence, the third order Taylor expansion can be expressed as:
        //Pr(pi = (xi,yi)) = Pr(p1) +
        //  diff(Pr(p1))/diff(x)*(xi - x1) +
        //  diff(Pr(p1))/diff(y)*(yi - y1) +
        //  1/2*(diff(Pr(p1))/diff(x^2)*(xi - x1)^2 +
        //      diff(Pr(p1))/diff(y^2)*(yi - y1)^2 +
        //      2*diff(Pr(p1))/diff(x*y)*(xi - x1)*(yi - y1)) +
        //  1/6*(diff(Pr(p1))/diff(x^3)*(xi - x1)^3 +
        //      diff(Pr(p1))/diff(y^3)*(yi - y1)^3 +
        //      3*diff(Pr(p1))/diff(x^2*y)*(xi - x1)^2*(yi - y1) +
        //      3*diff(Pr(p1))/diff(x*y^2)*(xi - x1)*(yi - y1)^2)

        //Pr(pi) = Pr(p1)
        //  -10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1) +
        //  -10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1) +
        //  -5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2 +
        //  -5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*d1a^4)*(yi - y1)^2 +
        //  20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1) +
        //  -10/6*n/ln(10)*(-2*(x1 - xa)*dia^4 - ((y1 - ya)^2 - (x1 - xa)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)^3 +
        //  -10/6*n/ln(10)*(-2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*4*d1a^2*(y1 - ya))/d1a^8*(yi - y1)^3 +
        //  -5*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((y1 - ya)^2 - (x1 - xa)^2)*4*d1a^2*(y1 - ya))/d1a^8*(xi - x1)^2*(yi - y1) +
        //  -5*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)*(yi - y1)^2


        //The equation above can be solved using a non-linear fitter such as Levenberg-Marquardt

        //This method implements received power at point pi = (xi, yi) and its derivatives

        final double xi = params[0];
        final double yi = params[1];

        //received power
        final double pr = point[0];

        //fingerprint coordinates
        final double x1 = point[1];
        final double y1 = point[2];

        //radio source coordinates
        final double xa = point[3];
        final double ya = point[4];

        //path loss exponent
        final double n = point[5];

        final double ln10 = Math.log(10.0);

        final double diffXi1 = xi - x1;
        final double diffYi1 = yi - y1;

        final double diffX1a = x1 - xa;
        final double diffY1a = y1 - ya;

        final double diffXi12 = diffXi1 * diffXi1;
        final double diffYi12 = diffYi1 * diffYi1;

        final double diffXi13 = diffXi12 * diffXi1;
        final double diffYi13 = diffYi12 * diffYi1;

        final double diffX1a2 = diffX1a * diffX1a;
        final double diffY1a2 = diffY1a * diffY1a;

        final double d1a2 = diffX1a2 + diffY1a2;
        final double d1a4 = d1a2 * d1a2;
        final double d1a8 = d1a4 * d1a4;

        final double value1 = -10.0 * n * diffX1a / (ln10 * d1a2);
        final double value2 = -10.0 * n * diffY1a / (ln10 * d1a2);
        final double value3 = -5.0 * n * (-diffX1a2 + diffY1a2) / (ln10 * d1a4);
        final double value4 = -5.0 * n * (diffX1a2 - diffY1a2) / (ln10 * d1a4);
        final double value5 = 20.0 * n * diffX1a * diffY1a / (ln10 * d1a4);
        final double value6 = -10.0 / 6.0 * n / ln10 * (-2.0 * diffX1a * d1a4 - (-diffX1a2 + diffY1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
        final double value7 = -10.0 / 6.0 * n / ln10 * (-2.0 * diffY1a * d1a4 - (diffX1a2 - diffY1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
        final double value8 = -5.0 * n / ln10 * (2.0 * diffY1a * d1a4 - (-diffX1a2 + diffY1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
        final double value9 = -5.0 * n / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 - diffY1a2) * 4.0 * d1a2 * diffX1a) / d1a8;

        //hence:
        //Pr(pi) = Pr(p1) +
        //  value1*(xi - x1) +
        //  value2*(yi - y1) +
        //  value3*(xi - x1)^2 +
        //  value4*(yi - y1)^2 +
        //  value5*(xi - x1)*(yi - y1) +
        //  value6*(xi - x1)^3 +
        //  value7*(yi - y1)^3 +
        //  value8*(xi - x1)^2*(yi - y1) +
        //  value9*(xi - x1)*(yi - y1)^2 +

        final double result = pr
                + value1 * diffXi1
                + value2 * diffYi1
                + value3 * diffXi12
                + value4 * diffYi12
                + value5 * diffXi1 * diffYi1
                + value6 * diffXi13
                + value7 * diffYi13
                + value8 * diffXi12 * diffYi1
                + value9 * diffXi1 * diffYi12;

        //derivative respect xi

        //diff(Pr(pi))/diff(xi) = value1 +
        //  2*value3*(xi - x1) +
        //  value5*(yi - y1) +
        //  3*value6*(xi - x1)^2 +
        //  2*value8*(xi - x1)*(yi - y1) +
        //  value9*(yi - y1)^2

        derivatives[0] = value1 + 2.0 * value3 * diffXi1 + value5 * diffYi1 +
                3.0 * value6 * diffXi12 + 2.0 * value8 * diffXi1 * diffYi1 +
                value9 * diffYi12;

        //derivative respect yi

        //diff(Pr(pi))/diff(yi) = value2 +
        //  2*value4*(yi - y1) +
        //  value5*(xi - x1) +
        //  3*value7*(yi - y1)^2 +
        //  value8*(xi - x1)^2 +
        //  2*value9*(xi - x1)*(yi - y1)

        derivatives[1] = value2 + 2.0 * value4 * diffYi1 + value5 * diffXi1 +
                3.0 * value7 * diffYi12 + value8 * diffXi12 +
                2.0 * value9 * diffXi1 * diffYi1;

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
     *                                      initial position used by a non linear algorithm.
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
            final double fingerprintRssi, final double pathlossExponent,
            final Point2D fingerprintPosition, final Point2D radioSourcePosition,
            final Point2D estimatedPosition, final Double fingerprintRssiVariance,
            final Double pathlossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance) {
        try {
            final MultivariateNormalDist dist =
                    Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(
                            fingerprintRssi, pathlossExponent, fingerprintPosition,
                            radioSourcePosition, estimatedPosition, fingerprintRssiVariance,
                            pathlossExponentVariance, fingerprintPositionCovariance,
                            radioSourcePositionCovariance, null);
            if (dist == null) {
                return null;
            }

            final Matrix covariance = dist.getCovariance();
            if (covariance == null) {
                return null;
            }

            return covariance.getElementAt(0, 0);

        } catch (IndoorException e) {
            return null;
        }
    }

}
