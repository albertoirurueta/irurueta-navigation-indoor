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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint2D;
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
 * Position and radio source estimator in 2D based only on located fingerprints
 * containing RSSI readings.
 * This class estimates 2D position of a new fingerprint and the 2D position of all
 * radio sources associated to fingerprints whose location is known.
 */
public class NonLinearFingerprintPositionAndRadioSourceEstimator2D extends
        NonLinearFingerprintPositionAndRadioSourceEstimator<Point2D> {

    /**
     * Constructor.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D() {
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final FingerprintPositionAndRadioSourceEstimatorListener<Point2D> listener) {
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
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
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
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final FingerprintPositionAndRadioSourceEstimatorListener<Point2D> listener) {
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
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final Point2D initialPosition) {
        super(locatedFingerprints, fingerprint, initialPosition);
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
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final Point2D initialPosition,
            final FingerprintPositionAndRadioSourceEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, initialPosition, listener);
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
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> initialLocatedSources) {
        super(locatedFingerprints, fingerprint, initialLocatedSources);
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
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final List<? extends RadioSourceLocated<Point2D>> initialLocatedSources,
            final FingerprintPositionAndRadioSourceEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, initialLocatedSources, listener);
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
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final Point2D initialPosition,
            final List<? extends RadioSourceLocated<Point2D>> initialLocatedSources) {
        super(locatedFingerprints, fingerprint, initialPosition, initialLocatedSources);
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
    public NonLinearFingerprintPositionAndRadioSourceEstimator2D(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, Point2D>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final Point2D initialPosition,
            final List<? extends RadioSourceLocated<Point2D>> initialLocatedSources,
            final FingerprintPositionAndRadioSourceEstimatorListener<Point2D> listener) {
        super(locatedFingerprints, fingerprint, initialPosition, initialLocatedSources, listener);
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
    @Override
    @SuppressWarnings("Duplicates")
    protected Double propagateVariances(
            final double pathlossExponent, final Point2D fingerprintPosition, final Point2D radioSourcePosition,
            final Point2D estimatedPosition, final Double pathlossExponentVariance,
            final Matrix fingerprintPositionCovariance, final Matrix radioSourcePositionCovariance) {
        try {
            final var dist = Utils.propagateVariancesToRssiDifferenceVariance2D(pathlossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, pathlossExponentVariance, fingerprintPositionCovariance,
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

    /**
     * Gets number of dimensions of points.
     *
     * @return number of dimensions of points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Create a point.
     *
     * @return point to be created.
     */
    @Override
    protected Point2D createPoint() {
        return new InhomogeneousPoint2D();
    }
}
