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

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiFingerprintLocated;
import com.irurueta.navigation.indoor.RssiReading;

import java.util.List;

/**
 * Base class for position and radio source estimators based on located fingerprints
 * containing only RSSI readings without any prior knowledge of radio sources.
 *
 * @param <P> a {@link Point} type.
 * @param <L> a {@link BaseFingerprintPositionAndRadioSourceEstimatorListener} type.
 */
public abstract class BaseFingerprintPositionAndRadioSourceEstimator<P extends Point<?>,
        L extends BaseFingerprintPositionAndRadioSourceEstimatorListener<?>> {

    /**
     * Default minimum number of nearest fingerprints to search.
     * This is the minimum required value to be used for the estimation of all
     * radio sources and fingerprint position.
     */
    public static final int DEFAULT_MIN_NEAREST_FINGERPRINTS = -1;

    /**
     * Default maximum number of nearest fingerprints to search (no limit).
     */
    public static final int DEFAULT_MAX_NEAREST_FINGERPRINTS = -1;

    /**
     * Default exponent typically used on free space for path loss propagation in
     * terms of distance. This value is used for free space environments.
     */
    public static final double DEFAULT_PATH_LOSS_EXPONENT = 2.0;

    /**
     * Located fingerprints containing RSSI readings.
     */
    protected List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints;

    /**
     * Fingerprint containing readings at an unknown location.
     * Readings need to belong to the same radio sources as those given at located
     * fingerprints.
     */
    protected RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint;

    /**
     * Minimum number of nearest fingerprints to search or -1 if there is no limit
     * and all required fingerprints are initially used.
     */
    protected int minNearestFingerprints = DEFAULT_MIN_NEAREST_FINGERPRINTS;

    /**
     * Maximum number of nearest fingerprints to search or -1 if there is no limit and
     * all provided fingerprints are used.
     */
    protected int maxNearestFingerprints = DEFAULT_MAX_NEAREST_FINGERPRINTS;

    /**
     * Path loss exponent to be used by default.
     * This is typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     */
    protected double pathLossExponent = DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Listener to be notified of events raised by this instance.
     */
    protected L listener;

    /**
     * Estimated inhomogeneous position coordinates.
     */
    protected double[] estimatedPositionCoordinates;

    /**
     * Radio sources provided along with located fingerprints containing
     * their estimated locations.
     */
    protected List<RadioSourceLocated<P>> estimatedLocatedSources;

    /**
     * Nearest located fingerprints based on their RSSI readings respect to provided fingerprint.
     * These are the fingerprints that are probably located close to the unknown location to be estimated,
     * however their location is approximate due to errors on RSSI readings.
     */
    protected List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>> nearestFingerprints;

    /**
     * Indicates if this instance is locked.
     */
    protected boolean locked;

    /**
     * Constructor.
     */
    protected BaseFingerprintPositionAndRadioSourceEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    protected BaseFingerprintPositionAndRadioSourceEstimator(final L listener) {
        this.listener = listener;
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
    protected BaseFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint) {
        internalSetLocatedFingerprints(locatedFingerprints);
        internalSetFingerprint(fingerprint);
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
    protected BaseFingerprintPositionAndRadioSourceEstimator(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            final RssiFingerprint<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>> fingerprint,
            final L listener) {
        this(listener);
        internalSetLocatedFingerprints(locatedFingerprints);
        internalSetFingerprint(fingerprint);
    }

    /**
     * Gets located fingerprints containing RSSI readings.
     *
     * @return located fingerprints containing RSSI readings.
     */
    public List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>> getLocatedFingerprints() {
        //noinspection unchecked
        return (List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>,P>>) locatedFingerprints;
    }

    /**
     * Sets located fingerprints containing RSSI readings.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is null or there are not enough
     *                                  fingerprints or readings within provided fingerprints (for 2D position estimation at
     *                                  least 2 readings are required in a single fingerprint, or at least 2 fingerprints
     *                                  at different locations containing a single reading are required. For 3D position
     *                                  estimation 3 reading in a single fingerprint, or 3 fingerprints containing a single
     *                                  reading or any combination resulting in at least 3 readings at different locations
     *                                  are required).
     */
    public void setLocatedFingerprints(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetLocatedFingerprints(locatedFingerprints);
    }

    /**
     * Gets fingerprint containing readings at an unknown location for provided located
     * fingerprints.
     *
     * @return fingerprint containing readings at an unknown location for provided located
     * fingerprints.
     */
    public RssiFingerprint<RadioSource, RssiReading<RadioSource>> getFingerprint() {
        //noinspection unchecked
        return (RssiFingerprint<RadioSource, RssiReading<RadioSource>>) fingerprint;
    }

    /**
     * Sets fingerprint containing readings at an unknown location for provided located fingerprints.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for provided located fingerprints.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided value is null.
     */
    public void setFingerprint(
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetFingerprint(fingerprint);
    }

    /**
     * Get minimum number of nearest fingerprints to search.
     *
     * @return minimum number of nearest fingerprints, -1 indicates to initially use
     * all fingerprints needed to estimate available radio sources.
     */
    public int getMinNearestFingerprints() {
        return minNearestFingerprints;
    }

    /**
     * Gets maximum number of nearest fingerprints to search.
     *
     * @return maximum number of nearest fingerprints, -1 indicates to use all available
     * fingerprints.
     */
    public int getMaxNearestFingerprints() {
        return maxNearestFingerprints;
    }

    /**
     * Sets minimum and maximum number of nearest fingerprints to search.
     * If minimum value is -1, then the minimum required number of fingerprints needed
     * to estimate available radio sources is used.
     * If maximum value is -1, then the problem is attempted to be solved until all
     * available fingerprints are used.
     *
     * @param minNearestFingerprints minimum number of nearest fingerprints or -1.
     * @param maxNearestFingerprints maximum number of nearest fingerprints or -1.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if minimum value is larger than maximum value (as
     *                                  long as it has a limit defined), or if maximum value is not negative when
     *                                  minimum one is less than 1, or if minimum value is zero.
     */
    public void setMinMaxNearestFingerprints(
            final int minNearestFingerprints, final int maxNearestFingerprints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetMinMaxNearestFingerprints(minNearestFingerprints, maxNearestFingerprints);
    }

    /**
     * Gets path loss exponent to be used by default.
     * This is typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     *
     * @return path loss exponent to be used by default.
     */
    public double getPathLossExponent() {
        return pathLossExponent;
    }

    /**
     * Sets path loss exponent to be used by default.
     * This is typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     *
     * @param pathLossExponent path loss exponent to be used by default.
     * @throws LockedException if estimator is locked.
     */
    public void setPathLossExponent(final double pathLossExponent) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.pathLossExponent = pathLossExponent;
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     *
     * @return listener to be notified of events raised by this instance.
     */
    public L getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     *
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(final L listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Gets estimated inhomogeneous position coordinates.
     *
     * @return estimated inhomogeneous position coordinates.
     */
    public double[] getEstimatedPositionCoordinates() {
        return estimatedPositionCoordinates;
    }

    /**
     * Gets estimated position and stores result into provided instance.
     *
     * @param estimatedPosition instance where estimated position will be stored.
     */
    public void getEstimatedPosition(final P estimatedPosition) {
        if (estimatedPositionCoordinates != null) {
            for (var i = 0; i < estimatedPositionCoordinates.length; i++) {
                estimatedPosition.setInhomogeneousCoordinate(i, estimatedPositionCoordinates[i]);
            }
        }
    }

    /**
     * Gets estimated position or null if not available yet.
     *
     * @return estimated position or null.
     */
    public P getEstimatedPosition() {
        if (estimatedPositionCoordinates == null) {
            return null;
        }

        final var result = createPoint();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Gets radio sources provided along with located fingerprints containing
     * their estimated locations.
     *
     * @return radio sources containing estimated locations.
     */
    public List<RadioSourceLocated<P>> getEstimatedLocatedSources() {
        return estimatedLocatedSources;
    }

    /**
     * Gets nearest found located fingerprints based on their RSSI readings respect to provided fingerprint.
     * These are the fingerprints that are probably located close to the unknown location to be estimated,
     * however their location is approximate due to errors on RSSI readings.
     *
     * @return nearest located fingerprints based on their RSSI readings or null if estimation has not been done yet.
     */
    public List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>> getNearestFingerprints() {
        return nearestFingerprints;
    }

    /**
     * Returns boolean indicating whether this estimator is locked because an estimation
     * is already in progress.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return locatedFingerprints != null && fingerprint != null;
    }

    /**
     * Gets number of dimensions of points.
     *
     * @return number of dimensions of points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Starts estimation of position of unknown fingerprint and position of all radio sources
     * associated to located fingerprints.
     *
     * @throws LockedException                if estimator is locked.
     * @throws NotReadyException              if estimator is not ready.
     * @throws FingerprintEstimationException if estimation fails for some other reason.
     */
    public abstract void estimate() throws LockedException, NotReadyException, FingerprintEstimationException;

    /**
     * Create a point.
     *
     * @return point to be created.
     */
    protected abstract P createPoint();

    /**
     * Internally sets located fingerprints containing RSSI readings.
     *
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @throws IllegalArgumentException if provided value is null.
     */
    private void internalSetLocatedFingerprints(
            final List<? extends RssiFingerprintLocated<? extends RadioSource,
                    ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints) {
        if (locatedFingerprints == null) {
            throw new IllegalArgumentException();
        }

        this.locatedFingerprints = locatedFingerprints;
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for provided
     * located fingerprints.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located fingerprints.
     * @throws IllegalArgumentException if provided value is null.
     */
    private void internalSetFingerprint(
            final RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>> fingerprint) {
        if (fingerprint == null) {
            throw new IllegalArgumentException();
        }

        this.fingerprint = fingerprint;
    }

    /**
     * Sets minimum and maximum number of nearest fingerprints to search.
     *
     * @param minNearestFingerprints minimum number of nearest fingerprints.
     * @param maxNearestFingerprints maximum number of nearest fingerprints.
     * @throws IllegalArgumentException if minimum value is larger than maximum value (as
     *                                  long as it has a limit defined), or if maximum value is not negative when
     *                                  minimum one is less than 1.
     */
    @SuppressWarnings("Duplicates")
    private void internalSetMinMaxNearestFingerprints(
            final int minNearestFingerprints, final int maxNearestFingerprints) {
        if (minNearestFingerprints == 0 || (minNearestFingerprints < 1 && maxNearestFingerprints >= 0)
                || (maxNearestFingerprints >= 0 && minNearestFingerprints > maxNearestFingerprints)) {
            throw new IllegalArgumentException();
        }

        this.minNearestFingerprints = minNearestFingerprints;
        this.maxNearestFingerprints = maxNearestFingerprints;
    }
}
