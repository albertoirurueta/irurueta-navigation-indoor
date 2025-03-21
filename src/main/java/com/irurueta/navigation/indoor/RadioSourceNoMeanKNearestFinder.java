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
package com.irurueta.navigation.indoor;

import com.irurueta.geometry.Point;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Finds k-nearest radio source fingerprints based on their signal Euclidean distances
 * (not their actual location) where mean values are removed to account for possible
 * biases when measuring from different devices.
 * Typically, this class should be preferred over {@link RadioSourceKNearestFinder}.
 *
 * @param <P> a {@link Point} type.
 * @param <S> a {@link RadioSource type}.
 */
public class RadioSourceNoMeanKNearestFinder<P extends Point<?>, S extends RadioSource> {

    /**
     * Collection of fingerprints to match against.
     */
    private final Collection<? extends RssiFingerprintLocated<S, RssiReading<S>, P>> mFingerprints;

    /**
     * Constructor.
     *
     * @param fingerprints collection of fingerprints to match against.
     * @throws IllegalArgumentException if collection of fingerprints is null.
     */
    public RadioSourceNoMeanKNearestFinder(
            final Collection<? extends RssiFingerprintLocated<S, RssiReading<S>, P>> fingerprints) {
        if (fingerprints == null) {
            throw new IllegalArgumentException();
        }
        mFingerprints = fingerprints;
    }

    /**
     * Finds nearest fingerprint to provided one, in terms of signal Euclidean distances
     * (with removed signal means), within the collection of provided fingerprints.
     *
     * @param fingerprint fingerprint to find the nearest to.
     * @return nearest fingerprint or null if none could be found.
     */
    public RssiFingerprintLocated<S, RssiReading<S>, P> findNearestTo(
            final RssiFingerprint<S, RssiReading<S>> fingerprint) {
        return findNearestTo(fingerprint, mFingerprints);
    }

    /**
     * Finds k-nearest fingerprints to provided one, in terms of signal Euclidean distances
     * (with removed signal means), within the collection of provided fingerprints.
     *
     * @param fingerprint fingerprint to find the k-nearest ones to.
     * @param k           number of nearest fingerprints to find.
     * @return nearest fingerprints ordered from closest to farthest or an empty list if none could be found.
     * @throws IllegalArgumentException if either fingerprint is null or k is less than 1.
     */
    public List<RssiFingerprintLocated<S, RssiReading<S>, P>> findKNearestTo(
            final RssiFingerprint<S, RssiReading<S>> fingerprint, final int k) {
        return findKNearestTo(fingerprint, mFingerprints, k);
    }

    /**
     * Finds k-nearest fingerprints to provided one, in terms of signal Euclidean distances
     * (with removed signal means), within the collection of provided fingerprints.
     *
     * @param fingerprint         fingerprint to find the k-nearest ones to.
     * @param k                   number of nearest fingerprints to find.
     * @param nearestFingerprints list where found nearest fingerprints will be stored ordered from closest to farthest
     *                            or an empty list if none could be found.
     * @param nearestSqrDistances list where squared signal Euclidean distances corresponding to found fingerprints will
     *                            be stored or an empty list if no fingerprint is found.
     * @throws IllegalArgumentException if any parameter is null or k is less than 1.
     */
    public void findKNearestTo(
            final RssiFingerprint<S, RssiReading<S>> fingerprint, final int k,
            final List<RssiFingerprintLocated<S, RssiReading<S>, P>> nearestFingerprints,
            final List<Double> nearestSqrDistances) {
        findKNearestTo(fingerprint, mFingerprints, k, nearestFingerprints, nearestSqrDistances);
    }

    /**
     * Gets collection of fingerprints to match against.
     *
     * @return collection of fingerprints to match against.
     */
    public Collection<RssiFingerprintLocated<S, RssiReading<S>, P>> getFingerprints() {
        //noinspection unchecked
        return (Collection<RssiFingerprintLocated<S, RssiReading<S>,P>>) mFingerprints;
    }

    /**
     * Finds nearest fingerprint to provided one, in terms of signal Euclidean distances
     * (with removed signal means), within the collection of provided fingerprints.
     *
     * @param fingerprint  fingerprint to find the nearest to.
     * @param fingerprints collection of fingerprints to make the search for the nearest one.
     * @param <P>          a {@link Point} type.
     * @param <S>          a {@link RadioSource} type.
     * @return nearest fingerprint or null if none could be found.
     * @throws IllegalArgumentException if either fingerprint or collection of fingerprints is null.
     */
    @SuppressWarnings("Duplicates")
    public static <P extends Point<?>, S extends RadioSource> RssiFingerprintLocated<S, RssiReading<S>, P>
    findNearestTo(final RssiFingerprint<S, RssiReading<S>> fingerprint,
                  final Collection<? extends RssiFingerprintLocated<S, RssiReading<S>, P>> fingerprints) {
        if (fingerprint == null || fingerprints == null) {
            throw new IllegalArgumentException();
        }

        var bestSqrDist = Double.MAX_VALUE;
        RssiFingerprintLocated<S, RssiReading<S>, P> result = null;
        for (final var f : fingerprints) {
            var sqrDist = f.noMeanSqrDistanceTo(fingerprint);
            if (sqrDist < bestSqrDist) {
                bestSqrDist = sqrDist;
                result = f;
            }
        }

        return result;
    }

    /**
     * Finds k-nearest fingerprints to provided one, in terms of signal Euclidean distances, within the collection
     * of provided fingerprints.
     *
     * @param fingerprint  fingerprint to find the k-nearest ones to.
     * @param fingerprints collection of fingerprints to make the search for the nearest ones.
     * @param k            number of nearest fingerprints to find.
     * @param <P>          a {@link Point} type.
     * @param <S>          a {@link RadioSource} type.
     * @return nearest fingerprints ordered from closest to farthest or an empty list if none could be found.
     * @throws IllegalArgumentException if either fingerprint or collection of fingerprints is null, or k is less than
     *                                  1.
     */
    @SuppressWarnings("DuplicatedCode")
    public static <P extends Point<?>, S extends RadioSource> List<RssiFingerprintLocated<S, RssiReading<S>, P>>
    findKNearestTo(final RssiFingerprint<S, RssiReading<S>> fingerprint,
                   final Collection<? extends RssiFingerprintLocated<S, RssiReading<S>, P>> fingerprints,
                   final int k) {

        final var result = new ArrayList<RssiFingerprintLocated<S, RssiReading<S>, P>>();
        final var nearestSqrDistances = new ArrayList<Double>();
        findKNearestTo(fingerprint, fingerprints, k, result, nearestSqrDistances);

        return result;
    }

    /**
     * Finds k-nearest fingerprints to provided one, in terms of signal Euclidean distances, within the collection
     * of provided fingerprints.
     *
     * @param fingerprint         fingerprint to find the k-nearest ones to.
     * @param fingerprints        collection of fingerprints ot make the search for the nearest ones.
     * @param k                   number of nearest fingerprints to find.
     * @param nearestFingerprints list where found nearest fingerprints will be stored ordered from closest to farthest
     *                            or an empty list if none could be found.
     * @param nearestSqrDistances list where squared signal Euclidean distances corresponding to found fingerprints will
     *                            be stored or an empty list if no fingerprint is found.
     * @param <P>                 a {@link Point} type.
     * @param <S>                 a {@link RadioSource} type.
     * @throws IllegalArgumentException if any parameter is null or k is less than 1.
     */
    @SuppressWarnings("Duplicates")
    public static <P extends Point<?>, S extends RadioSource> void findKNearestTo(
            final RssiFingerprint<S, RssiReading<S>> fingerprint,
            final Collection<? extends RssiFingerprintLocated<S, RssiReading<S>, P>> fingerprints, final int k,
            final List<RssiFingerprintLocated<S, RssiReading<S>, P>> nearestFingerprints,
            final List<Double> nearestSqrDistances) {

        if (fingerprint == null || fingerprints == null || k < 1 || nearestFingerprints == null
                || nearestSqrDistances == null) {
            throw new IllegalArgumentException();
        }

        nearestSqrDistances.clear();
        nearestFingerprints.clear();

        var maxSqrDist = Double.MAX_VALUE;
        for (final var f : fingerprints) {
            final var sqrDist = f.noMeanSqrDistanceTo(fingerprint);
            if (sqrDist < maxSqrDist || nearestSqrDistances.size() < k) {

                // find insertion point
                var pos = -1;
                var i = 0;
                for (final var sd : nearestSqrDistances) {
                    if (sqrDist < sd) {
                        // insertion point found
                        pos = i;
                        break;
                    }
                    i++;
                }

                if (pos >= 0) {
                    nearestSqrDistances.add(pos, sqrDist);
                    nearestFingerprints.add(pos, f);
                } else {
                    nearestSqrDistances.add(sqrDist);
                    nearestFingerprints.add(f);
                }

                // remove results exceeding required number of k neighbours to be found
                if (nearestFingerprints.size() > k) {
                    nearestSqrDistances.remove(k);
                    nearestFingerprints.remove(k);
                }

                // update maxSqrDist to the largest squared distance value contained in result list distances
                maxSqrDist = nearestSqrDistances.get(nearestSqrDistances.size() - 1);
            }
        }
    }
}
