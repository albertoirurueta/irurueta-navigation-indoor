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

import com.irurueta.geometry.Point;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.indoor.ReadingType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

/**
 * Evenly sorts readings of a fingerprint among different radio sources taking into
 * account their respective quality scores as well.
 *
 * @param <P> a {@link Point} type.
 * @param <R> a {@link Reading} type.
 */
public class ReadingSorter<P extends Point<?>, R extends Reading<? extends RadioSource>> {

    /**
     * Sources to be taken into account within readings.
     */
    private final List<? extends RadioSourceLocated<P>> sources;

    /**
     * Fingerprint containing readings of different sources.
     */
    private final Fingerprint<? extends RadioSource, ? extends R> fingerprint;

    /**
     * Quality scores associated to each radio source.
     */
    private final double[] sourceQualityScores;

    /**
     * Quality scores associated to each reading within the fingerprint.
     */
    private final double[] fingerprintReadingsQualityScores;

    /**
     * Contains sorted sources and readings.
     */
    private List<RadioSourceSourceWithQualityScore<P, R>> sortedSourcesAndReadings;

    /**
     * Constructor.
     *
     * @param sources                          sources to take into account within
     *                                         readings.
     * @param fingerprint                      fingerprint containing readings of
     *                                         different sources.
     * @param sourceQualityScores              quality scores associated to each radio
     *                                         source.
     * @param fingerprintReadingsQualityScores quality scores associated to each reading
     *                                         within the fingerprint.
     * @throws IllegalArgumentException if number of source quality scores is not equal
     *                                  to the number of sources, or if number of
     *                                  fingerprint reading quality scores is not equal
     *                                  to the number of readings within fingerprint.
     */
    ReadingSorter(final List<? extends RadioSourceLocated<P>> sources,
                  final Fingerprint<? extends RadioSource, ? extends R> fingerprint,
                  final double[] sourceQualityScores, final double[] fingerprintReadingsQualityScores) {
        if (sources.size() != sourceQualityScores.length) {
            throw new IllegalArgumentException();
        }

        if (fingerprint.getReadings().size() != fingerprintReadingsQualityScores.length) {
            throw new IllegalArgumentException();
        }

        this.sources = sources;
        this.fingerprint = fingerprint;
        this.sourceQualityScores = sourceQualityScores;
        this.fingerprintReadingsQualityScores = fingerprintReadingsQualityScores;
    }

    /**
     * Gets sources to be taken into account within readings.
     *
     * @return sources to be taken into account within readings.
     */
    public List<RadioSourceLocated<P>> getSources() {
        //noinspection unchecked
        return (List<RadioSourceLocated<P>>) sources;
    }

    /**
     * Gets fingerprint containing readings of different sources.
     *
     * @return fingerprint containing readings of different sources.
     */
    public Fingerprint<RadioSource, Reading<RadioSource>> getFingerprint() {
        //noinspection unchecked
        return (Fingerprint<RadioSource, Reading<RadioSource>>) fingerprint;
    }

    /**
     * Gets quality scores associated to each radio source.
     *
     * @return quality scores associated to each radio source.
     */
    double[] getSourceQualityScores() {
        return sourceQualityScores;
    }

    /**
     * Gets quality scores associated to each reading within the fingerprint.
     *
     * @return quality scores associated to each reading within the fingerprint.
     */
    double[] getFingerprintReadingsQualityScores() {
        return fingerprintReadingsQualityScores;
    }

    /**
     * Sorts readings with quality scores.
     */
    void sort() {

        final var sourcesMap = new HashMap<RadioSourceLocated<P>, RadioSourceSourceWithQualityScore<P, R>>();

        // build sources
        final var sourcesWithQualityScores = new ArrayList<RadioSourceSourceWithQualityScore<P, R>>();
        var sourcePosition = 0;
        for (final var source : sources) {
            final var sourceWithQualityScore = new RadioSourceSourceWithQualityScore<P, R>();
            sourceWithQualityScore.source = source;
            sourceWithQualityScore.qualityScore = sourceQualityScores[sourcePosition];
            sourceWithQualityScore.position = sourcePosition;
            sourceWithQualityScore.readingsWithQualityScores = new ArrayList<>();

            sourcesWithQualityScores.add(sourceWithQualityScore);

            sourcesMap.put(source, sourceWithQualityScore);

            sourcePosition++;
        }

        // build readings
        var readingPosition = 0;
        for (@SuppressWarnings("unchecked") final var reading : fingerprint.getReadings()) {
            if (!(reading.getSource() instanceof RadioSourceLocated)) {
                continue;
            }

            //noinspection unchecked
            final var radioSourceLocated = (RadioSourceLocated<P>) reading.getSource();
            final var sourceWithQualityScore = sourcesMap.get(radioSourceLocated);

            if (sourceWithQualityScore == null) {
                continue;
            }

            final var readingsWithQualityScores = sourceWithQualityScore.readingsWithQualityScores;

            final var readingWithQualityScore = new ReadingWithQualityScore<R>();
            readingWithQualityScore.reading = reading;
            readingWithQualityScore.qualityScore = fingerprintReadingsQualityScores[readingPosition];
            readingWithQualityScore.position = readingPosition;

            readingsWithQualityScores.add(readingWithQualityScore);

            readingPosition++;
        }

        // sort all readings within sources
        for (final var sourceWithQualityScore : sourcesWithQualityScores) {
            // sort all readings for this source from highest to lowest quality

            // noinspection unchecked
            final ReadingWithQualityScore<R>[] readingsWithQualityScoresArray =
                    new ReadingWithQualityScore[sourceWithQualityScore.readingsWithQualityScores.size()];
            sourceWithQualityScore.readingsWithQualityScores.toArray(readingsWithQualityScoresArray);
            Arrays.sort(readingsWithQualityScoresArray, new ReadingComparator<>());

            sourceWithQualityScore.readingsWithQualityScores = Arrays.asList(readingsWithQualityScoresArray);
        }

        // sort all sources from highest to lowest quality

        // noinspection unchecked
        final RadioSourceSourceWithQualityScore<P, R>[] sourcesWithQualityScoresArray =
                new RadioSourceSourceWithQualityScore[sourcesWithQualityScores.size()];
        sourcesWithQualityScores.toArray(sourcesWithQualityScoresArray);
        Arrays.sort(sourcesWithQualityScoresArray, new RadioSourceComparator<>());

        sortedSourcesAndReadings = Arrays.asList(sourcesWithQualityScoresArray);
    }

    /**
     * Gets sorted sources and readings.
     *
     * @return sorted sources and readings.
     */
    List<RadioSourceSourceWithQualityScore<P, R>> getSortedSourcesAndReadings() {
        return sortedSourcesAndReadings;
    }

    /**
     * Contains a reading with its associated quality score.
     *
     * @param <R> a {@link Reading} type.
     */
    static class ReadingWithQualityScore<R extends Reading<? extends RadioSource>> {
        /**
         * A reading.
         */
        R reading;

        /**
         * Reading quality score.
         */
        double qualityScore;

        /**
         * Original position.
         */
        int position;
    }

    /**
     * Contains a radio source with its associated quality score.
     *
     * @param <P> a {@link Point} type.
     * @param <R> a {@link Reading} type.
     */
    static class RadioSourceSourceWithQualityScore<P extends Point<?>, R extends Reading<? extends RadioSource>> {
        /**
         * Radio source.
         */
        RadioSourceLocated<P> source;

        /**
         * Radio source quality score.
         */
        double qualityScore;

        /**
         * Original position.
         */
        int position;

        List<ReadingWithQualityScore<R>> readingsWithQualityScores;
    }

    /**
     * Comparator to order readings based on their quality scores.
     *
     * @param <R> a {@link Reading} type.
     */
    private static class ReadingComparator<R extends Reading<? extends RadioSource>> implements
            Comparator<ReadingWithQualityScore<R>> {

        /**
         * Orders readings so that readings with higher scores go first.
         *
         * @param o1 1st item to be compared.
         * @param o2 2nd item to be compared.
         * @return -1 if first item goes first, 0 if both are equal and 1 if first item
         * goes second.
         */
        @Override
        public int compare(final ReadingWithQualityScore<R> o1, final ReadingWithQualityScore<R> o2) {

            // Take reading type into account so that ranging readings go first,
            // then ranging+RSSI and finally RSSI readings.
            if (o1.reading.getType() != o2.reading.getType()) {
                switch (o1.reading.getType()) {
                    case RANGING_READING:
                        // o1 goes first (o2 last)
                        return -1;

                    case RANGING_AND_RSSI_READING:
                        if (o2.reading.getType() == ReadingType.RANGING_READING) {
                            // RANGING_READING
                            // o2 goes first (o1 last)
                            return 1;
                        } else {
                            // RSSI_READING
                            // o1 goes first (o2 last)
                            return -1;
                        }
                    case RSSI_READING:
                    default:
                        // o2 goes first (o1 last)
                        return 1;
                }
            }

            // same reading type

            // order by reading quality score
            return Double.compare(o2.qualityScore, o1.qualityScore);
        }
    }

    /**
     * Comparator to order radio sources based on their quality scores.
     *
     * @param <P> a {@link Point} type.
     * @param <R> a {@link Reading} type.
     */
    private static class RadioSourceComparator<P extends Point<?>, R extends Reading<? extends RadioSource>> implements
            Comparator<RadioSourceSourceWithQualityScore<P, R>> {

        /**
         * Orders radio sources so that radio sources with higher scores go first.
         *
         * @param o1 1st item to be compared.
         * @param o2 2nd item to be compared.
         * @return -1 if first item goes first, 0 if both are equal and 1 if first item
         * goes second.
         */
        @Override
        public int compare(final RadioSourceSourceWithQualityScore<P, R> o1,
                           final RadioSourceSourceWithQualityScore<P, R> o2) {
            return Double.compare(o2.qualityScore, o1.qualityScore);
        }
    }
}
