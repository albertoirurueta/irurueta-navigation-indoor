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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RangingAndRssiFingerprint;
import com.irurueta.navigation.indoor.RangingAndRssiReading;
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.indoor.ReadingType;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated2D;
import com.irurueta.statistics.UniformRandomizer;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ReadingSorterTest {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final int MIN_READINGS = 6;
    private static final int MAX_READINGS = 20;

    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 10.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    @Test
    void testConstructor() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var sourceQualityScores = new double[numSources];
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
        }

        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        final var readingsQualityScores = new double[numReadings];
        for (var i = 0; i < numReadings; i++) {
            final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            readings.add(new RssiReading<>(sources.get(0), rssi));
        }

        final var fingerprint = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        assertSame(sources, sorter.getSources());
        assertSame(fingerprint, sorter.getFingerprint());
        assertSame(sourceQualityScores, sorter.getSourceQualityScores());
        assertSame(readingsQualityScores, sorter.getFingerprintReadingsQualityScores());
        assertNull(sorter.getSortedSourcesAndReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ReadingSorter<>(sources, fingerprint, new double[1],
                readingsQualityScores));
        assertThrows(IllegalArgumentException.class, () -> new ReadingSorter<>(sources, fingerprint,
                sourceQualityScores, new double[1]));
    }

    @Test
    void testSortSameSourceRangingReadingsDifferentQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = 1;
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(1, result.size());

        var previousQuality = Double.MAX_VALUE;
        for (var j = 0; j < numReadings; j++) {
            final var readingWithQualityScore = result.get(0).readingsWithQualityScores.get(j);
            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    void testSortSameSourceRangingAndRssiReadingsDifferentQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = 1;
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(1, result.size());

        var previousQuality = Double.MAX_VALUE;
        for (var j = 0; j < numReadings; j++) {
            final var readingWithQualityScore = result.get(0).readingsWithQualityScores.get(j);
            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    void testSortSameSourceRssiReadingsDifferentQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = 1;
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(1, result.size());

        var previousQuality = Double.MAX_VALUE;
        for (var j = 0; j < numReadings; j++) {
            final var readingWithQualityScore = result.get(0).readingsWithQualityScores.get(j);
            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    void testSortSameSourceMixedReadingsDifferentQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = 1;
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[3 * numSources * numReadings];

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<Reading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(1, result.size());

        var previousQuality = Double.MAX_VALUE;
        var previousType = ReadingType.RANGING_READING;
        for (k = 0; k < 3 * numReadings; k++) {
            final var readingWithQualityScore = result.get(0).readingsWithQualityScores.get(k);

            final var type = readingWithQualityScore.reading.getType();
            if (type != previousType) {
                // check correct order of type changes
                if (type == ReadingType.RANGING_AND_RSSI_READING) {
                    assertEquals(ReadingType.RANGING_READING, previousType);
                }
                if (type == ReadingType.RSSI_READING) {
                    assertEquals(ReadingType.RANGING_AND_RSSI_READING, previousType);
                }

                // when type changes reset quality score
                previousQuality = Double.MAX_VALUE;
                previousType = readingWithQualityScore.reading.getType();
            }

            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    void testSortSameSourceRangingReadingsSameQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = 1;
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(1, result.size());

        for (var j = 0; j < numReadings; j++) {
            final var readingWithQualityScore = result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    void testSortSameSourceRangingAndRssiReadingsSameQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = 1;
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint, distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(1, result.size());

        for (var j = 0; j < numReadings; j++) {
            final var readingWithQualityScore = result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    void testSortSameSourceRssiReadingsSameQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = 1;
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(1, result.size());

        for (var j = 0; j < numReadings; j++) {
            final var readingWithQualityScore = result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    void testSortSameSourceMixedReadingsSameQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = 1;
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[3 * numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<Reading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint = new Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(1, result.size());

        var previousType = ReadingType.RANGING_READING;
        for (k = 0; k < 3 * numReadings; k++) {
            final var readingWithQualityScore = result.get(0).readingsWithQualityScores.get(k);

            final var type = readingWithQualityScore.reading.getType();
            if (type != previousType) {
                // check correct order of type changes
                if (type == ReadingType.RANGING_AND_RSSI_READING) {
                    assertEquals(ReadingType.RANGING_READING, previousType);
                }
                if (type == ReadingType.RSSI_READING) {
                    assertEquals(ReadingType.RANGING_AND_RSSI_READING, previousType);
                }

                // when type changes reset quality score
                previousType = readingWithQualityScore.reading.getType();
            }

            assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
        }
    }

    @Test
    void testSortMultipleSourcesRangingReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        var previousSourceQuality = Double.MAX_VALUE;
        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            var previousReadingQuality = Double.MAX_VALUE;
            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    void testSortMultipleSourcesRangingAndRssiReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        var previousSourceQuality = Double.MAX_VALUE;
        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            var previousReadingQuality = Double.MAX_VALUE;
            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    void testSortMultipleSourcesRssiReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        var previousSourceQuality = Double.MAX_VALUE;
        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            var previousReadingQuality = Double.MAX_VALUE;
            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    void testSortMultipleSourcesMixedReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[3 * numSources * numReadings];

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<Reading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        var previousSourceQuality = Double.MAX_VALUE;
        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            var previousReadingQuality = Double.MAX_VALUE;
            var previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(k);

                final var type = readingWithQualityScore.reading.getType();
                if (type != previousType) {
                    // check correct order of type changes
                    if (type == ReadingType.RANGING_AND_RSSI_READING) {
                        assertEquals(ReadingType.RANGING_READING, previousType);
                    }
                    if (type == ReadingType.RSSI_READING) {
                        assertEquals(ReadingType.RANGING_AND_RSSI_READING, previousType);
                    }

                    // when type changes reset quality score
                    previousReadingQuality = Double.MAX_VALUE;
                    previousType = readingWithQualityScore.reading.getType();
                }

                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    void testSortMultipleSourcesRangingReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        var previousSourceQuality = Double.MAX_VALUE;
        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    void testSortMultipleSourcesRangingAndRssiReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        var previousSourceQuality = Double.MAX_VALUE;
        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    void testSortMultipleSourcesRssiReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        var fingerprint = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        var previousSourceQuality = Double.MAX_VALUE;
        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    void testSortMultipleSourcesMixedReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[3 * numSources * numReadings];
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<Reading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint = new Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        var previousSourceQuality = Double.MAX_VALUE;
        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            var previousType = ReadingType.RANGING_READING;
            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);

                final var type = readingWithQualityScore.reading.getType();
                if (type != previousType) {
                    // check correct order of type changes
                    if (type == ReadingType.RANGING_AND_RSSI_READING) {
                        assertEquals(ReadingType.RANGING_READING, previousType);
                    }
                    if (type == ReadingType.RSSI_READING) {
                        assertEquals(ReadingType.RANGING_AND_RSSI_READING, previousType);
                    }

                    // when type changes reset quality score
                    previousType = readingWithQualityScore.reading.getType();
                }

                assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    void testSortMultipleSourcesRangingReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue, 0.0);

            var previousReadingQuality = Double.MAX_VALUE;
            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    void testSortMultipleSourcesRangingAndRssiReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue, 0.0);

            var previousReadingQuality = Double.MAX_VALUE;
            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    void testSortMultipleSourcesRssiReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue, 0.0);

            var previousReadingQuality = Double.MAX_VALUE;
            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    void testSortMultipleSourcesMixedReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[3 * numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<Reading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final var fingerprint = new Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue, 0.0);

            var previousReadingQuality = Double.MAX_VALUE;
            var previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(k);

                final var type = readingWithQualityScore.reading.getType();
                if (type != previousType) {
                    // check correct order of type changes
                    if (type == ReadingType.RANGING_AND_RSSI_READING) {
                        assertEquals(ReadingType.RANGING_READING, previousType);
                    }
                    if (type == ReadingType.RSSI_READING) {
                        assertEquals(ReadingType.RANGING_AND_RSSI_READING, previousType);
                    }

                    // when type changes reset quality score
                    previousReadingQuality = Double.MAX_VALUE;
                    previousType = readingWithQualityScore.reading.getType();
                }

                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    void testSortMultipleSourcesRangingReadingsSameSourceQualityScoresSameReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue, 0.0);

            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    void testSortMultipleSourcesRangingAndRssiReadingsSameSourceQualityScoresSameReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RangingAndRssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint =
                new RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue, 0.0);

            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    void testSortMultipleSourcesRssiReadingsSameSourceQualityScoresSameReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue, 0.0);

            for (var j = 0; j < numReadings; j++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    void testSortMultipleSourcesMixedReadingsSameSourceQualityScoresSameReadingQualityScores() {
        final var randomizer = new UniformRandomizer();
        final var numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final var sourceQualityScores = new double[numSources];
        final var readingsQualityScores = new double[3 * numSources * numReadings];
        final var sourceQualityScoreValue = randomizer.nextDouble();
        final var readingQualityScoreValue = randomizer.nextDouble();

        final var sources = new ArrayList<WifiAccessPointLocated2D>();
        final var readings = new ArrayList<Reading<WifiAccessPoint>>();
        var k = 0;
        for (var i = 0; i < numSources; i++) {
            final var accessPoint = new WifiAccessPointLocated2D("id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (var j = 0; j < numReadings; j++) {
                final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final var rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
                readings.add(new RangingReading<>(accessPoint, distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final var fingerprint = new Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>>();
        fingerprint.setReadings(readings);

        final var sorter = new ReadingSorter<>(sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final var result = sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (var i = 0; i < numSources; i++) {
            final var sourceWithQualityScore = result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue, 0.0);

            var previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                final var readingWithQualityScore = sourceWithQualityScore.readingsWithQualityScores.get(k);

                final var type = readingWithQualityScore.reading.getType();
                if (type != previousType) {
                    // check correct order of type changes
                    if (type == ReadingType.RANGING_AND_RSSI_READING) {
                        assertEquals(ReadingType.RANGING_READING, previousType);
                    }
                    if (type == ReadingType.RSSI_READING) {
                        assertEquals(ReadingType.RANGING_AND_RSSI_READING, previousType);
                    }

                    // when type changes reset quality score
                    previousType = readingWithQualityScore.reading.getType();
                }

                assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }
}
