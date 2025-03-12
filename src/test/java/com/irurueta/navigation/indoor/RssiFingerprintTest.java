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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RssiFingerprintTest {

    private static final int MIN_RSSI = -100;
    private static final int MAX_RSSI = -50;

    private static final int MIN_READINGS = 1;
    private static final int MAX_READINGS = 5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double FREQUENCY = 2.4e9;

    @Test
    void testConstructor() {
        // test empty constructor
        var f = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();

        // check default values
        assertTrue(f.getReadings().isEmpty());

        // test constructor with readings
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        f = new RssiFingerprint<>(readings);

        // check
        assertEquals(readings, f.getReadings());
        assertNotSame(readings, f.getReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RssiFingerprint<>(null));
    }

    @Test
    void testGetSetReadings() {
        final var f = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();

        // check default value
        assertTrue(f.getReadings().isEmpty());

        // set new value
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        f.setReadings(readings);

        // check
        assertEquals(readings, f.getReadings());
        assertNotSame(readings, f.getReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> f.setReadings(null));
    }

    @Test
    void testDistanceToAndSqrDistanceTo() {
        final var randomizer = new UniformRandomizer();

        // test fingerprint with empty readings
        var f = new RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>();

        assertEquals(Double.MAX_VALUE, f.sqrDistanceTo(f), 0.0);

        // test equal fingerprints
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        for (var i = 0; i < numReadings; i++) {
            final var ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final var rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            final var reading = new RssiReading<>(ap, rssi);
            readings.add(reading);
        }

        f = new RssiFingerprint<>(readings);

        assertEquals(0.0, f.sqrDistanceTo(f), ABSOLUTE_ERROR);
        assertEquals(0.0, f.distanceTo(f), ABSOLUTE_ERROR);

        // test different fingerprint RSSI values
        final var readings2 = new ArrayList<RssiReading<WifiAccessPoint>>();
        for (var i = 0; i < numReadings; i++) {
            final var ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final var rssi = readings.get(i).getRssi() + 1.0;
            final var reading = new RssiReading<>(ap, rssi);
            readings2.add(reading);
        }

        final var f2 = new RssiFingerprint<>(readings2);

        assertEquals(numReadings, f.sqrDistanceTo(f2), ABSOLUTE_ERROR);
        assertEquals(Math.sqrt(numReadings), f.distanceTo(f2), ABSOLUTE_ERROR);

        // test different fingerprint access points
        final var readings3 = new ArrayList<RssiReading<WifiAccessPoint>>();
        for (var i = 0; i < numReadings + 1; i++) {
            final var ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            double rssi;
            if (i < numReadings) {
                rssi = readings.get(i).getRssi();
            } else {
                rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            }
            final var reading = new RssiReading<>(ap, rssi);
            readings3.add(reading);
        }

        final var f3 = new RssiFingerprint<>(readings3);

        assertEquals(0.0, f.sqrDistanceTo(f3), ABSOLUTE_ERROR);
        assertEquals(0.0, f.distanceTo(f3), ABSOLUTE_ERROR);

        // test with null fingerprints
        assertEquals(Double.MAX_VALUE, f.sqrDistanceTo(null), 0.0);
    }

    @Test
    void testGetMeanRssi() {
        final var randomizer = new UniformRandomizer();

        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        var meanRssi = 0.0;
        for (var i = 0; i < numReadings; i++) {
            final var ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final var rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            meanRssi += (double) rssi / (double) numReadings;

            final var reading = new RssiReading<>(ap, rssi);
            readings.add(reading);
        }

        final var f = new RssiFingerprint<>(readings);
        assertEquals(f.getMeanRssi(), meanRssi, ABSOLUTE_ERROR);
    }

    @Test
    void testNoMeanDistanceToAndSqrDistanceTo() {
        final var randomizer = new UniformRandomizer();

        // test fingerprint with empty readings
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> f = new RssiFingerprint<>();

        assertEquals(Double.MAX_VALUE, f.noMeanSqrDistanceTo(f), 0.0);

        // test equal fingerprints
        final var numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        for (var i = 0; i < numReadings; i++) {
            final var ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final var rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            final var reading = new RssiReading<>(ap, rssi);
            readings.add(reading);
        }

        f = new RssiFingerprint<>(readings);

        assertEquals(0.0, f.noMeanSqrDistanceTo(f), ABSOLUTE_ERROR);
        assertEquals(0.0, f.noMeanDistanceTo(f), ABSOLUTE_ERROR);

        // test different fingerprint RSSI values
        final var readings2 = new ArrayList<RssiReading<WifiAccessPoint>>();
        for (var i = 0; i < numReadings; i++) {
            final var ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            final var rssi = readings.get(i).getRssi() + 1.0;
            final var reading = new RssiReading<>(ap, rssi);
            readings2.add(reading);
        }

        final var f2 = new RssiFingerprint<>(readings2);

        assertEquals(0.0, f.noMeanSqrDistanceTo(f2), ABSOLUTE_ERROR);
        assertEquals(0.0, f.noMeanDistanceTo(f2), ABSOLUTE_ERROR);

        // test different fingerprint access points
        final var readings3 = new ArrayList<RssiReading<WifiAccessPoint>>();
        for (var i = 0; i < numReadings + 1; i++) {
            final var ap = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            double rssi;
            if (i < numReadings) {
                rssi = readings.get(i).getRssi();
            } else {
                rssi = randomizer.nextInt(MIN_RSSI, MAX_RSSI);
            }
            final var reading = new RssiReading<>(ap, rssi);
            readings3.add(reading);
        }

        final var f3 = new RssiFingerprint<>(readings3);

        assertEquals(0.0, f.noMeanSqrDistanceTo(f3), ABSOLUTE_ERROR);
        assertEquals(0.0, f.noMeanDistanceTo(f3), ABSOLUTE_ERROR);

        // test with null fingerprints
        assertEquals(Double.MAX_VALUE, f.noMeanSqrDistanceTo(null), 0.0);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        final var f1 = new RssiFingerprint<>(readings);

        // check
        assertEquals(readings, f1.getReadings());
        assertNotSame(readings, f1.getReadings());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(f1);
        final var f2 =
                SerializationHelper.<RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>>deserialize(bytes);

        // check
        assertNotSame(f1, f2);
        assertEquals(f1.getReadings(), f2.getReadings());
    }
}
