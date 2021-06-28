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

import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class RangingAndRssiReadingTest {

    private static final double FREQUENCY = 2.4e9;

    @Test
    public void testConstructor() {
        // test empty constructor
        RangingAndRssiReading<WifiAccessPoint> reading = new RangingAndRssiReading<>();

        // check
        assertNull(reading.getSource());
        assertEquals(reading.getDistance(), 0.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), 0.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // test constructor with access point, distance and RSSI
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new RangingAndRssiReading<>(ap, 1.2, -50.0);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // Force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReading<>(null, 1.2,
                    -50.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, -1.0, -50.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, RSSI and number of measurements.
        reading = new RangingAndRssiReading<>(ap, 1.2, -50.0,
                8, 7);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // Force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReading<>(null, 1.2, -50,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, -1.0, -50.0,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.2, -50.0,
                    0, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.2, -50.0,
                    8, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, RSSI and standard deviations
        reading = new RangingAndRssiReading<>(ap, 1.5, -50.0,
                0.1, 5.5);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 5.5, 0.0);
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // Force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReading<>(null, 1.5,
                    -50.0, 0.1, 5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, -1.0, -50.0,
                    0.1, 5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.0, -50.0,
                    0.0, 5.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.0, -50.0,
                    0.1, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, RSS, standard deviations and
        // number of measurements
        reading = new RangingAndRssiReading<>(ap, 1.5, -50.0,
                0.1, 5.5,
                8, 7);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 5.5, 0.0);
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // Force IllegalArgumentException.
        reading = null;
        try {
            reading = new RangingAndRssiReading<>(null, 1.5, -50.0,
                    0.1, 5.5,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, -1.0, -50.0,
                    0.1, 5.5,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.0, -50.0,
                    0.0, 5.5,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.0, -50.0,
                    0.1, 0.0,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.5, -50.0,
                    0.1, 5.5,
                    0, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReading<>(ap, 1.5, -50.0,
                    0.1, 5.5,
                    8, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        final WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        final RangingAndRssiReading<WifiAccessPoint> reading1 = new RangingAndRssiReading<>(ap1,
                1.5, -50.0);
        final RangingAndRssiReading<WifiAccessPoint> reading2 = new RangingAndRssiReading<>(ap1,
                1.5, -50.0);
        final RangingAndRssiReading<WifiAccessPoint> reading3 = new RangingAndRssiReading<>(ap2,
                1.5, -50.0);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        final RangingAndRssiReading<WifiAccessPoint> reading1 = new RangingAndRssiReading<>(
                ap, 1.5, -50.0,
                0.1, 5.5,
                8, 7);

        // check
        assertSame(reading1.getSource(), ap);
        assertEquals(reading1.getDistance(), 1.5, 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading1.getRssi(), -50.0, 0.0);
        assertEquals(reading1.getRssiStandardDeviation(), 5.5, 0.0);
        assertEquals(reading1.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading1.getNumAttemptedMeasurements(), 8);
        assertEquals(reading1.getNumSuccessfulMeasurements(), 7);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(reading1);
        final RangingAndRssiReading<WifiAccessPoint> reading2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getDistance(), reading2.getDistance(), 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(),
                reading2.getDistanceStandardDeviation(), 0.0);
        assertEquals(reading1.getRssi(), reading2.getRssi(), 0.0);
        assertEquals(reading1.getRssiStandardDeviation(),
                reading2.getRssiStandardDeviation(), 0.0);
        assertEquals(reading1.getType(), reading2.getType());
        assertEquals(reading1.getNumAttemptedMeasurements(),
                reading2.getNumAttemptedMeasurements());
        assertEquals(reading1.getNumSuccessfulMeasurements(),
                reading2.getNumSuccessfulMeasurements());
    }
}
