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

public class RangingReadingTest {

    private static final double FREQUENCY = 2.4e9;

    @Test
    public void testConstructor() {
        // test empty constructor
        RangingReading<WifiAccessPoint> reading = new RangingReading<>();

        // check
        assertNull(reading.getSource());
        assertEquals(reading.getDistance(), 0.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingReading.DEFAULT_NUM_MEASUREMENTS);

        // test constructor with access point and distance
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new RangingReading<>(ap, 1.2);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReading<>(null, 1.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance and number of measurements
        reading = new RangingReading<>(ap, 1.2, 8,
                7);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReading<>(null, 1.2,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, -1.0,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, 1.2,
                    0, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, 1.2,
                    8, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance and distance standard deviation
        reading = new RangingReading<>(ap, 1.5, 0.1);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingReading.DEFAULT_NUM_MEASUREMENTS);

        reading = new RangingReading<>(ap, 1.5, null);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReading<>(null, 1.5,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, -1.0,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, 1.5,
                    0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, distance standard deviation an
        // number of measurements
        reading = new RangingReading<>(ap, 1.5, 0.1,
                8, 7);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReading<>(null, 1.5,
                    0.1, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, -1.0,
                    0.1, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, 1.5,
                    0.0, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, 1.5,
                    0.1, 0,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReading<>(ap, 1.5,
                    0.1, 8,
                    -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);
    }

    @Test
    public void testHasSameAccessPoint() {
        final WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final WifiAccessPoint ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        final RangingReading<WifiAccessPoint> reading1 = new RangingReading<>(ap1, 50.0);
        final RangingReading<WifiAccessPoint> reading2 = new RangingReading<>(ap1, 50.0);
        final RangingReading<WifiAccessPoint> reading3 = new RangingReading<>(ap2, 50.0);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        final RangingReading<WifiAccessPoint> reading1 = new RangingReading<>(
                ap, 1.5, 0.1,
                8, 7);

        // check
        assertSame(reading1.getSource(), ap);
        assertEquals(reading1.getDistance(), 1.5, 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading1.getType(), ReadingType.RANGING_READING);
        assertEquals(reading1.getNumAttemptedMeasurements(), 8);
        assertEquals(reading1.getNumSuccessfulMeasurements(), 7);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(reading1);
        final RangingReading<WifiAccessPoint> reading2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getDistance(), reading2.getDistance(), 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(),
                reading2.getDistanceStandardDeviation(), 0.0);
        assertEquals(reading1.getType(), reading2.getType());
        assertEquals(reading1.getNumAttemptedMeasurements(),
                reading2.getNumAttemptedMeasurements());
        assertEquals(reading1.getNumSuccessfulMeasurements(),
                reading2.getNumSuccessfulMeasurements());
    }
}
