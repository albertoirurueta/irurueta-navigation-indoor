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

import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class RangingReadingTest {

    private static final double FREQUENCY = 2.4e9;

    @Test
    void testConstructor() {
        // test empty constructor
        var reading = new RangingReading<WifiAccessPoint>();

        // check
        assertNull(reading.getSource());
        assertEquals(0.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // test constructor with access point and distance
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new RangingReading<>(ap, 1.2);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.2, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(null, 1.2));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, -1.0));

        // test constructor with access point, distance and number of measurements
        reading = new RangingReading<>(ap, 1.2, 8, 7);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.2, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(null, 1.2,
                8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, -1.0,
                8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, 1.2,
                0, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, 1.2,
                8, -1));

        // test constructor with access point, distance and distance standard deviation
        reading = new RangingReading<>(ap, 1.5, 0.1);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        reading = new RangingReading<>(ap, 1.5, null);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(null, 1.5,
                0.1));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, -1.0,
                0.1));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, 1.5,
                0.0));

        // test constructor with access point, distance, distance standard deviation and number of measurements
        reading = new RangingReading<>(ap, 1.5, 0.1, 8,
                7);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(null, 1.5,
                0.1, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, -1.0,
                0.1, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, 1.5,
                0.0, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, 1.5,
                0.1, 0, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReading<>(ap, 1.5,
                0.1, 8, -1));
    }

    @Test
    void testHasSameAccessPoint() {
        final var ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final var ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        final var reading1 = new RangingReading<>(ap1, 50.0);
        final var reading2 = new RangingReading<>(ap1, 50.0);
        final var reading3 = new RangingReading<>(ap2, 50.0);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        final var reading1 = new RangingReading<>(ap, 1.5, 0.1, 8,
                7);

        // check
        assertSame(ap, reading1.getSource());
        assertEquals(1.5, reading1.getDistance(), 0.0);
        assertEquals(0.1, reading1.getDistanceStandardDeviation(), 0.0);
        assertEquals(ReadingType.RANGING_READING, reading1.getType());
        assertEquals(8, reading1.getNumAttemptedMeasurements());
        assertEquals(7, reading1.getNumSuccessfulMeasurements());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(reading1);
        final var reading2 = SerializationHelper.<RangingReading<WifiAccessPoint>>deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getDistance(), reading2.getDistance(), 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(), reading2.getDistanceStandardDeviation(), 0.0);
        assertEquals(reading1.getType(), reading2.getType());
        assertEquals(reading1.getNumAttemptedMeasurements(), reading2.getNumAttemptedMeasurements());
        assertEquals(reading1.getNumSuccessfulMeasurements(), reading2.getNumSuccessfulMeasurements());
    }
}
