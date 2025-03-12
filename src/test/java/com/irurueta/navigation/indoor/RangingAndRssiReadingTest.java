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

class RangingAndRssiReadingTest {

    private static final double FREQUENCY = 2.4e9;

    @Test
    void testConstructor() {
        // test empty constructor
        var reading = new RangingAndRssiReading<WifiAccessPoint>();

        // check
        assertNull(reading.getSource());
        assertEquals(0.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(0.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // test constructor with access point, distance and RSSI
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new RangingAndRssiReading<>(ap, 1.2, -50.0);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.2, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(null, 1.2,
                -50.0));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, -1.0, -50.0));

        // test constructor with access point, distance, RSSI and number of measurements.
        reading = new RangingAndRssiReading<>(ap, 1.2, -50.0, 8,
                7);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.2, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(null, 1.2,
                -50, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, -1.0, -50.0,
                8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, 1.2, -50.0,
                0, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, 1.2, -50.0,
                8, -1));

        // test constructor with access point, distance, RSSI and standard deviations
        reading = new RangingAndRssiReading<>(ap, 1.5, -50.0, 0.1,
                5.5);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(5.5, reading.getRssiStandardDeviation(), 0.0);
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(null, 1.5,
                -50.0, 0.1, 5.5));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, -1.0, -50.0,
                0.1, 5.5));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, 1.0, -50.0,
                0.0, 5.5));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, 1.0, -50.0,
                0.1, 0.0));

        // test constructor with access point, distance, RSS, standard deviations and number of measurements
        reading = new RangingAndRssiReading<>(ap, 1.5, -50.0, 0.1,
                5.5, 8, 7);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(5.5, reading.getRssiStandardDeviation(), 0.0);
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // Force IllegalArgumentException.
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(null, 1.5,
                -50.0, 0.1, 5.5, 8,
                7));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, -1.0, -50.0,
                0.1, 5.5, 8,
                7));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, 1.0, -50.0,
                0.0, 5.5, 8,
                7));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, 1.0, -50.0,
                0.1, 0.0, 8,
                7));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, 1.5, -50.0,
                0.1, 5.5, 0,
                7));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiReading<>(ap, 1.5, -50.0,
                0.1, 5.5, 8,
                -1));
    }

    @Test
    void testHasSameAccessPoint() {
        final var ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final var ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        final var reading1 = new RangingAndRssiReading<>(ap1, 1.5, -50.0);
        final var reading2 = new RangingAndRssiReading<>(ap1, 1.5, -50.0);
        final var reading3 = new RangingAndRssiReading<>(ap2, 1.5, -50.0);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        final var reading1 = new RangingAndRssiReading<>(ap, 1.5, -50.0, 0.1,
                5.5, 8, 7);

        // check
        assertSame(ap, reading1.getSource());
        assertEquals(1.5, reading1.getDistance(), 0.0);
        assertEquals(0.1, reading1.getDistanceStandardDeviation(), 0.0);
        assertEquals(-50.0, reading1.getRssi(), 0.0);
        assertEquals(5.5, reading1.getRssiStandardDeviation(), 0.0);
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading1.getType());
        assertEquals(8, reading1.getNumAttemptedMeasurements());
        assertEquals(7, reading1.getNumSuccessfulMeasurements());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(reading1);
        final var reading2 = SerializationHelper.<RangingAndRssiReading<WifiAccessPoint>>deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getDistance(), reading2.getDistance(), 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(), reading2.getDistanceStandardDeviation(), 0.0);
        assertEquals(reading1.getRssi(), reading2.getRssi(), 0.0);
        assertEquals(reading1.getRssiStandardDeviation(), reading2.getRssiStandardDeviation(), 0.0);
        assertEquals(reading1.getType(), reading2.getType());
        assertEquals(reading1.getNumAttemptedMeasurements(), reading2.getNumAttemptedMeasurements());
        assertEquals(reading1.getNumSuccessfulMeasurements(), reading2.getNumSuccessfulMeasurements());
    }
}
