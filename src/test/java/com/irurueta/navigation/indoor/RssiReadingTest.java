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

class RssiReadingTest {

    private static final double FREQUENCY = 2.4e9;

    @Test
    void testConstructor() {
        // test empty constructor
        var reading = new RssiReading<WifiAccessPoint>();

        // check
        assertNull(reading.getSource());
        assertEquals(0.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        // test constructor with access point and RSSI
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        reading = new RssiReading<>(ap, -50.0);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RssiReading<>(null, -50.0));

        // test constructor with access point, RSSI and RSSI standard deviation
        reading = new RssiReading<>(ap, -50.0, 5.5);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(5.5, reading.getRssiStandardDeviation(), 0.0);
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new RssiReading<>(null, -50.0, 5.5));
        assertThrows(IllegalArgumentException.class, () -> new RssiReading<>(ap, -50.0, 0.0));
    }

    @Test
    void testHasSameAccessPoint() {
        final var ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final var ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        final var reading1 = new RssiReading<>(ap1, -50.0);
        final var reading2 = new RssiReading<>(ap1, -50.0);
        final var reading3 = new RssiReading<>(ap2, -50.0);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        final var reading1 = new RssiReading<>(ap, -50.0, 5.5);

        // check
        assertSame(ap, reading1.getSource());
        assertEquals(-50.0, reading1.getRssi(), 0.0);
        assertEquals(5.5, reading1.getRssiStandardDeviation(), 0.0);
        assertEquals(ReadingType.RSSI_READING, reading1.getType());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(reading1);
        final var reading2 = SerializationHelper.<RssiReading<WifiAccessPoint>>deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getRssi(), reading2.getRssi(), 0.0);
        assertEquals(reading1.getRssiStandardDeviation(), reading2.getRssiStandardDeviation(), 0.0);
        assertEquals(reading1.getType(), reading2.getType());
    }
}
