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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class RssiReadingLocated2DTest {

    private static final double FREQUENCY = 2.4e9;

    @Test
    void testConstructor() throws AlgebraException {
        // test empty constructor
        var reading = new RssiReadingLocated2D<WifiAccessPoint>();

        // check
        assertNull(reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertNull(reading.getSource());
        assertEquals(0.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        // test constructor with access point, rssi and position
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        final var position = new InhomogeneousPoint2D();
        reading = new RssiReadingLocated2D<>(ap, -50.0, position);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertSame(ap, reading.getSource());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(null, -50.0,
                position));
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(ap, -50.0, null));

        // test constructor with access point, rssi, position and rssi standard deviation
        reading = new RssiReadingLocated2D<>(ap, -50.0, position, 5.5);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertSame(ap, reading.getSource());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(5.5, reading.getRssiStandardDeviation(), 0.0);
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(null, -50.0, position,
                5.5));
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(ap, -50.0, null,
                5.5));
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(ap, -50.0, position,
                0.0));

        // test constructor with access point, rssi, position and covariance
        final var cov = new Matrix(2, 2);
        reading = new RssiReadingLocated2D<>(ap, -50.0, position, cov);

        // check
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertSame(ap, reading.getSource());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        reading = new RssiReadingLocated2D<>(ap, -50.0, position, (Matrix) null);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertSame(ap, reading.getSource());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(null, -50.0, position,
                cov));
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(ap, -50.0, null,
                cov));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(ap, -50.0, position, m));

        // test constructor with access point, rssi, position, rssi standard deviation, and position covariance
        reading = new RssiReadingLocated2D<>(ap, -50.0, position, 5.5, cov);

        // check
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertSame(ap, reading.getSource());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(5.5, reading.getRssiStandardDeviation(), 0.0);
        assertEquals(ReadingType.RSSI_READING, reading.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(null, -50.0, position,
                5.5, cov));
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(ap, -50.0, null,
                5.5, cov));
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(ap, -50.0, position,
                0.0, cov));
        assertThrows(IllegalArgumentException.class, () -> new RssiReadingLocated2D<>(ap, -50.0, position,
                5.5, m));
    }

    @Test
    void testHasSameAccessPoint() {
        final var ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final var ap2 = new WifiAccessPoint("bssid2", FREQUENCY);

        final var position = new InhomogeneousPoint2D();
        final var reading1 = new RssiReadingLocated2D<>(ap1, -50.0, position);
        final var reading2 = new RssiReadingLocated2D<>(ap1, -50.0, position);
        final var reading3 = new RssiReadingLocated2D<>(ap2, -50.0, position);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        final var position = new InhomogeneousPoint2D();
        final var cov = new Matrix(2, 2);
        final var reading1 = new RssiReadingLocated2D<>(ap, -50.0, position, 5.5, cov);

        // check
        assertSame(position, reading1.getPosition());
        assertSame(cov, reading1.getPositionCovariance());
        assertSame(ap, reading1.getSource());
        assertEquals(-50.0, reading1.getRssi(), 0.0);
        assertEquals(5.5, reading1.getRssiStandardDeviation(), 0.0);
        assertEquals(ReadingType.RSSI_READING, reading1.getType());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(reading1);
        final var reading2 = SerializationHelper.<RssiReadingLocated2D<WifiAccessPoint>>deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getPosition(), reading2.getPosition());
        assertEquals(reading1.getPositionCovariance(), reading2.getPositionCovariance());
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getType(), reading2.getType());
    }
}
