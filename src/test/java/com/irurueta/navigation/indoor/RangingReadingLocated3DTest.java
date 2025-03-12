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
import com.irurueta.geometry.InhomogeneousPoint3D;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class RangingReadingLocated3DTest {
    private static final double FREQUENCY = 2.4e9;

    @Test
    void testConstructor() throws AlgebraException {
        // test empty constructor
        var reading = new RangingReadingLocated3D<WifiAccessPoint>();

        // check
        assertNull(reading.getSource());
        assertEquals(0.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertNull(reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // test constructor with access point, distance and position
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        final var position = new InhomogeneousPoint3D();
        reading = new RangingReadingLocated3D<>(ap, 1.2, position);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(1.2, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(null, 1.2,
                position));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, -1.0, position));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.2,
                null));

        // test constructor with access point, distance, position and number of
        // measurements.
        reading = new RangingReadingLocated3D<>(ap, 1.2, position, 8,
                7);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(1.2, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(null, 1.2,
                position, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, -1.0, position,
                8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.2, null,
                8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.2, position,
                0, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.2, position,
                8, -1));

        // test constructor with access point, distance, position and distance standard deviation
        reading = new RangingReadingLocated3D<>(ap, 1.5, position, 0.1);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(null, 1.5,
                position, 0.1));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, -1.0, position,
                0.1));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.5, null,
                0.1));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.5, position,
                0.0));

        // test constructor with access point, distance, position, distance standard deviation and number of
        // measurements.
        reading = new RangingReadingLocated3D<>(ap, 1.5, position, 0.1,
                8, 7);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(null, 1.5,
                position, 0.1, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, -1.0, position,
                0.1, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.5, null,
                0.1, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.5, position,
                0.0, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.5, position,
                0.1, 0, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 1.5, position,
                0.1, 8, -1));

        // test constructor with access point, distance, position and position covariance
        final var cov = new Matrix(3, 3);
        reading = new RangingReadingLocated3D<>(ap, 2.0, position, cov);

        // check
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertEquals(2.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        reading = new RangingReadingLocated3D<>(ap, 2.0, position, (Matrix) null);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(2.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(null, 2.0,
                position, cov));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, -1.0, position,
                cov));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.0, null,
                cov));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.0, position,
                m1));
        final var m3 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.0, position,
                m3));

        // test constructor with access point, distance, position, position covariance and number of measurements.
        reading = new RangingReadingLocated3D<>(ap, 2.0, position, cov, 8,
                7);

        // check
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertEquals(2.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(null, 2.0,
                position, cov, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, -1.0, position,
                cov, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.0, null,
                cov, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.0, position,
                m1, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.0, position, m3,
                8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.0, position, cov,
                0, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.0, position, cov,
                8, -1));

        // test constructor with access point, distance, position, distance standard deviation and position covariance
        reading = new RangingReadingLocated3D<>(ap, 2.5, position, 0.2, cov);

        // check
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertEquals(2.5, reading.getDistance(), 0.0);
        assertEquals(0.2, reading.getDistanceStandardDeviation(), 0.0);
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        reading = new RangingReadingLocated3D<>(ap, 2.5, position, 0.2,
                null);

        // check
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(2.5, reading.getDistance(), 0.0);
        assertEquals(0.2, reading.getDistanceStandardDeviation(), 0.0);
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(null, 2.5,
                position, 0.2, cov));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, -1.0, position,
                0.2, cov));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, null,
                0.2, cov));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, position,
                0.0, cov));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, position,
                0.2, m1));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, position,
                0.2, m3));

        // test constructor with access point, distance, position, distance standard deviation, position covariance and
        // number of measurements.
        reading = new RangingReadingLocated3D<>(ap, 2.5, position, 0.2, cov,
                8, 7);

        // check
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertEquals(2.5, reading.getDistance(), 0.0);
        assertEquals(0.2, reading.getDistanceStandardDeviation(), 0.0);
        assertSame(ap, reading.getSource());
        assertEquals(ReadingType.RANGING_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(null, 2.5,
                position, 0.2, cov, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, -1.0, position,
                0.2, cov, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, null,
                0.2, cov, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, position,
                0.0, cov, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, position,
                0.2, m1, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, position,
                0.2, m3, 8, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, position,
                0.2, cov, 0, 7));
        assertThrows(IllegalArgumentException.class, () -> new RangingReadingLocated3D<>(ap, 2.5, position,
                0.2, cov, 8, -1));
    }

    @Test
    void testHasSameAccessPoint() {
        final var ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final var ap2 = new WifiAccessPoint("bssid2", FREQUENCY);
        final var position = new InhomogeneousPoint3D();

        final var reading1 = new RangingReadingLocated3D<>(ap1, 50.0, position);
        final var reading2 = new RangingReadingLocated3D<>(ap1, 50.0, position);
        final var reading3 = new RangingReadingLocated3D<>(ap2, 50.0, position);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var ap = new WifiAccessPoint("bssid", FREQUENCY);
        final var position = new InhomogeneousPoint3D();
        final var cov = new Matrix(3, 3);
        final var reading1 = new RangingReadingLocated3D<>(ap, 2.5, position, 0.2, cov,
                8, 7);

        // check
        assertSame(position, reading1.getPosition());
        assertSame(cov, reading1.getPositionCovariance());
        assertEquals(2.5, reading1.getDistance(), 0.0);
        assertEquals(0.2, reading1.getDistanceStandardDeviation(), 0.0);
        assertSame(ap, reading1.getSource());
        assertEquals(ReadingType.RANGING_READING, reading1.getType());
        assertEquals(8, reading1.getNumAttemptedMeasurements());
        assertEquals(7, reading1.getNumSuccessfulMeasurements());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(reading1);
        final var reading2 = SerializationHelper.<RangingReadingLocated3D<WifiAccessPoint>>deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getPosition(), reading2.getPosition());
        assertEquals(reading1.getPositionCovariance(), reading2.getPositionCovariance());
        assertEquals(reading1.getDistance(), reading2.getDistance(), 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(), reading2.getDistanceStandardDeviation(), 0.0);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getType(), reading2.getType());
        assertEquals(reading1.getNumAttemptedMeasurements(), reading2.getNumAttemptedMeasurements());
        assertEquals(reading1.getNumSuccessfulMeasurements(), reading2.getNumSuccessfulMeasurements());
    }
}
