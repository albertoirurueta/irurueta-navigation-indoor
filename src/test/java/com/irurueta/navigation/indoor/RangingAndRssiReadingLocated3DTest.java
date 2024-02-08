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
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class RangingAndRssiReadingLocated3DTest {
    private static final double FREQUENCY = 2.4e9;

    @Test
    public void testConstructor() throws AlgebraException {
        // test empty constructor
        RangingAndRssiReadingLocated3D<WifiAccessPoint> reading = new RangingAndRssiReadingLocated3D<>();

        // check
        assertNull(reading.getSource());
        assertEquals(0.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(0.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertNull(reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // test constructor with access point, distance, rssi and position
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        reading = new RangingAndRssiReadingLocated3D<>(ap, 1.2, -50.0, position);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.2, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated3D<>(null, 1.2, -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, -1.0, -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.2, -50.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position and number
        // of measurements.
        reading = new RangingAndRssiReadingLocated3D<>(ap, 1.2, -50.0, position,
                8, 7);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.2, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated3D<>(null, 1.2, -50.0, position,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, -1.0, -50.0, position,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.2, -50.0, null,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.2, -50.0, position,
                    0, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.2, -50.0, position,
                    8, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position,
        // distance standard deviation and rssi standard deviation
        reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                0.1, 0.2);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(0.2, reading.getRssiStandardDeviation(), 0.0);
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                null, null);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated3D<>(null, 1.5, -50.0, position,
                    0.1, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, -1.0, -50.0, position,
                    0.1, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, null,
                    0.1, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                    0.0, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                    0.1, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position, distance
        // standard deviation, rssi standard deviation and number of measurements
        reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                0.1, 0.2, 8,
                7);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(1.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(0.2, reading.getRssiStandardDeviation(), 0.0);
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated3D<>(null, 1.5, -50.0, position,
                    0.1, 0.2, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, -1.0, -50.0, position,
                    0.1, 0.2, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, null,
                    0.1, 0.2, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                    0.0, 0.2, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                    0.1, 0.0, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                    0.1, 0.2, 0,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 1.5, -50.0, position,
                    0.1, 0.2, 8,
                    -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position and position covariance
        final Matrix cov = new Matrix(3, 3);
        reading = new RangingAndRssiReadingLocated3D<>(ap, 2.0, -50.0, position, cov);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(2.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated3D<>(null, 2.0, -50.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, -1.0, -50.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.0, -50.0, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.0, -50.0,
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position, position
        // covariance and number of measurements.
        reading = new RangingAndRssiReadingLocated3D<>(ap, 2.0, -50.0, position, cov,
                8, 7);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(2.0, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated3D<>(null, 2.0, -50.0, position, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, -1.0, -50.0, position, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.0, -50.0, null, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.0, -50.0, position,
                    new Matrix(1, 1), 8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.0, -50.0, position, cov,
                    0, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.0, -50.0, position, cov,
                    8, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position,
        // distance standard deviation, rssi standard deviation and position
        // covariance
        reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                0.1, 0.2, cov);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(2.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(0.2, reading.getRssiStandardDeviation(), 0.0);
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                null, null, null);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(2.5, reading.getDistance(), 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(position, reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumAttemptedMeasurements());
        assertEquals(RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated3D<>(null, 2.5, -50.0, position,
                    0.1, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, -1.0, -50.0, position,
                    0.1, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, null,
                    0.1, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.0, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.1, 0.0, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.1, 0.2, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position,
        // distance standard deviation, rssi standard deviation, position covariance
        // and number of measurements.
        reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                0.1, 0.2, cov, 8,
                7);

        // check
        assertSame(ap, reading.getSource());
        assertEquals(2.5, reading.getDistance(), 0.0);
        assertEquals(0.1, reading.getDistanceStandardDeviation(), 0.0);
        assertEquals(-50.0, reading.getRssi(), 0.0);
        assertEquals(0.2, reading.getRssiStandardDeviation(), 0.0);
        assertSame(position, reading.getPosition());
        assertSame(cov, reading.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading.getType());
        assertEquals(8, reading.getNumAttemptedMeasurements());
        assertEquals(7, reading.getNumSuccessfulMeasurements());

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated3D<>(null, 2.5, -50.0, position,
                    0.1, 0.2, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, -1.0, -50.0, position,
                    0.1, 0.2, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, null,
                    0.1, 0.2, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.0, 0.2, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.1, 0.0, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.1, 0.2, new Matrix(1, 1),
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.1, 0.2, new Matrix(2, 1),
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.1, 0.2, cov, 0,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated3D<>(ap, 2.5, -50.0, position,
                    0.1, 0.2, cov, 8,
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

        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();

        final RangingAndRssiReadingLocated3D<WifiAccessPoint> reading1 = new RangingAndRssiReadingLocated3D<>(
                ap1, 1.5, -50.0, position);
        final RangingAndRssiReadingLocated3D<WifiAccessPoint> reading2 = new RangingAndRssiReadingLocated3D<>(
                ap1, 1.5, -50.0, position);
        final RangingAndRssiReadingLocated3D<WifiAccessPoint> reading3 = new RangingAndRssiReadingLocated3D<>(
                ap2, 1.5, -50.0, position);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        final Matrix cov = new Matrix(3, 3);
        final RangingAndRssiReadingLocated3D<WifiAccessPoint> reading1 = new RangingAndRssiReadingLocated3D<>(
                ap, 2.5, -50.0, position, 0.1, 0.2, cov,
                8, 7);

        // check
        assertSame(ap, reading1.getSource());
        assertEquals(2.5, reading1.getDistance(), 0.0);
        assertEquals(0.1, reading1.getDistanceStandardDeviation(), 0.0);
        assertEquals(-50.0, reading1.getRssi(), 0.0);
        assertEquals(0.2, reading1.getRssiStandardDeviation(), 0.0);
        assertSame(position, reading1.getPosition());
        assertSame(cov, reading1.getPositionCovariance());
        assertEquals(ReadingType.RANGING_AND_RSSI_READING, reading1.getType());
        assertEquals(8, reading1.getNumAttemptedMeasurements());
        assertEquals(7, reading1.getNumSuccessfulMeasurements());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(reading1);
        final RangingAndRssiReadingLocated3D<WifiAccessPoint> reading2 = SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getDistance(), reading2.getDistance(), 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(), reading2.getDistanceStandardDeviation(), 0.0);
        assertEquals(reading1.getRssi(), reading2.getRssi(), 0.0);
        assertEquals(reading1.getRssiStandardDeviation(), reading2.getRssiStandardDeviation());
        assertEquals(reading1.getPosition(), reading2.getPosition());
        assertEquals(reading1.getPositionCovariance(), reading2.getPositionCovariance());
        assertEquals(reading1.getType(), reading2.getType());
        assertEquals(reading1.getNumAttemptedMeasurements(), reading2.getNumAttemptedMeasurements());
        assertEquals(reading1.getNumSuccessfulMeasurements(), reading2.getNumSuccessfulMeasurements());
    }
}
