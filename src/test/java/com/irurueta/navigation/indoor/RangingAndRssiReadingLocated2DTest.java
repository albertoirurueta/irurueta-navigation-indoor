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
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class RangingAndRssiReadingLocated2DTest {

    private static final double FREQUENCY = 2.4e9;

    @Test
    public void testConstructor() throws AlgebraException {
        // test empty constructor
        RangingAndRssiReadingLocated2D<WifiAccessPoint> reading =
                new RangingAndRssiReadingLocated2D<>();

        // check
        assertNull(reading.getSource());
        assertEquals(reading.getDistance(), 0.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), 0.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertNull(reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // test constructor with access point, distance, rssi and position
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        reading = new RangingAndRssiReadingLocated2D<>(ap, 1.2,
                -50.0, position);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated2D<>(null,
                    1.2, -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, -1.0,
                    -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.2,
                    -50.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position and number
        // of measurements.
        reading = new RangingAndRssiReadingLocated2D<>(ap, 1.2, -50.0,
                position, 8, 7);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated2D<>(null, 1.2,
                    -50.0, position, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, -1.0,
                    -50.0, position, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.2,
                    -50.0, null, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.2,
                    -50.0, position, 0,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.2,
                    -50.0, position, 8,
                    -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position,
        // distance standard deviation and rssi standard deviation
        reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5, -50.0,
                position, 0.1, 0.2);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5, -50.0,
                position, null, null);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated2D<>(null, 1.5,
                    -50.0, position, 0.1,
                    0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, -1.0, -50.0,
                    position, 0.1, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5, -50.0,
                    null, 0.1, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5, -50.0,
                    position, 0.0, 0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5, -50.0,
                    position, 0.1, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position, distance
        // standard deviation, rssi standard deviation and number of measurements
        reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5, -50.0,
                position, 0.1, 0.2,
                8, 7);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated2D<>(null, 1.5,
                    -50.0, position, 0.1,
                    0.2, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, -1.0,
                    -50.0, position, 0.1,
                    0.2, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5,
                    -50.0, null, 0.1,
                    0.2, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5,
                    -50.0, position, 0.0,
                    0.2, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5,
                    -50.0, position, 0.1,
                    0.0, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5,
                    -50.0, position, 0.1,
                    0.2, 0,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 1.5,
                    -50.0, position, 0.1,
                    0.2, 8,
                    -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position and position covariance
        final Matrix cov = new Matrix(2, 2);
        reading = new RangingAndRssiReadingLocated2D<>(ap, 2.0, -50.0,
                position, cov);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 2.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated2D<>(null,
                    2.0, -50.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, -1.0, -50.0,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.0, -50.0,
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.0, -50.0,
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position, position
        // covariance and number of measurements.
        reading = new RangingAndRssiReadingLocated2D<>(ap, 2.0, -50.0,
                position, cov, 8, 7);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 2.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated2D<>(null,
                    2.0, -50.0, position, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, -1.0, -50.0,
                    position, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.0, -50.0,
                    null, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.0, -50.0,
                    position, new Matrix(1, 1),
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.0, -50.0,
                    position, cov, 0,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.0, -50.0,
                    position, cov, 8,
                    -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position,
        // distance standard deviation, rssi standard deviation and position
        // covariance
        reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                position, 0.1, 0.2, cov);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                position, null, null,
                null);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertNull(reading.getRssiStandardDeviation());
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated2D<>(null, 2.5,
                    -50.0, position, 0.1,
                    0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, -1.0, -50.0,
                    position, 0.1, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    null, 0.1, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.0, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.1, 0.0, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.1, 0.2,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, rssi, position,
        // distance standard deviation, rssi standard deviation, position covariance
        // and number of measurements.
        reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                position, 0.1, 0.2, cov,
                8, 7);

        // check
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading.getRssi(), -50.0, 0.0);
        assertEquals(reading.getRssiStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingAndRssiReadingLocated2D<>(null, 2.5, -50.0,
                    position, 0.1, 0.2, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, -1.0, -50.0,
                    position, 0.1, 0.2, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    null, 0.1, 0.2, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.0, 0.2, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.1, 0.0, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.1, 0.2,
                    new Matrix(1, 1),
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.1, 0.2,
                    new Matrix(2, 1),
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.1, 0.2, cov,
                    0, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                    position, 0.1, 0.2, cov,
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

        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();

        final RangingAndRssiReadingLocated2D<WifiAccessPoint> reading1 = new RangingAndRssiReadingLocated2D<>(
                ap1, 1.5, -50.0, position);
        final RangingAndRssiReadingLocated2D<WifiAccessPoint> reading2 = new RangingAndRssiReadingLocated2D<>(
                ap1, 1.5, -50.0, position);
        final RangingAndRssiReadingLocated2D<WifiAccessPoint> reading3 = new RangingAndRssiReadingLocated2D<>(
                ap2, 1.5, -50.0, position);

        // check
        assertTrue(reading1.hasSameSource(reading1));
        assertTrue(reading1.hasSameSource(reading2));
        assertFalse(reading1.hasSameSource(reading3));
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        final Matrix cov = new Matrix(2, 2);
        final RangingAndRssiReadingLocated2D<WifiAccessPoint> reading1 =
                new RangingAndRssiReadingLocated2D<>(ap, 2.5, -50.0,
                        position, 0.1, 0.2, cov,
                        8, 7);

        // check
        assertSame(reading1.getSource(), ap);
        assertEquals(reading1.getDistance(), 2.5, 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(), 0.1, 0.0);
        assertEquals(reading1.getRssi(), -50.0, 0.0);
        assertEquals(reading1.getRssiStandardDeviation(), 0.2, 0.0);
        assertSame(reading1.getPosition(), position);
        assertSame(reading1.getPositionCovariance(), cov);
        assertEquals(reading1.getType(), ReadingType.RANGING_AND_RSSI_READING);
        assertEquals(reading1.getNumAttemptedMeasurements(), 8);
        assertEquals(reading1.getNumSuccessfulMeasurements(), 7);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(reading1);
        final RangingAndRssiReadingLocated2D<WifiAccessPoint> reading2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getDistance(), reading2.getDistance(), 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(),
                reading2.getDistanceStandardDeviation(), 0.0);
        assertEquals(reading1.getRssi(), reading2.getRssi(), 0.0);
        assertEquals(reading1.getRssiStandardDeviation(),
                reading2.getRssiStandardDeviation());
        assertEquals(reading1.getPosition(), reading2.getPosition());
        assertEquals(reading1.getPositionCovariance(), reading2.getPositionCovariance());
        assertEquals(reading1.getType(), reading2.getType());
        assertEquals(reading1.getNumAttemptedMeasurements(),
                reading2.getNumAttemptedMeasurements());
        assertEquals(reading1.getNumSuccessfulMeasurements(),
                reading2.getNumSuccessfulMeasurements());
    }
}
