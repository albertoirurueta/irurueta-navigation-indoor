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

public class RangingReadingLocated2DTest {

    private static final double FREQUENCY = 2.4e9;

    @Test
    public void testConstructor() throws AlgebraException {
        // test empty constructor
        RangingReadingLocated2D<WifiAccessPoint> reading = new RangingReadingLocated2D<>();

        // check
        assertNull(reading.getSource());
        assertEquals(reading.getDistance(), 0.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertNull(reading.getPosition());
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // test constructor with access point, distance and position
        final WifiAccessPoint ap = new WifiAccessPoint("bssid", FREQUENCY);
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        reading = new RangingReadingLocated2D<>(ap, 1.2, position);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated2D<>(null,
                    1.2, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, -1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.2, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, position and number of
        // measurements.
        reading = new RangingReadingLocated2D<>(ap, 1.2, position,
                8, 7);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 1.2, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated2D<>(null, 1.2, position,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, -1.0, position,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.2, null,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.2, position,
                    0, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.2, position,
                    8, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, position and distance standard deviation
        reading = new RangingReadingLocated2D<>(ap, 1.5, position,
                0.1);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated2D<>(null, 1.5, position,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, -1.0, position,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.5, null,
                    0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.5, position,
                    0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, position, distance standard
        // deviation and number of measurements
        reading = new RangingReadingLocated2D<>(ap, 1.5, position,
                0.1, 8,
                7);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 1.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.1, 0.0);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated2D<>(null, 1.5, position,
                    0.1, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, -1.0, position,
                    0.1, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.5, null,
                    0.1, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.5, position,
                    0.0, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.5, position,
                    0.1, 0,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 1.5, position,
                    0.1, 8,
                    -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, position and position covariance
        final Matrix cov = new Matrix(2, 2);
        reading = new RangingReadingLocated2D<>(ap, 2.0, position, cov);

        // check
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getDistance(), 2.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        reading = new RangingReadingLocated2D<>(ap, 2.0, position,
                (Matrix) null);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 2.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated2D<>(null,
                    2.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, -1.0,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.0,
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.0,
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.0,
                    position, new Matrix(2, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, position, position covariance
        // and number of measurements.
        reading = new RangingReadingLocated2D<>(ap, 2.0, position, cov,
                8, 7);

        // check
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getDistance(), 2.0, 0.0);
        assertNull(reading.getDistanceStandardDeviation());
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated2D<>(null, 2.0,
                    position, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, -1.0, position,
                    cov, 8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.0, null,
                    cov, 8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.0, position,
                    new Matrix(1, 1), 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.0, position,
                    new Matrix(2, 1), 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.0, position,
                    cov, 0, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.0, position,
                    cov, 8, -1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, position, distance standard deviation and position
        // covariance
        reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                0.2, cov);

        // check
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                0.2, null);

        // check
        assertSame(reading.getPosition(), position);
        assertNull(reading.getPositionCovariance());
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);
        assertEquals(reading.getNumSuccessfulMeasurements(),
                RangingAndRssiReading.DEFAULT_NUM_MEASUREMENTS);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated2D<>(null, 2.5,
                    position, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, -1.0,
                    position, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5,
                    null, 0.2, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                    0.0, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                    0.2, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                    0.2, new Matrix(2, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(reading);

        // test constructor with access point, distance, position, distance standard
        // deviation, position covariance and number of measurements.
        reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                0.2, cov, 8,
                7);

        // check
        assertSame(reading.getPosition(), position);
        assertSame(reading.getPositionCovariance(), cov);
        assertEquals(reading.getDistance(), 2.5, 0.0);
        assertEquals(reading.getDistanceStandardDeviation(), 0.2, 0.0);
        assertSame(reading.getSource(), ap);
        assertEquals(reading.getType(), ReadingType.RANGING_READING);
        assertEquals(reading.getNumAttemptedMeasurements(), 8);
        assertEquals(reading.getNumSuccessfulMeasurements(), 7);

        // force IllegalArgumentException
        reading = null;
        try {
            reading = new RangingReadingLocated2D<>(null, 2.5,
                    position, 0.2, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, -1.0,
                    position, 0.2, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, null,
                    0.2, cov,
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                    0.0, cov, 8,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                    0.2, new Matrix(1, 1),
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                    0.2, new Matrix(2, 1),
                    8, 7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                    0.2, cov, 0,
                    7);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            reading = new RangingReadingLocated2D<>(ap, 2.5, position,
                    0.2, cov, 8,
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
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();

        final RangingReadingLocated2D<WifiAccessPoint> reading1 = new RangingReadingLocated2D<>(ap1,
                50.0, position);
        final RangingReadingLocated2D<WifiAccessPoint> reading2 = new RangingReadingLocated2D<>(ap1,
                50.0, position);
        final RangingReadingLocated2D<WifiAccessPoint> reading3 = new RangingReadingLocated2D<>(ap2,
                50.0, position);

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
        final RangingReadingLocated2D<WifiAccessPoint> reading1 =
                new RangingReadingLocated2D<>(ap, 2.5, position,
                        0.2, cov, 8,
                        7);

        // check
        assertSame(reading1.getPosition(), position);
        assertSame(reading1.getPositionCovariance(), cov);
        assertEquals(reading1.getDistance(), 2.5, 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(), 0.2, 0.0);
        assertSame(reading1.getSource(), ap);
        assertEquals(reading1.getType(), ReadingType.RANGING_READING);
        assertEquals(reading1.getNumAttemptedMeasurements(), 8);
        assertEquals(reading1.getNumSuccessfulMeasurements(), 7);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(reading1);
        final RangingReadingLocated2D<WifiAccessPoint> reading2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(reading1, reading2);
        assertEquals(reading1.getPosition(), reading2.getPosition());
        assertEquals(reading1.getPositionCovariance(),
                reading2.getPositionCovariance());
        assertEquals(reading1.getDistance(), reading2.getDistance(), 0.0);
        assertEquals(reading1.getDistanceStandardDeviation(),
                reading2.getDistanceStandardDeviation(), 0.0);
        assertEquals(reading1.getSource(), reading2.getSource());
        assertEquals(reading1.getType(), reading2.getType());
        assertEquals(reading1.getNumAttemptedMeasurements(),
                reading2.getNumAttemptedMeasurements());
        assertEquals(reading1.getNumSuccessfulMeasurements(),
                reading2.getNumSuccessfulMeasurements());
    }
}
