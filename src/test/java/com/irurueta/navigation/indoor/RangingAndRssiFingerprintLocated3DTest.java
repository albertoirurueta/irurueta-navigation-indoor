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
import com.irurueta.geometry.Point3D;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RangingAndRssiFingerprintLocated3DTest {

    @Test
    public void testConstructor() throws AlgebraException {
        // empty constructor
        RangingAndRssiFingerprintLocated3D<RadioSource, RangingAndRssiReading<RadioSource>> fingerprint =
                new RangingAndRssiFingerprintLocated3D<>();

        // check
        assertNotNull(fingerprint.getReadings());
        assertTrue(fingerprint.getReadings().isEmpty());
        assertNull(fingerprint.getPosition());
        assertNull(fingerprint.getPositionCovariance());

        // constructor with readings and position
        final List<RangingAndRssiReading<RadioSource>> readings = new ArrayList<>();
        final Point3D position = new InhomogeneousPoint3D();
        fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position);

        // check
        assertEquals(readings, fingerprint.getReadings());
        assertNotSame(readings, fingerprint.getReadings());
        assertSame(position, fingerprint.getPosition());
        assertNull(fingerprint.getPositionCovariance());

        // force IllegalArgumentException
        fingerprint = null;
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(null, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(fingerprint);

        // constructor with readings, position and covariance
        final Matrix cov = new Matrix(3, 3);
        fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position, cov);

        // check
        assertEquals(readings, fingerprint.getReadings());
        assertNotSame(readings, fingerprint.getReadings());
        assertSame(position, fingerprint.getPosition());
        assertSame(cov, fingerprint.getPositionCovariance());

        fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position, null);

        // check
        assertEquals(readings, fingerprint.getReadings());
        assertNotSame(readings, fingerprint.getReadings());
        assertSame(position, fingerprint.getPosition());
        assertNull(fingerprint.getPositionCovariance());

        // force IllegalArgumentException
        fingerprint = null;
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(null, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(fingerprint);
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final List<RangingAndRssiReading<RadioSource>> readings = new ArrayList<>();
        final Point3D position = new InhomogeneousPoint3D();
        final Matrix cov = new Matrix(3, 3);
        final RangingAndRssiFingerprintLocated3D<RadioSource, RangingAndRssiReading<RadioSource>> fingerprint1 =
                new RangingAndRssiFingerprintLocated3D<>(readings, position, cov);

        // check
        assertEquals(readings, fingerprint1.getReadings());
        assertNotSame(readings, fingerprint1.getReadings());
        assertSame(position, fingerprint1.getPosition());
        assertSame(cov, fingerprint1.getPositionCovariance());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(fingerprint1);
        final RangingAndRssiFingerprintLocated3D<RadioSource, RangingAndRssiReading<RadioSource>> fingerprint2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(fingerprint1, fingerprint2);
        assertEquals(fingerprint1.getReadings(), fingerprint2.getReadings());
        assertEquals(fingerprint1.getPosition(), fingerprint2.getPosition());
        assertEquals(fingerprint1.getPositionCovariance(), fingerprint2.getPositionCovariance());
    }
}
