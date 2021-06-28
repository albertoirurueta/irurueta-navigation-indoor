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
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RangingFingerprintLocated2DTest {

    @Test
    public void testConstructor() throws AlgebraException {
        // empty constructor
        RangingFingerprintLocated2D<RadioSource, RangingReading<RadioSource>> fingerprint =
                new RangingFingerprintLocated2D<>();

        // check
        assertNotNull(fingerprint.getReadings());
        assertTrue(fingerprint.getReadings().isEmpty());
        assertNull(fingerprint.getPosition());
        assertNull(fingerprint.getPositionCovariance());

        // constructor with readings and position
        final List<RangingReading<RadioSource>> readings = new ArrayList<>();
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        fingerprint = new RangingFingerprintLocated2D<>(readings, position);

        // check
        assertEquals(fingerprint.getReadings(), readings);
        assertNotSame(fingerprint.getReadings(), readings);
        assertSame(fingerprint.getPosition(), position);
        assertNull(fingerprint.getPositionCovariance());

        // force IllegalArgumentException
        fingerprint = null;
        try {
            fingerprint = new RangingFingerprintLocated2D<>(null, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fingerprint = new RangingFingerprintLocated2D<>(readings, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(fingerprint);

        // constructor with readings, position and position covariance
        final Matrix cov = new Matrix(2, 2);
        fingerprint = new RangingFingerprintLocated2D<>(readings, position, cov);

        // check
        assertEquals(fingerprint.getReadings(), readings);
        assertNotSame(fingerprint.getReadings(), readings);
        assertSame(fingerprint.getPosition(), position);
        assertSame(fingerprint.getPositionCovariance(), cov);

        fingerprint = new RangingFingerprintLocated2D<>(readings, position, null);

        // check
        assertEquals(fingerprint.getReadings(), readings);
        assertNotSame(fingerprint.getReadings(), readings);
        assertSame(fingerprint.getPosition(), position);
        assertNull(fingerprint.getPositionCovariance());

        // force IllegalArgumentException
        fingerprint = null;
        try {
            fingerprint = new RangingFingerprintLocated2D<>(null,
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fingerprint = new RangingFingerprintLocated2D<>(readings,
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            fingerprint = new RangingFingerprintLocated2D<>(readings,
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(fingerprint);
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final List<RangingReading<RadioSource>> readings = new ArrayList<>();
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        final Matrix cov = new Matrix(2, 2);
        final RangingFingerprintLocated2D<RadioSource, RangingReading<RadioSource>> fingerprint1 =
                new RangingFingerprintLocated2D<>(readings, position, cov);

        // check
        assertEquals(fingerprint1.getReadings(), readings);
        assertNotSame(fingerprint1.getReadings(), readings);
        assertSame(fingerprint1.getPosition(), position);
        assertSame(fingerprint1.getPositionCovariance(), cov);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(fingerprint1);
        final RangingFingerprintLocated2D<RadioSource, RangingReading<RadioSource>> fingerprint2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(fingerprint1, fingerprint2);
        assertEquals(fingerprint1.getReadings(), fingerprint2.getReadings());
        assertEquals(fingerprint1.getPosition(), fingerprint2.getPosition());
        assertEquals(fingerprint1.getPositionCovariance(), fingerprint2.getPositionCovariance());
    }
}
