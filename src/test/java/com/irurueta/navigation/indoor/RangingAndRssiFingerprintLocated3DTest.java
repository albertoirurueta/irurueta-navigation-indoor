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
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RangingAndRssiFingerprintLocated3DTest {

    @Test
    void testConstructor() throws AlgebraException {
        // empty constructor
        var fingerprint = new RangingAndRssiFingerprintLocated3D<RadioSource, RangingAndRssiReading<RadioSource>>();

        // check
        assertNotNull(fingerprint.getReadings());
        assertTrue(fingerprint.getReadings().isEmpty());
        assertNull(fingerprint.getPosition());
        assertNull(fingerprint.getPositionCovariance());

        // constructor with readings and position
        final var readings = new ArrayList<RangingAndRssiReading<RadioSource>>();
        final var position = new InhomogeneousPoint3D();
        fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position);

        // check
        assertEquals(readings, fingerprint.getReadings());
        assertNotSame(readings, fingerprint.getReadings());
        assertSame(position, fingerprint.getPosition());
        assertNull(fingerprint.getPositionCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiFingerprintLocated3D<>(null,
                position));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiFingerprintLocated3D<>(readings,
                null));

        // constructor with readings, position and covariance
        final var cov = new Matrix(3, 3);
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
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiFingerprintLocated3D<>(null,
                position, cov));
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiFingerprintLocated3D<>(readings,
                null, cov));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RangingAndRssiFingerprintLocated3D<>(readings, position,
                m));
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var readings = new ArrayList<RangingAndRssiReading<RadioSource>>();
        final var position = new InhomogeneousPoint3D();
        final var cov = new Matrix(3, 3);
        final var fingerprint1 = new RangingAndRssiFingerprintLocated3D<>(readings, position, cov);

        // check
        assertEquals(readings, fingerprint1.getReadings());
        assertNotSame(readings, fingerprint1.getReadings());
        assertSame(position, fingerprint1.getPosition());
        assertSame(cov, fingerprint1.getPositionCovariance());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(fingerprint1);
        final var fingerprint2 = SerializationHelper
                .<RangingAndRssiFingerprintLocated3D<RadioSource, RangingAndRssiReading<RadioSource>>>deserialize(
                        bytes);

        // check
        assertNotSame(fingerprint1, fingerprint2);
        assertEquals(fingerprint1.getReadings(), fingerprint2.getReadings());
        assertEquals(fingerprint1.getPosition(), fingerprint2.getPosition());
        assertEquals(fingerprint1.getPositionCovariance(), fingerprint2.getPositionCovariance());
    }
}
