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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RssiFingerprintLocated2DTest {

    @Test
    void testConstructor() throws WrongSizeException {
        // test empty constructor
        var f = new RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>();

        // check default values
        assertTrue(f.getReadings().isEmpty());
        assertNull(f.getPosition());
        assertNull(f.getPositionCovariance());

        // test with readings and position
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        final var position = new InhomogeneousPoint2D();
        f = new RssiFingerprintLocated2D<>(readings, position);

        // check
        assertEquals(readings, f.getReadings());
        assertNotSame(readings, f.getReadings());
        assertSame(position, f.getPosition());
        assertNull(f.getPositionCovariance());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RssiFingerprintLocated2D<>(null, position));
        assertThrows(IllegalArgumentException.class, () -> new RssiFingerprintLocated2D<>(readings, null));

        // test with readings, position and covariance
        final var cov = new Matrix(2, 2);
        f = new RssiFingerprintLocated2D<>(readings, position, cov);

        // check
        assertEquals(readings, f.getReadings());
        assertNotSame(readings, f.getReadings());
        assertSame(position, f.getPosition());
        assertSame(cov, f.getPositionCovariance());

        assertThrows(IllegalArgumentException.class, () -> new RssiFingerprintLocated2D<>(null, position,
                cov));
        assertThrows(IllegalArgumentException.class, () -> new RssiFingerprintLocated2D<>(readings, null, cov));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RssiFingerprintLocated2D<>(readings, position, m));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var readings = new ArrayList<RssiReading<WifiAccessPoint>>();
        final var position = new InhomogeneousPoint2D();
        final var f1 = new RssiFingerprintLocated2D<>(readings, position);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(f1);
        final var f2 = SerializationHelper
                .<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>>deserialize(bytes);

        // check
        assertNotSame(f1, f2);
        assertEquals(f1.getReadings(), f2.getReadings());
        assertEquals(f1.getPosition(), f2.getPosition());
    }
}
