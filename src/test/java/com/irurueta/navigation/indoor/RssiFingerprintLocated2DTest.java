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
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RssiFingerprintLocated2DTest {

    @Test
    public void testConstructor() throws WrongSizeException {
        // test empty constructor
        RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>> f = new RssiFingerprintLocated2D<>();

        // check default values
        assertTrue(f.getReadings().isEmpty());
        assertNull(f.getPosition());
        assertNull(f.getPositionCovariance());

        // test with readings and position
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        f = new RssiFingerprintLocated2D<>(readings, position);

        // check
        assertEquals(readings, f.getReadings());
        assertNotSame(readings, f.getReadings());
        assertSame(position, f.getPosition());
        assertNull(f.getPositionCovariance());

        // force IllegalArgumentException
        f = null;
        try {
            f = new RssiFingerprintLocated2D<>(null, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            f = new RssiFingerprintLocated2D<>(readings, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(f);

        // test with readings, position and covariance
        final Matrix cov = new Matrix(2, 2);
        f = new RssiFingerprintLocated2D<>(readings, position, cov);

        // check
        assertEquals(readings, f.getReadings());
        assertNotSame(readings, f.getReadings());
        assertSame(position, f.getPosition());
        assertSame(cov, f.getPositionCovariance());

        f = null;
        try {
            f = new RssiFingerprintLocated2D<>(null, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            f = new RssiFingerprintLocated2D<>(readings, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            f = new RssiFingerprintLocated2D<>(readings, position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(f);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        final RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>> f1 =
                new RssiFingerprintLocated2D<>(readings, position);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(f1);
        final RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>> f2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(f1, f2);
        assertEquals(f1.getReadings(), f2.getReadings());
        assertEquals(f1.getPosition(), f2.getPosition());
    }
}
