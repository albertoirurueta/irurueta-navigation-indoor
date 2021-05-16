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
import com.irurueta.geometry.InhomogeneousPoint3D;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RssiFingerprintLocated3DTest {

    @Test
    public void testConstructor() throws WrongSizeException {
        // test empty constructor
        RssiFingerprintLocated3D<WifiAccessPoint, RssiReading<WifiAccessPoint>> f =
                new RssiFingerprintLocated3D<>();

        // check default values
        assertTrue(f.getReadings().isEmpty());
        assertNull(f.getPosition());
        assertNull(f.getPositionCovariance());

        // test with readings and position
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        f = new RssiFingerprintLocated3D<>(readings, position);

        // check
        assertSame(f.getReadings(), readings);
        assertSame(f.getPosition(), position);
        assertNull(f.getPositionCovariance());

        // force IllegalArgumentException
        f = null;
        try {
            f = new RssiFingerprintLocated3D<>(null, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            f = new RssiFingerprintLocated3D<>(readings, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(f);

        // test with readings, position and covariance
        final Matrix cov = new Matrix(3, 3);
        f = new RssiFingerprintLocated3D<>(readings, position, cov);

        // check
        assertSame(f.getReadings(), readings);
        assertSame(f.getPosition(), position);
        assertSame(f.getPositionCovariance(), cov);

        f = null;
        try {
            f = new RssiFingerprintLocated3D<>(null, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            f = new RssiFingerprintLocated3D<>(readings, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            f = new RssiFingerprintLocated3D<>(readings, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(f);
    }
}
