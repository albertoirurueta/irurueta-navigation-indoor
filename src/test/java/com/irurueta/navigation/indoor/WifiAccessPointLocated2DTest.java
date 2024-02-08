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
import com.irurueta.geometry.Point2D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class WifiAccessPointLocated2DTest {

    private static final String BSSID = "bssid";
    private static final String SSID = "ssid";
    private static final double FREQUENCY = 2.4e9;
    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    @Test
    public void testConstructor() throws AlgebraException {
        // test empty constructor
        WifiAccessPointLocated2D ap = new WifiAccessPointLocated2D();

        // check default values
        assertNull(ap.getBssid());
        assertEquals(0.0, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertNull(ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // test constructor with bssid, frequency and position
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointLocated2D(null, FREQUENCY, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, -FREQUENCY, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test constructor with bssid, frequency, ssid and position
        ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, SSID, position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointLocated2D(null, FREQUENCY, SSID, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, -FREQUENCY, SSID, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, SSID, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test constructor with bssid, frequency, position and position covariance
        final Matrix cov = new Matrix(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
        ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, position, cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, position, null);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointLocated2D(null, FREQUENCY, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, -FREQUENCY, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test constructor with bssid, frequency, ssid, position and
        // position covariance
        ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, SSID, position, cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, SSID, position, null);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointLocated2D(null, FREQUENCY, SSID, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, -FREQUENCY, SSID, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, SSID, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointLocated2D(BSSID, FREQUENCY, SSID, position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final WifiAccessPointLocated2D ap1 = new WifiAccessPointLocated2D("bssid1", FREQUENCY, position);
        final WifiAccessPointLocated2D ap2 = new WifiAccessPointLocated2D("bssid1", FREQUENCY, position);
        final WifiAccessPointLocated2D ap3 = new WifiAccessPointLocated2D("bssid2", FREQUENCY, position);

        // check
        //noinspection EqualsWithItself
        assertEquals(ap1, ap1);
        assertEquals(ap1, ap2);
        assertNotEquals(ap1, ap3);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final WifiAccessPointLocated2D ap1 = new WifiAccessPointLocated2D("bssid1", FREQUENCY, position);
        final WifiAccessPointLocated2D ap2 = new WifiAccessPointLocated2D("bssid1", FREQUENCY, position);
        final WifiAccessPointLocated2D ap3 = new WifiAccessPointLocated2D("bssid2", FREQUENCY, position);

        // check
        assertEquals(ap1.hashCode(), ap2.hashCode());
        assertNotEquals(ap1.hashCode(), ap3.hashCode());
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final Matrix cov = new Matrix(Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
        final WifiAccessPointLocated2D ap1 = new WifiAccessPointLocated2D(BSSID, FREQUENCY, SSID, position, cov);

        // check
        assertEquals(BSSID, ap1.getBssid());
        assertEquals(FREQUENCY, ap1.getFrequency(), 0.0);
        assertEquals(SSID, ap1.getSsid());
        assertSame(position, ap1.getPosition());
        assertSame(cov, ap1.getPositionCovariance());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap1.getType());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(ap1);
        final WifiAccessPointLocated2D ap2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(ap1, ap2);
        assertNotSame(ap1, ap2);
        assertEquals(ap1.getBssid(), ap2.getBssid());
        assertEquals(ap1.getFrequency(), ap2.getFrequency(), 0.0);
        assertEquals(ap1.getSsid(), ap2.getSsid());
        assertEquals(ap1.getPosition(), ap2.getPosition());
        assertEquals(ap1.getPositionCovariance(), ap2.getPositionCovariance());
        assertEquals(ap1.getType(), ap2.getType());
    }
}
