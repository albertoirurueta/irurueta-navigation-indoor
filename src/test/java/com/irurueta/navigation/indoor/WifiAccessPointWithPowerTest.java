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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class WifiAccessPointWithPowerTest {

    private static final String BSSID = "bssid";
    private static final String SSID = "ssid";
    private static final double FREQUENCY = 2.4e9;
    private static final double TRANSMITTED_POWER = -50.0;
    private static final double TRANSMITTED_POWER_STD = 0.5;
    private static final double PATH_LOSS_STD = 0.1;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    @Test
    public void testConstructor() {
        // test empty constructor
        WifiAccessPointWithPower ap = new WifiAccessPointWithPower();

        // check default values
        assertNull(ap.getBssid());
        assertNull(ap.getSsid());
        assertEquals(0.0, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // test with bssid, frequency and transmitted power
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, TRANSMITTED_POWER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, TRANSMITTED_POWER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test with bssid, frequency, ssid and transmitted power
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, SSID, TRANSMITTED_POWER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, SSID, TRANSMITTED_POWER);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test with bssid, frequency, transmitted power, transmitted power
        // standard deviation
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER, Double.valueOf(TRANSMITTED_POWER_STD));

        // heck default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, TRANSMITTED_POWER,
                    Double.valueOf(TRANSMITTED_POWER_STD));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, TRANSMITTED_POWER,
                    Double.valueOf(TRANSMITTED_POWER_STD));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER,
                    Double.valueOf(-TRANSMITTED_POWER_STD));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test with bssid, frequency, ssid, transmitted power and transmitted power
        // standard deviation
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test with bssid, frequency, transmitted power, transmitted power standard
        // deviation and path loss exponent
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER, pathLossExponent);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, TRANSMITTED_POWER, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, TRANSMITTED_POWER, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test with bssid, frequency, transmitted power, transmitted power standard
        // deviation and path loss exponent
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test constructor with bssid, frequency, tx power, tx power std
        // deviation and path-loss and path-loss std deviation
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, PATH_LOSS_STD);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(ap.getPathLossExponent(), pathLossExponent, 0.0);
        assertEquals(PATH_LOSS_STD, ap.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent, PATH_LOSS_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent, PATH_LOSS_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD,
                    pathLossExponent, PATH_LOSS_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent, -PATH_LOSS_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test constructor with bssid, frequency, ssid, tx power, tx power
        // std deviation, path loss, path-loss std deviation
        ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, PATH_LOSS_STD);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertEquals(PATH_LOSS_STD, ap.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPower(null, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent, PATH_LOSS_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, -FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent, PATH_LOSS_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD,
                    pathLossExponent, PATH_LOSS_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPointWithPower(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                    pathLossExponent, -PATH_LOSS_STD);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);
    }

    @Test
    public void testEquals() {
        final WifiAccessPointWithPower ap1 = new WifiAccessPointWithPower("bssid1", FREQUENCY,
                TRANSMITTED_POWER);
        final WifiAccessPointWithPower ap2 = new WifiAccessPointWithPower("bssid1", FREQUENCY,
                TRANSMITTED_POWER);
        final WifiAccessPointWithPower ap3 = new WifiAccessPointWithPower("bssid2", FREQUENCY,
                TRANSMITTED_POWER);

        // check
        //noinspection EqualsWithItself
        assertEquals(ap1, ap1);
        assertEquals(ap1, ap2);
        assertNotEquals(ap1, ap3);
    }

    @Test
    public void testHashCode() {
        final WifiAccessPointWithPower ap1 = new WifiAccessPointWithPower("bssid1", FREQUENCY,
                TRANSMITTED_POWER);
        final WifiAccessPointWithPower ap2 = new WifiAccessPointWithPower("bssid1", FREQUENCY,
                TRANSMITTED_POWER);
        final WifiAccessPointWithPower ap3 = new WifiAccessPointWithPower("bssid2", FREQUENCY,
                TRANSMITTED_POWER);

        // check
        assertEquals(ap1.hashCode(), ap2.hashCode());
        assertNotEquals(ap1.hashCode(), ap3.hashCode());
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

        final WifiAccessPointWithPower ap1 = new WifiAccessPointWithPower(BSSID,
                FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD);

        // check
        assertEquals(BSSID, ap1.getBssid());
        assertEquals(FREQUENCY, ap1.getFrequency(), 0.0);
        assertEquals(SSID, ap1.getSsid());
        assertEquals(TRANSMITTED_POWER, ap1.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap1.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(pathLossExponent, ap1.getPathLossExponent(), 0.0);
        assertEquals(PATH_LOSS_STD, ap1.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap1.getType());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(ap1);
        final WifiAccessPointWithPower ap2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(ap1, ap2);
        assertNotSame(ap1, ap2);
        assertEquals(ap1.getBssid(), ap2.getBssid());
        assertEquals(ap1.getFrequency(), ap2.getFrequency(), 0.0);
        assertEquals(ap1.getSsid(), ap2.getSsid());
        assertEquals(ap1.getTransmittedPower(), ap2.getTransmittedPower(), 0.0);
        assertEquals(ap1.getTransmittedPowerStandardDeviation(), ap2.getTransmittedPowerStandardDeviation(),
                0.0);
        assertEquals(ap1.getPathLossExponent(), ap2.getPathLossExponent(), 0.0);
        assertEquals(ap1.getPathLossExponentStandardDeviation(), ap2.getPathLossExponentStandardDeviation(),
                0.0);
        assertEquals(ap1.getType(), ap2.getType());
    }
}
