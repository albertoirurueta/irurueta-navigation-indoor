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

import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class WifiAccessPointTest {

    private static final String BSSID = "bssid";
    private static final String SSID = "ssid";
    private static final double FREQUENCY = 2.4e9;

    @Test
    public void testConstructor() {
        // test empty constructor
        WifiAccessPoint ap = new WifiAccessPoint();

        // check default values
        assertNull(ap.getBssid());
        assertEquals(0.0, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // test constructor with BSSID
        ap = new WifiAccessPoint(BSSID, FREQUENCY);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // Force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPoint(null, FREQUENCY);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPoint(BSSID, -FREQUENCY);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);

        // test constructor with BSSID and SSID
        ap = new WifiAccessPoint(BSSID, FREQUENCY, SSID);

        // check default value
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // Force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPoint(null, FREQUENCY, SSID);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ap = new WifiAccessPoint(BSSID, -FREQUENCY, SSID);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(ap);
    }

    @Test
    public void testEquals() {
        final WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final WifiAccessPoint ap2 = new WifiAccessPoint("bssid1", FREQUENCY);
        final WifiAccessPoint ap3 = new WifiAccessPoint("bssid2", FREQUENCY);

        // check
        //noinspection EqualsWithItself
        assertEquals(ap1, ap1);
        assertEquals(ap1, ap2);
        assertNotEquals(ap1, ap3);

        assertNotEquals(null, ap1);
        assertNotEquals(new Object(), ap1);
    }

    @Test
    public void testHashCode() {
        final WifiAccessPoint ap1 = new WifiAccessPoint("bssid1", FREQUENCY);
        final WifiAccessPoint ap2 = new WifiAccessPoint("bssid1", FREQUENCY);
        final WifiAccessPoint ap3 = new WifiAccessPoint("bssid2", FREQUENCY);

        // check
        assertEquals(ap1.hashCode(), ap2.hashCode());
        assertNotEquals(ap1.hashCode(), ap3.hashCode());
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        // test constructor with BSSID and SSID
        final WifiAccessPoint ap1 = new WifiAccessPoint(BSSID, FREQUENCY, SSID);

        // check
        assertEquals(BSSID, ap1.getBssid());
        assertEquals(FREQUENCY, ap1.getFrequency(), 0.0);
        assertEquals(SSID, ap1.getSsid());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap1.getType());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(ap1);
        final WifiAccessPoint ap2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(ap1, ap2);
        assertNotSame(ap1, ap2);
        assertEquals(ap1.getBssid(), ap2.getBssid());
        assertEquals(ap1.getFrequency(), ap2.getFrequency(), 0.0);
        assertEquals(ap1.getSsid(), ap2.getSsid());
        assertEquals(ap1.getType(), ap2.getType());
    }
}
