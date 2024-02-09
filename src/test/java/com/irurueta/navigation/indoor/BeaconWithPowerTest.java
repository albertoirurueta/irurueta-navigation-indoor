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
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class BeaconWithPowerTest {

    @Test
    public void testConstructor() {
        // test empty constructor
        BeaconWithPower b = new BeaconWithPower();

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertNull(b.getIdentifiers());
        assertEquals(0.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // test constructor with identifiers and transmitted power
        final List<BeaconIdentifier> identifiers = new ArrayList<>();
        b = new BeaconWithPower(identifiers, -50.0);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        //noinspection ConstantConditions
        assertNull(b);

        // test constructor with all data
        b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                2, 3, "name");

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, "address", 1,
                    2, 3, "name");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power and
        // transmitted power standard deviation
        b = new BeaconWithPower(identifiers, -50.0, Double.valueOf(1.0));

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPower(identifiers, -50.0, null);

        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, Double.valueOf(1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, Double.valueOf(-1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data and transmitted power standard deviation
        b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                2, 3, "name", Double.valueOf(1.0));

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                2, 3, "name", null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, "address", 1,
                    2, 3, "name", Double.valueOf(1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                    2, 3, "name", Double.valueOf(-1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data and frequency
        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, "address",
                1, 2, 3, "name");

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 5.0e9, "address",
                    1, 2, 3, "name");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, -5.0e9, "address",
                    1, 2, 3, "name");
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power,
        // transmitted power standard deviation and frequency
        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, Double.valueOf(1.0));

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 5.0e9, Double.valueOf(1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, -5.0e9, Double.valueOf(1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, 5.0e9, Double.valueOf(-1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data and transmitted power standard deviation
        b = new BeaconWithPower(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                Double.valueOf(1.0));

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPower(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 5.0e9,
                    "address", 1, 2, 3, "name",
                    Double.valueOf(1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, -5.0e9,
                    "address", 1, 2, 3, "name",
                    Double.valueOf(1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, 5.0e9,
                    "address", 1, 2, 3, "name",
                    Double.valueOf(-1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power and path loss
        b = new BeaconWithPower(identifiers, -50.0, 1.6);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 1.6);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, all data and path loss
        b = new BeaconWithPower(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, "address",
                    1, 2, 3, "name", 1.6);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power, tx power std and path loss
        b = new BeaconWithPower(identifiers, -50.0, Double.valueOf(1.0), 1.6);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPower(identifiers, -50.0, null,
                1.6);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, Double.valueOf(1.0), 1.6);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, Double.valueOf(-1.0), 1.6);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data and path loss
        b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                2, 3, "name", 1.6,
                1.0);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                2, 3, "name", 1.6,
                null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, "address", 1,
                    2, 3, "name", 1.6,
                    1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                    2, 3, "name", 1.6,
                    -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, frequency and path loss
        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, "address",
                1, 2, 3, "name", 1.6);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 5.0e9,
                    "address", 1, 2, 3, "name",
                    1.6);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, frequency, path loss and tx power std
        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, 1.6,
                Double.valueOf(1.0));

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, 1.6,
                null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, 5.0e9, 1.6,
                    Double.valueOf(-1.0));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, frequency and path loss
        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, "address",
                1, 2, 3, "name", 1.6,
                1.0);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, "address",
                1, 2, 3, "name", 1.6,
                null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 5.0e9, "address",
                    1, 2, 3, "name", 1.6,
                    1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, 5.0e9, "address",
                    1, 2, 3, "name", 1.6,
                    -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power and std deviation, path loss and std deviation
        b = new BeaconWithPower(identifiers, -50.0, Double.valueOf(1.0), 1.6,
                Double.valueOf(0.1));

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, Double.valueOf(1.0), 1.6,
                    Double.valueOf(0.1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, Double.valueOf(-1.0), 1.6,
                    Double.valueOf(0.1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, Double.valueOf(1.0), 1.6,
                    Double.valueOf(-0.1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, path loss and std deviations
        b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                2, 3, "name", 1.6,
                1.0, 0.1);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, "address", 1,
                    2, 3, "name", 1.6,
                    1.0, 0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                    2, 3, "name", 1.6,
                    -1.0, 0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, "address", 1,
                    2, 3, "name", 1.6,
                    1.0, -0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, frequency, path loss and std deviations
        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, 1.6,
                1.0, 0.1);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(0, b.getBeaconTypeCode());
        assertNull(b.getBluetoothName());
        assertEquals(0, b.getManufacturer());
        assertEquals(-1, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 5.0e9, 1.6,
                    1.0, 0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, 5.0e9, 1.6,
                    -1.0, 0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, 5.0e9, 1.6,
                    1.0, -0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, frequency, path loss and std deviations
        b = new BeaconWithPower(identifiers, -50.0, 5.0e9, "address",
                1, 2, 3, "name", 1.6,
                1.0, 0.1);

        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPower(null, -50.0, 5.0e9, "address",
                    1, 2, 3, "name", 1.6,
                    1.0, 0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, 5.0e9, "address",
                    1, 2, 3, "name", 1.6,
                    -1.0, 0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPower(identifiers, -50.0, 5.0e9, "address",
                    1, 2, 3, "name", 1.6,
                    1.0, -0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final long value1 = randomizer.nextLong();
        final BeaconIdentifier id1 = BeaconIdentifier.fromLong(value1, Long.SIZE / Byte.SIZE);

        final long value2 = randomizer.nextLong();
        final BeaconIdentifier id2 = BeaconIdentifier.fromLong(value2, Long.SIZE / Byte.SIZE);

        final long value3 = randomizer.nextLong();
        final BeaconIdentifier id3 = BeaconIdentifier.fromLong(value3, Long.SIZE / Byte.SIZE);

        final List<BeaconIdentifier> identifiers1 = new ArrayList<>();
        identifiers1.add(id1);
        identifiers1.add(id2);
        identifiers1.add(id3);

        final long value4 = value1 + 1;
        final BeaconIdentifier id4 = BeaconIdentifier.fromLong(value4, Long.SIZE / Byte.SIZE);

        final long value5 = value2 + 1;
        final BeaconIdentifier id5 = BeaconIdentifier.fromLong(value5, Long.SIZE / Byte.SIZE);

        final long value6 = value3 + 1;
        final BeaconIdentifier id6 = BeaconIdentifier.fromLong(value6, Long.SIZE / Byte.SIZE);

        final List<BeaconIdentifier> identifiers2 = new ArrayList<>();
        identifiers2.add(id4);
        identifiers2.add(id5);
        identifiers2.add(id6);

        final BeaconWithPower b1 = new BeaconWithPower(identifiers1, -60.0, 1.0);
        final BeaconWithPower b2 = new BeaconWithPower(identifiers1, -50.0, 1.0);
        final BeaconWithPower b3 = new BeaconWithPower(identifiers2, -60.0, 1.0);

        // check
        //noinspection EqualsWithItself
        assertEquals(b1, b1);
        assertEquals(b1, b2);
        assertNotEquals(b1, b3);

        assertNotEquals(new Object(), b1);

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final long value1 = randomizer.nextLong();
        final BeaconIdentifier id1 = BeaconIdentifier.fromLong(value1, Long.SIZE / Byte.SIZE);

        final long value2 = randomizer.nextLong();
        final BeaconIdentifier id2 = BeaconIdentifier.fromLong(value2, Long.SIZE / Byte.SIZE);

        final long value3 = randomizer.nextLong();
        final BeaconIdentifier id3 = BeaconIdentifier.fromLong(value3, Long.SIZE / Byte.SIZE);

        final List<BeaconIdentifier> identifiers = new ArrayList<>();
        identifiers.add(id1);
        identifiers.add(id2);
        identifiers.add(id3);

        final BeaconWithPower b1 = new BeaconWithPower(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, 1.0, 0.1);

        assertEquals(b1.getId1(), id1);
        assertEquals(b1.getId2(), id2);
        assertEquals(b1.getId3(), id3);
        assertEquals(identifiers, b1.getIdentifiers());
        assertEquals(-50.0, b1.getTransmittedPower(), 0.0);
        assertEquals("address", b1.getBluetoothAddress());
        assertEquals(1, b1.getBeaconTypeCode());
        assertEquals("name", b1.getBluetoothName());
        assertEquals(2, b1.getManufacturer());
        assertEquals(3, b1.getServiceUuid());
        assertEquals(1.0, b1.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(1.6, b1.getPathLossExponent(), 0.0);
        assertEquals(0.1, b1.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b1.getType());
        assertEquals(5.0e9, b1.getFrequency(), 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(b1);
        final BeaconWithPower b2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(b1.getId1(), b2.getId1());
        assertEquals(b1.getId2(), b2.getId2());
        assertEquals(b1.getId3(), b2.getId3());
        assertEquals(b1.getIdentifiers(), b2.getIdentifiers());
        assertEquals(b1.getTransmittedPower(), b2.getTransmittedPower(), 0.0);
        assertEquals(b1.getBluetoothAddress(), b2.getBluetoothAddress());
        assertEquals(b1.getBeaconTypeCode(), b2.getBeaconTypeCode());
        assertEquals(b1.getBluetoothName(), b2.getBluetoothName());
        assertEquals(b1.getManufacturer(), b2.getManufacturer());
        assertEquals(b1.getServiceUuid(), b2.getServiceUuid());
        assertEquals(b1.getTransmittedPowerStandardDeviation(), b2.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(b1.getPathLossExponent(), b2.getPathLossExponent(), 0.0);
        assertEquals(b1.getPathLossExponentStandardDeviation(), b2.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(b1.getType(), b2.getType());
        assertEquals(b1.getFrequency(), b2.getFrequency(), 0.0);
    }
}
