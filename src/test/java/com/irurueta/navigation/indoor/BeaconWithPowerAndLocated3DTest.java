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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class BeaconWithPowerAndLocated3DTest {

    @Test
    public void testConstructor() throws AlgebraException {
        // test empty constructor
        BeaconWithPowerAndLocated3D b = new BeaconWithPowerAndLocated3D();

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertNull(b.getIdentifiers());
        assertEquals(b.getTransmittedPower(), 0.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertNull(b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // test constructor with identifiers, transmitted power and position
        final List<BeaconIdentifier> identifiers = new ArrayList<>();
        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power, transmitted
        // power standard deviation and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                Double.valueOf(1.0), position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                null, position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, transmitted power standard deviation
        // and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", Double.valueOf(1.0),
                position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", null,
                position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(-1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power, position and
        // position covariance
        final Matrix cov = new Matrix(3, 3);
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                position, null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, position and position covariance
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    "address", 1,
                    2, 3, "name",
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power, transmitted
        // power standard deviation, position and position covariance
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                Double.valueOf(1.0), position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                null, position, null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    1.0, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, transmitted power standard deviation,
        // position and position covariance
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", Double.valueOf(1.0),
                position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", null,
                position, null);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(-1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    "address", 1, 2,
                    3, "name", Double.valueOf(1.0),
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, position and frequency
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power, transmitted
        // power standard deviation and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0, 5.0e9,
                Double.valueOf(1.0), position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0, 5.0e9,
                null, position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, Double.valueOf(-1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, Double.valueOf(1.0),
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, transmitted power standard deviation
        // and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", Double.valueOf(1.0),
                position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", null,
                position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(-1.0),
                    position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, position and position covariance
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, transmitted power, transmitted
        // power standard deviation, position and position covariance
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, Double.valueOf(1.0), position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, null, position,
                null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, Double.valueOf(-1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, Double.valueOf(1.0),
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, Double.valueOf(1.0), position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, transmitted power standard deviation,
        // position and position covariance
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", Double.valueOf(1.0),
                position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", null,
                position, null);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(),
                BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(-1.0),
                    position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", Double.valueOf(1.0),
                    null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", Double.valueOf(1.0),
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, path loss and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                1.6, position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 1.6, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, path-loss and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6, position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, std dev, path loss and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                Double.valueOf(1.0), 1.6, position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                null, 1.6,
                position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, Double.valueOf(1.0),
                    1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, tx power, std, position and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                1.0, position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                null, position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx, power, position, position covariance
        // and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                1.6, position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                1.6, position, null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 1.6, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    1.6, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, position, position covariance and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    "address", 1,
                    2, 3, "name",
                    1.6, position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, tx power std deviation,
        // position, position covariance and path loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                Double.valueOf(1.0), 1.6, position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                null, 1.6,
                position, null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, Double.valueOf(1.0),
                    1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    Double.valueOf(1.0), 1.6, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, tx power std deviation, position,
        // position covariance and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                1.0, position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                null, position,
                null);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    "address", 1, 2,
                    3, "name", 1.6,
                    1.0, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, position, frequency and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", 1.6,
                position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, tx power std deviation,
        // position and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, 1.6, Double.valueOf(1.0),
                position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, 1.6,
                null, position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, 1.6,
                    Double.valueOf(1.0), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(-1.0), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0), null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, tx power std deviation, position and
        // path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", 1.6,
                1.0, position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0, 5.0e9,
                "address", 1, 2,
                3, "name", 1.6,
                null, position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, position, position covariance and path loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", 1.6,
                position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, tx power std deviation,
        // position, position covariance and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, 1.6, Double.valueOf(1.0),
                position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, 1.6,
                null, position,
                null);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, 1.6,
                    Double.valueOf(1.0), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(-1.0), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    Double.valueOf(1.0), null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, 1.6, Double.valueOf(1.0),
                    position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, tx power std deviation, position,
        // position covariance and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", 1.6,
                1.0, position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", 1.6,
                null, position,
                null);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0, position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, path loss, path loss
        // std deviation and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                Double.valueOf(1.0), 1.6,
                Double.valueOf(0.1), position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(),
                0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    1.6, Double.valueOf(0.1), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(-0.1), position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, path loss, path loss std deviation and
        // position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                1.0,
                0.1, position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(), 1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    -0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    0.1, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, std dev, path loss,
        // path loss std deviation and position
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                Double.valueOf(1.0), 1.6, Double.valueOf(0.1),
                position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                null, 1.6,
                null, position, cov);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(-1.0),
                    1.6, Double.valueOf(0.1), position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), null,
                    cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, Double.valueOf(1.0),
                    1.6, Double.valueOf(0.1), position,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, tx power, stds, position and path-loss
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                1.0,
                0.1, position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                "address", 1, 2,
                3, "name", 1.6,
                null,
                null, position, cov);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), Beacon.DEFAULT_FREQUENCY, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    -1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    -0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, "address",
                    1, 2, 3,
                    "name", 1.6,
                    1.0,
                    0.1, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx, power, position, path loss and
        // path-loss std deviation
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, 1.6,
                1.0,
                0.1, position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(), 1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null,
                    -50.0, 5.0e9, 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, -5.0e9, 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    -1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    1.0,
                    -0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers,
                    -50.0, 5.0e9, 1.6,
                    1.0,
                    0.1, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, position, path loss and path-loss std
        // deviation
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1, 2,
                3, "name", 1.6,
                1.0,
                0.1, position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(), 1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertNull(b.getPositionCovariance());
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    -5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    -1.0,
                    0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0,
                    -0.1, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1, 2,
                    3, "name", 1.6,
                    1.0,
                    0.1, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with identifiers, tx power, tx power std deviation,
        // position, position covariance, path-loss and path-loss std deviation
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, 1.6,
                1.0,
                0.1, position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertNull(b.getBluetoothAddress());
        assertEquals(b.getBeaconTypeCode(), 0);
        assertNull(b.getBluetoothName());
        assertEquals(b.getManufacturer(), 0);
        assertEquals(b.getServiceUuid(), -1);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null, -50.0,
                    5.0e9, 1.6,
                    1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    -5.0e9, 1.6,
                    1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, 1.6,
                    -1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, 1.6,
                    1.0,
                    -0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, 1.6,
                    1.0,
                    0.1, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);

        // test constructor with all data, tx power std deviation, position,
        // position covariance, path-loss and path-loss std deviation
        b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1,
                2, 3, "name",
                1.6, 1.0,
                0.1, position, cov);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(b.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b.getBluetoothAddress(), "address");
        assertEquals(b.getBeaconTypeCode(), 1);
        assertEquals(b.getBluetoothName(), "name");
        assertEquals(b.getManufacturer(), 2);
        assertEquals(b.getServiceUuid(), 3);
        assertEquals(b.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b.getPosition(), position);
        assertSame(b.getPositionCovariance(), cov);
        assertEquals(b.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b.getType(), RadioSourceType.BEACON);
        assertEquals(b.getFrequency(), 5.0e9, 0.0);

        // force IllegalArgumentException
        b = null;
        try {
            b = new BeaconWithPowerAndLocated3D(null, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, 1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    -5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, 1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, -1.0,
                    0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, 1.0,
                    -0.1, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            b = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                    5.0e9, "address", 1,
                    2, 3, "name",
                    1.6, 1.0,
                    0.1, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(b);
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final long value1 = randomizer.nextLong();
        final BeaconIdentifier id1 = BeaconIdentifier.fromLong(value1,
                Long.SIZE / Byte.SIZE);

        final long value2 = randomizer.nextLong();
        final BeaconIdentifier id2 = BeaconIdentifier.fromLong(value2,
                Long.SIZE / Byte.SIZE);

        final long value3 = randomizer.nextLong();
        final BeaconIdentifier id3 = BeaconIdentifier.fromLong(value3,
                Long.SIZE / Byte.SIZE);

        final List<BeaconIdentifier> identifiers1 = new ArrayList<>();
        identifiers1.add(id1);
        identifiers1.add(id2);
        identifiers1.add(id3);

        final long value4 = value1 + 1;
        final BeaconIdentifier id4 = BeaconIdentifier.fromLong(value4,
                Long.SIZE / Byte.SIZE);

        final long value5 = value2 + 1;
        final BeaconIdentifier id5 = BeaconIdentifier.fromLong(value5,
                Long.SIZE / Byte.SIZE);

        final long value6 = value3 + 1;
        final BeaconIdentifier id6 = BeaconIdentifier.fromLong(value6,
                Long.SIZE / Byte.SIZE);

        final List<BeaconIdentifier> identifiers2 = new ArrayList<>();
        identifiers2.add(id4);
        identifiers2.add(id5);
        identifiers2.add(id6);

        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();

        final BeaconWithPowerAndLocated3D b1 = new BeaconWithPowerAndLocated3D(
                identifiers1, -60.0, position);
        final BeaconWithPowerAndLocated3D b2 = new BeaconWithPowerAndLocated3D(
                identifiers1, -50.0, position);
        final BeaconWithPowerAndLocated3D b3 = new BeaconWithPowerAndLocated3D(
                identifiers2, -60.0, position);

        // check
        assertEquals(b1, b1);
        assertEquals(b1, b2);
        assertNotEquals(b1, b3);

        assertNotEquals(b1, new Object());

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final long value1 = randomizer.nextLong();
        final BeaconIdentifier id1 = BeaconIdentifier.fromLong(value1,
                Long.SIZE / Byte.SIZE);

        final long value2 = randomizer.nextLong();
        final BeaconIdentifier id2 = BeaconIdentifier.fromLong(value2,
                Long.SIZE / Byte.SIZE);

        final long value3 = randomizer.nextLong();
        final BeaconIdentifier id3 = BeaconIdentifier.fromLong(value3,
                Long.SIZE / Byte.SIZE);

        final List<BeaconIdentifier> identifiers = new ArrayList<>();
        identifiers.add(id1);
        identifiers.add(id2);
        identifiers.add(id3);

        final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
        final Matrix cov = new Matrix(3, 3);
        final BeaconWithPowerAndLocated3D b1 = new BeaconWithPowerAndLocated3D(identifiers, -50.0,
                5.0e9, "address", 1,
                2, 3, "name",
                1.6, 1.0,
                0.1, position, cov);

        // check default values
        assertEquals(b1.getId1(), id1);
        assertEquals(b1.getId2(), id2);
        assertEquals(b1.getId3(), id3);
        assertEquals(b1.getIdentifiers(), identifiers);
        assertEquals(b1.getTransmittedPower(), -50.0, 0.0);
        assertEquals(b1.getBluetoothAddress(), "address");
        assertEquals(b1.getBeaconTypeCode(), 1);
        assertEquals(b1.getBluetoothName(), "name");
        assertEquals(b1.getManufacturer(), 2);
        assertEquals(b1.getServiceUuid(), 3);
        assertEquals(b1.getTransmittedPowerStandardDeviation(),
                1.0, 0.0);
        assertSame(b1.getPosition(), position);
        assertSame(b1.getPositionCovariance(), cov);
        assertEquals(b1.getPathLossExponent(), 1.6, 0.0);
        assertEquals(b1.getPathLossExponentStandardDeviation(), 0.1, 0.0);
        assertEquals(b1.getType(), RadioSourceType.BEACON);
        assertEquals(b1.getFrequency(), 5.0e9, 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(b1);
        final BeaconWithPowerAndLocated3D b2 =
                SerializationHelper.deserialize(bytes);

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
        assertEquals(b1.getTransmittedPowerStandardDeviation(),
                b2.getTransmittedPowerStandardDeviation(), 0.0);
        assertEquals(b1.getPosition(), b2.getPosition());
        assertEquals(b1.getPositionCovariance(), b2.getPositionCovariance());
        assertEquals(b1.getPathLossExponent(),
                b2.getPathLossExponent(), 0.0);
        assertEquals(b1.getPathLossExponentStandardDeviation(),
                b2.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(b1.getType(), b2.getType());
        assertEquals(b1.getFrequency(), b2.getFrequency(), 0.0);
    }
}
