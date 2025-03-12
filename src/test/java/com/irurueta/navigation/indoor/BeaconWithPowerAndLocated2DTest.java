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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class BeaconWithPowerAndLocated2DTest {

    @Test
    void testConstructor() throws AlgebraException {
        // test empty constructor
        var b = new BeaconWithPowerAndLocated2D();

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
        assertNull(b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // test constructor with identifiers, transmitted power and position
        final var identifiers = new ArrayList<BeaconIdentifier>();
        final var position = new InhomogeneousPoint2D();
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, null));

        // test constructor with all data and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", null));

        // test constructor with identifiers, transmitted power, transmitted power standard deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, Double.valueOf(1.0), position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, null,
                position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        final var std = Double.valueOf(1.0);
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, std, position));
        final var wrongStd = Double.valueOf(-1.0);
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, wrongStd, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, null));

        // test constructor with all data, transmitted power standard deviation
        // and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", Double.valueOf(1.0), position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", null,
                position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", wrongStd, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", std, null));

        // test constructor with identifiers, transmitted power, position and
        // position covariance
        final var cov = new Matrix(2, 2);
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, null, cov));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, position, m));

        // test constructor with all data, position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", position, m));

        // test constructor with identifiers, transmitted power, transmitted
        // power standard deviation, position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, Double.valueOf(1.0), position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, null,
                position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, wrongStd, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, position, m));

        // test constructor with all data, transmitted power standard deviation,
        // position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", Double.valueOf(1.0), position,
                cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", null,
                position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", wrongStd, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", std, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", std, position, m));

        // test constructor with all data, position and frequency
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", null));

        // test constructor with identifiers, transmitted power, transmitted power standard deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9, Double.valueOf(1.0),
                position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                null, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, wrongStd, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, std, null));

        // test constructor with all data, transmitted power standard deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                Double.valueOf(1.0), position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                null, position);

        // check
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(b.getIdentifiers(), identifiers);
        assertEquals(-50.0, b.getTransmittedPower(), 0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertNull(b.getTransmittedPowerStandardDeviation());
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", wrongStd, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", std, null));

        // test constructor with all data, position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name", position,
                cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", position, m));

        // test constructor with identifiers, transmitted power, transmitted
        // power standard deviation, position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9, Double.valueOf(1.0),
                position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                null, position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, wrongStd, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, std, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, std, position, m));

        // test constructor with all data, transmitted power standard deviation,
        // position and position covariance
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                Double.valueOf(1.0), position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                null, position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(BeaconWithPower.DEFAULT_PATH_LOSS_EXPONENT, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", wrongStd, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", std, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", std, position, m));

        // test constructor with identifiers, tx power, path loss and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 1.6, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 1.6, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 1.6, null));

        // test constructor with all data, path-loss and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, null));

        // test constructor with identifiers, tx power, std dev, path loss and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, Double.valueOf(1.0),
                1.6, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, null,
                1.6, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, std, 1.6, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, wrongStd, 1.6, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, 1.6, null));

        // test constructor with all data, tx power, std, position and path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6,
                1.0, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6,
                null, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, -1.0, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0, null));

        // test constructor with identifiers, tx, power, position, position covariance and path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 1.6, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 1.6, position,
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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 1.6, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 1.6, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 1.6, position, m));

        // test constructor with all data, position, position covariance and path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6, position,
                cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, position, m));

        // test constructor with identifiers, tx power, tx power std deviation,
        // position, position covariance and path loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, Double.valueOf(1.0),
                1.6, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, null,
                1.6, position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, std, 1.6, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, wrongStd, 1.6, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, 1.6, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, 1.6, position, m));

        // test constructor with all data, tx power std deviation, position,
        // position covariance and path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6,
                1.0, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6,
                null, position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, -1.0, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0, position, m));

        // test constructor with all data, position, frequency and path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", 1.6, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, null));

        // test constructor with identifiers, tx power, tx power std deviation,
        // position and path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9, 1.6,
                Double.valueOf(1.0), position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9, 1.6,
                null, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, 1.6, std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, 1.6, std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, wrongStd, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, std, null));

        // test constructor with all data, tx power std deviation, position and
        // path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, 1.0, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, null, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, -1.0, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0, null));

        // test constructor with all data, position, position covariance and path loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", 1.6, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, position, m));

        // test constructor with identifiers, tx power, tx power std deviation, position, position covariance and
        // path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9, 1.6,
                Double.valueOf(1.0), position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9, 1.6,
                null, position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, 1.6, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, 1.6, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, wrongStd, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, std, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, std, position, m));

        // test constructor with all data, tx power std deviation, position,
        // position covariance and path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, 1.0, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, null, position, null);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0, position,
                cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0, position,
                cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, -1.0, position,
                cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0, null,
                cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0, position, m));

        // test constructor with identifiers, tx power, path loss, path loss
        // std deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, Double.valueOf(1.0),
                1.6, Double.valueOf(0.1), position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, std, 1.6, std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, wrongStd, 1.6, std, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, 1.6, wrongStd, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, 1.6, std, null));

        // test constructor with all data, path loss, path loss std deviation and
        // position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6,
                1.0, 0.1, position);

        // check default values
        assertNull(b.getId1());
        assertNull(b.getId2());
        assertNull(b.getId3());
        assertEquals(identifiers, b.getIdentifiers());
        assertEquals(-50.0, b.getTransmittedPower(),  0.0);
        assertEquals("address", b.getBluetoothAddress());
        assertEquals(1, b.getBeaconTypeCode());
        assertEquals("name", b.getBluetoothName());
        assertEquals(2, b.getManufacturer());
        assertEquals(3, b.getServiceUuid());
        assertEquals(1.0, b.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0,
                0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, -1.0,
                0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0,
                -0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0,
                0.1, null));

        // test constructor with identifiers, tx power, std dev, path loss,
        // path loss std deviation and position
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, Double.valueOf(1.0),
                1.6, Double.valueOf(0.1), position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, null,
                1.6, null, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, std, 1.6, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, wrongStd, 1.6, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, 1.6, std, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, std, 1.6, std, position, m));

        // test constructor with all data, tx power, stds, position and path-loss
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6,
                1.0, 0.1, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, "address",
                1, 2, 3, "name", 1.6,
                null, null, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertNull(b.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0,
                0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, -1.0,
                0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0,
                -0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, "address", 1, 2, 3,
                "name", 1.6, 1.0,
                0.1, null, cov));

        // test constructor with identifiers, tx, power, position, path loss and
        // path-loss std deviation
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9, 1.6,
                1.0, 0.1, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, 1.6, 1.0,
                0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, 1.6, 1.0,
                0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, -1.0,
                0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, 1.0,
                -0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, 1.0,
                0.1, null));

        // test constructor with all data, position, path loss and path-loss std deviation
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, 1.0, 0.1, position);

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
        assertSame(position, b.getPosition());
        assertNull(b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0,
                0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0,
                0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, -1.0,
                0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0,
                -0.1, position));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0,
                0.1, null));

        // test constructor with identifiers, tx power, tx power std deviation,
        // position, position covariance, path-loss and path-loss std deviation
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9, 1.6,
                1.0, 0.1, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, 1.6, 1.0,
                0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, 1.6, 1.0,
                0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, -1.0,
                0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, 1.0,
                -0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, 1.6, 1.0,
                0.1, null, cov));

        // test constructor with all data, tx power std deviation, position,
        // position covariance, path-loss and path-loss std deviation
        b = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, 1.0, 0.1, position, cov);

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
        assertSame(position, b.getPosition());
        assertSame(cov, b.getPositionCovariance());
        assertEquals(1.6, b.getPathLossExponent(), 0.0);
        assertEquals(0.1, b.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(null,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0,
                0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, -5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0,
                0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, -1.0,
                0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0,
                -0.1, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new BeaconWithPowerAndLocated2D(identifiers,
                -50.0, 5.0e9, "address", 1, 2,
                3, "name", 1.6, 1.0,
                0.1, null, cov));
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();

        final var value1 = randomizer.nextLong();
        final var id1 = BeaconIdentifier.fromLong(value1, Long.SIZE / Byte.SIZE);

        final var value2 = randomizer.nextLong();
        final var id2 = BeaconIdentifier.fromLong(value2, Long.SIZE / Byte.SIZE);

        final var value3 = randomizer.nextLong();
        final var id3 = BeaconIdentifier.fromLong(value3, Long.SIZE / Byte.SIZE);

        final var identifiers1 = new ArrayList<BeaconIdentifier>();
        identifiers1.add(id1);
        identifiers1.add(id2);
        identifiers1.add(id3);

        final var value4 = value1 + 1;
        final var id4 = BeaconIdentifier.fromLong(value4, Long.SIZE / Byte.SIZE);

        final var value5 = value2 + 1;
        final var id5 = BeaconIdentifier.fromLong(value5, Long.SIZE / Byte.SIZE);

        final var value6 = value3 + 1;
        final var id6 = BeaconIdentifier.fromLong(value6, Long.SIZE / Byte.SIZE);

        final var identifiers2 = new ArrayList<BeaconIdentifier>();
        identifiers2.add(id4);
        identifiers2.add(id5);
        identifiers2.add(id6);

        final var position = new InhomogeneousPoint2D();

        final var b1 = new BeaconWithPowerAndLocated2D(identifiers1, -60.0, position);
        final var b2 = new BeaconWithPowerAndLocated2D(identifiers1, -50.0, position);
        final var b3 = new BeaconWithPowerAndLocated2D(identifiers2, -60.0, position);

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
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var value1 = randomizer.nextLong();
        final var id1 = BeaconIdentifier.fromLong(value1, Long.SIZE / Byte.SIZE);

        final var value2 = randomizer.nextLong();
        final var id2 = BeaconIdentifier.fromLong(value2, Long.SIZE / Byte.SIZE);

        final var value3 = randomizer.nextLong();
        final var id3 = BeaconIdentifier.fromLong(value3, Long.SIZE / Byte.SIZE);

        final var identifiers = new ArrayList<BeaconIdentifier>();
        identifiers.add(id1);
        identifiers.add(id2);
        identifiers.add(id3);

        final var position = new InhomogeneousPoint2D();
        final var cov = new Matrix(2, 2);
        final var b1 = new BeaconWithPowerAndLocated2D(identifiers, -50.0, 5.0e9,
                "address", 1, 2, 3, "name",
                1.6, 1.0, 0.1, position, cov);

        // check default values
        assertEquals(id1, b1.getId1());
        assertEquals(id2, b1.getId2());
        assertEquals(id3, b1.getId3());
        assertEquals(identifiers, b1.getIdentifiers());
        assertEquals(-50.0, b1.getTransmittedPower(), 0.0);
        assertEquals("address", b1.getBluetoothAddress());
        assertEquals(1, b1.getBeaconTypeCode());
        assertEquals("name", b1.getBluetoothName());
        assertEquals(2, b1.getManufacturer());
        assertEquals(3, b1.getServiceUuid());
        assertEquals(1.0, b1.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, b1.getPosition());
        assertSame(cov, b1.getPositionCovariance());
        assertEquals(1.6, b1.getPathLossExponent(), 0.0);
        assertEquals(0.1, b1.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.BEACON, b1.getType());
        assertEquals(5.0e9, b1.getFrequency(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(b1);
        final var b2 = SerializationHelper.<BeaconWithPowerAndLocated2D>deserialize(bytes);

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
        assertEquals(b1.getPosition(), b2.getPosition());
        assertEquals(b1.getPositionCovariance(), b2.getPositionCovariance());
        assertEquals(b1.getPathLossExponent(), b2.getPathLossExponent(), 0.0);
        assertEquals(b1.getPathLossExponentStandardDeviation(), b2.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(b1.getType(), b2.getType());
        assertEquals(b1.getFrequency(), b2.getFrequency(), 0.0);
    }
}
