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
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class BeaconTest {

    @Test
    void testConstructor() {
        // test empty constructor
        var b = new Beacon();

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
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // test constructor with identifiers and transmitted power
        final var identifiers = new ArrayList<BeaconIdentifier>();
        b = new Beacon(identifiers, -50.0);

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
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Beacon(null, -50.0));

        // test constructor with all data
        b = new Beacon(identifiers, -50.0, "address", 1, 2,
                3, "name");

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
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(Beacon.DEFAULT_FREQUENCY, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Beacon(null, -50.0,
                "address", 1, 2, 3, "name"));

        // test constructor with identifiers, transmitted power and frequency
        b = new Beacon(identifiers, -50.0, 5.0e9);

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
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Beacon(null, -50.0,
                5.0e9));
        assertThrows(IllegalArgumentException.class, () -> new Beacon(identifiers, -50.0,
                -5.0e9));

        // test constructor with all data and frequency
        b = new Beacon(identifiers, -50.0, 5.0e9, "address", 1,
                2, 3, "name");

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
        assertEquals(RadioSourceType.BEACON, b.getType());
        assertEquals(5.0e9, b.getFrequency(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new Beacon(null, -50.0,
                5.0e9, "address", 1, 2, 3,
                "name"));
        assertThrows(IllegalArgumentException.class, () -> new Beacon(identifiers, -50.0,
                -5.0e9, "address", 1, 2, 3,
                "name"));
    }

    @Test
    void testGetIdentifier() {
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

        final var b = new Beacon(identifiers, -60.0);

        // check
        assertEquals(b.getIdentifier(0), id1);
        assertEquals(b.getIdentifier(1), id2);
        assertEquals(b.getIdentifier(2), id3);

        assertEquals(b.getId1(), id1);
        assertEquals(b.getId2(), id2);
        assertEquals(b.getId3(), id3);

        assertNull(b.getIdentifier(-1));
        assertNull(b.getIdentifier(identifiers.size()));
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

        final var b1 = new Beacon(identifiers1, -60.0);
        final var b2 = new Beacon(identifiers1, -50.0);
        final var b3 = new Beacon(identifiers2, -60.0);

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
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
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

        final var b1 = new Beacon(identifiers, -50.0, 5.0e9, "address",
                1, 2, 3, "name");

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
        assertEquals(RadioSourceType.BEACON, b1.getType());
        assertEquals(5.0e9, b1.getFrequency(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(b1);
        final var b2 = SerializationHelper.<Beacon>deserialize(bytes);

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
        assertEquals(b1.getType(), b2.getType());
        assertEquals(b1.getFrequency(), b2.getFrequency(), 0.0);
    }
}
