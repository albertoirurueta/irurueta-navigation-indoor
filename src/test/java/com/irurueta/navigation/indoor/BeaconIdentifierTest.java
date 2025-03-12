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
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class BeaconIdentifierTest {

    @Test
    void testConstructor() {
        // test empty constructor
        var id = new BeaconIdentifier();

        // check
        assertNotNull(id.toString());
        assertEquals(0, id.toInt());
        assertNull(id.toByteArrayOfSpecifiedEndianness(true));
        assertEquals(0, id.getByteCount());
        assertNull(id.toHexString());
        assertNull(id.toUuid());
        assertNull(id.toByteArray());
        assertEquals(0, id.hashCode());

        // test constructor from byte array
        final var r = new Random();

        // length 2
        var value = new byte[2];
        r.nextBytes(value);

        id = new BeaconIdentifier(value);

        // check
        assertEquals(Integer.toString(id.toInt()), id.toString());
        assertTrue(id.toInt() > 0);
        assertArrayEquals(value, id.toByteArrayOfSpecifiedEndianness(true));
        final var reversedValue = id.toByteArrayOfSpecifiedEndianness(false);
        for (var i = 0; i < value.length; i++) {
            assertEquals(reversedValue[value.length - 1 - i], value[i]);
        }
        assertEquals(value.length, id.getByteCount());

        assertNotNull(id.toHexString());
        BeaconIdentifier id2 = BeaconIdentifier.parse(id.toHexString());
        assertArrayEquals(id.toByteArray(), id2.toByteArray());
        assertEquals(id, id2);
        assertEquals(id.hashCode(), id2.hashCode());

        assertThrows(UnsupportedOperationException.class, id::toUuid);

        // length 16
        value = new byte[16];
        r.nextBytes(value);
        id = new BeaconIdentifier(value);
        assertNotNull(id.toUuid());
        id2 = BeaconIdentifier.parse(id.toUuid().toString());
        assertArrayEquals(id.toByteArray(), id2.toByteArray());
        assertEquals(id, id2);
        assertEquals(id.hashCode(), id2.hashCode());

        // force NullPointerException
        assertThrows(NullPointerException.class, () -> new BeaconIdentifier(null));
    }

    @Test
    void testParseFromLong() {
        // create identifier
        final var randomizer = new UniformRandomizer();

        final var value = randomizer.nextLong();
        final var id = BeaconIdentifier.fromLong(value, Long.SIZE / Byte.SIZE);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> BeaconIdentifier.fromLong(value, -1));

        // parse
        final var str = id.toString();
        final var id2 = BeaconIdentifier.parse(str);
        final var id3 = BeaconIdentifier.parse(str, Long.SIZE / Byte.SIZE);

        assertEquals(id, id2);
        assertEquals(id, id3);

        // force NullPointerException
        assertThrows(NullPointerException.class, () -> BeaconIdentifier.parse(null));

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> BeaconIdentifier.parse("hello"));
    }

    @Test
    void testParseFromInt() {
        // create identifier
        final var randomizer = new UniformRandomizer();

        final var value = randomizer.nextInt(0, 65535);
        final var id = BeaconIdentifier.fromInt(value);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> BeaconIdentifier.fromInt(-1));
        assertThrows(IllegalArgumentException.class, () -> BeaconIdentifier.fromInt(65536));

        // parse
        final var str = id.toString();
        final var id2 = BeaconIdentifier.parse(str);

        assertEquals(id, id2);

        // force NullPointerException
        assertThrows(NullPointerException.class, () -> BeaconIdentifier.parse(null));

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> BeaconIdentifier.parse("hello"));
    }

    @Test
    void testParseFromBytes() {
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextLong();
        final var id = BeaconIdentifier.fromLong(value, Long.SIZE / Byte.SIZE);
        final var bytes = id.toByteArray();

        var id2 = BeaconIdentifier.fromBytes(bytes, 0, bytes.length, false);
        assertEquals(id, id2);
        assertArrayEquals(id.toByteArray(), id2.toByteArray());

        // test with little endian
        id2 = BeaconIdentifier.fromBytes(bytes, 0, bytes.length, true);
        final var bytes2 = id2.toByteArray();
        for (var i = 0; i < bytes.length; i++) {
            assertEquals(bytes[i], bytes2[bytes.length - 1 - i]);
        }

        // force NullPointerException
        assertThrows(NullPointerException.class, () -> BeaconIdentifier.fromBytes(null, 0, 1,
                true));

        // force ArrayIndexOutOfBoundsException
        assertThrows(ArrayIndexOutOfBoundsException.class, () -> BeaconIdentifier.fromBytes(bytes, -1,
                bytes.length, true));
        assertThrows(ArrayIndexOutOfBoundsException.class, () -> BeaconIdentifier.fromBytes(bytes, 0,
                bytes.length + 1, true));

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> BeaconIdentifier.fromBytes(bytes, bytes.length, 0,
                true));
        assertThrows(IllegalArgumentException.class, () -> BeaconIdentifier.parse("hello"));
    }

    @Test
    void testFromUuid() {
        final var value = new byte[16];

        final var r = new Random();
        r.nextBytes(value);
        final var id = new BeaconIdentifier(value);

        final var uuid = id.toUuid();
        BeaconIdentifier id2 = BeaconIdentifier.fromUuid(uuid);

        // check
        assertEquals(id, id2);
    }

    @Test
    void testToString() {
        final var r = new Random();
        final var bytes = new byte[16];
        r.nextBytes(bytes);

        final var id = BeaconIdentifier.fromBytes(bytes, 0, bytes.length, false);

        final var str = id.toString();
        assertEquals("0x" + str.replaceAll("-", ""), id.toHexString());

        final var id2 = BeaconIdentifier.parse(str);
        assertEquals(id, id2);
    }

    @Test
    void testParseHexNoPrefix() {
        final var r = new Random();
        final var bytes = new byte[32];
        r.nextBytes(bytes);

        final var id = BeaconIdentifier.fromBytes(bytes, 0, bytes.length, false);

        final var str = id.toHexString().substring(2);

        final var id2 = BeaconIdentifier.parse(str);
        final var id3 = BeaconIdentifier.parse(str, 32);
        assertEquals(id, id2);
        assertEquals(id, id3);

        final var id4 = BeaconIdentifier.parse(str, 33);
        assertEquals(str, id4.toHexString().substring(4));

        final var id5 = BeaconIdentifier.parse(str, 31);
        assertEquals(str.substring(2), id5.toHexString().substring(2));

    }

    @Test
    void testParseLongDecimal() {
        final var value = 65536;
        final var id = BeaconIdentifier.fromLong(value, 4);

        final var id2 = BeaconIdentifier.parse(String.valueOf(value), 4);
        assertEquals(id, id2);
    }

    @Test
    void testToInt() {
        final var randomizer = new UniformRandomizer();
        var value = randomizer.nextInt(0, 65535);
        var id = BeaconIdentifier.fromInt(value);

        assertEquals(value, id.toInt());

        // force UnsupportedOperationException
        value = 65536;
        id = BeaconIdentifier.fromLong(value, 4);
        assertThrows(UnsupportedOperationException.class, id::toInt);
    }

    @Test
    void testEqualsAndCompareTo() {
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextInt(2, 65535);
        final var id = BeaconIdentifier.fromInt(value);
        final var id2 = BeaconIdentifier.fromInt(value);
        final var id3 = BeaconIdentifier.fromLong(value, 4);
        final var id4 = BeaconIdentifier.fromLong(value, 5);
        final var id5 = BeaconIdentifier.fromInt(value - 1);

        assertEquals(id, id2);
        assertEquals(0, id.compareTo(id2));

        assertNotEquals(new Object(), id);

        assertEquals(-1, id3.compareTo(id4));
        assertEquals(1, id4.compareTo(id3));

        assertEquals(1, id.compareTo(id5));
        assertEquals(-1, id5.compareTo(id));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var r = new Random();

        final var value = new byte[2];
        r.nextBytes(value);

        final var id1 = new BeaconIdentifier(value);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(id1);
        final var id2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(id1, id2);
        assertNotSame(id1, id2);
    }
}
