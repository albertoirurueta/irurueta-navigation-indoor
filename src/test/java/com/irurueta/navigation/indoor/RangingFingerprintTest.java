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

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RangingFingerprintTest {

    @Test
    void testConstructor() {
        var fingerprint = new RangingFingerprint<>();

        // check
        assertNotNull(fingerprint.getReadings());
        assertTrue(fingerprint.getReadings().isEmpty());

        // constructor with readings
        final var readings = new ArrayList<RangingReading<RadioSource>>();
        fingerprint = new RangingFingerprint<>(readings);

        // check
        assertEquals(readings, fingerprint.getReadings());
        assertNotSame(readings, fingerprint.getReadings());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RangingFingerprint<>(null));
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var readings = new ArrayList<RangingReading<RadioSource>>();
        final var fingerprint1 = new RangingFingerprint<>(readings);

        // check
        assertEquals(readings, fingerprint1.getReadings());
        assertNotSame(readings, fingerprint1.getReadings());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(fingerprint1);
        final var fingerprint2 =
                SerializationHelper.<RangingFingerprint<RadioSource, RangingReading<RadioSource>>>deserialize(bytes);

        // check
        assertNotSame(fingerprint1, fingerprint2);
        assertEquals(fingerprint1.getReadings(), fingerprint2.getReadings());
        assertNotSame(fingerprint1.getReadings(), fingerprint2.getReadings());
    }
}
