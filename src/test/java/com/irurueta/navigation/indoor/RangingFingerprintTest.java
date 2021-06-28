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
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RangingFingerprintTest {

    @Test
    public void testConstructor() {
        RangingFingerprint<RadioSource, RangingReading<RadioSource>> fingerprint =
                new RangingFingerprint<>();

        // check
        assertNotNull(fingerprint.getReadings());
        assertTrue(fingerprint.getReadings().isEmpty());

        // constructor with readings
        final List<RangingReading<RadioSource>> readings = new ArrayList<>();
        fingerprint = new RangingFingerprint<>(readings);

        // check
        assertEquals(fingerprint.getReadings(), readings);
        assertNotSame(fingerprint.getReadings(), readings);

        // force IllegalArgumentException
        fingerprint = null;
        try {
            fingerprint = new RangingFingerprint<>(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        //noinspection ConstantConditions
        assertNull(fingerprint);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final List<RangingReading<RadioSource>> readings = new ArrayList<>();
        final RangingFingerprint<RadioSource, RangingReading<RadioSource>> fingerprint1 =
                new RangingFingerprint<>(readings);

        // check
        assertEquals(fingerprint1.getReadings(), readings);
        assertNotSame(fingerprint1.getReadings(), readings);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(fingerprint1);
        final RangingFingerprint<RadioSource, RangingReading<RadioSource>> fingerprint2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(fingerprint1, fingerprint2);
        assertEquals(fingerprint1.getReadings(), fingerprint2.getReadings());
        assertNotSame(fingerprint1.getReadings(), fingerprint2.getReadings());
    }
}
