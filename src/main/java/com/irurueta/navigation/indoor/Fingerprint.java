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

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Contains readings from several radio sources for an unknown location
 * to be determined.
 *
 * @param <S> a {@link RadioSource} type.
 * @param <R> a {@link RssiReading} type.
 */
public class Fingerprint<S extends RadioSource, R extends Reading<S>> implements Serializable {

    /**
     * Non-located ranging and RSSI readings.
     */
    protected ArrayList<R> readings = new ArrayList<>();

    /**
     * Constructor.
     */
    public Fingerprint() {
    }

    /**
     * Constructor.
     *
     * @param readings non-located readings.
     * @throws IllegalArgumentException if provided readings is null.
     */
    public Fingerprint(final List<R> readings) {
        if (readings == null) {
            throw new IllegalArgumentException();
        }
        this.readings = new ArrayList<>(readings);
    }

    /**
     * Gets non-located ranging readings.
     *
     * @return non-located ranging readings.
     */
    public List<R> getReadings() {
        return readings;
    }

    /**
     * Sets non-located ranging readings.
     *
     * @param readings non-located ranging readings.
     * @throws IllegalArgumentException if provided readings is null.
     */
    public void setReadings(final List<R> readings) {
        if (readings == null) {
            throw new IllegalArgumentException();
        }
        this.readings = new ArrayList<>(readings);
    }
}
