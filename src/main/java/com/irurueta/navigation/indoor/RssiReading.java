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

/**
 * Contains a signal strength reading associated to a given radio source
 * (e.g. Wi-Fi access point or bluetooth beacon).
 *
 * @param <S> a {@link RadioSource} type.
 */
public class RssiReading<S extends RadioSource> extends Reading<S> {

    /**
     * Received signal strength indicator of a 802.11 network or bluetooth beacon, in dBm.
     */
    private double rssi;

    /**
     * Standard deviation of RSSI, if available.
     */
    private Double rssiStandardDeviation;

    /**
     * Constructor.
     *
     * @param source radio source associated to this reading.
     * @param rssi   received signal strength indicator in dBm.
     * @throws IllegalArgumentException if radio source data is null.
     */
    public RssiReading(final S source, final double rssi) {
        super(source);
        this.rssi = rssi;
    }

    /**
     * Constructor.
     *
     * @param source                radio source associated to this reading.
     * @param rssi                  received signal strength indicator in dBm.
     * @param rssiStandardDeviation standard deviation of RSSI, if available.
     * @throws IllegalArgumentException if radio source data is null or
     *                                  standard deviation is zero or negative.
     */
    public RssiReading(final S source, final double rssi, final Double rssiStandardDeviation) {
        this(source, rssi);

        if (rssiStandardDeviation != null && rssiStandardDeviation <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.rssiStandardDeviation = rssiStandardDeviation;
    }

    /**
     * Empty constructor.
     */
    protected RssiReading() {
        super();
    }

    /**
     * Contains radio source reading type.
     *
     * @return reading type.
     */
    @Override
    public ReadingType getType() {
        return ReadingType.RSSI_READING;
    }

    /**
     * Gets received signal strength indicator of this 802.11 network, in dBm.
     *
     * @return received signal strength indicator.
     */
    public double getRssi() {
        return rssi;
    }

    /**
     * Gets standard deviation of RSSI, if available.
     *
     * @return standard deviation of RSSI, if available.
     */
    public Double getRssiStandardDeviation() {
        return rssiStandardDeviation;
    }
}
