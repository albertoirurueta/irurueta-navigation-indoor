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

/**
 * Data related to a WiFi access point.
 */
public class WifiAccessPoint implements Serializable, RadioSource {

    /**
     * Basic service set identifier of this access point in the form of a six-byte MAC address:
     * xx:xx:xx:xx:xx:xx.
     */
    private String bssid;

    /**
     * Frequency used by this Access Point (expressed in Hz).
     */
    private double frequency;

    /**
     * Service set identifier (SSID) of this 802.11 network. This value is optional.
     */
    private String ssid;

    /**
     * Constructor.
     *
     * @param bssid     basic service set identifier of this access point in the form of a six-byte MAC address:
     *                  xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPoint(final String bssid, final double frequency) {
        if (bssid == null) {
            throw new IllegalArgumentException();
        }

        if (frequency < 0.0) {
            throw new IllegalArgumentException();
        }

        this.bssid = bssid;
        this.frequency = frequency;
    }

    /**
     * Constructor.
     *
     * @param bssid     basic service set identifier of this access point in the form of a six-byte MAC address:
     *                  xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid      service set identifier (SSID) of this 802.11 network.
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPoint(final String bssid, final double frequency, final String ssid) {
        this(bssid, frequency);
        this.ssid = ssid;
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPoint() {
    }

    /**
     * Gets the basic service set identifier of this access point in the form of a six-byte MAC address:
     * xx:xx:xx:xx:xx:xx.
     *
     * @return the basic service set identifier.
     */
    public String getBssid() {
        return bssid;
    }

    /**
     * Gets frequency used by this Access Point (expressed in Hz).
     *
     * @return frequency used by this Access Point (expressed in Hz).
     */
    @Override
    public double getFrequency() {
        return frequency;
    }

    /**
     * Gets service set identifier (SSID) of this 802.11 network.
     *
     * @return service set identifier (SSID).
     */
    public String getSsid() {
        return ssid;
    }

    /**
     * Indicates whether two access points are considered to be equal or not.
     * Two access points are considered equal if they have the same BSSID.
     *
     * @param obj other object to be compared.
     * @return true if both access points are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }

        if (!(obj instanceof WifiAccessPoint other)) {
            return false;
        }

        return bssid.equals(other.bssid);
    }

    /**
     * Returns hashcode associated to this access point.
     *
     * @return hashcode associated to this access point.
     */
    @Override
    public int hashCode() {
        return bssid.hashCode();
    }

    /**
     * Gets radio source type, which can be either a Wi-Fi Access point or a bluetooth Beacon.
     *
     * @return radio source type.
     */
    @Override
    public RadioSourceType getType() {
        return RadioSourceType.WIFI_ACCESS_POINT;
    }
}
