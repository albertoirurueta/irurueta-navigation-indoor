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
 * Data related to a Wi-Fi access point with estimated transmitted power.
 */
public class WifiAccessPointWithPower extends WifiAccessPoint implements RadioSourceWithPower {

    /**
     * Default exponent typically used on free space for path loss propagation in
     * terms of distance. This value is used for free space environments.
     */
    public static final double DEFAULT_PATH_LOSS_EXPONENT = 2.0;

    /**
     * Transmitted power expressed in dBm's.
     */
    private double transmittedPower;

    /**
     * Standard deviation of transmitted power value or null if unknown.
     */
    private Double transmittedPowerStandardDeviation;

    /**
     * Exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link #DEFAULT_PATH_LOSS_EXPONENT}
     */
    private double pathLossExponent = DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Standard deviation of path loss exponent or null if unknown.
     */
    private Double pathLossExponentStandardDeviation;

    /**
     * Constructor.
     *
     * @param bssid            basic service set identifier of this access point in the form of a six-byte MAC address:
     *                         xx:xx:xx:xx:xx:xx.
     * @param frequency        frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPower(final String bssid, final double frequency, final double transmittedPower) {
        super(bssid, frequency);
        this.transmittedPower = transmittedPower;
    }

    /**
     * Constructor.
     *
     * @param bssid            basic service set identifier of this access point in the form of a six-byte MAC address:
     *                         xx:xx:xx:xx:xx:xx.
     * @param frequency        frequency used by this Access Point (expressed in Hz).
     * @param ssid             service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPower(
            final String bssid, final double frequency, final String ssid, final double transmittedPower) {
        super(bssid, frequency, ssid);
        this.transmittedPower = transmittedPower;
    }

    /**
     * Constructor.
     *
     * @param bssid                             basic service set identifier of this access point in the form of a six-byte MAC address:
     *                                          xx:xx:xx:xx:xx:xx.
     * @param frequency                         frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower                  transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null if
     *                                          unknown.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     *                                  or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPower(
            final String bssid, final double frequency, final double transmittedPower,
            final Double transmittedPowerStandardDeviation) {
        this(bssid, frequency, transmittedPower);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        this.transmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     *
     * @param bssid                             basic service set identifier of this access point in the form of a six-byte MAC address:
     *                                          xx:xx:xx:xx:xx:xx.
     * @param frequency                         frequency used by this Access Point (expressed in Hz).
     * @param ssid                              service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower                  transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null if unknown.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     *                                  or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPower(
            final String bssid, final double frequency, final String ssid, final double transmittedPower,
            final Double transmittedPowerStandardDeviation) {
        this(bssid, frequency, ssid, transmittedPower);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        this.transmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     *
     * @param bssid            basic service set identifier of this access point in the form of a six-byte MAC address:
     *                         xx:xx:xx:xx:xx:xx.
     * @param frequency        frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param pathLossExponent path loss exponent. By default, this is 2.0.
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPower(
            final String bssid, final double frequency, final double transmittedPower, final double pathLossExponent) {
        this(bssid, frequency, transmittedPower);
        this.pathLossExponent = pathLossExponent;
    }

    /**
     * Constructor.
     *
     * @param bssid                             basic service set identifier of this access point in the form of a six-byte MAC address:
     *                                          xx:xx:xx:xx:xx:xx.
     * @param frequency                         frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower                  transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null if
     *                                          unknown.
     * @param pathLossExponent                  path loss exponent. By default, this is 2.0.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     *                                  or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPower(
            final String bssid, final double frequency, final double transmittedPower,
            final Double transmittedPowerStandardDeviation, final double pathLossExponent) {
        this(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation);
        this.pathLossExponent = pathLossExponent;
    }

    /**
     * Constructor.
     *
     * @param bssid                             basic service set identifier of this access point in the form of a six-byte MAC address:
     *                                          xx:xx:xx:xx:xx:xx.
     * @param frequency                         frequency used by this Access Point (expressed in Hz).
     * @param ssid                              service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower                  transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null if
     *                                          unknown.
     * @param pathLossExponent                  path loss exponent. By default, this is 2.0.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     *                                  or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPower(
            final String bssid, final double frequency, final String ssid, final double transmittedPower,
            final Double transmittedPowerStandardDeviation, final double pathLossExponent) {
        this(bssid, frequency, ssid, transmittedPower, transmittedPowerStandardDeviation);
        this.pathLossExponent = pathLossExponent;
    }

    /**
     * Constructor.
     *
     * @param bssid                             basic service set identifier of this access point in the form of a six-byte MAC address:
     *                                          xx:xx:xx:xx:xx:xx.
     * @param frequency                         frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower                  transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null if
     *                                          unknown.
     * @param pathLossExponent                  path loss exponent. By default, this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     *                                  or any standard deviation is negative.
     */
    public WifiAccessPointWithPower(
            final String bssid, final double frequency, final double transmittedPower,
            final Double transmittedPowerStandardDeviation, final double pathLossExponent,
            final Double pathLossExponentStandardDeviation) {
        this(bssid, frequency, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent);

        if (pathLossExponentStandardDeviation != null && pathLossExponentStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        this.pathLossExponentStandardDeviation = pathLossExponentStandardDeviation;
    }

    /**
     * Constructor.
     *
     * @param bssid                             basic service set identifier of this access point in the form of a six-byte MAC address:
     *                                          xx:xx:xx:xx:xx:xx.
     * @param frequency                         frequency used by this Access Point (expressed in Hz).
     * @param ssid                              service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower                  transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null if
     *                                          unknown.
     * @param pathLossExponent                  path loss exponent. By default, this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     *                                  or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPower(
            final String bssid, final double frequency, final String ssid, final double transmittedPower,
            final Double transmittedPowerStandardDeviation, final double pathLossExponent,
            final Double pathLossExponentStandardDeviation) {
        this(bssid, frequency, ssid, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent);

        if (pathLossExponentStandardDeviation != null && pathLossExponentStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        this.pathLossExponentStandardDeviation = pathLossExponentStandardDeviation;
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPointWithPower() {
        super();
    }

    /**
     * Gets transmitted power expressed in dBm's.
     *
     * @return transmitted power expressed in dBm's.
     */
    @Override
    public double getTransmittedPower() {
        return transmittedPower;
    }

    /**
     * Gets standard deviation of transmitted power value or null if unknown.
     *
     * @return standard deviation of transmitted power value or null if unknown.
     */
    @Override
    public Double getTransmittedPowerStandardDeviation() {
        return transmittedPowerStandardDeviation;
    }

    /**
     * Gets exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link #DEFAULT_PATH_LOSS_EXPONENT}
     *
     * @return path loss exponent.
     */
    @Override
    public double getPathLossExponent() {
        return pathLossExponent;
    }

    /**
     * Gets standard deviation of path loss exponent or null if unknown.
     *
     * @return standard deviation of path loss exponent or null if unknown.
     */
    @Override
    public Double getPathLossExponentStandardDeviation() {
        return pathLossExponentStandardDeviation;
    }
}
