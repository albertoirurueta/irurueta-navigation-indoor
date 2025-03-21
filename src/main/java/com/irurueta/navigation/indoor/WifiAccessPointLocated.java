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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;

/**
 * Data related to a Wi-Fi access point whose location is known.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class WifiAccessPointLocated<P extends Point<?>> extends WifiAccessPoint implements
        RadioSourceLocated<P> {

    /**
     * Position where access point is located.
     */
    private P position;

    /**
     * Covariance of inhomogeneous coordinates of current position (if available).
     */
    private Matrix positionCovariance;

    /**
     * Constructor.
     *
     * @param bssid     basic service set identifier of this access point in the form of a six-byte MAC address:
     *                  xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param position  position where access point is located.
     * @throws IllegalArgumentException if either BSSID or position are null or
     *                                  frequency is negative.
     */
    protected WifiAccessPointLocated(final String bssid, final double frequency, final P position) {
        super(bssid, frequency);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        this.position = position;
    }

    /**
     * Constructor.
     *
     * @param bssid     basic service set identifier of this access point in the form of a six-byte MAC address:
     *                  xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid      service set identifier (SSID) of this 802.11 network.
     * @param position  position where access point is located.
     * @throws IllegalArgumentException if either BSSID or position are null or
     *                                  frequency is negative.
     */
    protected WifiAccessPointLocated(
            final String bssid, final double frequency, final String ssid, final P position) {
        super(bssid, frequency, ssid);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        this.position = position;
    }

    /**
     * Constructor.
     *
     * @param bssid              basic service set identifier of this access point in the form of a six-byte MAC address:
     *                           xx:xx:xx:xx:xx:xx.
     * @param frequency          frequency used by this Access Point (expressed in Hz).
     * @param position           position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID or position are null or
     *                                  frequency is negative, or covariance does not have proper size.
     */
    protected WifiAccessPointLocated(
            final String bssid, final double frequency, final P position, final Matrix positionCovariance) {
        this(bssid, frequency, position);

        if (positionCovariance != null) {
            final var dims = position.getDimensions();
            if (positionCovariance.getRows() != dims || positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        this.positionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     *
     * @param bssid              basic service set identifier of this access point in the form of a six-byte MAC address:
     *                           xx:xx:xx:xx:xx:xx.
     * @param frequency          frequency used by this Access Point (expressed in Hz).
     * @param ssid               service set identifier (SSID) of this 802.11 network.
     * @param position           position where access point is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either BSSID or position are null or
     *                                  frequency is negative, or covariance does not have proper size.
     */
    protected WifiAccessPointLocated(
            final String bssid, final double frequency, final String ssid, final P position,
            final Matrix positionCovariance) {
        this(bssid, frequency, ssid, position);

        if (positionCovariance != null) {
            final var dims = position.getDimensions();
            if (positionCovariance.getRows() != dims || positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        this.positionCovariance = positionCovariance;
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPointLocated() {
        super();
    }

    /**
     * Gets position where access point is located.
     *
     * @return position where access point is located.
     */
    public P getPosition() {
        return position;
    }

    /**
     * Gets covariance of inhomogeneous coordinates of current position (if available).
     *
     * @return covariance of position or null.
     */
    public Matrix getPositionCovariance() {
        return positionCovariance;
    }
}
