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
import java.util.Collections;
import java.util.List;

/**
 * The <code>Beacon</code> class represents a single hardware Beacon detected by
 * an Android device.
 *
 * <pre>A Beacon is identified by a unique multi-part identifier, with the first of the ordered
 * identifiers being more significant for the purposes of grouping beacons.
 *
 * A Beacon sends a Bluetooth Low Energy (BLE) advertisement that contains these
 * three identifiers, along with the calibrated tx power (in RSSI) of the Beacon's
 * Bluetooth transmitter.
 * </pre>
 * <p>
 * Based on:
 * <a href="https://github.com/AltBeacon/android-beacon-library/blob/master/src/main/java/org/altbeacon/beacon/Beacon.java">
 *   https://github.com/AltBeacon/android-beacon-library/blob/master/src/main/java/org/altbeacon/beacon/Beacon.java
 * </a>
 */
public class Beacon implements Serializable, RadioSource {

    /**
     * Default frequency used by a Beacon when none is specified (expressed in Hz).
     */
    public static final double DEFAULT_FREQUENCY = 2.4e9;

    /**
     * The list of the multipart identifiers of the beacon. Together, these identifiers signify
     * a unique beacon. The identifiers are ordered by significance for the purpose of grouping
     * beacons.
     */
    private ArrayList<BeaconIdentifier> identifiers;

    /**
     * The calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * This value is baked into a Beacon when it is manufactured, and it is
     * transmitted with each packet to aid in the distance estimate.
     */
    private double transmittedPower;

    /**
     * The Bluetooth mac address.
     */
    private String bluetoothAddress;

    /**
     * The two byte value indicating the type of beacon that this is, which is used for figuring
     * out the byte layout of the beacon advertisement.
     */
    private int beaconTypeCode;

    /**
     * A two byte code indicating the beacon manufacturer. A list of registered manufacturer codes
     * may be found here:
     * <a href="https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers">
     *   https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
     * </a>
     * <p>
     * If the beacon is a GATT-based beacon, this field will be set to -1.
     */
    private int manufacturer;

    /**
     * A 32 bit service uuid for the beacon.
     * This is valid only for GATT-based beacons. If the beacon is a manufacturer data-based
     * beacon, this field will be -1
     */
    private int serviceUuid = -1;

    /**
     * The Bluetooth device name.  This is a field transmitted by the remote beacon device separate
     * from the advertisement data
     */
    private String bluetoothName;

    /**
     * Frequency used by this Beacon(expressed in Hz).
     */
    private double frequency = DEFAULT_FREQUENCY;

    /**
     * Constructor.
     *
     * @param identifiers      list of the multipart identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @throws IllegalArgumentException if identifiers is null.
     */
    public Beacon(final List<BeaconIdentifier> identifiers, final double transmittedPower) {
        if (identifiers == null) {
            throw new IllegalArgumentException();
        }

        this.identifiers = new ArrayList<>(identifiers);
        this.transmittedPower = transmittedPower;
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multipart identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress the bluetooth mac address.
     * @param beaconTypeCode   the two byte value indicating the type of beacon.
     * @param manufacturer     a two byte code indicating the beacon manufacturer.
     * @param serviceUuid      a 32 bit service uuid for the beacon.
     * @param bluetoothName    the bluetooth device name.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public Beacon(final List<BeaconIdentifier> identifiers, final double transmittedPower,
                  final String bluetoothAddress, final int beaconTypeCode, final int manufacturer,
                  final int serviceUuid, final String bluetoothName) {
        this(identifiers, transmittedPower);

        this.bluetoothAddress = bluetoothAddress;
        this.beaconTypeCode = beaconTypeCode;
        this.manufacturer = manufacturer;
        this.serviceUuid = serviceUuid;
        this.bluetoothName = bluetoothName;
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multipart identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency        frequency used by this Beacon.
     * @throws IllegalArgumentException if identifiers is null or frequency is negative.
     */
    public Beacon(final List<BeaconIdentifier> identifiers, final double transmittedPower, final double frequency) {
        this(identifiers, transmittedPower);

        if (frequency < 0.0) {
            throw new IllegalArgumentException();
        }

        this.frequency = frequency;
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multipart identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency        frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac address.
     * @param beaconTypeCode   the two byte value indicating the type of beacon.
     * @param manufacturer     a two byte code indicating the beacon manufacturer.
     * @param serviceUuid      a 32 bit service uuid for the beacon.
     * @param bluetoothName    the bluetooth device name.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public Beacon(final List<BeaconIdentifier> identifiers, final double transmittedPower, final double frequency,
                  final String bluetoothAddress, final int beaconTypeCode, final int manufacturer,
                  final int serviceUuid, final String bluetoothName) {
        this(identifiers, transmittedPower, bluetoothAddress, beaconTypeCode, manufacturer, serviceUuid, bluetoothName);

        if (frequency < 0.0) {
            throw new IllegalArgumentException();
        }

        this.frequency = frequency;
    }

    /**
     * Empty constructor.
     */
    protected Beacon() {
    }

    /**
     * Returns the specified identifier - 0 indexed.
     * Note: to read id1, call getIdentifier(0);
     *
     * @param i index identifier.
     * @return identifier or null if not available.
     */
    public BeaconIdentifier getIdentifier(final int i) {
        if (i < 0 || identifiers == null || identifiers.size() <= i) {
            return null;
        }
        return identifiers.get(i);
    }

    /**
     * Convenience method to get the first identifier.
     *
     * @return first identifier or null if not available.
     */
    public BeaconIdentifier getId1() {
        if (identifiers == null || identifiers.isEmpty()) {
            return null;
        }
        return identifiers.get(0);
    }

    /**
     * Convenience method to get the second identifier.
     *
     * @return second identifier or null if not available.
     */
    public BeaconIdentifier getId2() {
        if (identifiers == null || identifiers.size() < 2) {
            return null;
        }
        return identifiers.get(1);
    }

    /**
     * Convenience method to get the third identifier.
     *
     * @return third identifier or null if not available.
     */
    public BeaconIdentifier getId3() {
        if (identifiers == null || identifiers.size() < 3) {
            return null;
        }
        return identifiers.get(2);
    }

    /**
     * Gets the list of the multipart identifiers of the beacon. Together, these identifiers signify
     * a unique beacon. The identifiers are ordered by significance for the purpose of grouping
     * beacons.
     *
     * @return list of identifiers of the beacon or null if not available.
     */
    public List<BeaconIdentifier> getIdentifiers() {
        return identifiers != null ? Collections.unmodifiableList(identifiers) : null;
    }

    /**
     * Returns the calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * This value is baked into a Beacon when it is manufactured, and it is
     * transmitted with each packet to aid in the distance estimate.
     *
     * @return the calibrated measured Tx power.
     */
    public double getTransmittedPower() {
        return transmittedPower;
    }

    /**
     * Gets the bluetooth mac address.
     *
     * @return bluetooth mac address.
     */
    public String getBluetoothAddress() {
        return bluetoothAddress;
    }

    /**
     * Gets the two byte value indicating the type of beacon that this is, which is used for figuring
     * out the byte layout of the beacon advertisement.
     *
     * @return two byte value indicating the type of beacon.
     */
    public int getBeaconTypeCode() {
        return beaconTypeCode;
    }


    /**
     * Gets the Bluetooth device name. This is a field transmitted by the remote beacon device separate
     * from the advertisement data.
     *
     * @return bluetooth device name.
     */
    public String getBluetoothName() {
        return bluetoothName;
    }

    /**
     * Gets a two byte code indicating the beacon manufacturer. A list of registered manufacturer codes
     * may be found here:
     * <a href="https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers">
     *   https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
     * </a>
     * <p>
     * If the beacon is a GATT-based beacon, this field will be set to -1.
     *
     * @return a two byte code indicating the beacon manufacturer.
     */
    public int getManufacturer() {
        return manufacturer;
    }

    /**
     * Gets a 32 bit service uuid for the beacon.
     * This is valid only for GATT-based beacons. If the beacon is a manufacturer data-based
     * beacon, this field will be -1
     *
     * @return service uuid.
     */
    public int getServiceUuid() {
        return serviceUuid;
    }

    /**
     * Gets frequency used by this radio source (expressed in Hz).
     *
     * @return frequency used by this radio source (expressed in Hz).
     */
    @Override
    public double getFrequency() {
        return frequency;
    }

    /**
     * Checks whether two beacons are considered equal if they share the same identifiers.
     *
     * @param that beacon to be compared.
     * @return true if both beacons are considered equal, false otherwise.
     */
    @Override
    public boolean equals(final Object that) {
        if (!(that instanceof Beacon thatBeacon)) {
            return false;
        }

        return identifiers != null && identifiers.equals(thatBeacon.identifiers);
    }

    /**
     * Computes hash code for this instance.
     *
     * @return this instance hash code.
     */
    @Override
    public int hashCode() {
        return identifiers.hashCode();
    }

    /**
     * Gets radio source type, which can be either a Wi-Fi Access point or a bluetooth Beacon.
     *
     * @return radio source type.
     */
    @Override
    public RadioSourceType getType() {
        return RadioSourceType.BEACON;
    }
}
