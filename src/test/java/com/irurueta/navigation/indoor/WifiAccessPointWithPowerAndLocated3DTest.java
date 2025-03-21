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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class WifiAccessPointWithPowerAndLocated3DTest {

    private static final String BSSID = "bssid";
    private static final String SSID = "ssid";
    private static final double FREQUENCY = 2.4e9;
    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;
    private static final double TRANSMITTED_POWER = -50.0;
    private static final double TRANSMITTED_POWER_STD = 0.5;
    private static final double PATH_LOSS_STD = 0.1;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    @Test
    void testConstructor() throws AlgebraException {
        // test empty constructor
        var ap = new WifiAccessPointWithPowerAndLocated3D();

        // check default values
        assertNull(ap.getBssid());
        assertEquals(0.0, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(0.0, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertNull(ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // test with bssid, frequency, transmitted power and position
        final var randomizer = new UniformRandomizer();
        final var position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, null));

        // test with bssid, frequency, ssid, transmitted power and position
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, SSID, TRANSMITTED_POWER, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                SSID, TRANSMITTED_POWER, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, null));

        // test with bssid, frequency, transmitted power, transmitted power standard deviation
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER,
                Double.valueOf(TRANSMITTED_POWER_STD), position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER,
                null, position);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        final var std = Double.valueOf(TRANSMITTED_POWER_STD);
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, std, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, std, position));
        final var wrongStd = Double.valueOf(-TRANSMITTED_POWER_STD);
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, wrongStd, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, std, null));

        // test with bssid, frequency, ssid, transmitted power and transmitted power standard deviation
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER,
                null, position);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, null));

        // test with bssid, frequency, transmitted power, position and position covariance
        final var cov = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, position, cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, position,
                null);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, null, cov));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, position, m));

        // test with bssid, frequency, ssid, transmitted power, position and position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, position, cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, position,
                null);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, SSID, TRANSMITTED_POWER, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                SSID, TRANSMITTED_POWER, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, position, m));

        // test with bssid, frequency, transmitted power, transmitted power standard deviation, position and position
        // covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER,
                Double.valueOf(TRANSMITTED_POWER_STD), position, cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER,
                null, position, null);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, std, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, wrongStd, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, std, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, std, position, m));

        // test with bssid, frequency, ssid, transmitted power, transmitted power standard deviation, position and
        // position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                position, cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER,
                null, position, null);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(WifiAccessPointWithPower.DEFAULT_PATH_LOSS_EXPONENT, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position, m));

        // test with bssid, frequency, transmitted power, path-loss and position
        final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, pathLossExponent, position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, pathLossExponent, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, pathLossExponent, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, pathLossExponent, null));

        // test with bssid, frequency, transmitted power, transmitted power standard deviation, path loss exponent and
        // position
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER,
                null, pathLossExponent, position);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, null));

        // test with bssid, frequency, ssid, transmitted power, transmitted power standard deviation, path loss
        // exponent and position
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER,
                null, pathLossExponent, position);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, null));

        // test with bssid, frequency, transmitted power, path loss, position and position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, pathLossExponent, position,
                cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, pathLossExponent, position,
                null);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, pathLossExponent, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, pathLossExponent, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, pathLossExponent, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, pathLossExponent, position, m));

        // test with bssid, frequency, transmitted power, transmitted power standard deviation, path loss exponent,
        // position and position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, position, cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER,
                null, pathLossExponent, position, null);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position, m));

        // test with bssid, frequency, ssid, transmitted power, transmitted power standard deviation, path loss
        // exponent, position and position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, position, cov);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER,
                null, pathLossExponent, position, null);

        // check default values
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, position, m));

        // test constructor with bssid, frequency, tx power, tx power std deviation, path-loss, path-loss std deviation
        // and position
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, PATH_LOSS_STD, position);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertEquals(PATH_LOSS_STD, ap.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER,
                null, pathLossExponent, null, position);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // fail IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, -PATH_LOSS_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, null));

        // test constructor with bssid, frequency, ssid, tx power, tx power std deviation, path-loss, path-loss std
        // deviation and position
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, PATH_LOSS_STD, position);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertEquals(PATH_LOSS_STD, ap.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER,
                null, pathLossExponent, null, position);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // fail IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, -PATH_LOSS_STD, position));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, null));

        // test constructor with bssid, frequency, tx power, tx power std deviation, path-loss, path-loss std deviation,
        // position and position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, PATH_LOSS_STD, position, cov);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertEquals(PATH_LOSS_STD, ap.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, TRANSMITTED_POWER,
                null, pathLossExponent, null, position,
                null);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertNull(ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // fail IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, -PATH_LOSS_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position, m));

        // test constructor with bssid, frequency, ssid, tx power, tx power std deviation,
        // path-loss, path-loss std deviation, position and position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD,
                pathLossExponent, PATH_LOSS_STD, position, cov);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap.getPosition());
        assertSame(cov, ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertEquals(PATH_LOSS_STD, ap.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER,
                null, pathLossExponent, null, position,
                null);

        // check
        assertEquals(BSSID, ap.getBssid());
        assertEquals(FREQUENCY, ap.getFrequency(), 0.0);
        assertEquals(SSID, ap.getSsid());
        assertEquals(TRANSMITTED_POWER, ap.getTransmittedPower(), 0.0);
        assertNull(ap.getTransmittedPowerStandardDeviation());
        assertSame(position, ap.getPosition());
        assertNull(ap.getPositionCovariance());
        assertEquals(pathLossExponent, ap.getPathLossExponent(), 0.0);
        assertNull(ap.getPathLossExponentStandardDeviation());
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap.getType());

        // fail IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(null,
                FREQUENCY, SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position,
                cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, -PATH_LOSS_STD, position, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, null, cov));
        assertThrows(IllegalArgumentException.class, () -> new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position, m));
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final var ap1 = new WifiAccessPointWithPowerAndLocated3D("bssid1", FREQUENCY, TRANSMITTED_POWER,
                position);
        final var ap2 = new WifiAccessPointWithPowerAndLocated3D("bssid1", FREQUENCY, TRANSMITTED_POWER,
                position);
        final var ap3 = new WifiAccessPointWithPowerAndLocated3D("bssid2", FREQUENCY, TRANSMITTED_POWER,
                position);

        // check
        //noinspection EqualsWithItself
        assertEquals(ap1, ap1);
        assertEquals(ap1, ap2);
        assertNotEquals(ap1, ap3);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final var ap1 = new WifiAccessPointWithPowerAndLocated3D("bssid1", FREQUENCY, TRANSMITTED_POWER,
                position);
        final var ap2 = new WifiAccessPointWithPowerAndLocated3D("bssid1", FREQUENCY, TRANSMITTED_POWER,
                position);
        final var ap3 = new WifiAccessPointWithPowerAndLocated3D("bssid2", FREQUENCY, TRANSMITTED_POWER,
                position);

        // check
        assertEquals(ap1.hashCode(), ap2.hashCode());
        assertNotEquals(ap1.hashCode(), ap3.hashCode());
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
        final var cov = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);

        final var ap1 = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID, TRANSMITTED_POWER,
                TRANSMITTED_POWER_STD, pathLossExponent, PATH_LOSS_STD, position, cov);

        // check
        assertEquals(BSSID, ap1.getBssid());
        assertEquals(FREQUENCY, ap1.getFrequency(), 0.0);
        assertEquals(SSID, ap1.getSsid());
        assertEquals(TRANSMITTED_POWER, ap1.getTransmittedPower(), 0.0);
        assertEquals(TRANSMITTED_POWER_STD, ap1.getTransmittedPowerStandardDeviation(), 0.0);
        assertSame(position, ap1.getPosition());
        assertSame(cov, ap1.getPositionCovariance());
        assertEquals(pathLossExponent, ap1.getPathLossExponent(), 0.0);
        assertEquals(PATH_LOSS_STD, ap1.getPathLossExponentStandardDeviation(), 0.0);
        assertEquals(RadioSourceType.WIFI_ACCESS_POINT, ap1.getType());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(ap1);
        final var ap2 = SerializationHelper.<WifiAccessPointWithPowerAndLocated3D>deserialize(bytes);

        // check
        assertEquals(ap1, ap2);
        assertNotSame(ap1, ap2);
        assertEquals(ap1.getBssid(), ap2.getBssid());
        assertEquals(ap1.getFrequency(), ap2.getFrequency(), 0.0);
        assertEquals(ap1.getSsid(), ap2.getSsid());
        assertEquals(ap1.getTransmittedPower(), ap2.getTransmittedPower(), 0.0);
        assertEquals(ap1.getTransmittedPowerStandardDeviation(), ap2.getTransmittedPowerStandardDeviation(),
                0.0);
        assertEquals(ap1.getPosition(), ap2.getPosition());
        assertEquals(ap1.getPositionCovariance(), ap2.getPositionCovariance());
        assertEquals(ap1.getPathLossExponent(), ap2.getPathLossExponent(), 0.0);
        assertEquals(ap1.getPathLossExponentStandardDeviation(), ap2.getPathLossExponentStandardDeviation(),
                0.0);
        assertEquals(ap1.getType(), ap2.getType());
    }
}
