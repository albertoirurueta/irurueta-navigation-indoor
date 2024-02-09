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
package com.irurueta.navigation.indoor.radiosource;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated3D;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustRangingAndRssiRadioSourceEstimator3DTest implements
        RobustRangingAndRssiRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final double MAX_RSSI = -50.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustRangingAndRssiRadioSourceEstimator3D<WifiAccessPoint> estimator =
                RobustRangingAndRssiRadioSourceEstimator3D.create(RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with readings and method
        final List<RangingAndRssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
            readings.add(new RangingAndRssiReadingLocated3D<>(accessPoint,
                    0.0, 0.0, position));
        }

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with readings, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with readings, initial position and method

        // RANSAC
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with initial position and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with initial position, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(
                initialPosition, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(
                initialPosition, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // check with readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, RobustEstimatorMethod.LMEDS);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, RobustEstimatorMethod.MSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, RobustEstimatorMethod.PROSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, RobustEstimatorMethod.PROMEDS);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with readings, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // crate with readings, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with readings, initial position, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with initial position, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with readings, initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with readings, initial position, initial transmitted power,
        // initial path loss exponent and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with initial position, initial transmitted power,
        // initial path loss exponent and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with readings, initial position, initial transmitted power,
        // initial path loss exponent, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores and method
        final double[] qualityScores = new double[5];

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, initial position and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, initial position and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, initial position, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMeds
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, initial position, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, initial position, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, initial position,
        // initial transmitted power, initial path loss exponent and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, initial position, initial transmitted power,
        // initial path loss exponent and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, initial position, initial transmitted power,
        // initial path loss exponent, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with quality scores, readings, initial position,
        // initial transmitted power, initial path loss exponent, listener and
        // method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiRadioSourceEstimator3D);

        // create with default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create();

        // check
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(this);

        // check
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, this);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, this);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, this);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(MAX_RSSI, this);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, MAX_RSSI, this);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI, this);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                this);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings,
                initialPosition, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, MAX_RSSI, this);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, MAX_RSSI, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition,
                MAX_RSSI, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, initial transmitted power, listener and
        // default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, initial transmitted power, listener and
        // default method
        estimator = RobustRangingAndRssiRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());
    }

    @Override
    public void onEstimateStart(final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
    }

    @Override
    public void onEstimateEnd(final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
    }

    @Override
    public void onEstimateNextIteration(
            final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator, final int iteration) {
    }

    @Override
    public void onEstimateProgressChange(
            final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point3D> estimator, final float progress) {
    }
}
