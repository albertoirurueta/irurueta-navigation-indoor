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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.RangingAndRssiReadingLocated2D;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RobustRangingAndRssiRadioSourceEstimator2DTest implements
        RobustRangingAndRssiRadioSourceEstimatorListener<WifiAccessPoint, Point2D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final double MAX_RSSI = -50.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var estimator = RobustRangingAndRssiRadioSourceEstimator2D.<WifiAccessPoint>create(
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with readings and method
        final var readings = new ArrayList<RangingAndRssiReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 4; i++) {
            final var position = new InhomogeneousPoint2D();
            readings.add(new RangingAndRssiReadingLocated2D<>(accessPoint, 0.0, 0.0, position));
        }

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with readings, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with readings, initial position and method

        // RANSAC
        final var initialPosition = new InhomogeneousPoint2D();
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with initial position and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with initial position, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // check with readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, RobustEstimatorMethod.LMEDS);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, RobustEstimatorMethod.MSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, RobustEstimatorMethod.PROSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, RobustEstimatorMethod.PROMEDS);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with readings, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, RobustEstimatorMethod.MSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // crate with readings, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with readings, initial position, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with initial position, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with readings, initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with readings, initial position, initial transmitted power,
        // initial path loss exponent and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with initial position, initial transmitted power,
        // initial path loss exponent and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with readings, initial position, initial transmitted power,
        // initial path loss exponent, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores and method
        final var qualityScores = new double[4];

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial position and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial position and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial position, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMeds
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial position, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial position, initial transmitted power and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial position, initial transmitted power, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial position,
        // initial transmitted power, initial path loss exponent and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial position, initial transmitted power,
        // initial path loss exponent and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial position, initial transmitted power,
        // initial path loss exponent, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial position,
        // initial transmitted power, initial path loss exponent, listener and
        // method

        // RANSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingAndRssiRadioSourceEstimator2D.class, estimator);

        // create with default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create();

        // check
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings);

        // check
        assertSame(estimator.getReadings(), readings);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(this);

        // check
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, this);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, this);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, this);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(MAX_RSSI, this);

        // check
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, MAX_RSSI, this);

        // check
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI, this);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                this);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(initialPosition,
                MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with readings, initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(readings, initialPosition, MAX_RSSI,
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
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, MAX_RSSI, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, MAX_RSSI, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                MAX_RSSI);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, initial transmitted power and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, initial transmitted power, listener
        // and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
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
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
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
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, initial position, initial transmitted power, listener and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertEquals(MAX_RSSI, estimator.getInitialTransmittedPowerdBm(), 0.0);
        assertEquals(Utils.dBmToPower(MAX_RSSI), estimator.getInitialTransmittedPower(), 0.0);
        assertEquals(MIN_PATH_LOSS_EXPONENT, estimator.getInitialPathLossExponent(), 0.0);
        assertSame(this, estimator.getListener());
        assertEquals(RobustRssiRadioSourceEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // create with quality scores, readings, initial position, initial transmitted power, listener
        // and default method
        estimator = RobustRangingAndRssiRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
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
    public void onEstimateStart(final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        // no action needed
    }

    @Override
    public void onEstimateEnd(final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        // no action needed
    }

    @Override
    public void onEstimateNextIteration(
            final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator, final int iteration) {
        // no action needed
    }

    @Override
    public void onEstimateProgressChange(
            final RobustRangingAndRssiRadioSourceEstimator<WifiAccessPoint, Point2D> estimator, final float progress) {
        // no action needed
    }
}
