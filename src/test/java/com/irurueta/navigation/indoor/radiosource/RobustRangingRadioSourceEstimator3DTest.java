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
import com.irurueta.navigation.indoor.RangingReadingLocated3D;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustRangingRadioSourceEstimator3DTest implements
        RobustRangingRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator = RobustRangingRadioSourceEstimator3D.create(
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // crete with readings and method
        final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 4; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with readings, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with initial position and method
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with readings, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with quality scores and method
        final double[] qualityScores = new double[4];

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // crete with quality scores, readings and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with quality scores, readings, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with quality scores, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with quality scores, readings, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with quality scores, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);

        // create with quality scores, readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings, initialPosition,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);
    }

    @Override
    public void onEstimateStart(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
    }

    @Override
    public void onEstimateEnd(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
    }

    @Override
    public void onEstimateNextIteration(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                        final int iteration) {
    }

    @Override
    public void onEstimateProgressChange(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
                                         final float progress) {
    }
}
