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
import com.irurueta.navigation.indoor.RangingReadingLocated2D;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RobustRangingRadioSourceEstimator2DTest implements
        RobustRangingRadioSourceEstimatorListener<WifiAccessPoint, Point2D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var estimator = RobustRangingRadioSourceEstimator2D.<WifiAccessPoint>create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // crete with readings and method
        final var readings = new ArrayList<RangingReadingLocated2D<WifiAccessPoint>>();
        final var accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (var i = 0; i < 4; i++) {
            final var position = new InhomogeneousPoint2D();
            readings.add(new RangingReadingLocated2D<>(accessPoint, 0.0, position));
        }

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with readings, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with initial position and method
        final var initialPosition = new InhomogeneousPoint2D();

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with readings, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with quality scores and method
        final var qualityScores = new double[4];

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // crete with quality scores, readings and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with quality scores, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // create with quality scores, readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(RANSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(LMedSRobustRangingRadioSourceEstimator2D.class, estimator);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(MSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROSACRobustRangingRadioSourceEstimator2D.class, estimator);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings, initialPosition, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, estimator.getQualityScores());
        assertSame(readings, estimator.getReadings());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
        assertInstanceOf(PROMedSRobustRangingRadioSourceEstimator2D.class, estimator);
    }

    @Override
    public void onEstimateStart(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        // no action needed
    }

    @Override
    public void onEstimateEnd(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) {
        // no action needed
    }

    @Override
    public void onEstimateNextIteration(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator,
                                        final int iteration) {
        // no action needed
    }

    @Override
    public void onEstimateProgressChange(final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator,
                                         final float progress) {
        // no action needed
    }
}
