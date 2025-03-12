/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RobustRangingPositionEstimator3DTest implements RobustRangingPositionEstimatorListener<Point3D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var estimator = RobustRangingPositionEstimator3D.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);

        // create with sources and method
        final var sources = new ArrayList<WifiAccessPointLocated3D>();
        for (var i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY, new InhomogeneousPoint3D()));
        }

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sources, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sources, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());

        // create with fingerprint and method
        final var fingerprint = new RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>();

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());

        // create with sources, fingerprint and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(this, estimator.getListener());

        // create with sources, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sources, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sources, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with sources, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores and method
        final var sourceQualityScores = new double[4];
        final var fingerprintReadingQualityScores = new double[4];

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // create with quality scores, sources and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // create with quality scores, fingerprint and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, sources, fingerprint and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with quality scores, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustRangingPositionEstimator3D.class, estimator);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with default method
        estimator = RobustRangingPositionEstimator3D.create();

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);

        // create with sources and default method
        estimator = RobustRangingPositionEstimator3D.create(sources);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());


        // create with fingerprint and default method
        estimator = RobustRangingPositionEstimator3D.create(fingerprint);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());

        // create with sources, fingerprint and default method
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with listener and default method
        estimator = RobustRangingPositionEstimator3D.create(this);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(this, estimator.getListener());

        // create with sources, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sources, this);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with fingerprint, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(fingerprint, this);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with sources, fingerprint, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sources, fingerprint, this);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // create with quality scores, sources and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // create with quality scores, fingerprint and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, sources, fingerprint and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                this);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, this);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with quality scores, fingerprint, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                fingerprint, this);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, fingerprint, listener and default
        // method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores, fingerprintReadingQualityScores,
                sources, fingerprint, this);

        // check
        assertInstanceOf(PROMedSRobustRangingPositionEstimator3D.class, estimator);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
    }

    @Override
    public void onEstimateStart(final RobustRangingPositionEstimator<Point3D> estimator) {
        // no action needed
    }

    @Override
    public void onEstimateEnd(final RobustRangingPositionEstimator<Point3D> estimator) {
        // no action needed
    }

    @Override
    public void onEstimateNextIteration(final RobustRangingPositionEstimator<Point3D> estimator, final int iteration) {
        // no action needed
    }

    @Override
    public void onEstimateProgressChange(final RobustRangingPositionEstimator<Point3D> estimator,
                                         final float progress) {
        // no action needed
    }
}
