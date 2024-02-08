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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.RangingAndRssiFingerprint;
import com.irurueta.navigation.indoor.RangingAndRssiReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustRangingAndRssiPositionEstimator2DTest implements
        RobustRangingAndRssiPositionEstimatorListener<Point2D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustRangingAndRssiPositionEstimator2D estimator =
                RobustRangingAndRssiPositionEstimator2D.create(RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);

        // create with sources and method
        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY, new InhomogeneousPoint2D()));
        }

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());

        // create with fingerprint and method
        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());

        // create with sources, fingerprint and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with listener and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(this, estimator.getListener());

        // create with sources, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with sources, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores and method
        final double[] sourceQualityScores = new double[3];
        final double[] fingerprintReadingQualityScores = new double[3];

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // create with quality scores, sources and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // create with quality scores, fingerprint and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, sources, fingerprint and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with quality scores, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingAndRssiPositionEstimator2D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create();

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);

        // create with sources and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());

        // create with fingerprint and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());

        // create with sources, fingerprint and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with listener and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(this, estimator.getListener());

        // create with sources, listener and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with fingerprint, listener and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with sources, fingerprint, listener and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sources, fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // create with quality scores, sources and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // create with quality scores, fingerprint and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, sources, fingerprint and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, listener and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, listener and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with quality scores, fingerprint, listener and default method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, fingerprint, listener and default
        // method
        estimator = RobustRangingAndRssiPositionEstimator2D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingAndRssiPositionEstimator2D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
    }

    @Override
    public void onEstimateStart(final RobustRangingAndRssiPositionEstimator<Point2D> estimator) {
    }

    @Override
    public void onEstimateEnd(final RobustRangingAndRssiPositionEstimator<Point2D> estimator) {
    }

    @Override
    public void onEstimateNextIteration(final RobustRangingAndRssiPositionEstimator<Point2D> estimator,
                                        final int iteration) {
    }

    @Override
    public void onEstimateProgressChange(final RobustRangingAndRssiPositionEstimator<Point2D> estimator,
                                         final float progress) {
    }
}
