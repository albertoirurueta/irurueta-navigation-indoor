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
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustMixedPositionEstimator3DTest implements RobustMixedPositionEstimatorListener<Point3D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustMixedPositionEstimator3D estimator = RobustMixedPositionEstimator3D.create(
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);

        // create with sources and method
        final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY, new InhomogeneousPoint3D()));
        }

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sources, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sources, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());

        // create with fingerprint and method
        final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint = new Fingerprint<>();

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());

        // create with sources, fingerprint and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with listener and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(this, estimator.getListener());

        // create with sources, listener and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sources, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sources, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with fingerprint, listener and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with sources, fingerprint, listener and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores and method
        final double[] sourceQualityScores = new double[4];
        final double[] fingerprintReadingQualityScores = new double[4];

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // create with quality scores, sources and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // create with quality scores, fingerprint and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, sources, fingerprint and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, listener and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with quality scores, fingerprint, listener and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, fingerprint, listener and method

        // RANSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // LMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(estimator instanceof LMedSRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // MSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustMixedPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROSAC
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // PROMedS
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with default method
        estimator = RobustMixedPositionEstimator3D.create();

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);

        // create with sources and default method
        estimator = RobustMixedPositionEstimator3D.create(sources);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());

        // create with fingerprint and default method
        estimator = RobustMixedPositionEstimator3D.create(fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());

        // create with sources, fingerprint and default method
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with listener and default method
        estimator = RobustMixedPositionEstimator3D.create(this);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(this, estimator.getListener());

        // create with sources, listener and default method
        estimator = RobustMixedPositionEstimator3D.create(sources, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with fingerprint, listener and default method
        estimator = RobustMixedPositionEstimator3D.create(fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with sources, fingerprint, listener and default method
        estimator = RobustMixedPositionEstimator3D.create(sources, fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores and default method
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());

        // create with quality scores, sources and default method
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());

        // create with quality scores, fingerprint and default method
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, sources, fingerprint and default method
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());

        // create with quality scores, listener and default method
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, listener and default method
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(this, estimator.getListener());

        // create with quality scores, fingerprint, listener and default method
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());

        // create with quality scores, sources, fingerprint, listener and default
        // method
        estimator = RobustMixedPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustMixedPositionEstimator3D);
        assertSame(sourceQualityScores, estimator.getSourceQualityScores());
        assertSame(fingerprintReadingQualityScores, estimator.getFingerprintReadingsQualityScores());
        assertSame(sources, estimator.getSources());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(this, estimator.getListener());
    }

    @Override
    public void onEstimateStart(final RobustMixedPositionEstimator<Point3D> estimator) {
    }

    @Override
    public void onEstimateEnd(final RobustMixedPositionEstimator<Point3D> estimator) {
    }

    @Override
    public void onEstimateNextIteration(
            final RobustMixedPositionEstimator<Point3D> estimator, final int iteration) {
    }

    @Override
    public void onEstimateProgressChange(
            final RobustMixedPositionEstimator<Point3D> estimator, final float progress) {
    }
}
