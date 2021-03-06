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

import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class RobustRangingPositionEstimator3DTest implements
        RobustRangingPositionEstimatorListener<Point3D> {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustRangingPositionEstimator3D estimator =
                RobustRangingPositionEstimator3D.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);

        // create with sources and method
        final List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sources,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sources,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);

        // create with fingerprint and method
        final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);

        // create with sources, fingerprint and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // create with listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getListener(), this);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getListener(), this);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getListener(), this);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getListener(), this);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getListener(), this);

        // create with sources, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sources,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sources,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // create with fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // create with sources, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // create with quality scores and method
        final double[] sourceQualityScores = new double[4];
        final double[] fingerprintReadingQualityScores = new double[4];

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);

        // create with quality scores, sources and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);

        // create with quality scores, fingerprint and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getFingerprint(), fingerprint);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getFingerprint(), fingerprint);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getFingerprint(), fingerprint);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getFingerprint(), fingerprint);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getFingerprint(), fingerprint);

        // create with quality scores, sources, fingerprint and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getListener(), this);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getListener(), this);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getListener(), this);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getListener(), this);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getListener(), this);

        // create with quality scores, sources, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // create with quality scores, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // create with quality scores, sources, fingerprint, listener and method

        // RANSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // LMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // MSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingPositionEstimator3D);
        assertNull(estimator.getSourceQualityScores());
        assertNull(estimator.getFingerprintReadingsQualityScores());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // PROSAC
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // PROMedS
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // create with default method
        estimator = RobustRangingPositionEstimator3D.create();

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);

        // create with sources and default method
        estimator = RobustRangingPositionEstimator3D.create(sources);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);


        // create with fingerprint and default method
        estimator = RobustRangingPositionEstimator3D.create(fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);

        // create with sources, fingerprint and default method
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // create with listener and default method
        estimator = RobustRangingPositionEstimator3D.create(this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getListener(), this);

        // create with sources, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sources,
                this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // create with fingerprint, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(fingerprint,
                this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // create with sources, fingerprint, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sources,
                fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // create with quality scores and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);

        // create with quality scores, sources and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);

        // create with quality scores, fingerprint and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getFingerprint(), fingerprint);

        // create with quality scores, sources, fingerprint and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        // create with quality scores, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getListener(), this);

        // create with quality scores, sources, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        // create with quality scores, fingerprint, listener and default method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        // create with quality scores, sources, fingerprint, listener and default
        // method
        estimator = RobustRangingPositionEstimator3D.create(sourceQualityScores,
                fingerprintReadingQualityScores, sources, fingerprint, this);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingPositionEstimator3D);
        assertSame(estimator.getSourceQualityScores(), sourceQualityScores);
        assertSame(estimator.getFingerprintReadingsQualityScores(),
                fingerprintReadingQualityScores);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
    }

    @Override
    public void onEstimateStart(
            final RobustRangingPositionEstimator<Point3D> estimator) {
    }

    @Override
    public void onEstimateEnd(
            final RobustRangingPositionEstimator<Point3D> estimator) {
    }

    @Override
    public void onEstimateNextIteration(
            final RobustRangingPositionEstimator<Point3D> estimator,
            final int iteration) {
    }

    @Override
    public void onEstimateProgressChange(
            final RobustRangingPositionEstimator<Point3D> estimator,
            final float progress) {
    }
}
