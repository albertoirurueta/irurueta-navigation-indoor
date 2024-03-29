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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class NonLinearFingerprintPositionEstimator3DTest implements
        FingerprintPositionEstimatorListener<Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            NonLinearFingerprintPositionEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final int MIN_FINGERPRINTS = 100;
    private static final int MAX_FINGERPRINTS = 1000;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 100;

    @Test
    public void testCreate() {
        // test create empty

        // first order
        NonLinearFingerprintPositionEstimator3D estimator =
                NonLinearFingerprintPositionEstimator3D.create(
                        NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator3D.create(
                NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator3D.create(
                NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator3D.create();

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // test create with listener

        // first order
        estimator = NonLinearFingerprintPositionEstimator3D.create(this,
                NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator3D.create(this,
                NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator3D.create(this,
                NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator3D.create(this);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // test create with located fingerprints, fingerprint and sources
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final List<RssiReading<RadioSource>> readings = new ArrayList<>();
        for (int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i, FREQUENCY);
            final double rssi = randomizer.nextDouble();

            final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        final RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                new RssiFingerprintLocated3D<>(readings, Point3D.create());

        final List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                new ArrayList<>();
        locatedFingerprints.add(locatedFingerprint);

        final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);

        final List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
        for (int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            final WifiAccessPointLocated3D source = new WifiAccessPointLocated3D("bssid" + 1,
                    FREQUENCY, Point3D.create());
            sources.add(source);
        }

        // first order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // test create with located fingerprints, fingerprint, sources and listener

        // first order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, this, NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, this, NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, this, NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, this);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // test create with located fingerprints, fingerprint, sources and initial position
        final Point3D initialPosition = Point3D.create();

        // first order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, initialPosition,
                NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, initialPosition,
                NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, initialPosition,
                NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, initialPosition);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertNull(estimator.getListener());

        // test create with located fingerprints, fingerprint, sources, initial position and listener

        // first order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, initialPosition, this,
                NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.FIRST_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // second order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, initialPosition, this,
                NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.SECOND_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // third order
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, initialPosition, this,
                NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());

        // default type
        estimator = NonLinearFingerprintPositionEstimator3D.create(locatedFingerprints,
                fingerprint, sources, initialPosition, this);

        // check
        assertEquals(NonLinearFingerprintPositionEstimatorType.THIRD_ORDER, estimator.getType());
        assertSame(locatedFingerprints, estimator.getLocatedFingerprints());
        assertSame(fingerprint, estimator.getFingerprint());
        assertSame(sources, estimator.getSources());
        assertSame(initialPosition, estimator.getInitialPosition());
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testEstimateOrderComparison() throws LockedException, NotReadyException,
            FingerprintEstimationException {
        double avgRssiErrorFirstOrder = 0.0;
        double avgRssiErrorSecondOrder = 0.0;
        double avgRssiErrorThirdOrder = 0.0;

        double avgPositionErrorFirstOrder = 0.0;
        double avgPositionErrorSecondOrder = 0.0;
        double avgPositionErrorThirdOrder = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // build sources
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            final List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                final WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            // build located fingerprints
            final int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                final double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                final InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                final List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (final RadioSourceLocated<Point3D> source : sources) {
                    final double distance = source.getPosition().distanceTo(position);
                    final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    final double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance));
                    final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                final RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            // build non-located fingerprint
            final double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double z = randomizer.nextDouble(MIN_POS, MAX_POS);
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

            final List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (final RadioSourceLocated<Point3D> source : sources) {
                final double distance = source.getPosition().distanceTo(position);
                final double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                final double receivedRssi = Utils.powerTodBm(
                        receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance));
                final RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            final RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find the closest fingerprint based on RSSI without mean
            final RadioSourceNoMeanKNearestFinder<Point3D, RadioSource> fingerprintFinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            final RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprint =
                    fingerprintFinder.findNearestTo(fingerprint);
            final Point3D nearestFingerprintPosition = nearestFingerprint.getPosition();


            final NonLinearFingerprintPositionEstimator3D firstOrderEstimator =
                    NonLinearFingerprintPositionEstimator3D.create(
                            locatedFingerprints, fingerprint, sources, this,
                            NonLinearFingerprintPositionEstimatorType.FIRST_ORDER);
            final NonLinearFingerprintPositionEstimator3D secondOrderEstimator =
                    NonLinearFingerprintPositionEstimator3D.create(
                            locatedFingerprints, fingerprint, sources, this,
                            NonLinearFingerprintPositionEstimatorType.SECOND_ORDER);
            final NonLinearFingerprintPositionEstimator3D thirdOrderEstimator =
                    NonLinearFingerprintPositionEstimator3D.create(
                            locatedFingerprints, fingerprint, sources, this,
                            NonLinearFingerprintPositionEstimatorType.THIRD_ORDER);

            // compute average RSSI error for each order
            final double[] params = new double[3];
            params[0] = x;
            params[1] = y;
            params[2] = z;

            final double[] point = new double[8];
            final double[] derivatives = new double[3];
            for (int i = 0; i < numSources; i++) {
                final RadioSourceLocated<Point3D> source = sources.get(i);
                final RssiReading<RadioSource> reading = readings.get(i);

                final Point3D sourcePosition = source.getPosition();

                final double readingRssi = reading.getRssi();

                point[0] = readingRssi;

                point[1] = nearestFingerprintPosition.getInhomX();
                point[2] = nearestFingerprintPosition.getInhomY();
                point[3] = nearestFingerprintPosition.getInhomZ();

                point[4] = sourcePosition.getInhomX();
                point[5] = sourcePosition.getInhomY();
                point[6] = sourcePosition.getInhomZ();

                point[7] = LinearFingerprintPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT;

                final double estimatedRssiFirstOrder = firstOrderEstimator.evaluate(
                        0, point, params, derivatives);
                final double estimatedRssiSecondOrder = secondOrderEstimator.evaluate(
                        0, point, params, derivatives);
                final double estimatedRssiThirdOrder = thirdOrderEstimator.evaluate(
                        0, point, params, derivatives);

                final double rssiErrorFirstOrder = Math.abs(estimatedRssiFirstOrder - readingRssi);
                final double rssiErrorSecondOrder = Math.abs(estimatedRssiSecondOrder - readingRssi);
                final double rssiErrorThirdOrder = Math.abs(estimatedRssiThirdOrder - readingRssi);

                avgRssiErrorFirstOrder += rssiErrorFirstOrder / TIMES;
                avgRssiErrorSecondOrder += rssiErrorSecondOrder / TIMES;
                avgRssiErrorThirdOrder += rssiErrorThirdOrder / TIMES;

                firstOrderEstimator.estimate();
                secondOrderEstimator.estimate();
                thirdOrderEstimator.estimate();

                final Point3D estimatedPositionFirstOrder = firstOrderEstimator.getEstimatedPosition();
                final Point3D estimatedPositionSecondOrder = secondOrderEstimator.getEstimatedPosition();
                final Point3D estimatedPositionThirdOrder = thirdOrderEstimator.getEstimatedPosition();

                final double positionErrorFirstOrder = estimatedPositionFirstOrder.distanceTo(position);
                final double positionErrorSecondOrder = estimatedPositionSecondOrder.distanceTo(position);
                final double positionErrorThirdOrder = estimatedPositionThirdOrder.distanceTo(position);

                avgPositionErrorFirstOrder += positionErrorFirstOrder / TIMES;
                avgPositionErrorSecondOrder += positionErrorSecondOrder / TIMES;
                avgPositionErrorThirdOrder += positionErrorThirdOrder / TIMES;
            }
        }

        LOGGER.log(Level.INFO, "Avg rssi error 1st order: {0} dBm", avgRssiErrorFirstOrder);
        LOGGER.log(Level.INFO, "Avg rssi error 2nd order: {0} dBm", avgRssiErrorSecondOrder);
        LOGGER.log(Level.INFO, "Avg rssi error 3rd order: {0} dBm", avgRssiErrorThirdOrder);

        LOGGER.log(Level.INFO, "Avg position error 1st order: {0} m", avgPositionErrorFirstOrder);
        LOGGER.log(Level.INFO, "Avg position error 2nd order: {0} m", avgPositionErrorSecondOrder);
        LOGGER.log(Level.INFO, "Avg position error 3rd order: {0} m", avgPositionErrorThirdOrder);

        assertTrue(avgPositionErrorFirstOrder >= avgPositionErrorSecondOrder);
        assertTrue(avgPositionErrorFirstOrder >= avgPositionErrorThirdOrder);
    }

    @Override
    public void onEstimateStart(final FingerprintPositionEstimator<Point3D> estimator) {
        checkLocked((NonLinearFingerprintPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateEnd(final FingerprintPositionEstimator<Point3D> estimator) {
        checkLocked((NonLinearFingerprintPositionEstimator3D) estimator);
    }

    private double receivedPower(final double equivalentTransmittedPower,
                                 final double distance) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY),
                BaseFingerprintPositionEstimator.DEFAULT_PATH_LOSS_EXPONENT);
        return equivalentTransmittedPower * k /
                Math.pow(distance, BaseFingerprintPositionEstimator.DEFAULT_PATH_LOSS_EXPONENT);
    }

    private void checkLocked(final NonLinearFingerprintPositionEstimator3D estimator) {
        try {
            estimator.setLocatedFingerprints(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMinMaxNearestFingerprints(1, 1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPathLossExponent(2.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMeansFromFingerprintReadingsRemoved(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFallbackRssiStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}

