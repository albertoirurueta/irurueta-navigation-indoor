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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RangingAndRssiFingerprint;
import com.irurueta.navigation.indoor.RangingAndRssiReading;
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.navigation.indoor.Utils;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated2D;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated2D;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.Test;

public class PositionEstimatorHelperTest {

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double FALLBACK_DISTANCE_STANDARD_DEVIATION = 1e-3;

    private static final double TX_POWER_VARIANCE = 0.1;
    private static final double RX_POWER_VARIANCE = 0.5;
    private static final double PATH_LOSS_EXPONENT_VARIANCE = 0.001;

    private static final double POSITION_VARIANCE = 0.01;

    @Test
    public void testBuildPositionsAndDistancesRssiReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint, positions,
                    distances);

            // check that positions and distances are not modified if no sources or
            // fingerprint are provided
            PositionEstimatorHelper.buildPositionsAndDistances(
                    null, null, positions, distances);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsAndDistancesRangingReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance));
            }

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint, positions, distances);

            // check that positions and distances are not modified if no sources or
            // fingerprint are provided
            PositionEstimatorHelper.buildPositionsAndDistances(
                    null, null, positions, distances);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertEquals(distances.get(i), readings.get(i).getDistance(), 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsAndDistancesRangingAndRssiReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint, positions, distances);

            // check that positions and distances are not modified if no sources or
            // fingerprint are provided
            PositionEstimatorHelper.buildPositionsAndDistances(
                    null, null, positions, distances);

            // check
            assertEquals(2 * numSources, positions.size());
            assertEquals(2 * numSources, distances.size());

            for (int i = 0, j = 0; i < numSources; i++, j += 2) {
                assertEquals(sources.get(i).getPosition(), positions.get(j));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 1));
                assertEquals(distances.get(j), distances.get(j + 1), ABSOLUTE_ERROR);
                assertEquals(distances.get(j), readings.get(i).getDistance(), 0.0);
                assertEquals(distances.get(j + 1), readings.get(i).getDistance(), ABSOLUTE_ERROR);
            }
        }
    }

    @Test
    public void testBuildPositionsAndDistancesNonRadioSourceWithPower() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated2D locatedAccessPoint =
                        new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint, positions, distances);

            // check
            assertTrue(positions.isEmpty());
            assertTrue(distances.isEmpty());
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsRssiReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());
            assertEquals(numSources, distanceStandardDeviations.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsRangingReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION));
            }

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());
            assertEquals(numSources, distanceStandardDeviations.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertEquals(distances.get(i), readings.get(i).getDistance(), 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsRangingAndRssiReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertEquals(2 * numSources, positions.size());
            assertEquals(2 * numSources, distances.size());
            assertEquals(2 * numSources, distanceStandardDeviations.size());

            for (int i = 0, j = 0; i < numSources; i++, j += 2) {
                assertEquals(sources.get(i).getPosition(), positions.get(j));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 1));

                assertEquals(distances.get(j), distances.get(j + 1), ABSOLUTE_ERROR);
                assertEquals(distances.get(j), readings.get(i).getDistance(), 0.0);
                assertEquals(distances.get(j + 1), readings.get(i).getDistance(), ABSOLUTE_ERROR);

                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsNoRadioSourceWithPower() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointLocated2D locatedAccessPoint =
                        new WifiAccessPointLocated2D(bssid, FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertTrue(positions.isEmpty());
            assertTrue(distances.isEmpty());
            assertTrue(distanceStandardDeviations.isEmpty());
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsForceIllegalArgumentException() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

        final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        final double pathLossExponent = randomizer.nextDouble(
                MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

        final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        for (int i = 0; i < numSources; i++) {
            final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            final String bssid = String.valueOf(i);

            final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                    new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                            Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                            Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
            sources.add(locatedAccessPoint);

            final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

            final double distance = position.distanceTo(accessPointPosition);

            final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                    pathLossExponent));

            readings.add(new RssiReading<>(accessPoint, rssi, Math.sqrt(RX_POWER_VARIANCE)));
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>(readings);

        final List<Point2D> positions = new ArrayList<>();
        final List<Double> distances = new ArrayList<>();
        final List<Double> distanceStandardDeviations = new ArrayList<>();
        try {
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    -FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsRssiReadingsWithPositionCovariance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final Matrix positionCovariance = Matrix.diagonal(
                        new double[]{POSITION_VARIANCE, POSITION_VARIANCE});

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition,
                                positionCovariance);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());
            assertEquals(numSources, distanceStandardDeviations.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsRangingReadingsWithPositionCovariance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final Matrix positionCovariance = Matrix.diagonal(
                        new double[]{POSITION_VARIANCE, POSITION_VARIANCE});

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition,
                                positionCovariance);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION));
            }

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());
            assertEquals(numSources, distanceStandardDeviations.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertEquals(distances.get(i), readings.get(i).getDistance(), 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsRssiReadingsNoVariances() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());
            assertEquals(numSources, distanceStandardDeviations.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertEquals(FALLBACK_DISTANCE_STANDARD_DEVIATION,
                        distanceStandardDeviations.get(i), 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsRangingReadingsNoVariances() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                pathLossExponent, accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance));
            }

            final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());
            assertEquals(numSources, distanceStandardDeviations.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertEquals(distances.get(i), readings.get(i).getDistance(), 0.0);
                assertEquals(FALLBACK_DISTANCE_STANDARD_DEVIATION,
                        distanceStandardDeviations.get(i), 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistancesStandardDeviationsInvalidPositionCovariance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final Matrix positionCovariance = Matrix.diagonal(
                        new double[]{Double.NaN, Double.NaN});

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition,
                                positionCovariance);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances, distanceStandardDeviations);

            // check
            assertEquals(numSources, positions.size());
            assertEquals(numSources, distances.size());
            assertEquals(numSources, distanceStandardDeviations.size());

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesDistanceStandardDeviationsAndSourcesQualityScoresReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourcesQualityScores = new double[numSources];
            for (int i = 0; i < numSources; i++) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);
                sourcesQualityScores[i] = randomizer.nextDouble();

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi, Math.sqrt(RX_POWER_VARIANCE)));
                readings.add(new RangingReading<>(accessPoint, distance,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION));
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, Math.sqrt(RX_POWER_VARIANCE)));
            }

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                    new Fingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            final List<Double> distanceQualityScores = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                    sources, fingerprint, sourcesQualityScores, null,
                    true, FALLBACK_DISTANCE_STANDARD_DEVIATION, positions,
                    distances, distanceStandardDeviations, distanceQualityScores);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                    null, null, sourcesQualityScores,
                    null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations, distanceQualityScores);

            // check
            assertEquals(4 * numSources, positions.size());
            assertEquals(4 * numSources, distances.size());
            assertEquals(4 * numSources, distanceStandardDeviations.size());
            assertEquals(4 * numSources, distanceQualityScores.size());

            for (int i = 0, j = 0; i < numSources; i++, j += 4) {
                assertEquals(sources.get(i).getPosition(), positions.get(j));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 1));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 2));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 3));

                assertTrue(distances.get(j) > 0.0);
                assertTrue(distances.get(j + 1) > 0.0);
                assertTrue(distances.get(j + 2) > 0.0);
                assertTrue(distances.get(j + 3) > 0.0);
                assertEquals(distances.get(j), distances.get(j + 1), ABSOLUTE_ERROR);
                assertEquals(distances.get(j + 1),
                        ((RangingReading<WifiAccessPoint>) readings.get(3 * i + 1)).getDistance(),
                        0.0);
                assertEquals(distances.get(j + 2),
                        ((RangingAndRssiReading<WifiAccessPoint>) readings.get(3 * i + 2)).getDistance(),
                        0.0);
                assertEquals(distances.get(j), distances.get(j + 3), ABSOLUTE_ERROR);

                assertTrue(distanceStandardDeviations.get(j) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);

                assertEquals(sourcesQualityScores[i], distanceQualityScores.get(j), 0.0);
                assertEquals(sourcesQualityScores[i], distanceQualityScores.get(j + 1), 0.0);
                assertEquals(sourcesQualityScores[i], distanceQualityScores.get(j + 2), 0.0);
                assertEquals(sourcesQualityScores[i], distanceQualityScores.get(j + 3), 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesDistanceStandardDeviationsAndReadingsQualityScoresReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] readingsQualityScores = new double[3 * numSources];
            for (int i = 0, j = 0; i < numSources; i++, j += 3) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi, Math.sqrt(RX_POWER_VARIANCE)));
                readingsQualityScores[j] = randomizer.nextDouble();
                readings.add(new RangingReading<>(accessPoint, distance,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION));
                readingsQualityScores[j + 1] = randomizer.nextDouble();
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, Math.sqrt(RX_POWER_VARIANCE)));
                readingsQualityScores[j + 2] = randomizer.nextDouble();
            }

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                    new Fingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            final List<Double> distanceQualityScores = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                    sources, fingerprint, null, readingsQualityScores,
                    true, FALLBACK_DISTANCE_STANDARD_DEVIATION, positions,
                    distances, distanceStandardDeviations, distanceQualityScores);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                    null, null, null,
                    readingsQualityScores, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations, distanceQualityScores);

            // check
            assertEquals(4 * numSources, positions.size());
            assertEquals(4 * numSources, distances.size());
            assertEquals(4 * numSources, distanceStandardDeviations.size());
            assertEquals(4 * numSources, distanceQualityScores.size());

            for (int i = 0, j = 0, k = 0; i < numSources; i++, j += 4, k += 3) {
                assertEquals(sources.get(i).getPosition(), positions.get(j));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 1));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 2));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 3));

                assertTrue(distances.get(j) > 0.0);
                assertTrue(distances.get(j + 1) > 0.0);
                assertTrue(distances.get(j + 2) > 0.0);
                assertTrue(distances.get(j + 3) > 0.0);
                assertEquals(distances.get(j), distances.get(j + 1), ABSOLUTE_ERROR);
                assertEquals(distances.get(j + 1),
                        ((RangingReading<WifiAccessPoint>) readings.get(3 * i + 1)).getDistance(),
                        0.0);
                assertEquals(distances.get(j + 2),
                        ((RangingAndRssiReading<WifiAccessPoint>) readings.get(3 * i + 2)).getDistance(),
                        0.0);
                assertEquals(distances.get(j), distances.get(j + 3), ABSOLUTE_ERROR);

                assertTrue(distanceStandardDeviations.get(j) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);

                assertEquals(readingsQualityScores[k], distanceQualityScores.get(j), 0.0);
                assertEquals(readingsQualityScores[k + 1], distanceQualityScores.get(j + 1), 0.0);
                assertEquals(readingsQualityScores[k + 2], distanceQualityScores.get(j + 2), 0.0);
                assertEquals(readingsQualityScores[k + 2], distanceQualityScores.get(j + 3), 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesDistanceStandardDeviationsSourcesAndReadingsQualityScoresReadings() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
            final double[] sourcesQualityScores = new double[numSources];
            final double[] readingsQualityScores = new double[3 * numSources];
            for (int i = 0, j = 0; i < numSources; i++, j += 3) {
                final InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                final double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                final double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                final String bssid = String.valueOf(i);

                final WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid, FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE), pathLossExponent,
                                Math.sqrt(PATH_LOSS_EXPONENT_VARIANCE), accessPointPosition);
                sources.add(locatedAccessPoint);
                sourcesQualityScores[i] = randomizer.nextDouble();

                final WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                final double distance = position.distanceTo(accessPointPosition);

                final double rssi = Utils.powerTodBm(receivedPower(transmittedPower, distance,
                        pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi, Math.sqrt(RX_POWER_VARIANCE)));
                readingsQualityScores[j] = randomizer.nextDouble();
                readings.add(new RangingReading<>(accessPoint, distance,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION));
                readingsQualityScores[j + 1] = randomizer.nextDouble();
                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, Math.sqrt(RX_POWER_VARIANCE)));
                readingsQualityScores[j + 2] = randomizer.nextDouble();
            }

            final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                    new Fingerprint<>(readings);

            final List<Point2D> positions = new ArrayList<>();
            final List<Double> distances = new ArrayList<>();
            final List<Double> distanceStandardDeviations = new ArrayList<>();
            final List<Double> distanceQualityScores = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                    sources, fingerprint, sourcesQualityScores, readingsQualityScores,
                    true, FALLBACK_DISTANCE_STANDARD_DEVIATION, positions,
                    distances, distanceStandardDeviations, distanceQualityScores);

            // check that positions, distances and distance standard deviations are not
            // modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                    null, null, sourcesQualityScores, readingsQualityScores,
                    true, FALLBACK_DISTANCE_STANDARD_DEVIATION, positions,
                    distances, distanceStandardDeviations, distanceQualityScores);

            // check
            assertEquals(4 * numSources, positions.size());
            assertEquals(4 * numSources, distances.size());
            assertEquals(4 * numSources, distanceStandardDeviations.size());
            assertEquals(4 * numSources, distanceQualityScores.size());

            for (int i = 0, j = 0, k = 0; i < numSources; i++, j += 4, k += 3) {
                assertEquals(sources.get(i).getPosition(), positions.get(j));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 1));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 2));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 3));

                assertTrue(distances.get(j) > 0.0);
                assertTrue(distances.get(j + 1) > 0.0);
                assertTrue(distances.get(j + 2) > 0.0);
                assertTrue(distances.get(j + 3) > 0.0);
                assertEquals(distances.get(j), distances.get(j + 1), ABSOLUTE_ERROR);
                assertEquals(distances.get(j + 1),
                        ((RangingReading<WifiAccessPoint>) readings.get(3 * i + 1)).getDistance(),
                        0.0);
                assertEquals(distances.get(j + 2),
                        ((RangingAndRssiReading<WifiAccessPoint>) readings.get(3 * i + 2)).getDistance(),
                        0.0);
                assertEquals(distances.get(j), distances.get(j + 3), ABSOLUTE_ERROR);

                assertTrue(distanceStandardDeviations.get(j) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);
                assertTrue(distanceStandardDeviations.get(j + 1) > 0.0);

                assertEquals(sourcesQualityScores[i] + readingsQualityScores[k],
                        distanceQualityScores.get(j), 0.0);
                assertEquals(sourcesQualityScores[i] + readingsQualityScores[k + 1],
                        distanceQualityScores.get(j + 1), 0.0);
                assertEquals(sourcesQualityScores[i] + readingsQualityScores[k + 2],
                        distanceQualityScores.get(j + 2), 0.0);
                assertEquals(sourcesQualityScores[i] + readingsQualityScores[k + 2],
                        distanceQualityScores.get(j + 3), 0.0);
            }
        }
    }

    private double receivedPower(final double equivalentTransmittedPower,
                                 final double distance, final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k / Math.pow(distance, pathLossExponent);
    }
}
