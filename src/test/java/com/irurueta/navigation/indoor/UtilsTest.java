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
package com.irurueta.navigation.indoor;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.navigation.indoor.radiosource.RssiRadioSourceEstimator;
import com.irurueta.numerical.DerivativeEstimator;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class UtilsTest {

    private static final Logger LOGGER = Logger.getLogger(UtilsTest.class.getName());

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final double FREQUENCY = 2.4e9; // (Hz)

    private static final double MIN_RSSI = -100.0;
    private static final double MAX_RSSI = 100.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double MIN_DISTANCE = 0.5;
    private static final double MAX_DISTANCE = 50.0;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double TX_POWER_VARIANCE = 0.1;
    private static final double RX_POWER_VARIANCE = 0.5;
    private static final double PATH_LOSS_EXPONENT_VARIANCE = 0.001;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 50;

    @Test
    void testdBmToPowerAndPowerTodBm() {
        final var randomizer = new UniformRandomizer();

        final var value = randomizer.nextDouble();

        assertEquals(Utils.powerTodBm(Utils.dBmToPower(value)), value, ABSOLUTE_ERROR);
        assertEquals(Utils.dBmToPower(Utils.powerTodBm(value)), value, ABSOLUTE_ERROR);
    }

    @Test
    void testPropagatePowerVarianceToDistanceVariance() throws EvaluationException {
        final var randomizer = new UniformRandomizer();

        var minDistanceVariance = Double.MAX_VALUE;
        var maxDistanceVariance = -Double.MAX_VALUE;
        var avgDistanceVariance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var txPower = Utils.dBmToPower(txPowerdBm);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final var rxPowerdBm = Utils.powerTodBm(receivedPower(txPower, distance, pathLossExponent));

            var distanceVariance = Utils.propagatePowerVarianceToDistanceVariance(txPowerdBm, rxPowerdBm,
                    pathLossExponent, FREQUENCY, RX_POWER_VARIANCE);
            assertTrue(distanceVariance > 0.0);

            final var k = SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
            final var kdB = 10.0 * Math.log10(k);
            final var derivative = -Math.log(10.0) / (10.0 * pathLossExponent) *
                    Math.pow(10.0, (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) / (10.0 * pathLossExponent));

            final var derivativeEstimator = new DerivativeEstimator(point -> {
                final var k1 = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
                final var kdB1 = 10.0 * Math.log10(k1);
                final var logSqrDistance = (pathLossExponent * kdB1 + txPowerdBm - point) / (5.0 * pathLossExponent);
                return Math.pow(10.0, logSqrDistance / 2.0);
            });

            final var derivative2 = derivativeEstimator.derivative(rxPowerdBm);
            assertEquals(derivative, derivative2, LARGE_ABSOLUTE_ERROR);

            assertEquals(derivative * derivative * RX_POWER_VARIANCE, distanceVariance, 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;

            // check that if rx power variance is zero, then distance variance is
            // zero as well
            distanceVariance = Utils.propagatePowerVarianceToDistanceVariance(txPowerdBm, rxPowerdBm, pathLossExponent,
                    FREQUENCY, 0.0);
            assertEquals(0.0, distanceVariance, 0.0);

            // test without rx power
            assertEquals(0.0, Utils.propagatePowerVarianceToDistanceVariance(txPowerdBm, rxPowerdBm,
                    pathLossExponent, FREQUENCY, null), 0.0);
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    void testPropagateVariancesToDistanceVariance() throws IndoorException, EvaluationException {
        final var randomizer = new UniformRandomizer();

        var minDistanceVariance = Double.MAX_VALUE;
        var maxDistanceVariance = -Double.MAX_VALUE;
        var avgDistanceVariance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var txPower = Utils.dBmToPower(txPowerdBm);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final var rxPowerdBm = Utils.powerTodBm(receivedPower(txPower, distance, pathLossExponent));

            final var dist = Utils.propagateVariancesToDistanceVariance(txPowerdBm, rxPowerdBm, pathLossExponent,
                    FREQUENCY, TX_POWER_VARIANCE, RX_POWER_VARIANCE, PATH_LOSS_EXPONENT_VARIANCE);

            final var k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
            final var kdB = 10.0 * Math.log10(k);

            final var tenPathLossExponent = 10.0 * pathLossExponent;
            final var tmp = (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) / tenPathLossExponent;
            final var derivativeTxPower = Math.log(10.0) / tenPathLossExponent * Math.pow(10.0, tmp);
            final var derivativeRxPower = -Math.log(10.0) / tenPathLossExponent * Math.pow(10.0, tmp);

            final var g = (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) / (10.0 * pathLossExponent);
            final var derivativeG = (kdB * 10.0 * pathLossExponent
                    - 10.0 * (pathLossExponent * kdB + txPowerdBm - rxPowerdBm))
                    / Math.pow(10.0 * pathLossExponent, 2.0);

            final var derivativePathLossExponent = Math.log(10.0) * derivativeG * Math.pow(10.0, g);

            final var gradient = new double[]{
                    derivativeTxPower,
                    derivativeRxPower,
                    derivativePathLossExponent
            };

            final var gradientEstimator = new GradientEstimator(point -> {
                final var txPower1 = point[0];
                final var rxPower = point[1];
                final var pathLossExponent1 = point[2];

                final var logSqrDistance = (pathLossExponent1 * kdB + txPower1 - rxPower) / (5.0 * pathLossExponent1);
                return Math.pow(10.0, logSqrDistance / 2.0);
            });

            final var gradient2 = gradientEstimator.gradient(new double[]{
                    txPowerdBm,
                    rxPowerdBm,
                    pathLossExponent});

            assertArrayEquals(gradient, gradient2, LARGE_ABSOLUTE_ERROR);

            assertEquals(distance, dist.getMean()[0], ABSOLUTE_ERROR);

            final var distanceVariance = dist.getCovariance().getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;

            // check without variances
            assertNull(Utils.propagateVariancesToDistanceVariance(txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    null, null, null));
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    void testPropagateTxPowerVarianceToDistanceVariance() throws IndoorException {
        final var randomizer = new UniformRandomizer();

        var minDistanceVariance = Double.MAX_VALUE;
        var maxDistanceVariance = -Double.MAX_VALUE;
        var avgDistanceVariance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var txPower = Utils.dBmToPower(txPowerdBm);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final var rxPowerdBm = Utils.powerTodBm(receivedPower(txPower, distance, pathLossExponent));

            final var dist = Utils.propagateVariancesToDistanceVariance(txPowerdBm, rxPowerdBm, pathLossExponent,
                    FREQUENCY, TX_POWER_VARIANCE, 0.0, 0.0);

            assertEquals(distance, dist.getMean()[0], ABSOLUTE_ERROR);

            final var distanceVariance = dist.getCovariance().getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    void testPropagateRxPowerVarianceToDistanceVariance() throws IndoorException {
        final var randomizer = new UniformRandomizer();

        var minDistanceVariance = Double.MAX_VALUE;
        var maxDistanceVariance = -Double.MAX_VALUE;
        var avgDistanceVariance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var txPower = Utils.dBmToPower(txPowerdBm);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final var rxPowerdBm = Utils.powerTodBm(receivedPower(txPower, distance, pathLossExponent));

            final var dist = Utils.propagateVariancesToDistanceVariance(txPowerdBm, rxPowerdBm, pathLossExponent,
                    FREQUENCY, 0.0, RX_POWER_VARIANCE, 0.0);

            assertEquals(distance, dist.getMean()[0], ABSOLUTE_ERROR);

            final var distanceVariance = dist.getCovariance().getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            final var distanceVariance2 = Utils.propagatePowerVarianceToDistanceVariance(txPowerdBm, rxPowerdBm,
                    pathLossExponent, FREQUENCY, RX_POWER_VARIANCE);
            assertEquals(distanceVariance, distanceVariance2, ABSOLUTE_ERROR);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    void testPropagatePathlossExponentVarianceToDistanceVariance() throws IndoorException {
        final var randomizer = new UniformRandomizer();

        var minDistanceVariance = Double.MAX_VALUE;
        var maxDistanceVariance = -Double.MAX_VALUE;
        var avgDistanceVariance = 0.0;
        for (var t = 0; t < TIMES; t++) {
            final var txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var txPower = Utils.dBmToPower(txPowerdBm);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final var rxPowerdBm = Utils.powerTodBm(receivedPower(txPower, distance, pathLossExponent));

            final var dist = Utils.propagateVariancesToDistanceVariance(txPowerdBm, rxPowerdBm, pathLossExponent,
                    FREQUENCY, 0.0, 0.0, PATH_LOSS_EXPONENT_VARIANCE);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            final var distanceVariance = dist.getCovariance().getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    void testPropagateNoVarianceToDistanceVariance() throws IndoorException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var txPower = Utils.dBmToPower(txPowerdBm);

            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final var rxPowerdBm = Utils.powerTodBm(receivedPower(txPower, distance, pathLossExponent));

            final var dist = Utils.propagateVariancesToDistanceVariance(txPowerdBm, rxPowerdBm, pathLossExponent,
                    FREQUENCY, 0.0, 0.0, 0.0);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            final var distanceVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, distanceVariance, ABSOLUTE_ERROR);
        }
    }

    @Test
    void testPropagateVariancesToRssiVarianceFirstOrderNonLinear2D() throws IndoorException, AlgebraException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            final var xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            final var xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            // test without variance values
            var dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, null,
                    null, null, null,
                    null);

            final var diffX1a = x1 - xa;
            final var diffY1a = y1 - ya;

            final var diffXi1 = xi - x1;
            final var diffYi1 = yi - y1;

            final var diffX1a2 = diffX1a * diffX1a;
            final var diffY1a2 = diffY1a * diffY1a;

            final var d1a2 = diffX1a2 + diffY1a2;

            final var rssi = fingerprintRssi - 10.0 * pathLossExponent * (diffX1a * diffXi1 + diffY1a * diffYi1)
                    / (Math.log(10.0) * d1a2);

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            var rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(2, 2), new Matrix(2, 2),
                    new Matrix(2, 2));

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(fingerprintRssi, pathLossExponent,
                    null, radioSourcePosition, estimatedPosition, null,
                    null, null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, null, estimatedPosition, null,
                    null, null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition, null,
                    null, null, null,
                    null, null));
        }
    }

    @Test
    void testPropagateVariancesToRssiVarianceFirstOrderNonLinear3D() throws IndoorException, AlgebraException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            final var xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var za = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            final var xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            // test without variance values
            var dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, null,
                    null, null, null,
                    null);

            final var diffX1a = x1 - xa;
            final var diffY1a = y1 - ya;
            final var diffZ1a = z1 - za;

            final var diffXi1 = xi - x1;
            final var diffYi1 = yi - y1;
            final var diffZi1 = zi - z1;

            final var diffX1a2 = diffX1a * diffX1a;
            final var diffY1a2 = diffY1a * diffY1a;
            final var diffZ1a2 = diffZ1a * diffZ1a;

            final var d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;

            final var rssi = fingerprintRssi - 10.0 * pathLossExponent *
                    (diffX1a * diffXi1 + diffY1a * diffYi1 + diffZ1a * diffZi1) / (Math.log(10.0) * d1a2);

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            var rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(3, 3), new Matrix(3, 3),
                    new Matrix(3, 3));

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    null, radioSourcePosition, estimatedPosition, null,
                    null, null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, null, estimatedPosition, null,
                    null, null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, null, null,
                    null, null, null,
                    null));
        }
    }

    @Test
    void testPropagateVariancesToRssiVarianceSecondOrderNonLinear2D() throws IndoorException, AlgebraException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            final var xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            final var xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            // test without variance values
            var dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, null,
                    null, null, null,
                    null);

            final var diffX1a = x1 - xa;
            final var diffY1a = y1 - ya;

            final var diffXi1 = xi - x1;
            final var diffYi1 = yi - y1;

            final var diffX1a2 = diffX1a * diffX1a;
            final var diffY1a2 = diffY1a * diffY1a;

            final var diffXi12 = diffXi1 * diffXi1;
            final var diffYi12 = diffYi1 * diffYi1;

            final var d1a2 = diffX1a2 + diffY1a2;
            final var d1a4 = d1a2 * d1a2;
            final var ln10 = Math.log(10.0);

            final var rssi = fingerprintRssi - 10.0 * pathLossExponent *
                    (diffX1a * diffXi1 + diffY1a * diffYi1) / (ln10 * d1a2) +
                    5.0 * pathLossExponent * ((diffX1a2 - diffY1a2) * (diffXi12 - diffYi12)) / (ln10 * d1a4) +
                    20.0 * pathLossExponent * diffX1a * diffY1a * diffXi1 * diffYi1 / (ln10 * d1a4);

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);

            var rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(2, 2), new Matrix(2, 2),
                    new Matrix(2, 2));

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(fingerprintRssi,
                    pathLossExponent, null, radioSourcePosition, estimatedPosition,
                    null, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(fingerprintRssi,
                    pathLossExponent, fingerprintPosition, null, estimatedPosition,
                    null, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(fingerprintRssi,
                    pathLossExponent, fingerprintPosition, radioSourcePosition, null,
                    null, null, null,
                    null, null));
        }
    }

    @Test
    void testPropagateVariancesToRssiVarianceSecondOrderNonLinear3D() throws IndoorException, AlgebraException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            final var xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var za = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            final var xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            // test without variance values
            var dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, null,
                    null, null, null,
                    null);

            final var diffX1a = x1 - xa;
            final var diffY1a = y1 - ya;
            final var diffZ1a = z1 - za;

            final var diffXi1 = xi - x1;
            final var diffYi1 = yi - y1;
            final var diffZi1 = zi - z1;

            final var diffX1a2 = diffX1a * diffX1a;
            final var diffY1a2 = diffY1a * diffY1a;
            final var diffZ1a2 = diffZ1a * diffZ1a;

            final var diffXi12 = diffXi1 * diffXi1;
            final var diffYi12 = diffYi1 * diffYi1;
            final var diffZi12 = diffZi1 * diffZi1;

            final var d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
            final var d1a4 = d1a2 * d1a2;
            final var ln10 = Math.log(10.0);

            final var rssi = fingerprintRssi
                    - 10.0 * pathLossExponent * (diffX1a * diffXi1 + diffY1a * diffYi1 + diffZ1a * diffZi1)
                    / (ln10 * d1a2)
                    - 5.0 * pathLossExponent * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4) * diffXi12
                    - 5.0 * pathLossExponent * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4) * diffYi12
                    - 5.0 * pathLossExponent * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4) * diffZi12
                    + 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1
                    + 20.0 * pathLossExponent * diffY1a * diffZ1a / (ln10 * d1a4) * diffYi1 * diffZi1
                    + 20.0 * pathLossExponent * diffX1a * diffZ1a / (ln10 * d1a4) * diffXi1 * diffZi1;

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);

            var rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(3, 3), new Matrix(3, 3),
                    new Matrix(3, 3));

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(fingerprintRssi,
                    pathLossExponent, null, radioSourcePosition, estimatedPosition,
                    null, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(fingerprintRssi,
                    pathLossExponent, fingerprintPosition, null, estimatedPosition,
                    null, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(fingerprintRssi,
                    pathLossExponent, fingerprintPosition, radioSourcePosition, null,
                    null, null, null,
                    null, null));
        }
    }

    @Test
    void testPropagateVariancesToRssiVarianceThirdOrderNonLinear2D() throws IndoorException, AlgebraException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            final var xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            final var xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            // test without variance values
            var dist = Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, null,
                    null, null, null,
                    null);

            final var diffX1a = x1 - xa;
            final var diffY1a = y1 - ya;

            final var diffXi1 = xi - x1;
            final var diffYi1 = yi - y1;

            final var diffX1a2 = diffX1a * diffX1a;
            final var diffY1a2 = diffY1a * diffY1a;

            final var diffXi12 = diffXi1 * diffXi1;
            final var diffYi12 = diffYi1 * diffYi1;

            final var diffXi13 = diffXi12 * diffXi1;
            final var diffYi13 = diffYi12 * diffYi1;

            final var d1a2 = diffX1a2 + diffY1a2;
            final var d1a4 = d1a2 * d1a2;
            final var d1a8 = d1a4 * d1a4;
            final var ln10 = Math.log(10.0);

            final var rssi = fingerprintRssi
                    - 10.0 * pathLossExponent * diffX1a / (ln10 * d1a2) * diffXi1
                    - 10.0 * pathLossExponent * diffY1a / (ln10 * d1a2) * diffYi1
                    - 5.0 * pathLossExponent * (-diffX1a2 + diffY1a2) / (ln10 * d1a4) * diffXi12
                    - 5.0 * pathLossExponent * (diffX1a2 - diffY1a2) / (ln10 * d1a4) * diffYi12
                    + 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1
                    - 10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * d1a4 - (-diffX1a2 + diffY1a2) * 4.0 * d1a2 * diffX1a) / d1a8 * diffXi13
                    - 10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffY1a * d1a4 - (diffX1a2 - diffY1a2) * 4.0 * d1a2 * diffY1a) / d1a8 * diffYi13
                    - 5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4 - (-diffX1a2 + diffY1a2) * 4.0 * d1a2 * diffY1a) / d1a8 * diffXi12 * diffYi1
                    - 5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 - diffY1a2) * 4.0 * d1a2 * diffX1a) / d1a8 * diffXi1 * diffYi12;

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);

            var rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(2, 2), new Matrix(2, 2),
                    new Matrix(2, 2));

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(fingerprintRssi,
                    pathLossExponent, null, radioSourcePosition, estimatedPosition,
                    null, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(fingerprintRssi,
                    pathLossExponent, fingerprintPosition, null, estimatedPosition,
                    null, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(fingerprintRssi,
                    pathLossExponent, fingerprintPosition, radioSourcePosition, null,
                    null, null, null,
                    null, null));
        }
    }

    @Test
    void testPropagateVariancesToRssiVarianceThirdOrderNonLinear3D() throws IndoorException, AlgebraException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            final var xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var za = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            final var xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            // test without variance values
            var dist = Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, null,
                    null, null, null,
                    null);

            final var diffX1a = x1 - xa;
            final var diffY1a = y1 - ya;
            final var diffZ1a = z1 - za;

            final var diffXi1 = xi - x1;
            final var diffYi1 = yi - y1;
            final var diffZi1 = zi - z1;

            final var diffX1a2 = diffX1a * diffX1a;
            final var diffY1a2 = diffY1a * diffY1a;
            final var diffZ1a2 = diffZ1a * diffZ1a;

            final var diffXi12 = diffXi1 * diffXi1;
            final var diffYi12 = diffYi1 * diffYi1;
            final var diffZi12 = diffZi1 * diffZi1;

            final var diffXi13 = diffXi12 * diffXi1;
            final var diffYi13 = diffYi12 * diffYi1;
            final var diffZi13 = diffZi12 * diffZi1;

            final var d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
            final var d1a4 = d1a2 * d1a2;
            final var d1a8 = d1a4 * d1a4;
            final var ln10 = Math.log(10.0);

            final var value1 = -10.0 * pathLossExponent * diffX1a / (ln10 * d1a2);
            final var value2 = -10.0 * pathLossExponent * diffY1a / (ln10 * d1a2);
            final var value3 = -10.0 * pathLossExponent * diffZ1a / (ln10 * d1a2);
            final var value4 = -5.0 * pathLossExponent * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4);
            final var value5 = -5.0 * pathLossExponent * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4);
            final var value6 = -5.0 * pathLossExponent * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4);
            final var value7 = 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4);
            final var value8 = 20.0 * pathLossExponent * diffY1a * diffZ1a / (ln10 * d1a4);
            final var value9 = 20.0 * pathLossExponent * diffX1a * diffZ1a / (ln10 * d1a4);
            final var value10 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * d1a4
                    - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
            final var value11 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffY1a * d1a4
                    - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
            final var value12 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffZ1a * d1a4
                    - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
            final var value13 = -5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4
                    - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
            final var value14 = -5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * d1a4
                    - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
            final var value15 = -5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4
                    - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
            final var value16 = -5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4
                    - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
            final var value17 = -5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * d1a4
                    - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
            final var value18 = -5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4
                    - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
            final var value19 = -80.0 * pathLossExponent / ln10 * (diffX1a * diffY1a * diffZ1a * d1a2) / d1a8;

            final var rssi = fingerprintRssi
                    + value1 * diffXi1
                    + value2 * diffYi1
                    + value3 * diffZi1
                    + value4 * diffXi12
                    + value5 * diffYi12
                    + value6 * diffZi12
                    + value7 * diffXi1 * diffYi1
                    + value8 * diffYi1 * diffZi1
                    + value9 * diffXi1 * diffZi1
                    + value10 * diffXi13
                    + value11 * diffYi13
                    + value12 * diffZi13
                    + value13 * diffXi12 * diffYi1
                    + value14 * diffXi12 * diffZi1
                    + value15 * diffXi1 * diffYi12
                    + value16 * diffXi1 * diffZi12
                    + value17 * diffYi12 * diffZi1
                    + value18 * diffYi1 * diffZi12
                    + value19 * diffXi1 * diffYi1 * diffZi1;

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            var rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(fingerprintRssi, pathLossExponent,
                    fingerprintPosition, radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(3, 3), new Matrix(3, 3),
                    new Matrix(3, 3));

            assertEquals(rssi, dist.getMean()[0], ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, rssiVariance, ABSOLUTE_ERROR);

            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(fingerprintRssi,
                    pathLossExponent, null, radioSourcePosition, estimatedPosition,
                    null, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(fingerprintRssi,
                    pathLossExponent, fingerprintPosition, null, estimatedPosition,
                    null, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(fingerprintRssi,
                    pathLossExponent, fingerprintPosition, radioSourcePosition, null,
                    null, null, null,
                    null, null));
        }
    }

    @Test
    void testPropagateVariancesToRssiDifferenceVariance2D() throws IndoorException, AlgebraException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            final var xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            final var xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            // test without variance values
            var dist = Utils.propagateVariancesToRssiDifferenceVariance2D(pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null, null,
                    null, null);

            final var diffX1a = x1 - xa;
            final var diffY1a = y1 - ya;

            final var diffXia = xi - xa;
            final var diffYia = yi - ya;

            final var diffX1a2 = diffX1a * diffX1a;
            final var diffY1a2 = diffY1a * diffY1a;

            final var diffXia2 = diffXia * diffXia;
            final var diffYia2 = diffYia * diffYia;

            final var d1a2 = diffX1a2 + diffY1a2;
            final var dia2 = diffXia2 + diffYia2;

            final var diffRssi = 5.0 * pathLossExponent * (Math.log10(d1a2) - Math.log10(dia2));

            assertEquals(diffRssi, dist.getMean()[0], ABSOLUTE_ERROR);

            var diffRssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, diffRssiVariance, ABSOLUTE_ERROR);

            // test with variance values
            dist = Utils.propagateVariancesToRssiDifferenceVariance2D(pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0, new Matrix(2, 2),
                    new Matrix(2, 2), new Matrix(2, 2));

            assertEquals(diffRssi, dist.getMean()[0], ABSOLUTE_ERROR);
            diffRssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, diffRssiVariance, ABSOLUTE_ERROR);

            assertNull(Utils.propagateVariancesToRssiDifferenceVariance2D(pathLossExponent, null,
                    radioSourcePosition, estimatedPosition, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiDifferenceVariance2D(pathLossExponent, fingerprintPosition,
                    null, estimatedPosition, null,
                    null, null, null));
            assertNull(Utils.propagateVariancesToRssiDifferenceVariance2D(pathLossExponent, fingerprintPosition,
                    radioSourcePosition, null, null,
                    null, null, null));
        }
    }

    @Test
    void testPropagateVariancesToRssiDifferenceVariance3D() throws IndoorException, AlgebraException {
        final var randomizer = new UniformRandomizer();

        for (var t = 0; t < TIMES; t++) {
            final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final var x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            final var xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var za = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            final var xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final var estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            // test without variance values
            var dist = Utils.propagateVariancesToRssiDifferenceVariance3D(pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null, null,
                    null, null);

            final var diffX1a = x1 - xa;
            final var diffY1a = y1 - ya;
            final var diffZ1a = z1 - za;

            final var diffXia = xi - xa;
            final var diffYia = yi - ya;
            final var diffZia = zi - za;

            final var diffX1a2 = diffX1a * diffX1a;
            final var diffY1a2 = diffY1a * diffY1a;
            final var diffZ1a2 = diffZ1a * diffZ1a;

            final var diffXia2 = diffXia * diffXia;
            final var diffYia2 = diffYia * diffYia;
            final var diffZia2 = diffZia * diffZia;

            final var d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
            final var dia2 = diffXia2 + diffYia2 + diffZia2;

            final var diffRssi = 5.0 * pathLossExponent * (Math.log10(d1a2) - Math.log10(dia2));

            assertEquals(diffRssi, dist.getMean()[0], ABSOLUTE_ERROR);

            var diffRssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, diffRssiVariance, ABSOLUTE_ERROR);

            // test with variance values
            dist = Utils.propagateVariancesToRssiDifferenceVariance3D(pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0, new Matrix(3, 3),
                    new Matrix(3, 3), new Matrix(3, 3));

            assertEquals(diffRssi, dist.getMean()[0], ABSOLUTE_ERROR);
            diffRssiVariance = dist.getCovariance().getElementAt(0, 0);
            assertEquals(0.0, diffRssiVariance, ABSOLUTE_ERROR);

            assertNull(Utils.propagateVariancesToRssiDifferenceVariance3D(pathLossExponent, null,
                    radioSourcePosition, estimatedPosition, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiDifferenceVariance3D(pathLossExponent, fingerprintPosition,
                    null, estimatedPosition, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiDifferenceVariance3D(pathLossExponent, fingerprintPosition,
                    radioSourcePosition, null, null, null,
                    null, null));
        }
    }

    private static double receivedPower(final double equivalentTransmittedPower, final double distance,
                                        final double pathLossExponent) {
        // Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        // Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final var k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k / Math.pow(distance, pathLossExponent);
    }
}
