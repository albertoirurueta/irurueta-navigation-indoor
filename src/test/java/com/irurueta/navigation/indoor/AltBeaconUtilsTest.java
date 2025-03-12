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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AltBeaconUtilsTest {

    private static final double MIN_FREQUENCY = 2.4e9;
    private static final double MAX_FREQUENCY = 2.45e9;

    private static final double MIN_PATH_LOSS = 1.6;
    private static final double MAX_PATH_LOSS = 2.0;

    private static final double MIN_K = 0.5;
    private static final double MAX_K = 0.6;

    private static final double MIN_C1 = 0.08;
    private static final double MAX_C1 = 0.09;

    private static final double MIN_C2 = -0.6;
    private static final double MAX_C2 = -0.5;

    private static final double MIN_C3 = 0.0;
    private static final double MAX_C3 = 1.0;

    private static final double MIN_RATIO = 0.5;
    private static final double MAX_RATIO = 1.0;

    private static final double MIN_POWER = 0.5;
    private static final double MAX_POWER = 1.0;

    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 5.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testGetK() {
        final var randomizer = new UniformRandomizer();

        final var frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);

        final var k = AltBeaconUtils.getK(frequency);

        assertEquals(frequency, AltBeaconUtils.getFrequency(k), ABSOLUTE_ERROR);
    }

    @Test
    void testGetKWithCoefficients() {
        final var randomizer = new UniformRandomizer();

        final var frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS, MAX_PATH_LOSS);

        final var coefficient1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency, pathLossExponent);
        final var coefficient2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        final var k = AltBeaconUtils.getK(coefficient1, coefficient2);

        assertEquals(frequency / 1e9, AltBeaconUtils.getFrequency(k) / 1e9, ABSOLUTE_ERROR);
    }

    @Test
    void testGetFrequency() {
        final var randomizer = new UniformRandomizer();

        final var k = randomizer.nextDouble(MIN_K, MAX_K);

        final var frequency = AltBeaconUtils.getFrequency(k);

        assertEquals(k, AltBeaconUtils.getK(frequency), ABSOLUTE_ERROR);
    }

    @Test
    void testGetFrequencyWithCoefficients() {
        final var randomizer = new UniformRandomizer();

        final var c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final var c2 = randomizer.nextDouble(MIN_C2, MAX_C2);

        final var frequency = AltBeaconUtils.getFrequency(c1, c2);
        final var n = AltBeaconUtils.getPathLossExponent(c2);

        assertEquals(c1, AltBeaconUtils.getCoefficient1WithFrequency(frequency, n), ABSOLUTE_ERROR);
        assertEquals(c2, AltBeaconUtils.getCoefficient2(n), ABSOLUTE_ERROR);
    }

    @Test
    void testGetPathLossExponent() {
        final var randomizer = new UniformRandomizer();

        final var c2 = randomizer.nextDouble(MIN_C2, MAX_C2);

        final var n = AltBeaconUtils.getPathLossExponent(c2);

        assertEquals(c2, AltBeaconUtils.getCoefficient2(n), ABSOLUTE_ERROR);
    }

    @Test
    void testGetDistance() {
        final var randomizer = new UniformRandomizer();

        final var c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final var c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final var c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final var ratio = randomizer.nextDouble(MIN_RATIO, MAX_RATIO);

        final var distance = AltBeaconUtils.getDistance(c1, c2, c3, ratio);

        assertEquals(ratio, AltBeaconUtils.getRatio(c1, c2, c3, distance), ABSOLUTE_ERROR);
        assertEquals(distance, c1 * Math.pow(ratio, c2) + c3, ABSOLUTE_ERROR);
    }

    @Test
    void testGetDistanceWithPower() {
        final var randomizer = new UniformRandomizer();

        final var c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final var c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final var c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final var transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        final var ratio = randomizer.nextDouble(MIN_RATIO, MAX_RATIO);

        final var receivedPower = AltBeaconUtils.getReceivedPower(ratio, transmittedPower);

        final var distance = AltBeaconUtils.getDistance(c1, c2, c3, receivedPower, transmittedPower);

        assertEquals(ratio, AltBeaconUtils.getRatio(c1, c2, c3, distance), ABSOLUTE_ERROR);
        assertEquals(distance, c1 * Math.pow(ratio, c2) + c3, ABSOLUTE_ERROR);
    }

    @Test
    void testGetRatio() {
        final var randomizer = new UniformRandomizer();

        final var c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final var c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final var c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

        final var ratio = AltBeaconUtils.getRatio(c1, c2, c3, distance);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3, ratio), ABSOLUTE_ERROR);
    }

    @Test
    void testGetRatioWithPower() {
        final var randomizer = new UniformRandomizer();

        final var receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        final var transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final var ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        // check
        assertEquals(ratio, receivedPower / transmittedPower, ABSOLUTE_ERROR);
    }

    @Test
    void testGetReceivedPower() {
        final var randomizer = new UniformRandomizer();

        final var receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        final var transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final var ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        assertEquals(receivedPower, AltBeaconUtils.getReceivedPower(ratio, transmittedPower), ABSOLUTE_ERROR);
    }

    @Test
    void testGetReceivedPowerWithCoefficients() {
        final var randomizer = new UniformRandomizer();

        final var c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final var c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final var c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
        final var transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final var receivedPower = AltBeaconUtils.getReceivedPower(c1, c2, c3, distance, transmittedPower);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3, receivedPower, transmittedPower), ABSOLUTE_ERROR);
    }

    @Test
    void testGetTransmittedPower() {
        final var randomizer = new UniformRandomizer();

        final var receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        final var transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final var ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        assertEquals(transmittedPower, AltBeaconUtils.getTransmittedPower(ratio, receivedPower), ABSOLUTE_ERROR);
    }

    @Test
    void testGetTransmittedPowerWithCoefficients() {
        final var randomizer = new UniformRandomizer();

        final var c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final var c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final var c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final var distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
        final var receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final var transmittedPower = AltBeaconUtils.getTransmittedPower(c1, c2, c3, distance, receivedPower);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3, receivedPower, transmittedPower), ABSOLUTE_ERROR);
    }

    @Test
    void testGetCoefficient1() {
        final var randomizer = new UniformRandomizer();

        final var frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS, MAX_PATH_LOSS);

        final var coefficient1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency, pathLossExponent);
        final var coefficient2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        final var k = AltBeaconUtils.getK(coefficient1, coefficient2);

        assertEquals(coefficient1, AltBeaconUtils.getCoefficient1(k, pathLossExponent), ABSOLUTE_ERROR);
    }

    @Test
    void testGetCoefficient1WithFrequency() {
        final var randomizer = new UniformRandomizer();

        final var frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS, MAX_PATH_LOSS);

        final var c1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency, pathLossExponent);
        final var c2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        assertEquals(frequency, AltBeaconUtils.getFrequency(c1, c2), 10.0 * ABSOLUTE_ERROR);
    }

    @Test
    void testGetCoefficient2() {
        final var randomizer = new UniformRandomizer();

        final var pathLossExponent = randomizer.nextDouble(MIN_PATH_LOSS, MAX_PATH_LOSS);

        final var c2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        assertEquals(pathLossExponent, AltBeaconUtils.getPathLossExponent(c2), ABSOLUTE_ERROR);
    }
}
