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

/**
 * Contains a ranging reading associated to a given radio source (e.g. Wi-Fi
 * access point or bluetooth beacon), indicating the distance to such source.
 *
 * @param <S> a {@link RadioSource} type.
 */
public class RangingReading<S extends RadioSource> extends Reading<S> {

    /**
     * Default number of measurements.
     */
    public static final int DEFAULT_NUM_MEASUREMENTS = 1;

    /**
     * Distance in meters to the radio source.
     */
    private double distance;

    /**
     * Standard deviation of distance, if available.
     */
    private Double distanceStandardDeviation;

    /**
     * Number of attempted measurements used in the RTT exchange.
     */
    private int numAttemptedMeasurements = DEFAULT_NUM_MEASUREMENTS;

    /**
     * Number of successful measurements used to calculate the distance and standard
     * deviation.
     */
    private int numSuccessfulMeasurements = DEFAULT_NUM_MEASUREMENTS;

    /**
     * Constructor.
     *
     * @param source   radio source associated to this reading.
     * @param distance distance in meters to the radio source.
     * @throws IllegalArgumentException if radio source data is null or distance is negative.
     */
    public RangingReading(final S source, final double distance) {
        super(source);

        if (distance < 0.0) {
            throw new IllegalArgumentException();
        }

        this.distance = distance;
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param numAttemptedMeasurements  number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative,
     *                                  number of attempted measures is less than 1 or number of successful measures is negative.
     */
    public RangingReading(
            final S source, final double distance, final int numAttemptedMeasurements,
            final int numSuccessfulMeasurements) {
        this(source, distance);

        if (numAttemptedMeasurements < DEFAULT_NUM_MEASUREMENTS || numSuccessfulMeasurements < 0) {
            throw new IllegalArgumentException();
        }

        this.numAttemptedMeasurements = numAttemptedMeasurements;
        this.numSuccessfulMeasurements = numSuccessfulMeasurements;
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @throws IllegalArgumentException if radio source data is null, distance is negative or
     *                                  standard deviation is zero or negative.
     */
    public RangingReading(
            final S source, final double distance, final Double distanceStandardDeviation) {
        this(source, distance);

        if (distanceStandardDeviation != null && distanceStandardDeviation <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.distanceStandardDeviation = distanceStandardDeviation;
    }

    /**
     * Constructor.
     *
     * @param source                    radio source associated to this reading.
     * @param distance                  distance in meters to the radio source.
     * @param distanceStandardDeviation standard deviation of distance, if available.
     * @param numAttemptedMeasurements  number of attempted measurements used in the RTT exchange.
     * @param numSuccessfulMeasurements number of successful measurements used to calculate the
     *                                  distance and standard deviation.
     * @throws IllegalArgumentException if radio source data is null, distance is negative or
     *                                  standard deviation is zero or negative, number of attempted measures is less
     *                                  than 1 or number of successful measures is negative.
     */
    public RangingReading(
            final S source, final double distance, final Double distanceStandardDeviation,
            final int numAttemptedMeasurements, final int numSuccessfulMeasurements) {
        this(source, distance, distanceStandardDeviation);

        if (numAttemptedMeasurements < DEFAULT_NUM_MEASUREMENTS || numSuccessfulMeasurements < 0) {
            throw new IllegalArgumentException();
        }

        this.numAttemptedMeasurements = numAttemptedMeasurements;
        this.numSuccessfulMeasurements = numSuccessfulMeasurements;
    }

    /**
     * Empty constructor.
     */
    protected RangingReading() {
        super();
    }

    /**
     * Contains radio source reading type.
     *
     * @return reading type.
     */
    @Override
    public ReadingType getType() {
        return ReadingType.RANGING_READING;
    }

    /**
     * Gets distance in meters to the radio source.
     *
     * @return distance in meters to the radio source.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Gets standard deviation of distance, if available.
     *
     * @return standard deviation of distance or null.
     */
    public Double getDistanceStandardDeviation() {
        return distanceStandardDeviation;
    }

    /**
     * Gets number of attempted measurements used in the RTT exchange.
     *
     * @return number of attempted measurements used in the RTT exchange.
     */
    public int getNumAttemptedMeasurements() {
        return numAttemptedMeasurements;
    }

    /**
     * Gets number of successful measurements used to calculate the distance and
     * standard deviation.
     *
     * @return number of successful measurements used to calculate the distance and
     * standard deviation.
     */
    public int getNumSuccessfulMeasurements() {
        return numSuccessfulMeasurements;
    }
}
