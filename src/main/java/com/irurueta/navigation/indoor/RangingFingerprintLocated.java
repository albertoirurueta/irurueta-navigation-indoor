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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;

import java.util.List;

/**
 * Contains located ranging readings from several radio sources.
 *
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 * @param <R> a {@link RangingReading} type.
 */
public class RangingFingerprintLocated<S extends RadioSource, R extends RangingReading<S>,
        P extends Point<?>> extends RangingFingerprint<S, R> implements FingerprintLocated<P> {

    /**
     * Position where fingerprint readings were made.
     */
    private P mPosition;

    /**
     * Covariance of inhomogeneous coordinates of current
     * position (if available).
     */
    private Matrix mPositionCovariance;

    /**
     * Constructor.
     *
     * @param readings non-located ranging readings defining the fingerprint.
     * @param position position where readings were made.
     * @throws IllegalArgumentException if either readings or position are
     *                                  null.
     */
    public RangingFingerprintLocated(
            final List<R> readings, final P position) {
        super(readings);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param readings           non-located ranging readings defining the fingerprint.
     * @param position           position where readings were made.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either readings or position are null, or
     *                                  covariance has invalid size.
     */
    public RangingFingerprintLocated(
            final List<R> readings, final P position,
            final Matrix positionCovariance) {
        this(readings, position);

        if (positionCovariance != null) {
            final int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Empty constructor.
     */
    public RangingFingerprintLocated() {
        super();
    }

    /**
     * Gets position where fingerprint readings were made.
     *
     * @return position where fingerprint readings were made.
     */
    public P getPosition() {
        return mPosition;
    }

    /**
     * Gets covariance of inhomogeneous coordinates of current position (if available).
     *
     * @return covariance of position or null.
     */
    public Matrix getPositionCovariance() {
        return mPositionCovariance;
    }
}
