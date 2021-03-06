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

/**
 * Listener to be notified of events such as when estimation starts or ends.
 *
 * @param <E> a {@link PositionEstimator} type.
 */
public interface PositionEstimatorListener<E extends PositionEstimator<?, ?, ?>> {

    /**
     * Called when estimation starts.
     *
     * @param estimator estimator raising the event.
     */
    void onEstimateStart(final E estimator);

    /**
     * Called when estimation ends.
     *
     * @param estimator estimator raising the event.
     */
    void onEstimateEnd(final E estimator);
}
