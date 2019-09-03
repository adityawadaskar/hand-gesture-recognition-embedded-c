/*
 * FusionBias.c
 *
 *  Created on: Mar 16, 2018
 *      Author: dali
 */

#include "FusionBias.h"

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Minimum stationary period (in seconds) after which the the algorithm
 * becomes active and begins sampling the gyroscope bias.
 */
#define STATIONARY_PERIOD (5.0f)

/**
 * @brief Corner frequency (in Hz) of the high-pass filter used to sample the
 * gyroscope bias.
 */
#define CORNER_FREQUENCY (0.02f)

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Initialises the bias correction algorithm with application settings.
 *
 * Example use:
 * @code
 * FusionBias fusionBias;
 * FusionBiasInitialise(&fusionBias, 50, 0.01f); // assumes 100 Hz sample rate
 * @endcode
 *
 * @param fusionBias FusionBias structure.
 * @param adcThreshold Gyroscope ADC threshold (in lsb) below which the
 * gyroscope is detected as being stationary.
 * @param samplePeriod Nominal sample period (in seconds) that FusionBiasUpdate
 * will be called by the application.
 */
void FusionBiasInitialise(FusionBias * const fusionBias, const int adcThreshold, const float samplePeriod) {
    fusionBias->adcThreshold = adcThreshold;
    fusionBias->samplePeriod = samplePeriod;
    fusionBias->stationaryTimer = 0.0f;
    fusionBias->gyroscopeBias = FUSION_VECTOR3_ZERO;
}

/**
 * @brief Updates the bias correction algorithm with the latest sensor
 * measurements.  This function should be called for each new gyroscope
 * measurement where the gyroscope is be sampled at the specified sample period.
 *
 * Example use:
 * @code
 * FusionBiasUpdate(&fusionBias, 0, 0, 0); // literal values should be replaced with sensor measurements
 * @endcode
 *
 * @param fusionBias FusionBias structure.
 * @param xAdc Gyroscope x axis ADC value in lsb.
 * @param yAdc Gyroscope y axis ADC value in lsb.
 * @param zAdc Gyroscope z axis ADC value in lsb.
 */
void FusionBiasUpdate(FusionBias * const fusionBias, const int xAdc, const int yAdc, const int zAdc) {
    if ((xAdc > fusionBias->adcThreshold) || (xAdc < (-1 * fusionBias->adcThreshold)) ||
            (yAdc > fusionBias->adcThreshold) || (yAdc < (-1 * fusionBias->adcThreshold)) ||
            (zAdc > fusionBias->adcThreshold) || (zAdc < (-1 * fusionBias->adcThreshold))) {
        fusionBias->stationaryTimer = 0.0f;
    } else {
        if (fusionBias->stationaryTimer >= STATIONARY_PERIOD) {
            FusionVector3 gyroscope = {
                .axis.x = (float) xAdc,
                .axis.y = (float) yAdc,
                .axis.z = (float) zAdc,
            };
            gyroscope = FusionVectorSubtract(gyroscope, fusionBias->gyroscopeBias);
            fusionBias->gyroscopeBias = FusionVectorAdd(fusionBias->gyroscopeBias, FusionVectorMultiplyScalar(gyroscope, (2.0f * M_PI * CORNER_FREQUENCY) * fusionBias->samplePeriod));
        } else {
            fusionBias->stationaryTimer += fusionBias->samplePeriod;
        }
    }
}

/**
 * @brief Returns true if the bias correction algorithm is active.
 *
 * Example use:
 * @code
 * if (FusionBiasIsActive(&fusionBias) == true) {
 *     // Bias correction algorithm is active
 * } else {
 *     // Bias correction algorithm is not active
 * }
 * @endcode
 *
 * @param fusionBias FusionBias structure.
 * @return True if the bias correction algorithm is active.
 */
bool FusionBiasIsActive(FusionBias * const fusionBias) {
    return fusionBias->stationaryTimer >= STATIONARY_PERIOD;
}
