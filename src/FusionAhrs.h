/*
 * FusionAhrs.h
 *
 *  Created on: Mar 16, 2018
 *      Author: dali
 */

#ifndef FUSIONAHRS_H_
#define FUSIONAHRS_H_

//------------------------------------------------------------------------------
// Includes

#include "FusionTypes.h"
#include <stdbool.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief AHRS algorithm structure.  Must be initialised using
 * FusionAhrsInitialise before use.
 */
typedef struct {
    // Adjustable parameters (may be modified at any time by application)
    float gain;
    float minMagneticFieldSquared;
    float maxMagneticFieldSquared;

    // Algorithm output and internal states (must not be modified by application)
    FusionQuaternion quaternion; // describes the Earth relative to the sensor
    FusionVector3 linearAcceleration;
    float rampedGain;
} FusionAhrs;

//------------------------------------------------------------------------------
// Function prototypes

void FusionAhrsInitialise(FusionAhrs * const fusionAhrs, const float gain, const float minMagneticField, const float maxMagneticField);
void FusionAhrsUpdate(FusionAhrs * const fusionAhrs, const FusionVector3 gyroscope, const FusionVector3 accelerometer, const FusionVector3 magnetometer, const float samplePeriod);
FusionVector3 FusionAhrsCalculateEarthAcceleration(const FusionAhrs * const fusionAhrs);
bool FusionAhrsIsInitialising(const FusionAhrs * const fusionAhrs);
void FusionAhrsReinitialise(FusionAhrs * const fusionAhrs);
void FusionAhrsZeroYaw(FusionAhrs * const fusionAhrs);

#endif /* FUSIONAHRS_H_ */
