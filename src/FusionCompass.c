/*
 * FusionCompass.c
 *
 *  Created on: Mar 16, 2018
 *      Author: dali
 */

#include "FusionCompass.h"
#include <math.h> // atan2f

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Calculates the tilt-compensated heading relative to magnetic north.
 *
 * Example use:
 * @code
 * const FusionVector3 accelerometer = {
 *    .axis.x = 0.0f,
 *    .axis.y = 0.0f,
 *    .axis.z = 1.0f,
 * }; // literal values should be replaced with sensor measurements
 *
 * const FusionVector3 magnetometer = {
 *    .axis.x = 1.0f,
 *    .axis.y = 0.0f,
 *    .axis.z = 0.0f,
 * }; // literal values should be replaced with sensor measurements
 *
 * const float heading = FusionCompassCalculateHeading(accelerometer, magnetometer);
 * @endcode
 *
 * @param accelerometer Accelerometer measurement in any calibrated units.
 * @param magnetometer Magnetometer measurement in any calibrated units.
 * @return Heading angle in degrees.
 */
float FusionCompassCalculateHeading(const FusionVector3 accelerometer, const FusionVector3 magnetometer) {

    // Compute direction of 'magnetic west' (Earth's y axis)
    const FusionVector3 magneticWest = FusionVectorFastNormalise(FusionVectorCrossProduct(accelerometer, magnetometer));

    // Compute direction of magnetic north (Earth's x axis)
    const FusionVector3 magneticNorth = FusionVectorFastNormalise(FusionVectorCrossProduct(magneticWest, accelerometer));

    // Calculate angular heading relative to magnetic north
    return FUSION_RADIANS_TO_DEGREES(atan2f(magneticWest.axis.x, magneticNorth.axis.x));
}
