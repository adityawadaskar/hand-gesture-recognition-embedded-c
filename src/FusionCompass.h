/*
 * FusionCompass.h
 *
 *  Created on: Mar 16, 2018
 *      Author: dali
 */

#ifndef FUSIONCOMPASS_H_
#define FUSIONCOMPASS_H_

//------------------------------------------------------------------------------
// Includes

#include "FusionTypes.h"

//------------------------------------------------------------------------------
// Function prototypes

float FusionCompassCalculateHeading(const FusionVector3 accelerometer, const FusionVector3 magnetometer);
#endif /* FUSIONCOMPASS_H_ */
