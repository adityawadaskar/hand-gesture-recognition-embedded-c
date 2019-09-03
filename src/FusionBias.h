/*
 * FusionBias.h
 *
 *  Created on: Mar 16, 2018
 *      Author: dali
 */

#ifndef FUSIONBIAS_H_
#define FUSIONBIAS_H_

//------------------------------------------------------------------------------
// Includes

#include "FusionTypes.h"
#include <stdbool.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Bias correction algorithm structure.  Must be initialised using
 * FusionBiasInitialise before use.
 */
typedef struct {
    int adcThreshold;
    float samplePeriod;
    float stationaryTimer; // internal state (must not be modified by the application)
    FusionVector3 gyroscopeBias; // algorithm output (may be modified at any time by the application)
} FusionBias;

//------------------------------------------------------------------------------
// Function prototypes

void FusionBiasInitialise(FusionBias * const fusionBias, const int adcThreshold, const float samplePeriod);
void FusionBiasUpdate(FusionBias * const fusionBias, const int xAdc, const int yAdc, const int zAdc);
bool FusionBiasIsActive(FusionBias * const fusionBias);


#endif /* FUSIONBIAS_H_ */
