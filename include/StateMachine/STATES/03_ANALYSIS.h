#pragma once

#include "system_states.h"

//* ************************************************************************
//* ************************ ANALYSIS ***************************
//* ************************************************************************
// This state handles basic analysis sequence initialization for the cutting cycle

void executeAnalysisState();
bool isAnalysisComplete();
void resetAnalysisState(); 