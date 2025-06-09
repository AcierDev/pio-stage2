#ifndef ALIGN_STATE_H
#define ALIGN_STATE_H

#include <Arduino.h>

//* ************************************************************************
//* ************************ ALIGN ***************************
//* ************************************************************************
// This state handles the initial clamp movements and alignment cylinder
// operations before motor movement begins

// Function declarations
void executeAlignState();
void executeAlignmentSequence();
void executeAnalysisSequence();
bool isAlignComplete();
bool isEmptyClassDetected();
bool isEndClassDetected();
void resetAlignState();

#endif // ALIGN_STATE_H 