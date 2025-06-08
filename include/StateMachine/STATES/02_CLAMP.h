#ifndef CLAMP_H
#define CLAMP_H

#include <Arduino.h>

//* ************************************************************************
//* ************************ CLAMP ***************************
//* ************************************************************************
// This state handles the clamp sequence including alignment cylinder
// operations and clamp positioning for the cutting cycle

// Function declarations
void executeClampState();
bool isClampComplete();
void resetClampState();

#endif // CLAMP_H 