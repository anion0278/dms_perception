#pragma once

#define Linux !true

#if Linux  // == true

#include "RoboMath.h"
using namespace RoboMath;

#else

#include "RoboMathSpec.h"
using namespace RoboMathSpec;

#endif




