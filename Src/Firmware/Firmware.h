#pragma once

#define MSP_FET

// Make sure the C basic types aren't used.
// They must be replaced by explicit-width types (eg uint16_t) to ensure
// execution that's consistent with the embedded device.
#define char    BADTYPE
#define short   BADTYPE
#define int     BADTYPE
#define long    BADTYPE

// static variables/functions aren't allowed since the entirety of firmware
// exists within the MSPProbeSim class, and class instances shouldn't share
// variables, while functions should only affect their respective instances.
// Globals should just be member variables, and static functions should just
// be normal functions.
#define static  BADSTATIC

#define extern  BADEXTERN

#include "USB.h"
#include "Descriptor.h"

#undef extern
#undef static
#undef char
#undef short
#undef int
#undef long
