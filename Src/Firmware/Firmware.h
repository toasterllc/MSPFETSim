#pragma once

#define MSP_FET

// Make sure the C basic types aren't used.
// They must be replaced by explicit-width types (eg uint16_t) to ensure
// execution that's consistent with the embedded device.
#define char    BADTYPE
#define short   BADTYPE
#define int     BADTYPE
#define long    BADTYPE
#define static  BADSTATIC

#include "USB.h"
#include "Descriptor.h"

#undef static
#undef char
#undef short
#undef int
#undef long
