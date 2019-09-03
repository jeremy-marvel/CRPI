#if HAVE_CONFIG_H
#include "rcs_config.h"
#endif

/*
   sincos.c

   Modification history:

   18-Dec-2001  FMP reworked with better def'ing
   30-Jan-1997  FMP added #ifdef __CPLUSPLUS__ around math.h include
   29-Jan-1997  FMP changed added PLATFORM_WITH_ to make more obvious
*/

#ifndef HAVE_SINCOS

#include "sincos.h"

#ifndef SINCOS_SUPPORT

#include <math.h>

void sincos(double x, double *sx, double *cx)
{
  *sx = sin(x);
  *cx = cos(x);
}
#else
void sincos_c_not_empty(void);

void sincos_c_not_empty(void)
{
}
#endif

#else
void sincos_c_not_empty2(void);

void sincos_c_not_empty2(void)
{
}
#endif



