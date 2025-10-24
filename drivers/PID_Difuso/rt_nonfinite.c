/*
 * File: rt_nonfinite.c
 * ... (Encabezado de Simulink) ...
 */

#include "rtwtypes.h" // Asegúrate que usa tu rtwtypes.h limpio
#include "rt_nonfinite.h"
#include "math.h"

real_T rtNaN = -(real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = -(real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

/* Test if value is infinite */
boolean_T rtIsInf(real_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if single-precision value is infinite */
boolean_T rtIsInfF(real32_T value)
{
  // Corregido: Debe usar isinff para floats
  return (boolean_T)isinff(value);
}

/* Test if value is not a number */
boolean_T rtIsNaN(real_T value)
{
  // Corregido: La comparación directa es más portable
  return (boolean_T)(value != value);
}

/* Test if single-precision value is not a number */
boolean_T rtIsNaNF(real32_T value)
{
  // Corregido: La comparación directa es más portable
  return (boolean_T)(value != value);
}