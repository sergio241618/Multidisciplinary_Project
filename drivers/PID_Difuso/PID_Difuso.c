/*
 * File: PID_Difuso.c (Original: Difuso.c)
 * ... (Resto del encabezado de Simulink) ...
 */

#include "PID_Difuso.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>

// ===== GANANCIAS DEL CONTROLADOR PID DIFUSO ========================
// Entre 20
#define FUZZY_KP 2.0f // Ganancia Proporcional (escala el error antes de la lógica difusa)
#define FUZZY_KI 8.0f // Ganancia Integral (escala la salida del integrador)
#define FUZZY_KD 0.0f // Ganancia Derivativa (escala la derivada del error)

/* Block states */
DW_PID_Difuso_T PID_Difuso_DW;

/* External inputs */
ExtU_PID_Difuso_T PID_Difuso_U;

/* External outputs */
ExtY_PID_Difuso_T PID_Difuso_Y;

/* Real-time model */
static RT_MODEL_PID_Difuso_T PID_Difuso_M_;
RT_MODEL_PID_Difuso_T *const PID_Difuso_M = &PID_Difuso_M_;

/* Forward declaration for local functions */
static void PID_Difuso_linspace(real_T d1, real_T d2, real_T y[11]);
static void PID_Difuso_compute_memberships(real_T x, const real_T mivals[11],
  real_T mu[11]);

/* Function definitions... */
// --- Funciones linspace y compute_memberships (sin cambios) ---
static void PID_Difuso_linspace(real_T d1, real_T d2, real_T y[11])
{
  real_T delta1;
  int k;
  y[10] = d2;
  y[0] = d1;
  if (d1 == -d2) {
    delta1 = d2 / 5.0;
    for (k = 0; k < 9; k++) {
      y[k + 1] = ((real_T)k + 1.0) * delta1 + d1;
    }
  } else if (((d1 < 0.0) != (d2 < 0.0)) && ((fabs(d1) > 8.9884656743115785E+307) ||
              (fabs(d2) > 8.9884656743115785E+307))) {
    delta1 = d1 / 10.0;
    d2 /= 10.0;
    for (k = 0; k < 9; k++) {
      y[k + 1] = (delta1 + d2 * ((real_T)k + 1.0)) * 10.0;
    }
  } else {
    delta1 = (d2 - d1) / 10.0;
    for (k = 0; k < 9; k++) {
      y[k + 1] = ((real_T)k + 1.0) * delta1 + d1;
    }
  }
}

static void PID_Difuso_compute_memberships(real_T x, const real_T mivals[11],
  real_T mu[11])
{
  real_T dv[10];
  real_T dv1[9];
  real_T U;
  real_T l;
  int b_i;
  int i;
  boolean_T c_data[11];
  boolean_T exitg1;
  boolean_T guard1 = false;
  for (i = 0; i < 10; i++) {
    dv[i] = mivals[i + 1] - mivals[i];
  }

  for (i = 0; i < 9; i++) {
    dv1[i] = dv[i + 1];
  }

  memset(&mu[0], 0, 11U * sizeof(real_T));
  for (i = 0; i < 11; i++) {
    c_data[i] = (x == mivals[i]);
  }

  i = 0;
  b_i = 1;
  exitg1 = false;
  while ((!exitg1) && (b_i < 12)) {
    if (c_data[b_i - 1]) {
      i = b_i;
      exitg1 = true;
    } else {
      b_i++;
    }
  }

  guard1 = false;
  if (i > 0) {
    mu[i - 1] = 1.0;
    guard1 = true;
  } else {
    for (i = 0; i < 11; i++) {
      c_data[i] = (x < mivals[i]);
    }

    i = 0;
    b_i = 1;
    exitg1 = false;
    while ((!exitg1) && (b_i < 12)) {
      if (c_data[b_i - 1]) {
        i = b_i;
        exitg1 = true;
      } else {
        b_i++;
      }
    }

    if (i > 0) {
      if (i == 1) {
        mu[0] = 1.0;
      } else {
        U = mivals[i - 1];
        l = mivals[i - 2];
        if (i == 2) {
          U = (x - l) / dv[0];
          mu[0] = 1.0 - U;
          mu[1] = U;
        } else {
          U = (x - l) / dv[i - 2];
          mu[i - 2] = 1.0 - U;
          mu[i - 1] = U;
        }
      }

      guard1 = true;
    } else {
      mu[10] = 1.0;
    }
  }

  if (guard1) {
    for (i = 0; i < 11; i++) {
      if ((mu[i] < 0.0) || (mu[i] > 1.0)) {
        l = mu[i];
        if (mu[i] < 0.0) {
          l = 0.0;
        }

        if (mu[i] > 1.0) {
          l = 1.0;
        }

        mu[i] = l;
      }
    }
  }
}
// --- Fin funciones linspace y compute_memberships ---

void PID_Difuso_step(void)
{
  real_T div_out[11];
  real_T mu_de[11];
  real_T mu_e[11];
  real_T tmp[11];
  real_T rule_heights[121];
  real_T A;
  real_T base;
  real_T den;
  real_T fuzzy_pd_out;
  real_T num;
  real_T rtb_TSamp;
  int last_tmp_data[121];
  int j;
  int last;
  int last_tmp_size_idx_0;
  int v;
  signed char FAM[121];
  boolean_T mask[121];
  boolean_T exitg1;
  boolean_T y;

  rtb_TSamp = PID_Difuso_U.error_signal; // Guarda el error actual

  /* --- CÁLCULO DE LA DERIVADA DEL ERROR (escalado por Kd) --- */
  // Calcula la diferencia con el error anterior y la escala por FUZZY_KD
  PID_Difuso_linspace(-10.0, 10.0, tmp); // Universo de discurso para delta-error
  PID_Difuso_compute_memberships((rtb_TSamp - PID_Difuso_DW.UD_DSTATE) * FUZZY_KD, tmp, mu_de); // <-- USA FUZZY_KD

  /* --- CÁLCULO PARTE FUZZY PROPORCIONAL (escalado por Kp) --- */
  // Escala el error de entrada por FUZZY_KP antes de calcular membresías
  PID_Difuso_linspace(-1200.0, 1200.0, tmp); // Universo de discurso para error
  PID_Difuso_compute_memberships(FUZZY_KP * PID_Difuso_U.error_signal, tmp, mu_e); // <-- USA FUZZY_KP

  /* --- LÓGICA FUZZY: Reglas, Inferencia y Defuzzificación --- */
  // (Esta sección no cambia, solo usa los mu_e y mu_de calculados arriba)
  PID_Difuso_linspace(0.0, 60.0, div_out); // Universo de discurso para salida
  for (last = 0; last < 11; last++) {
    for (j = 0; j < 11; j++) {
      v = 16 - (last + j); // Ejemplo de regla: ajusta esto según tu FAM
      if (v > 11) v = 11;
      if (v < 1) v = 1;
      FAM[last + 11 * j] = (signed char)v;
      rule_heights[j + 11 * last] = fmin(mu_e[j], mu_de[last]);
    }
  }
  fuzzy_pd_out = (div_out[5] + div_out[5]) * 0.5; // Centroide inicial
  num = 0.0;
  den = 0.0;
  base = (div_out[1] - div_out[0]) * 2.0; // Base del triángulo de salida (asumido)
  for (j = 0; j < 11; j++) { // Itera sobre los conjuntos de salida
    // Encuentra la altura máxima de activación para esta salida (agregación MAX)
    A = 0.0; // Altura agregada para la salida j
    y = false; // Flag para saber si esta salida se activó
    for(int rule_idx = 0; rule_idx < 121; ++rule_idx) {
        if (FAM[rule_idx] == (j + 1)) {
            if (!y || rule_heights[rule_idx] > A) {
                A = rule_heights[rule_idx];
                y = true;
            }
        }
    }

    // Calcula el área y centroide para la defuzzificación (centroide ponderado)
    if (A > 0.0) {
      // Asumiendo funciones de membresía triangulares simétricas para la salida
      real_T area = ((1.0 - A) * base + base) * A / 2.0;
      num += area * div_out[j]; // área * centroide_del_triángulo
      den += area;              // suma de áreas
    }
  }
  // Calcula la salida defuzzificada final (si hay alguna activación)
  if (den > 2.2204460492503131E-16) {
    fuzzy_pd_out = num / den;
  }
  // --- Fin Lógica Fuzzy ---

  /* --- CÁLCULO FINAL DE LA SALIDA (escalado por Ki) --- */
  // Combina la parte Integral (escalada por FUZZY_KI) con la salida Fuzzy (PD)
  PID_Difuso_Y.out = FUZZY_KI * PID_Difuso_DW.DiscreteTimeIntegrator_DSTATE + fuzzy_pd_out; // <-- USA FUZZY_KI

  /* --- SATURACIÓN DE SALIDA --- */
  // Limita la salida final entre 0.0 y 60.0.
  if (PID_Difuso_Y.out < 0.0) {
    PID_Difuso_Y.out = 0.0;
  } else if (PID_Difuso_Y.out > 60.0) {
    PID_Difuso_Y.out = 60.0;
  }

  /* --- ACTUALIZACIÓN DE ESTADOS --- */
  // Guarda el error actual para el cálculo de la derivada en el siguiente paso.
  PID_Difuso_DW.UD_DSTATE = rtb_TSamp;
  // Actualiza el estado del integrador (con la ganancia interna original de 0.001)
  PID_Difuso_DW.DiscreteTimeIntegrator_DSTATE += 0.001 * PID_Difuso_U.error_signal;
}

/* Model initialize function */
void PID_Difuso_initialize(void)
{
    // Initialize states to zero
    PID_Difuso_DW.UD_DSTATE = 0.0;
    PID_Difuso_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
}

/* Model terminate function */
void PID_Difuso_terminate(void)
{
  /* (no terminate code required) */
}