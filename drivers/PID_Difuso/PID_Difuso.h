/*
 * File: PID_Difuso.h (Original: Difuso.h)
 * ... (Resto del encabezado de Simulink) ...
 */

#ifndef PID_Difuso_h_ // CAMBIADO
#define PID_Difuso_h_ // CAMBIADO
#ifndef PID_Difuso_COMMON_INCLUDES_ // CAMBIADO
#define PID_Difuso_COMMON_INCLUDES_ // CAMBIADO
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif /* PID_Difuso_COMMON_INCLUDES_ */ // CAMBIADO

#include "PID_Difuso_types.h" // CAMBIADO

/* Macros... */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif
#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) */
typedef struct {
  real_T UD_DSTATE;                    /* '<S1>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<Root>/Discrete-Time Integrator' */
} DW_PID_Difuso_T; // CAMBIADO

/* External inputs */
typedef struct {
  real_T error_signal;                 /* '<Root>/In1' */
} ExtU_PID_Difuso_T; // CAMBIADO

/* External outputs */
typedef struct {
  real_T out;                          /* '<Root>/out' */
} ExtY_PID_Difuso_T; // CAMBIADO

/* Real-time Model Data Structure */
struct tag_RTM_PID_Difuso_T { // CAMBIADO
  const char_T * volatile errorStatus;
};

/* Block states */
extern DW_PID_Difuso_T PID_Difuso_DW; // CAMBIADO

/* External inputs */
extern ExtU_PID_Difuso_T PID_Difuso_U; // CAMBIADO

/* External outputs */
extern ExtY_PID_Difuso_T PID_Difuso_Y; // CAMBIADO

/* Model entry point functions */
extern void PID_Difuso_initialize(void); // CAMBIADO
extern void PID_Difuso_step(void);       // CAMBIADO
extern void PID_Difuso_terminate(void);  // CAMBIADO

/* Real-time Model object */
extern RT_MODEL_PID_Difuso_T *const PID_Difuso_M; // CAMBIADO

/* ... (Comentarios de jerarquía eliminados por brevedad) ... */

#endif /* PID_Difuso_h_ */ // CAMBIADO