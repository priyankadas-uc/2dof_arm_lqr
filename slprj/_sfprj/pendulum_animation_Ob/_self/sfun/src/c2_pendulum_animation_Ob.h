#ifndef __c2_pendulum_animation_Ob_h__
#define __c2_pendulum_animation_Ob_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_SFc2_pendulum_animation_ObInstanceStruct
#define typedef_SFc2_pendulum_animation_ObInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_pendulum_animation_Ob;
} SFc2_pendulum_animation_ObInstanceStruct;

#endif                                 /*typedef_SFc2_pendulum_animation_ObInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c2_pendulum_animation_Ob_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_pendulum_animation_Ob_get_check_sum(mxArray *plhs[]);
extern void c2_pendulum_animation_Ob_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
