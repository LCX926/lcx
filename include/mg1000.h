#ifndef MG1000_H
#define MG1000_H

#ifdef __cplusplus
extern "C" {
#endif

enum { MG1000_INPUT_COUNT = 26, MG1000_PARAM_COUNT = 23, MG1000_OUTPUT_COUNT = 17 };
/* SI units throughout; angles in rad. Order matches Simulink root ports. */
typedef enum {
    MG_IN_VL, MG_IN_VR, MG_IN_TCMD_L, MG_IN_TCMD_R,
    MG_IN_FX_L, MG_IN_FZ_L, MG_IN_FX_R, MG_IN_FZ_R,
    MG_IN_X_L, MG_IN_VCYL_L, MG_IN_Q_L, MG_IN_QDOT_L, MG_IN_QDDOT_L,
    MG_IN_X_R, MG_IN_VCYL_R, MG_IN_Q_R, MG_IN_QDOT_R, MG_IN_QDDOT_R,
    MG_IN_AX, MG_IN_PITCH, MG_IN_PITCH_RATE, MG_IN_PITCH_ACC,
    MG_IN_AN, MG_IN_GRADE, MG_IN_MDRUM_L, MG_IN_MDRUM_R
} Mg1000Input;
typedef enum {
    MG_OUT_TLOAD_L, MG_OUT_TLOAD_R, MG_OUT_FTRAC_L, MG_OUT_FTRAC_R,
    MG_OUT_FCYL_L, MG_OUT_FCYL_R, MG_OUT_PITCH, MG_OUT_Q_L, MG_OUT_Q_R,
    MG_OUT_N_L, MG_OUT_N_R, MG_OUT_MREACTION_L, MG_OUT_MREACTION_R,
    MG_OUT_MARGIN_L, MG_OUT_MARGIN_R, MG_OUT_PITCH_RESIDUAL,
    MG_OUT_LONGITUDINAL_RESIDUAL
} Mg1000Output;
typedef enum {
    MG_P_MASS, MG_P_UNUSED_2, MG_P_UNUSED_3, MG_P_ARM_MASS,
    MG_P_ARM_LENGTH, MG_P_ARM_COM_RADIUS, MG_P_ARM_INERTIA,
    MG_P_SUPPORT_SPAN, MG_P_PIVOT_SPAN, MG_P_TRACTION_HEIGHT,
    MG_P_SHOE_HEIGHT, MG_P_WHEEL_RADIUS, MG_P_TRACTION_LIMIT,
    MG_P_FRICTION, MG_P_PITCH_INERTIA, MG_P_LINK_L0,
    MG_P_LINK_L1, MG_P_LINK_L2, MG_P_MIN_LEVER, MG_P_GRAVITY,
    MG_P_ROT_DAMPING, MG_P_CYL_DAMPING, MG_P_PIVOT_OFFSET
} Mg1000Parameter;
typedef enum {
    MG1000_OK = 0, MG1000_BAD_ARGUMENT = 1,
    MG1000_BAD_PARAMETER = 2, MG1000_NUMERIC_ERROR = 3
} Mg1000Status;

void mg1000_default_parameters(double parameters[MG1000_PARAM_COUNT]);
/* No state, allocation or I/O. Caller schedules one call per model tick.
 * Returns error without modifying output. Null parameters are invalid.
 * Do not compile with fast-math: finite-value checks are part of the API. */
Mg1000Status mg1000_step(const double input[MG1000_INPUT_COUNT],
                         const double parameters[MG1000_PARAM_COUNT],
                         double output[MG1000_OUTPUT_COUNT]);
#ifdef __cplusplus
}
#endif
#endif
