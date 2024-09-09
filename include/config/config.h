#pragma once

/***************************/
/* 6D DOUBLE INTEGRATOR CONFIG  */
/***************************/

// #define MODEL 1

// #define MAX_TREE_SIZE 300000
// #define MAX_ITER 300
// #define STEP_SIZE 0.1f
// #define MAX_PROPAGATION_DURATION 10

// #define GOAL_THRESH 0.05f

// #define STATE_DIM 6
// #define CONTROL_DIM 3
// #define SAMPLE_DIM (STATE_DIM + CONTROL_DIM + 1)

// #define W_DIM 3
// #define C_DIM 1
// #define V_DIM 3

// #define W_MIN 0.0f
// #define W_MAX 1.0f
// #define W_SIZE 1.0f

// #define C_MIN -M_PI
// #define C_MAX M_PI

// #define V_MIN -0.3f
// #define V_MAX 0.3f

// #define A_MIN -0.2f
// #define A_MAX 0.2f

// #define W_R1_LENGTH 8
// #define C_R1_LENGTH 1
// #define V_R1_LENGTH 4

// #define W_R2_LENGTH 2
// #define C_R2_LENGTH 1
// #define V_R2_LENGTH 2

// #define W_R1_SIZE ((W_MAX - W_MIN) / W_R1_LENGTH)
// #define C_R1_SIZE ((C_MAX - C_MIN) / C_R1_LENGTH)
// #define V_R1_SIZE ((V_MAX - V_MIN) / V_R1_LENGTH)

// #define W_R1_VOL (W_R1_SIZE * W_R1_SIZE * W_R1_SIZE)

// #define NUM_R1_REGIONS (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH * V_R1_LENGTH * V_R1_LENGTH * V_R1_LENGTH)
// #define NUM_R2_REGIONS (NUM_R1_REGIONS * W_R2_LENGTH * W_R2_LENGTH * W_R2_LENGTH * V_R2_LENGTH * V_R2_LENGTH * V_R2_LENGTH)
// #define NUM_R2_PER_R1 W_R2_LENGTH *W_R2_LENGTH *W_R2_LENGTH *V_R2_LENGTH *V_R2_LENGTH *V_R2_LENGTH
// #define NUM_R1_REGIONS_KERNEL1 1024
// #define NUM_PARTIAL_SUMS 1024

// #define EPSILON 1e-2f
// #define VERBOSE 1

// // --- UNICYCLE MODEL: MODEL 0 ---
// #define UNI_MIN_STEERING -M_PI / 2
// #define UNI_MAX_STEERING M_PI / 2
// #define UNI_MIN_DT 0.1f
// #define UNI_MAX_DT 2.0f
// #define UNI_LENGTH 1.0f

// // --- DUBINS AIRPLANE: MODEL 2 ---
// #define DUBINS_AIRPLANE_MIN_PR (-M_PI / 4)
// #define DUBINS_AIRPLANE_MAX_PR (M_PI / 4)
// #define DUBINS_AIRPLANE_MIN_YR (-M_PI / 4)
// #define DUBINS_AIRPLANE_MAX_YR (M_PI / 4)
// #define DUBINS_AIRPLANE_MIN_YAW -M_PI
// #define DUBINS_AIRPLANE_MAX_YAW M_PI
// #define DUBINS_AIRPLANE_MIN_PITCH -M_PI / 3
// #define DUBINS_AIRPLANE_MAX_PITCH M_PI / 3

// // --- NON LINEAR QUAD: MODEL 3 ---
// #define QUAD_MIN_Zc -2.0f
// #define QUAD_MAX_Zc 2.0f
// #define QUAD_MIN_Lc -2.0f
// #define QUAD_MAX_Lc 2.0f
// #define QUAD_MIN_Mc -2.0f
// #define QUAD_MAX_Mc 2.0f
// #define QUAD_MIN_Nc -2.0f
// #define QUAD_MAX_Nc 2.0f
// #define QUAD_MIN_YAW -M_PI
// #define QUAD_MAX_YAW M_PI
// #define QUAD_MIN_PITCH -M_PI / 3
// #define QUAD_MAX_PITCH M_PI / 3
// #define QUAD_MIN_ROLL -M_PI / 3
// #define QUAD_MAX_ROLL M_PI / 3
// #define QUAD_MIN_ANGLE_RATE -30.0f
// #define QUAD_MAX_ANGLE_RATE 30.0f
// #define NU 10e-3f
// #define MU 2e-6f
// #define KM 0.03f
// #define IX 1.0f
// #define IY 1.0f
// #define IZ 2.0f
// #define GRAVITY -9.81f
// #define MASS 1.0f
// #define MASS_INV 1.0f / MASS

/***************************/
/* DUBINS AIRPLANE CONFIG  */
/***************************/

// #define MODEL 2

// #define MAX_TREE_SIZE 200000
// #define MAX_ITER 300
// #define STEP_SIZE 0.1f
// #define MAX_PROPAGATION_DURATION 10

// #define GOAL_THRESH 0.05f

// #define STATE_DIM 6
// #define CONTROL_DIM 3
// #define SAMPLE_DIM (STATE_DIM + CONTROL_DIM + 1)

// #define W_DIM 3
// #define C_DIM 2
// #define V_DIM 1

// #define W_MIN 0.0f
// #define W_MAX 1.0f
// #define W_SIZE 1.0f

// #define C_MIN -M_PI
// #define C_MAX M_PI

// #define V_MIN 0.0f
// #define V_MAX 0.3f

// #define A_MIN -0.3f
// #define A_MAX 0.3f

// #define W_R1_LENGTH 8
// #define C_R1_LENGTH 8
// #define V_R1_LENGTH 1

// #define W_R2_LENGTH 2
// #define C_R2_LENGTH 3
// #define V_R2_LENGTH 1

// #define W_R1_SIZE ((W_MAX - W_MIN) / W_R1_LENGTH)
// #define C_R1_SIZE ((C_MAX - C_MIN) / C_R1_LENGTH)
// #define V_R1_SIZE ((V_MAX - V_MIN) / V_R1_LENGTH)

// #define W_R1_VOL (W_R1_SIZE * W_R1_SIZE * W_R1_SIZE)

// #define NUM_R1_REGIONS (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH * V_R1_LENGTH)
// #define NUM_R2_REGIONS (NUM_R1_REGIONS * W_R2_LENGTH * W_R2_LENGTH * W_R2_LENGTH * C_R2_LENGTH * C_R2_LENGTH * V_R2_LENGTH)
// #define NUM_R2_PER_R1 W_R2_LENGTH *W_R2_LENGTH *W_R2_LENGTH *C_R2_LENGTH *C_R2_LENGTH *V_R2_LENGTH
// #define NUM_R1_REGIONS_KERNEL1 1024
// #define NUM_PARTIAL_SUMS 1024

// #define EPSILON 1e-2f
// #define VERBOSE 1

// // --- UNICYCLE MODEL: MODEL 0 ---
// #define UNI_MIN_STEERING -M_PI / 2
// #define UNI_MAX_STEERING M_PI / 2
// #define UNI_MIN_DT 0.1f
// #define UNI_MAX_DT 2.0f
// #define UNI_LENGTH 1.0f

// // --- DUBINS AIRPLANE: MODEL 2 ---
// #define DUBINS_AIRPLANE_MIN_PR (-M_PI / 4)
// #define DUBINS_AIRPLANE_MAX_PR (M_PI / 4)
// #define DUBINS_AIRPLANE_MIN_YR (-M_PI / 4)
// #define DUBINS_AIRPLANE_MAX_YR (M_PI / 4)
// #define DUBINS_AIRPLANE_MIN_YAW -M_PI
// #define DUBINS_AIRPLANE_MAX_YAW M_PI
// #define DUBINS_AIRPLANE_MIN_PITCH -M_PI / 3
// #define DUBINS_AIRPLANE_MAX_PITCH M_PI / 3

// // --- NON LINEAR QUAD: MODEL 3 ---
// #define QUAD_MIN_Zc -2.0f
// #define QUAD_MAX_Zc 2.0f
// #define QUAD_MIN_Lc -2.0f
// #define QUAD_MAX_Lc 2.0f
// #define QUAD_MIN_Mc -2.0f
// #define QUAD_MAX_Mc 2.0f
// #define QUAD_MIN_Nc -2.0f
// #define QUAD_MAX_Nc 2.0f
// #define QUAD_MIN_YAW -M_PI
// #define QUAD_MAX_YAW M_PI
// #define QUAD_MIN_PITCH -M_PI / 3
// #define QUAD_MAX_PITCH M_PI / 3
// #define QUAD_MIN_ROLL -M_PI / 3
// #define QUAD_MAX_ROLL M_PI / 3
// #define QUAD_MIN_ANGLE_RATE -30.0f
// #define QUAD_MAX_ANGLE_RATE 30.0f
// #define NU 10e-3f
// #define MU 2e-6f
// #define KM 0.03f
// #define IX 1.0f
// #define IY 1.0f
// #define IZ 2.0f
// #define GRAVITY -9.81f
// #define MASS 1.0f
// #define MASS_INV 1.0f / MASS

/***************************/
/* NON LINEAR QUAD CONFIG  */
/***************************/

#define MODEL 3

#define MAX_TREE_SIZE 400000
#define MAX_ITER 300
#define STEP_SIZE 0.1f
#define MAX_PROPAGATION_DURATION 10

#define GOAL_THRESH 5.0f

#define STATE_DIM 12
#define CONTROL_DIM 4
#define SAMPLE_DIM (STATE_DIM + CONTROL_DIM + 1)

#define W_DIM 3
#define C_DIM 3
#define V_DIM 3

#define W_MIN 0.0f
#define W_MAX 100.0f
#define W_SIZE 100.0f

#define C_MIN -M_PI
#define C_MAX M_PI

#define V_MIN -30.0f
#define V_MAX 30.0f

#define A_MIN -30.0f
#define A_MAX 30.0f

#define W_R1_LENGTH 8
#define C_R1_LENGTH 2
#define V_R1_LENGTH 2

#define W_R2_LENGTH 2
#define C_R2_LENGTH 1
#define V_R2_LENGTH 2

#define W_R1_SIZE ((W_MAX - W_MIN) / W_R1_LENGTH)
#define C_R1_SIZE ((C_MAX - C_MIN) / C_R1_LENGTH)
#define V_R1_SIZE ((V_MAX - V_MIN) / V_R1_LENGTH)

#define W_R1_VOL (W_R1_SIZE * W_R1_SIZE * W_R1_SIZE)

#define NUM_R1_REGIONS \
    (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH * V_R1_LENGTH * V_R1_LENGTH * V_R1_LENGTH)
#define NUM_R2_REGIONS                                                                                                                \
    (NUM_R1_REGIONS * W_R2_LENGTH * W_R2_LENGTH * W_R2_LENGTH * C_R2_LENGTH * C_R2_LENGTH * C_R2_LENGTH * V_R2_LENGTH * V_R2_LENGTH * \
     V_R2_LENGTH)
#define NUM_R2_PER_R1 W_R2_LENGTH *W_R2_LENGTH *W_R2_LENGTH *C_R2_LENGTH *C_R2_LENGTH *C_R2_LENGTH *V_R2_LENGTH *V_R2_LENGTH *V_R2_LENGTH
#define NUM_R1_REGIONS_KERNEL1 1024
#define NUM_PARTIAL_SUMS 1024

#define EPSILON 1e-2f
#define VERBOSE 1

// --- UNICYCLE MODEL: MODEL 0 ---
#define UNI_MIN_STEERING -M_PI / 2
#define UNI_MAX_STEERING M_PI / 2
#define UNI_MIN_DT 0.1f
#define UNI_MAX_DT 2.0f
#define UNI_LENGTH 1.0f

// --- DUBINS AIRPLANE: MODEL 2 ---
#define DUBINS_AIRPLANE_MIN_PR (-M_PI / 4)
#define DUBINS_AIRPLANE_MAX_PR (M_PI / 4)
#define DUBINS_AIRPLANE_MIN_YR (-M_PI / 4)
#define DUBINS_AIRPLANE_MAX_YR (M_PI / 4)
#define DUBINS_AIRPLANE_MIN_YAW -M_PI
#define DUBINS_AIRPLANE_MAX_YAW M_PI
#define DUBINS_AIRPLANE_MIN_PITCH -M_PI / 3
#define DUBINS_AIRPLANE_MAX_PITCH M_PI / 3

// --- NON LINEAR QUAD: MODEL 3 ---
#define QUAD_MIN_Zc 0.0f
#define QUAD_MAX_Zc 30.0f
#define QUAD_MIN_Lc -M_PI
#define QUAD_MAX_Lc M_PI
#define QUAD_MIN_Mc -M_PI
#define QUAD_MAX_Mc M_PI
#define QUAD_MIN_Nc -M_PI
#define QUAD_MAX_Nc M_PI
#define QUAD_MIN_YAW -M_PI
#define QUAD_MAX_YAW M_PI
#define QUAD_MIN_PITCH -M_PI
#define QUAD_MAX_PITCH M_PI
#define QUAD_MIN_ROLL -M_PI
#define QUAD_MAX_ROLL M_PI
#define QUAD_MIN_ANGLE_RATE -30.0f
#define QUAD_MAX_ANGLE_RATE 30.0f
#define NU 10e-3f
#define MU 2e-6f
#define KM 0.03f
#define IX 1.0f
#define IY 1.0f
#define IZ 2.0f
#define GRAVITY -9.81f
#define MASS 1.0f
#define MASS_INV 1.0f / MASS