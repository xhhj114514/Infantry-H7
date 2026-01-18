#ifndef MATRIX_H
#define MATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

# include "memory.h"
#include "arm_math.h"

typedef enum{
    NEGATIVE = -1,
    NOT_NEGATIVE = 1,
}NegaFlag_e;

typedef struct {
    uint16_t rows;
    uint16_t cols;
    NegaFlag_e negative;
}Matrix_Config_t;

typedef struct {
    float *data;
    uint16_t rows;
    uint16_t cols;
    NegaFlag_e negative;
    arm_matrix_instance_f32 M;
}Matrix_t;


Matrix_t* M_Init(Matrix_Config_t *Config);
Matrix_t* M_Multiply(Matrix_t* A ,Matrix_t* B);
Matrix_t* M_Inverse(Matrix_t* A);

#ifdef __cplusplus
}
#endif

#endif
