#include "matrix.h"
#include "arm_math.h"
#include <string.h>

Matrix_t* M_Init(Matrix_Config_t *Config) {
  if (Config == NULL || Config->rows == 0 || Config->cols == 0) {
    return NULL;
  }

  Matrix_t *m = (Matrix_t *)malloc(sizeof(Matrix_t));
  memset(m, 0, sizeof(Matrix_t));
  float *da = (float *)malloc(Config->rows * Config->cols * sizeof(float));
  // float *data = (float *)memalign(8,Config->rows * Config->cols * sizeof(float));// 对齐

  if (da == NULL) {
        free(m);
        free(da);
        return NULL;  // 分配失败
    }
  else {
  memset(da, 0, Config->rows * Config->cols * sizeof(float));
  m->rows = Config->rows;
  m->cols = Config->cols;
  m->negative = Config->negative;//似乎没必要
  m->data = da;
  }

  arm_mat_init_f32(&m->M, m->rows, m->cols, m->data);
  return m;
}

Matrix_t* M_Multiply(Matrix_t* A ,Matrix_t* B)
{ 
  if(A != NULL && B != NULL)
  {
    if(A->cols == B->rows)
    {
      Matrix_Config_t CConfig=
      {
        .rows = A->rows,
        .cols = B->cols,
      };
      Matrix_t* DST = M_Init(&CConfig);
      arm_status STA = arm_mat_mult_f32(&A->M, &B->M, &DST->M);
      if(STA == ARM_MATH_SUCCESS)return DST;
      else return NULL;
    }
    else  return NULL;
  }
  else return NULL;
}

Matrix_t* M_Inverse(Matrix_t* A)
{ 
  if(A != NULL )
  {
      Matrix_Config_t CConfig=
      {
        .rows = A->rows,
        .cols = A->cols,
      };
      Matrix_t* DST = M_Init(&CConfig);
      arm_status STA = arm_mat_inverse_f32(&A->M, &DST->M);
      if(STA == ARM_MATH_SUCCESS)return DST;
      else return NULL;
  }
  else return NULL;
}