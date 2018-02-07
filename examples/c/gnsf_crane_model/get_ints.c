/* This function was automatically generated by CasADi */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else /* CODEGEN_PREFIX */
  #define CASADI_PREFIX(ID) get_ints_ ## ID
#endif /* CODEGEN_PREFIX */

#include <math.h>

#ifndef real_t
#define real_t double
#endif /* real_t */

#define to_double(x) (double) x
#define to_int(x) (int) x
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
real_t CASADI_PREFIX(fmin)(real_t x, real_t y) { return x<y ? x : y;}
#define fmin(x,y) CASADI_PREFIX(fmin)(x,y)
real_t CASADI_PREFIX(fmax)(real_t x, real_t y) { return x>y ? x : y;}
#define fmax(x,y) CASADI_PREFIX(fmax)(x,y)
#endif

#define PRINTF printf
real_t CASADI_PREFIX(sq)(real_t x) { return x*x;}
#define sq(x) CASADI_PREFIX(sq)(x)

real_t CASADI_PREFIX(sign)(real_t x) { return x<0 ? -1 : x>0 ? 1 : x;}
#define sign(x) CASADI_PREFIX(sign)(x)

static const int CASADI_PREFIX(s0)[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
#define s0 CASADI_PREFIX(s0)
static const int CASADI_PREFIX(s1)[5] = {1, 1, 0, 1, 0};
#define s1 CASADI_PREFIX(s1)
/* get_ints */
int get_ints(const real_t** arg, real_t** res, int* iw, real_t* w, int mem) {
  real_t a0=9.;
  if (res[0]!=0) res[0][0]=a0;
  a0=2.;
  if (res[1]!=0) res[1][0]=a0;
  real_t a1=1.;
  if (res[2]!=0) res[2][0]=a1;
  real_t a2=8.;
  if (res[3]!=0) res[3][0]=a2;
  if (res[4]!=0) res[4][0]=a1;
  a1=4.;
  if (res[5]!=0) res[5][0]=a1;
  a1=40.;
  if (res[6]!=0) res[6][0]=a1;
  if (res[7]!=0) res[7][0]=a0;
  return 0;
}

void get_ints_incref(void) {
}

void get_ints_decref(void) {
}

int get_ints_n_in(void) { return 1;}

int get_ints_n_out(void) { return 8;}

const char* get_ints_name_in(int i){
  switch (i) {
  case 0: return "i0";
  default: return 0;
  }
}

const char* get_ints_name_out(int i){
  switch (i) {
  case 0: return "o0";
  case 1: return "o1";
  case 2: return "o2";
  case 3: return "o3";
  case 4: return "o4";
  case 5: return "o5";
  case 6: return "o6";
  case 7: return "o7";
  default: return 0;
  }
}

const int* get_ints_sparsity_in(int i) {
  switch (i) {
  case 0: return s0;
  default: return 0;
  }
}

const int* get_ints_sparsity_out(int i) {
  switch (i) {
  case 0: return s1;
  case 1: return s1;
  case 2: return s1;
  case 3: return s1;
  case 4: return s1;
  case 5: return s1;
  case 6: return s1;
  case 7: return s1;
  default: return 0;
  }
}

int get_ints_work(int *sz_arg, int* sz_res, int *sz_iw, int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 8;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 3;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
