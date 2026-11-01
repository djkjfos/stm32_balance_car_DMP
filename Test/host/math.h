/* TinyCC 0.9.27's Win64 math header contains unsupported x87 inline assembly.
 * Use Windows CRT double functions for this native test backend only. */
#ifdef __TINYC__
#ifndef TEST_MATH_H
#define TEST_MATH_H
double atan2(double,double);
double fabs(double);
double sqrt(double);
int _finite(double);
int _isnan(double);
static inline float atan2f(float a,float b) { return (float)atan2((double)a,(double)b); }
static inline float fabsf(float a) { return (float)fabs((double)a); }
static inline float sqrtf(float a) { return (float)sqrt((double)a); }
static inline float fminf(float a,float b) { return _isnan(a)?b:(_isnan(b)?a:(a<b?a:b)); }
static inline float fmaxf(float a,float b) { return _isnan(a)?b:(_isnan(b)?a:(a>b?a:b)); }
#define isfinite(x) _finite((double)(x))
#endif
#else
#include_next <math.h>
#endif
