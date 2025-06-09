#ifndef __UTIL_H__
#define __UTIL_H__

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define M_PI                        (3.14159265358f)
#define M_2PI                       (6.28318530716f)
#define ONE_BY_SQRT3                (0.57735026919f)
#define TWO_BY_SQRT3                (2.0f * 0.57735026919f)
#define SQRT3                       (1.73205080756f)
#define SQRT3_BY_2                  (0.86602540378f)

#define POW2(x)                     ((x)*(x))
#define POW3(x)                     ((x)*(x)*(x))
#define POW4(x)                     ((x)*(x)*(x)*(x))
#define POW5(x)                     ((x)*(x)*(x)*(x)*(x))
#define POW6(x)                     ((x)*(x)*(x)*(x)*(x)*(x))
#define ABS(x)                      ( (x)>0?(x):-(x) ) 
#define MAX(x, y)                   (((x) > (y)) ? (x) : (y))
#define MIN(x, y)                   (((x) < (y)) ? (x) : (y))
#define NORM2_f(x,y)		        (sqrtf(POW2(x) + POW2(y)))
#define CLAMP(x, lower, upper)      (MIN(upper, MAX(x, lower)))
#define FLOAT_EQU(floatA, floatB)   ((ABS((floatA)-(floatB))) < 1e-6f)

#define REG32(addr)         (*(volatile uint32_t *)(uint32_t)(addr))

#define ENTER_CRITICAL()    {                                           \
                                uint32_t primask = __get_PRIMASK();     \
                                __disable_irq();                        \

#define EXIT_CRITICAL()         __set_PRIMASK(primask);                 \
                            }

static inline float wrap_pm(float x, float y)
{
    float intval = (float)nearbyint(x / y);
    return x - intval * y;
}

static inline float fmodf_pos(float x, float y)
{
    float res = wrap_pm(x, y);
    if (res < 0) res += y;
    return res;
}

static inline bool utils_saturate_vector_2d(float *x, float *y, float max) {
	bool retval = false;
	float mag = NORM2_f(*x, *y);
	max = ABS(max);

	if (mag < 1e-10f) {
		mag = 1e-10f;
	}

	if (mag > max) {
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}

extern inline float arm_sin_f32(float x);
extern inline float arm_cos_f32(float x);

/* alpha-beta observer */
typedef struct
{
    float dt;
    float xk_1;
    float vk_1;
    
    float a, b;
} ab_observer_t;

void ab_observer_init(ab_observer_t *observer, float dt, float a, float b);
void ab_observer_update(ab_observer_t *observer, float xm);

uint8_t crc8(const uint8_t *data, const uint32_t size);
uint16_t crc16(const uint8_t *data, const uint32_t size);
uint32_t crc32(const uint8_t *data, const uint32_t size);

int uint32_to_data(uint32_t val, uint8_t *data);
int int32_to_data(int32_t val, uint8_t *data);
int uint16_to_data(uint16_t val, uint8_t *data);
int int16_to_data(int16_t val, uint8_t *data);
int float_to_data(float val, uint8_t *data);

uint32_t data_to_uint32(uint8_t *data);
int32_t data_to_int32(uint8_t *data);
uint16_t data_to_uint16(uint8_t *data);
int16_t data_to_int16(uint8_t *data);
float data_to_float(uint8_t *data);

#endif
