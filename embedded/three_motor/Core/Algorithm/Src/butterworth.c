#include "butterworth.h"
#include <math.h>

// 双线性变换并对截止频率预畸变，二阶Butterworth，Q=1/sqrt(2)。
void Butterworth_Design(ButterworthCoefficients *c, float cutoff_hz, float dt) {
  float k = tanf(3.14159265358979323846f * cutoff_hz * dt);
  float norm = 1.0f / (1.0f + 1.41421356237f * k + k * k);
  c->b0 = k * k * norm;
  c->a1 = 2.0f * (k * k - 1.0f) * norm;
  c->a2 = (1.0f - 1.41421356237f * k + k * k) * norm;
}

void Butterworth_Reset(ButterworthState *s, const ButterworthCoefficients *c, float value) {
  s->s1 = (1.0f - c->b0) * value;
  s->s2 = (c->b0 - c->a2) * value;
}

float Butterworth_Update(ButterworthState *s, const ButterworthCoefficients *c, float input) {
  float output = c->b0 * input + s->s1;
  s->s1 = 2.0f * c->b0 * input - c->a1 * output + s->s2;
  s->s2 = c->b0 * input - c->a2 * output;
  return output;
}
