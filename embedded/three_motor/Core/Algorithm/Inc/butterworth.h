#ifndef BUTTERWORTH_H_
#define BUTTERWORTH_H_

typedef struct { float b0, a1, a2; } ButterworthCoefficients;
typedef struct { float s1, s2; } ButterworthState;

void Butterworth_Design(ButterworthCoefficients *c, float cutoff_hz, float dt);
void Butterworth_Reset(ButterworthState *s, const ButterworthCoefficients *c, float value);
float Butterworth_Update(ButterworthState *s, const ButterworthCoefficients *c, float input);
#endif
