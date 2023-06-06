#include "struct_typedef.h"
#define MAX_PSC             1000
#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);
