#ifndef CAL_ACC_H_
#define CAL_ACC_H_

#include "sensfusion.h"
#include "hg_math.h"
#include <sys/time.h>

uint64_t get_time_usec(void);


uint8_t acc_zero_bias_cal(att_info_t &att, const Vector3<float> &acc);



#endif




