#ifndef __KEY_H__
#define __KEY_H__

#include "zf_common_typedef.h"

#define KEY_1   (0)
#define KEY_2   (1)
#define KEY_3   (2)
#define KEY_4   (3)

extern volatile uint8 key_data[4];

void Key_Init(void);
void Key_Update(void);

#endif
