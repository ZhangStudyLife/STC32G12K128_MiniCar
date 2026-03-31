#ifndef __TRACK_IO_H__
#define __TRACK_IO_H__

#include "zf_common_typedef.h"

#define TRACK_IO_L2     (0)
#define TRACK_IO_L1     (1)
#define TRACK_IO_MID    (2)
#define TRACK_IO_R1     (3)
#define TRACK_IO_R2     (4)

extern uint8 track_io_data[5];

void Track_IO_Init(void);
void Track_IO_Update(void);

#endif
