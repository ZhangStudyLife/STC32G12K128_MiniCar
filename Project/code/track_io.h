#ifndef __TRACK_IO_H__
#define __TRACK_IO_H__

#include "zf_common_typedef.h"

#define TRACK_IO_LEFT   (0)
#define TRACK_IO_RIGHT  (1)

extern uint8 track_io_data[2];

void Track_IO_Init(void);
void Track_IO_Update(void);

#endif
