#ifndef __DFU_H__
#define __DFU_H__

#include "main.h"

int DFU_start(void);
int DFU_data(uint8_t *data);
void DFU_jump_to_bootloader(void);

#endif
