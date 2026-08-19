#ifndef __DFU_H__
#define __DFU_H__

#include "main.h"

int DFU_start(void);
int DFU_data(uint8_t *data);
void DFU_jump_to_bootloader(void);
uint32_t calculate_crc32(const uint8_t *data, uint32_t length);
uint32_t DFU_get_downloaded_size(void);

#endif
