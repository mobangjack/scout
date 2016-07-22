#ifndef __CRC32_H__
#define __CRC32_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void CRC32GenTab(void);
uint32_t CRC32Update(uint32_t crc_accum, uint8_t *data_blk_ptr, uint32_t data_blk_size);

#ifdef __cplusplus
}
#endif

#endif
