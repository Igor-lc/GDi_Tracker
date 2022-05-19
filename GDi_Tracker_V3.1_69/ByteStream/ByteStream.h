
#include <stdint.h>

extern void ShortToBigEndianStream(uint8_t *buf,uint16_t sh);
extern void IntToBigEndianStream(uint8_t *buf,uint32_t in);
extern void LongToBigEndianStream(uint8_t *buf,uint64_t ln);
extern void BufToLittleEndianInt(uint8_t *buf,uint32_t *in);


