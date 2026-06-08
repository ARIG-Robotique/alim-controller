//
// Created by gregorydepuille@sglk.local on 13/03/2021.
//

#include "itm_log.h"

#include <string.h>

void itmPrint(char _char, uint8_t level) {
#ifdef DEBUG_MODE
  while (ITM->PORT[level].u32 == 0UL) {
    __NOP();
  }
  ITM->PORT[level].u8 = (uint8_t) _char;
#endif
}

void itmPrintLn(const char *msg, uint8_t level) {
#ifdef DEBUG_MODE
  if (msg == NULL || level > 31 || level < LOG_LEVEL_MIN) {
    return;
  }
  for (int i = 0; i < strlen(msg); i++) {
    itmPrint(msg[i], level);
  }
  itmPrint('\n', level);
#endif
}
