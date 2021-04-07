#include "MD_MAX72xx.h"
#include "MD_MAX72xx_lib.h"

void MD_MAX72XX_control1(MD_MAX72XX_t *m,enum controlRequest_t mode, int value) {
    MD_MAX72XX_control2(m,0, MD_MAX72XX_getDeviceCount(m)-1, mode, value);
}

void MD_MAX72XX_clear1(MD_MAX72XX_t *m) {
    MD_MAX72XX_clear(m,0, MD_MAX72XX_getDeviceCount(m)-1);
}

uint8_t MD_MAX72XX_getColumn1(MD_MAX72XX_t *m,uint8_t c) {
    return MD_MAX72XX_getColumn(m,(c / COL_SIZE), c % COL_SIZE);
}

void MD_MAX72XX_update(MD_MAX72XX_t *m,enum controlValue_t mode) {
    MD_MAX72XX_control1(m,UPDATE, mode);
}
