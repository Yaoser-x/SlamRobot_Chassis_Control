#ifndef ESP12F_BOOT_CONTROL_H
#define ESP12F_BOOT_CONTROL_H

/** Enter the ESP12F ROM download boot mode using the board control pins. */
void Esp12fBootControl_EnterDownload(void);

/** Enter the ESP12F normal boot mode using the board control pins. */
void Esp12fBootControl_EnterNormal(void);

#endif
