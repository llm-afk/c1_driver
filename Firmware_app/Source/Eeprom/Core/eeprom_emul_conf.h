/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_EMUL_CONF_H
#define __EEPROM_EMUL_CONF_H

#include "main.h"

/* Configuration of eeprom emulation in flash, can be custom */
#define START_PAGE_ADDRESS      (FLASH_BASE + EEPROM_PAGE * FLASH_PAGE_SIZE)    /*!< Start address of the 1st page in flash, for EEPROM emulation */
#define CYCLES_NUMBER           10U                                             /*!< Number of 10Kcycles requested, minimum 1 for 10Kcycles (default), for instance 10 to reach 100Kcycles. This factor will increase pages number */
#define GUARD_PAGES_NUMBER      2U                                              /*!< Number of guard pages avoiding frequent transfers (must be multiple of 2): 0,2,4.. */
#define NB_OF_VARIABLES         100U                                            /*!< Number of variables to handle in eeprom */

#endif
