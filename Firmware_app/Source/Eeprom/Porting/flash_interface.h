/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_INTERFACE_H
#define __FLASH_INTERFACE_H

/* Includes ------------------------------------------------------------------*/
#include "eeprom_emul_types.h"
#include "main.h"
#include "util.h"

#define BANK_SIZE               (0x20000U)      // 128KB

/* Page state header */
#define EE_PAGESTAT_ERASED      (uint64_t)0xFFFFFFFFFFFFFFFFU  /*!< State saved in 1st,2nd,3rd,4th data type of page header */
#define EE_PAGESTAT_RECEIVE     (uint64_t)0xAAAAAAAAAAAAAAAAU  /*!< State saved in 1st data type of page header */
#define EE_PAGESTAT_ACTIVE      (uint64_t)0xAAAAAAAAAAAAAAAAU  /*!< State saved in 2nd data type of page header */
#define EE_PAGESTAT_VALID       (uint64_t)0xAAAAAAAAAAAAAAAAU  /*!< State saved in 3rd data type of page header */
#define EE_PAGESTAT_ERASING     (uint64_t)0xAAAAAAAAAAAAAAAAU  /*!< State saved in 4th data type of page header */

/* Description of the 8 Bytes (64 bits) element in flash   */
/* Bit:  63                  32  31      16  15         0  */
/*       <--- Data Value ----->  <-unused->  <-VirtAddr->  */
#define EE_ELEMENT_SIZE         8U                            /*!< Size of element in Bytes */
#define EE_ELEMENT_TYPE         uint64_t                      /*!< Type of element */
#define EE_VIRTUALADDRESS_TYPE  uint16_t                      /*!< Type of Virtual Address */
#define EE_VIRTUALADDRESS_SHIFT 0U                            /*!< Bits Shifting to get Virtual Address in element */
#define EE_CRC_TYPE             uint16_t                      /*!< Type of Crc */
#define EE_CRC_SHIFT            16U                           /*!< Bits Shifting to get Crc in element */
#define EE_DATA_TYPE            uint32_t                      /*!< Type of Data */
#define EE_DATA_SHIFT           32U                           /*!< Bits Shifting to get Data value in element */
#define EE_MASK_VIRTUALADDRESS  (uint64_t)0x000000000000FFFFU
#define EE_MASK_CRC             (uint64_t)0x00000000FFFF0000U
#define EE_MASK_DATA            (uint64_t)0xFFFFFFFF00000000U
#define EE_MASK_FULL            (uint64_t)0xFFFFFFFFFFFFFFFFU


/* Private functions ---------------------------------------------------------*/
/** @addtogroup EEPROM_Private_Functions
  * @{
  */

void FI_flash_erase_page(uint32_t page_num);
void FI_flash_write(uint8_t *p_dest, uint8_t *p_src, uint32_t size_bytes);

EE_Status FI_WriteDoubleWord(uint32_t Address, uint64_t Data);
EE_Status FI_PageErase(uint32_t Page, uint16_t NbPages);
EE_Status FI_PageErase_IT(uint32_t Page, uint16_t NbPages);
EE_Status FI_DeleteCorruptedFlashAddress(uint32_t Address);
void FI_CacheFlush(void);

#endif
