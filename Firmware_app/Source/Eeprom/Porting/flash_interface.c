/* Includes ------------------------------------------------------------------*/
#include "eeprom_emul.h"
#include "flash_interface.h"

void FI_flash_erase_page(uint32_t page_num)
{
    ENTER_CRITICAL();
    
    watch_dog_feed();
    
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    fmc_page_erase(FLASH_BASE + page_num * FLASH_PAGE_SIZE);
    fmc_lock();
    
    watch_dog_feed();
    
    EXIT_CRITICAL();
}

void FI_flash_write(uint8_t *p_dest, uint8_t *p_src, uint32_t size_bytes)
{
    // The memory controller requires all flash writes to start on a 16-byte boundary and consist of 16 bytes in size
    // If the desired amount to be written is less than 16 bytes, this code writes the other bytes as 0xFF to preserve the contents.
    // The Memory Controller will automatically hold off the write of the next 16 bytes until the previous write is complete.
    // Note that reads or fetches from Flash should not take place until WBUSY=0 and an additional delay of 10 uSec has been added

    ENTER_CRITICAL();
    
    fmc_unlock();

    // Program
    for (int i = 0; i < size_bytes; i+=4) {
        watch_dog_feed();
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        if (FMC_READY != fmc_word_program((uint32_t)(p_dest + i), *(uint32_t*)(p_src + i))) {
            fmc_lock();
            return;
        }
    }

    fmc_lock();
    
    EXIT_CRITICAL();
}

/**
  * @brief  Write a double word at the given address in Flash
  * @param  Address Where to write
  * @param  Data What to write
  * @retval EE_Status
  */
EE_Status FI_WriteDoubleWord(uint32_t Address, uint64_t Data)
{
    FI_flash_write((uint8_t*)Address, (uint8_t*)&Data, 8);
    return EE_OK;
}

/**
  * @brief  Erase a page in polling mode
  * @param  Page Page number
  * @param  NbPages Number of pages to erase
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status FI_PageErase(uint32_t Page, uint16_t NbPages)
{
    for(int i=0; i<NbPages; i++){
        FI_flash_erase_page(Page+i);
    }
    return EE_OK;
}

/**
  * @brief  Erase a page with interrupt enabled
  * @param  Page Page number
  * @param  NbPages Number of pages to erase
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status FI_PageErase_IT(uint32_t Page, uint16_t NbPages)
{
    for(int i=0; i<NbPages; i++){
        FI_flash_erase_page(Page+i);
    }
    return EE_OK;
}

/**
  * @brief  Flush the caches if needed to keep coherency when the flash content is modified
  */
void FI_CacheFlush()
{
    return;
}

/**
  * @brief  Delete corrupted Flash address, can be called from NMI. No Timeout.
  * @param  Address Address of the FLASH Memory to delete
  * @retval EE_Status
  *           - EE_OK: on success
  *           - EE error code: if an error occurs
  */
EE_Status FI_DeleteCorruptedFlashAddress(uint32_t Address)
{
    uint64_t Data = 0;
    
    FI_flash_write((uint8_t*)Address, (uint8_t*)&Data, 8);
    
    return EE_OK;
}
