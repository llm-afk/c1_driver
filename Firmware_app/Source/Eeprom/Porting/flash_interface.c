/* Includes ------------------------------------------------------------------*/
#include "eeprom_emul.h"
#include "flash_interface.h"

static EE_Status FI_flash_erase_page_status(uint32_t page_num)
{
    uint32_t address;
    EE_Status status = EE_OK;

    if (page_num >= (BANK_SIZE / FLASH_PAGE_SIZE)) {
        return EE_ERASE_ERROR;
    }

    address = FLASH_BASE + page_num * FLASH_PAGE_SIZE;

    ENTER_CRITICAL();
    
    watch_dog_feed();
    
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    if (FMC_READY != fmc_page_erase(address)) {
        status = EE_ERASE_ERROR;
    }
    fmc_lock();
    
    watch_dog_feed();
    
    EXIT_CRITICAL();

    return status;
}

void FI_flash_erase_page(uint32_t page_num)
{
    (void)FI_flash_erase_page_status(page_num);
}

static EE_Status FI_flash_write_status(uint8_t *p_dest, uint8_t *p_src, uint32_t size_bytes)
{
    uint32_t address = (uint32_t)p_dest;
    EE_Status status = EE_OK;

    if ((address < FLASH_BASE) ||
        ((address & 0x3U) != 0U) ||
        ((size_bytes & 0x3U) != 0U) ||
        (size_bytes > BANK_SIZE) ||
        (address > (FLASH_BASE + BANK_SIZE - size_bytes))) {
        return EE_WRITE_ERROR;
    }

    // The memory controller requires all flash writes to start on a 16-byte boundary and consist of 16 bytes in size
    // If the desired amount to be written is less than 16 bytes, this code writes the other bytes as 0xFF to preserve the contents.
    // The Memory Controller will automatically hold off the write of the next 16 bytes until the previous write is complete.
    // Note that reads or fetches from Flash should not take place until WBUSY=0 and an additional delay of 10 uSec has been added

    ENTER_CRITICAL();
    
    fmc_unlock();

    // Program
    for (uint32_t i = 0U; i < size_bytes; i += 4U) {
        watch_dog_feed();
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        if (FMC_READY != fmc_word_program((uint32_t)(p_dest + i), *(uint32_t*)(p_src + i))) {
            status = EE_WRITE_ERROR;
            break;
        }
    }

    fmc_lock();
    
    EXIT_CRITICAL();

    return status;
}

void FI_flash_write(uint8_t *p_dest, uint8_t *p_src, uint32_t size_bytes)
{
    (void)FI_flash_write_status(p_dest, p_src, size_bytes);
}

/**
  * @brief  Write a double word at the given address in Flash
  * @param  Address Where to write
  * @param  Data What to write
  * @retval EE_Status
  */
EE_Status FI_WriteDoubleWord(uint32_t Address, uint64_t Data)
{
    EE_Status status = FI_flash_write_status((uint8_t*)Address, (uint8_t*)&Data, 8);

    if ((status == EE_OK) && (*(__IO uint64_t*)Address != Data)) {
        return EE_WRITE_ERROR;
    }

    return status;
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
        EE_Status status = FI_flash_erase_page_status(Page+i);
        if (status != EE_OK) {
            return status;
        }
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
        EE_Status status = FI_flash_erase_page_status(Page+i);
        if (status != EE_OK) {
            return status;
        }
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
    
    return FI_flash_write_status((uint8_t*)Address, (uint8_t*)&Data, 8);
}
