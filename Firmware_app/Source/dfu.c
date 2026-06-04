#include "dfu.h"
#include "util.h"
#include "eeprom_emul.h"

static uint32_t mByteCount = 0;

static int erase_app_back(void);
static int write_app_back(uint8_t *data, uint32_t offset, uint32_t len);

int DFU_start(void)
{
    if (erase_app_back()) {
        return -1;
    }

    mByteCount = 0;

    return 0;
}

int DFU_data(uint8_t *data)
{
    if ((mByteCount + 8) <= (APP_MAX_SIZE)) {
        write_app_back(data, mByteCount, 8);
        mByteCount += 8;
    }
    
    return 0;
}

/**
  * @brief  Auto-detect the actual bootloader address at runtime.
  *
  *          The bootloader may reside at either:
  *          - Page 80 (0x08014000) — old layout devices
  *          - Page 90 (0x08016800) — new layout devices
  *
  *          Detection: check if a valid stack pointer and reset vector
  *          exist at the candidate address. SP must be in RAM range
  *          (0x20000000–0x20008000), and reset vector must be in flash
  *          with Thumb bit set (odd address).
  *
  * @retval Detected bootloader base address
  */
static uint32_t detect_bootloader_addr(void)
{
    /* Candidate bootloader addresses: try new layout first, then old */
    const uint32_t candidates[] = {
        (0x8000000 + 90 * FLASH_PAGE_SIZE),  /* Page 90 — new layout */
        (0x8000000 + 80 * FLASH_PAGE_SIZE),  /* Page 80 — old layout */
    };

    for (int i = 0; i < (int)(sizeof(candidates) / sizeof(candidates[0])); i++)
    {
        uint32_t sp  = *(uint32_t *)candidates[i];
        uint32_t pc  = *(uint32_t *)(candidates[i] + 4);

        /* SP must point into RAM (0x20000000–0x20008000 for 32KB SRAM) */
        if (sp < 0x20000000 || sp > 0x20008000) {
            continue;
        }

        /* PC must be in flash and Thumb bit set */
        if ((pc & 1) == 0) {
            continue;
        }
        if ((pc & ~1) < 0x08000000 || (pc & ~1) > 0x08020000) {
            continue;
        }

        return candidates[i];
    }

    /* Fallback — use compiled-in address */
    return BOOTLOADER_ADDR;
}

void DFU_jump_to_bootloader(void)
{
    for (int i = 0; i < 8; i++) {
        // Disable all interrupts
        NVIC->ICER[i] = 0xFFFFFFFF;
        // Clear pending interrupts
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    LED_ACT_RESET();

    uint32_t boot_addr = detect_bootloader_addr();

    /* Initialize user application's Stack Pointer */
    __set_MSP(*(uint32_t *) boot_addr);

    /* Jump to the bootloader */
    (*(void (*)(void))(*(uint32_t *) (boot_addr + 4)))();
}

static int erase_app_back(void)
{
    uint32_t       addr;
    fmc_state_enum status;

    fmc_unlock();

    // Erase
    for (addr = APP_BACK_ADDR; addr < APP_BACK_ADDR + APP_MAX_SIZE; addr += PAGE_SIZE) {
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        status = fmc_page_erase(addr);
        if (status != FMC_READY) {
            fmc_lock();
            return -1;
        }
        
        watch_dog_feed();
    }

    fmc_lock();

    // Check
    for (addr = APP_BACK_ADDR; addr < APP_BACK_ADDR + APP_MAX_SIZE; addr += 4) {
        if (0xFFFFFFFF != *((uint32_t *) addr)) {
            return -2;
        }
        
        watch_dog_feed();
    }

    return 0;
}

static int write_app_back(uint8_t *data, uint32_t offset, uint32_t len)
{
    uint32_t       addr;
    uint32_t      *word;
    fmc_state_enum status;

    fmc_unlock();

    len /= 4;
    word = (uint32_t *) data;

    // Program
    for (uint32_t i = 0; i < len; i++) {
        addr = APP_BACK_ADDR + offset + 4 * i;
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        status = fmc_word_program(addr, word[i]);
        if (status != FMC_READY) {
            fmc_lock();
            return -1;
        }
    }

    fmc_lock();

    // Check
    for (uint32_t i = 0; i < len; i++) {
        addr = APP_BACK_ADDR + offset + 4 * i;
        if (word[i] != REG32(addr)) {
            return -2;
        }
    }

    return 0;
}
