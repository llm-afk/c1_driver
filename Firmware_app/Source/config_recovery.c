/**
  ******************************************************************************
  * @file    config_recovery.c
  * @brief   Configuration recovery from old flash layout to new layout.
  *
  *          Scans old EEPROM flash pages for valid configuration data and
  *          migrates missing variables to the current EEPROM area.
  *
  *          Background:
  *          - Old layout (pre-21e44d8): EEPROM at pages 101-127, encoder at 100
  *          - New layout (21e44d8+):    EEPROM at pages 116-127, encoder at 115
  *
  *          During OTA, the bootloader only rewrites APP area (pages 0-44).
  *          The old EEPROM/calibration pages are NOT erased, but the new
  *          firmware looks for them at different addresses, causing config loss.
  *
  *          This module bridges the gap by scanning old locations after EE_Init
  *          and migrating any missing configuration to the new location.
  ******************************************************************************
  */

#include "config_recovery.h"
#include "main.h"
#include "util.h"
#include "eeprom_emul.h"
#include "eeprom_emul_conf.h"
#include "flash_interface.h"
#include "encoder.h"
#include <string.h>

/* ─── EEPROM data element layout (64-bit) ─────────────────────────────────
   | 63 … 32 | 31 … 16 | 15 … 0  |
   |  Data   |  CRC    | VirtAddr|
   ─────────────────────────────────────────────────────────────────────── */

/**
  * @brief  Scan old EEPROM flash pages for valid configuration data and
  *         migrate any missing variables to the current EEPROM area.
  *
  *         For each page in the old EEPROM range:
  *           1. Skip fully-erased pages
  *           2. Skip ERASING-state pages (incomplete transfer data)
  *           3. For ACTIVE/VALID/RECEIVE pages, scan all data elements
  *           4. Verify CRC for each element
  *           5. Collect the latest value for each virtual address
  *
  *         After scanning, for each recovered variable that is NOT already
  *         present in the current EEPROM, write it via EE_WriteVariable32bits.
  *
  * @retval Number of variables recovered and written
  */
uint32_t config_recovery_from_old_eeprom(void)
{
    /* Storage for recovered (virtual_address, data) pairs.
       NB_OF_VARIABLES = maximum unique variables possible.
       Static to avoid stack pressure (~600 bytes) on constrained MCU. */
    static uint16_t var_addrs[NB_OF_VARIABLES];
    static uint32_t var_data[NB_OF_VARIABLES];
    uint32_t var_count = 0;

    /*
     * Phase 1: Scan old EEPROM pages for valid data.
     *
     * Old layout EEPROM: pages 101-127 (27 reserved, 22 actually used with
     * CYCLES_NUMBER=10, PAGES_NUMBER=22 → pages 101-122 active).
     *
     * We scan all 27 pages. Pages that contain bootloader code (in new-layout
     * devices) will fail CRC and yield no valid data — this is safe.
     */
    for (uint32_t page = OLD_EEPROM_START_PAGE;
         page < OLD_EEPROM_START_PAGE + OLD_EEPROM_PAGE_COUNT;
         page++)
    {
        uint64_t *page_addr = (uint64_t *)(FLASH_BASE + page * FLASH_PAGE_SIZE);

        /* Read page header (first 4 elements determine page state) */
        uint64_t hdr0 = page_addr[0];
        uint64_t hdr1 = page_addr[1];
        uint64_t hdr2 = page_addr[2];
        uint64_t hdr3 = page_addr[3];

        /* Skip fully-erased pages — no data here */
        if (hdr0 == EE_PAGESTAT_ERASED &&
            hdr1 == EE_PAGESTAT_ERASED &&
            hdr2 == EE_PAGESTAT_ERASED &&
            hdr3 == EE_PAGESTAT_ERASED)
        {
            continue;
        }

        /* Skip ERASING pages — data may be incomplete or being transferred.
           ERASING = element[3] is NOT erased. */
        if (hdr3 != EE_PAGESTAT_ERASED)
        {
            continue;
        }

        /*
         * At this point the page has some data (ACTIVE, VALID, or RECEIVE).
         * Scan all data elements starting after the 4-element header.
         *
         * PAGE_SIZE / 8 = 1024 / 8 = 128 total elements per page
         * Header = elements 0-3, Data = elements 4-127 (124 data slots)
         */
        for (uint32_t i = 4; i < (FLASH_PAGE_SIZE / 8); i++)
        {
            uint64_t element = page_addr[i];

            /* Erased element = end of data in this page */
            if (element == EE_PAGESTAT_ERASED)
            {
                break;
            }

            /* Extract fields from the 64-bit element */
            uint16_t virt_addr = (uint16_t)(element & EE_MASK_VIRTUALADDRESS);
            uint32_t data_val  = (uint32_t)((element & EE_MASK_DATA) >> EE_DATA_SHIFT);
            uint16_t crc_val   = (uint16_t)((element & EE_MASK_CRC) >> EE_CRC_SHIFT);

            /* Skip invalid virtual addresses (0x0000 and 0xFFFF are reserved) */
            if (virt_addr == 0x0000 || virt_addr == 0xFFFF)
            {
                continue;
            }

            /* Verify CRC using the EEPROM library's own CRC function */
            uint16_t crc_calc = CalculateCrc(data_val, virt_addr);
            if (crc_val != crc_calc)
            {
                /* CRC mismatch — this is not valid EEPROM data.
                   Could be bootloader code or corrupted flash. Skip it. */
                continue;
            }

            /*
             * Valid element found! Store it in our recovery table.
             * Later pages / later elements overwrite earlier ones for the
             * same virtual address (most recent value wins).
             */
            bool found = false;
            for (uint32_t j = 0; j < var_count; j++)
            {
                if (var_addrs[j] == virt_addr)
                {
                    var_data[j] = data_val;
                    found = true;
                    break;
                }
            }

            if (!found && var_count < NB_OF_VARIABLES)
            {
                var_addrs[var_count] = virt_addr;
                var_data[var_count] = data_val;
                var_count++;
            }
        }
    }

    /*
     * Phase 2: Write recovered variables to current EEPROM.
     *
     * For each recovered variable, check if it already exists in the
     * current EEPROM. Only write if missing — this preserves newer data
     * that may already be in the current EEPROM area (e.g. when pages
     * 116-127 of the old EEPROM overlap with the new EEPROM and EE_Init
     * already discovered some valid data there).
     */
    uint32_t written = 0;
    for (uint32_t i = 0; i < var_count; i++)
    {
        uint32_t existing_data;
        EE_Status status = EE_ReadVariable32bits(var_addrs[i], &existing_data);

        if (status != EE_OK)
        {
            /* Variable not found in current EEPROM — write recovered data */
            EE_Status wr_status = EE_WriteVariable32bits(var_addrs[i], var_data[i]);

            if ((wr_status & EE_STATUSMASK_ERROR) == EE_OK)
            {
                written++;

                /* Handle cleanup if needed */
                if ((wr_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP)
                {
                    EE_CleanUp();
                }
            }
        }
        /* else: variable already exists in current EEPROM, keep the
           existing (potentially newer) value — do not overwrite */
    }

    return written;
}

/**
  * @brief  Try to recover encoder calibration data from the old flash page.
  *
  *         Reads the tEncoderConfig struct from the old calibration page (100),
  *         verifies its CRC32, and if valid, migrates it to the current
  *         calibration page (115) using the standard flash write functions.
  *
  * @retval true   Valid calibration found at old location and migrated
  * @retval false  No valid calibration at old location
  */
bool config_recovery_encoder_calib(void)
{
    tEncoderConfig old_config;

    /* Read from old encoder calibration page (page 100) */
    memcpy(&old_config,
           (uint8_t *)(FLASH_BASE + OLD_ENCODER_CALIB_PAGE * FLASH_PAGE_SIZE),
           sizeof(tEncoderConfig));

    /* Verify CRC32 (last 4 bytes of struct are the CRC itself, excluded) */
    uint32_t crc = crc32((uint8_t *)&old_config, sizeof(tEncoderConfig) - 4);

    if (crc != old_config.crc)
    {
        /* CRC mismatch — old page contains no valid calibration data.
           This is expected for new-layout devices where page 100 is
           part of the bootloader area. */
        return false;
    }

    /* Valid calibration found! Migrate to current page.
       Erase current calibration page first, then write. */
    FI_flash_erase_page(ENCODER_CALIB_PAGE);
    FI_flash_write((uint8_t *)(FLASH_BASE + ENCODER_CALIB_PAGE * FLASH_PAGE_SIZE),
                   (uint8_t *)&old_config,
                   sizeof(tEncoderConfig));

    return true;
}
