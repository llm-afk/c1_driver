/**
  ******************************************************************************
  * @file    config_recovery.h
  * @brief   Configuration recovery from old flash layout to new layout.
  *
  *          When OTA updates from old layout firmware (EEPROM at page 101,
  *          encoder calib at page 100) to new layout firmware (EEPROM at page
  *          116, encoder calib at page 115), the configuration data stored in
  *          EEPROM and encoder calibration would be lost because the new
  *          firmware looks at different flash pages.
  *
  *          This module scans the old flash locations for valid configuration
  *          data and migrates it to the new locations, ensuring zero-config-loss
  *          OTA from any firmware version.
  ******************************************************************************
  */

#ifndef __CONFIG_RECOVERY_H
#define __CONFIG_RECOVERY_H

#include <stdint.h>
#include <stdbool.h>

/* Old layout flash page addresses (before commit 21e44d8) */
#define OLD_EEPROM_START_PAGE       101U    /* Old EEPROM start page */
#define OLD_EEPROM_PAGE_COUNT       27U     /* Old EEPROM reserved pages (101-127) */
#define OLD_ENCODER_CALIB_PAGE      100U    /* Old encoder calibration page */

/**
  * @brief  Scan old EEPROM flash pages for valid configuration data and
  *         migrate any missing variables to the current EEPROM area.
  *
  *         This function should be called AFTER EE_Init() has initialized
  *         the current EEPROM area. It scans the old EEPROM pages (101-127),
  *         verifies CRC of each data element, and for any variable NOT already
  *         present in the current EEPROM, writes it using the standard
  *         EE_WriteVariable32bits API.
  *
  * @note   Safe to call on devices that never had old-layout firmware —
  *         the old pages will contain bootloader code or erased flash,
  *         which will fail CRC checks and be ignored.
  *
  * @retval Number of variables successfully recovered and migrated
  */
uint32_t config_recovery_from_old_eeprom(void);

/**
  * @brief  Try to recover encoder calibration data from the old flash location.
  *
  *         If the current encoder calibration page (115) contains invalid data,
  *         this function attempts to read valid calibration from the old
  *         location (page 100) and migrates it to the current location.
  *
  * @note   Call this from ENCODER_init() as a fallback when CRC check fails
  *         on the current calibration page.
  *
  * @retval true   Valid calibration was recovered from old location
  * @retval false  No valid calibration found at old location
  */
bool config_recovery_encoder_calib(void);

#endif /* __CONFIG_RECOVERY_H */
