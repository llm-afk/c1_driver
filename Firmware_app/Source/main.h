/*
    Copyright 2021 codenocold codenocold@qq.com
    Address : https://github.com/codenocold/dgm
    This file is part of the dgm firmware.
    The dgm firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    The dgm firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32c10x.h"
#include <stdbool.h>
#include <stdlib.h>

extern volatile uint32_t SystickCount;

// LED ACT
#define LED_ACT_SET()   GPIO_BOP(GPIOB) = (uint32_t) GPIO_PIN_12;
#define LED_ACT_RESET() GPIO_BC(GPIOB) = (uint32_t) GPIO_PIN_12;
#define LED_ACT_GET()   (GPIO_OCTL(GPIOB) & (GPIO_PIN_12))
static inline void LED_ACT_TOGGLE(void)
{
    if (LED_ACT_GET()) {
        LED_ACT_RESET();
    } else {
        LED_ACT_SET();
    }
}

// brake
#define BRAKE_SET()   GPIO_BOP(GPIOB) = (uint32_t) GPIO_PIN_2;
#define BRAKE_RESET() GPIO_BC(GPIOB) = (uint32_t) GPIO_PIN_2;

// ENC SPI0 NCS
#define ENC_NCS_SET()       GPIO_BOP(GPIOA) = (uint32_t) GPIO_PIN_4;
#define ENC_NCS_RESET()     GPIO_BC(GPIOA) = (uint32_t) GPIO_PIN_4;

// EX SPI2 NCS
#define EX_NCS_SET()       GPIO_BOP(GPIOB) = (uint32_t) GPIO_PIN_6;
#define EX_NCS_RESET()     GPIO_BC(GPIOB) = (uint32_t) GPIO_PIN_6;

/* FLASH MAP ---------------------------------------------*/
#define FLASH_PAGE_SIZE				((unsigned int)0x400U)

#define APP_MAIN_ADDR        ((uint32_t) (0x8000000 + 0 * FLASH_PAGE_SIZE))  // Page 0
#define APP_BACK_ADDR        ((uint32_t) (0x8000000 + 45 * FLASH_PAGE_SIZE)) // Page 45
#define APP_MAX_SIZE         ((uint32_t) (45 * FLASH_PAGE_SIZE))             // 45KB

#define BOOTLOADER_ADDR      ((uint32_t) (0x8000000 + 90 * FLASH_PAGE_SIZE)) // Page 90
#define BOOTLOADER_MAX_SIZE  ((uint32_t) (25 * FLASH_PAGE_SIZE))             // 25KB

// 1 KB
#define ENCODER_CALIB_PAGE			(115)
#define ENCODER_CALIB_PAGE_COUNT  	(1)

// 12 KB
#define EEPROM_PAGE					(116)
#define EEPROM_PAGE_COUNT  			(12)

/* Exported functions prototypes ---------------------------------------------*/
static inline void watch_dog_feed(void) { FWDGT_CTL = FWDGT_KEY_RELOAD; }

void delay_ms(const uint16_t ms);
static inline uint32_t get_tick(void) { return SystickCount == 0 ? (SystickCount+1) : SystickCount; }
static inline uint32_t get_ms_since(uint32_t tick) { return (uint32_t)(SystickCount - tick); }

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
