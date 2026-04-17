/*
 * mx25_flash.h
 *
 *  Created on: Apr 6, 2026
 *      Author: spska
 */

#ifndef INC_MX25_FLASH_H_
#define INC_MX25_FLASH_H_

#include "stm32l4xx_hal.h"

uint32_t MX25_READID_QSPI(void);
void MX25_WriteEnable(void);
uint8_t MX25_ReadStatus(void);
void MX25_WaitForBusy(void);
void MX25_PageProgram(uint32_t Address, uint8_t *pData, uint16_t Size);
void MX25_SectorErase(uint32_t Address);
void MX25_ReadData(uint32_t Address, uint8_t *pData, uint16_t Size);

#endif /* INC_MX25_FLASH_H_ */
