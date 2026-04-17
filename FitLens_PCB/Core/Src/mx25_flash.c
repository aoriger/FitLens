/*
 * mx25_flash.c
 *
 *  Created on: Apr 6, 2026
 *      Author: spska
 */
#include "mx25_flash.h"
#include "main.h"
extern QSPI_HandleTypeDef hqspi;


uint32_t MX25_READID_QSPI(void){
	QSPI_CommandTypeDef s_command = {0};
	uint8_t id[3];

	s_command.Instruction = 0x9F;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.NbData = 3;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;


	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
	{
		HAL_QSPI_Receive(&hqspi, id, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	}


	return (id[0] << 16) | (id[1] << 8) | id[2];
}

void MX25_WriteEnable(void){
	QSPI_CommandTypeDef s_command = {0};

	s_command.Instruction = 0x06;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_NONE;
	s_command.NbData = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &s_command,HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
		Error_Handler();
	}

}

uint8_t MX25_ReadStatus(void){
	QSPI_CommandTypeDef s_command = {0};
	uint8_t status = 0;

	s_command.Instruction = 0x05;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.NbData = 1;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) {
	        HAL_QSPI_Receive(&hqspi, &status, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	}
	return status;
}

void MX25_WaitForBusy(void) {
    while ((MX25_ReadStatus() & 0x01) == 0x01) {
        HAL_Delay(1);
    }
}

void MX25_PageProgram(uint32_t Address, uint8_t *pData, uint16_t Size){
	QSPI_CommandTypeDef s_command = {0};

	MX25_WriteEnable();
	s_command.Instruction = 0x02;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Address = Address;
	s_command.AddressMode = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.NbData = Size; //256 bytes possible
	s_command.DummyCycles = 0;

	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) {
	    if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
	        Error_Handler();
	    }
	}
	//uses helper to wait for the chip to finish writing
	while ((MX25_ReadStatus() & 0x01) == 0x01);
}

void MX25_SectorErase(uint32_t Address){
	QSPI_CommandTypeDef s_command = {0};

	MX25_WriteEnable();

	s_command.Instruction = 0x20;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Address = Address;
	s_command.AddressMode = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
	s_command.DataMode = QSPI_DATA_NONE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;


	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
	    Error_Handler();
	}

	while ((MX25_ReadStatus() & 0x01) == 0x01);
}

void MX25_ReadData(uint32_t Address, uint8_t *pData, uint16_t Size){
	QSPI_CommandTypeDef s_command = {0};

	s_command.Instruction = 0x03;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Address = Address;
	s_command.AddressMode = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.NbData = Size;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) {
	    if (HAL_QSPI_Receive(&hqspi, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
	        Error_Handler();
	    }
	}
}


