/*
 * EEP_handler.h
 *
 *  Created on: 01.04.2024
 *      Author: Rudolf Herzog
 */

#ifndef EEP_HANDLER_H_
#define EEP_HANDLER_H_

#include "Arduino.h"
//#include <SPI.h>
#include <EEPROM_SPI_WE.h>
#include "CRC8.h"
#include <Streaming.h>      // http://arduiniana.org/libraries/streaming/

class EEP_handler {
public:
    // EEPROM address for CAPACITY
    #define EEPADR_CAPACITY   4000
    #define MAX_DATA_BLOCK_SIZE 2
    // eeprom buffer struct data,timestamp,crc
    uint8_t *bufE2pRd_CV;
    uint8_t bufE2pWr_CV[MAX_DATA_BLOCK_SIZE + 2];

	EEP_handler();
	virtual ~EEP_handler();
	void Init(void);
  uint8_t EepRead(uint16_t eepAdr, uint16_t *Data);
  void EepWrite(uint16_t eepAdr, uint16_t *Data, uint8_t numCop);
  uint8_t EepFindNewestCopy(uint16_t eepAdr, uint8_t szData_CV, uint8_t numCop);

protected:
  EEPROM_SPI_WE *_spi_we;
  //EEPROM_SPI_WE myEEPROM_SPI_WE;

};

#endif /*  */
/* $Log: EEP_handler.h $
 * 01-04-24: erste Version
 * */
