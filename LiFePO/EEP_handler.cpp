/*
 * EEP_handler.cpp
 *
 *  Created on: 01.04.2024
 *      Author: Rudolf Herzog
 */

#include "EEP_handler.h"

const int csPin = 10; // Chip select pin
EEPROM_SPI_WE myEEP = EEPROM_SPI_WE(csPin);
uint8_t bufE2pRd[MAX_DATA_BLOCK_SIZE + 2] = "";
uint8_t bufE2pWr[MAX_DATA_BLOCK_SIZE + 2] = "";
uint8_t inxLatestCop = 0;
uint8_t e2p_Status= 0;
 
uint8_t *bufE2pRd_CV = bufE2pRd;

// CRC
CRC8 crc8;

EEP_handler::EEP_handler() {
	// TODO Auto-generated constructor stub

}

EEP_handler::~EEP_handler() {
	// TODO Auto-generated destructor stub
}

void EEP_handler::Init(void) {

  crc8.begin();

  Serial.print(F("EEPROM "));
  if(myEEP.init()){
    Serial.println(F("connected"));
    myEEP.setPageSize(EEPROM_PAGE_SIZE_32);
    e2p_Status = 1;
  }
  else{
    Serial.println(F("does not respond"));
    e2p_Status = 0;
  }
 }

/***********************************************************

FUNCTION NAME     : EepFindNewestCopy
INPUT PARAMETERS  : hdlNum:  Index to active handle
OUTPUT PARAMETERS : TRUE: newest copy found; FALSE: no valid copy found.
DESCRIPTION       : This function finds the newest copy of the active handle.
                    The data block is present in the verify buffer after finish and the copy number is saved.

***********************************************************/
uint8_t EEP_handler::EepFindNewestCopy(uint16_t eepAdr, uint8_t szData_CV, uint8_t numCop)
{
   /* declare local variables */
   uint8_t copyNum;
   uint8_t timeStamp;
   uint8_t bytesRead;
   uint8_t checksum;

   /* Start with copy number 0 */
   copyNum = 0;
   bytesRead = szData_CV + 2; // add 1 byte for timestamp 1 byte for CRC
   do
   {
      /* Find first valid copy */
      if (copyNum <= numCop)
      {
         /* Read one copy into verify buffer */
         Serial << F("EEP ") << copyNum;

         myEEP.get(eepAdr+(copyNum*bytesRead), bufE2pRd);

         Serial << F(": ") << bufE2pRd[0];
         Serial << F(" ") << bufE2pRd[1];
         Serial << F(" ") << bufE2pRd[2];
         Serial << F(" ") << bufE2pRd[3];

         /* Select next copy */
         copyNum++;
         /* ... do verification over data block,
            time stamp and checksum;
            result must be zero */
         checksum = crc8.get_crc8(bufE2pRd, bytesRead);
         Serial << F("§ ") << checksum;
      }
      else
      {
         /* No valid copy found; assume last copy to be the newest one */
         inxLatestCop = copyNum - 1;
         /* Tell that search was unsuccessful */
         return(false);
      }
   }
   /* If data block is with checksum, repeat until the first copy with valid checksum is found */
   while (checksum != 0);
   /* Initialize newest copy */
   inxLatestCop = copyNum - 1;
   /* Initialize timestamp */
   timeStamp = bufE2pRd[szData_CV];
   /* Are there still copies to check? */
   while (copyNum <= numCop)
   {
      /* Read one copy into verify buffer */
      myEEP.get(eepAdr+(copyNum*bytesRead), bufE2pRd);
      Serial << F("; ") << copyNum;
         Serial << F(": ") << bufE2pRd[0];
         Serial << F(" ") << bufE2pRd[1];
         Serial << F(" ") << bufE2pRd[2];
         Serial << F(" ") << bufE2pRd[3];

      timeStamp++;
      /* Select next copy */
      copyNum++;
      /* ... do verification over data block, time stamp and checksum */
      checksum = crc8.get_crc8(bufE2pRd, bytesRead);
         Serial << F("# ") << checksum;

      if (checksum == 0)
      {
         /* Is this copy newer than the previous */
         if (timeStamp == bufE2pRd[szData_CV])
         {
            /* This copy was newer, proceed search */
            inxLatestCop = copyNum - 1;
         }
         else
         {
            /* No, assume that the previous copy was the newest one*/
            break;
         }
      }
      else
      {
         /* Invalid copy; Try the next one. */
      }
   }
   /* Read newest copy into verify buffer and check if reading was successful. */
   Serial << F("L ") << inxLatestCop;
   myEEP.get(eepAdr+(inxLatestCop*bytesRead), bufE2pRd);
   return(true);
}

/***********************************************************

FUNCTION NAME     : EepRead
INPUT PARAMETERS  : buf:  Ptr to buffer
                    uint8_t szData_CV, 
OUTPUT PARAMETERS : TRUE: newest copy found; FALSE: no valid copy found.
DESCRIPTION       : This function finds the newest copy of the data in the EEP
                    and copies the data to the destination.

***********************************************************/
uint8_t EEP_handler::EepRead(uint16_t eepAdr, uint16_t *Data)
{
  uint8_t i;
// EEP lesen
  if (EepFindNewestCopy(eepAdr, 2, 3))
  {
   /* copy data from read buffer */
   *Data = (uint16_t)bufE2pRd[1] << 8;
   *Data += bufE2pRd[0];
   ;
    return(true);
  }
  else
  {
    // don't copy data
    return (false);
  }
}

/***********************************************************

FUNCTION NAME     : EepWrite
INPUT PARAMETERS  : hdlNum:  Index to active handle
OUTPUT PARAMETERS : none
DESCRIPTION       : This function prepares writing to the EEPROM.
                    1. If there are copies the newest copy is searched
                    2. The copy following the newest one is selected for overwriting
                    3. Timestamp and checksum are appended to the write data if necessary
                    4. Write protection is cleared if it was set previously

***********************************************************/
void EEP_handler::EepWrite(uint16_t eepAdr, uint16_t *Data, uint8_t numCop)
{
  /* declare local variables */
  uint8_t timeStamp;
  uint8_t checksum;
  uint8_t i;
  /* Write Command preparation: number of bytes to write */
  uint8_t szData_CV = 2;
  uint8_t bytes2Wr_CV = szData_CV;
  /* Default copy number to write to */
  uint8_t inxCop_CV = 0;

   /* copy data to write buffer */
   bufE2pWr[0] = *Data & 0xFF;
   bufE2pWr[1] = *Data >> 8;
   /* Check if copies */
   if (numCop > 0)
   {
      /* Write Command preparation: find the newest copy => the next one will be overwritten */
      if (EepFindNewestCopy(eepAdr, szData_CV, 3) == true)
      {
         /* Newest copy found */
         /* The copy after the newest one shall be overwritten */
         inxCop_CV = inxLatestCop + 1;
         /* Timestamp of the copy to write is previous timestamp+1 */
         timeStamp = bufE2pRd[szData_CV] + 1;

      }
      else
      {
         /* No valid copy found;
            write to copy number 0 */
         /* Start with timestamp 0 */
         timeStamp = 0;
      }
      /* Check if copy number is within valid range */
      if (inxCop_CV > numCop)
      {
         /* If not, wrap around */
         inxCop_CV = 0;
      }
      
      /* Append timestamp to data block */
      bufE2pWr[bytes2Wr_CV] = timeStamp;
      /* Increment bytes2Wr_CV by timeStamp size */
      bytes2Wr_CV++;
   }
   else
   {
      /* Initialize newest copy */
      inxLatestCop = 0;
   }
   /* Write Command preparation: insert checksum */
   /* When the checksum bit is set, a CRC checksum over data block including timestamp is added */
      /* Calculate checksum */
      checksum = crc8.get_crc8(bufE2pWr, bytes2Wr_CV);
      /* Append checksum to data block */
      bufE2pWr[bytes2Wr_CV] = checksum;
      bytes2Wr_CV++;
   myEEP.put(eepAdr+(inxCop_CV*bytes2Wr_CV), bufE2pWr);
   Serial << F("EEPwritten ") << (eepAdr+(inxCop_CV*bytes2Wr_CV)) << endl;
   }
/* $Log: EEP_handler.cpp $
  * */
