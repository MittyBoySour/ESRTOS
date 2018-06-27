/*
 * Flash.c
 *
 *  Created on: 27 Jun 2018
 *      Author: 11238639
 */


#include "Flash.h"
#include "MK70F12.h"
#include "types.h"

#define FLASH_ERASE_SECTOR_COMMAND 0x09
#define FLASH_WRITE_PHRASE_COMMAND 0x07


typedef struct {
  uint8_t FCCOB3;
  uint8_t FCCOB2;
  uint8_t FCCOB1;
  uint8_t FCCOB0;
  uint8_t FCCOB7;
  uint8_t FCCOB6;
  uint8_t FCCOB5;
  uint8_t FCCOB4;
  uint8_t FCCOBB;
  uint8_t FCCOBA;
  uint8_t FCCOB9;
  uint8_t FCCOB8;
} TFCCOB;

// We need to keep track of which of the 8 bytes are currently in use
// to do this set up a unit8_t of all 1's and create masks to flip those
// 1's to zero to mark a memory address as full

/*! @brief Enables the Flash module.
 *
 *  @return bool - TRUE if the Flash was setup successfully.
 */
bool Flash_Init(void) {
  return true;
}


/*! @brief Launches constructed command object written in flash memory and awaits FSTAT register to be cleared before returning success of command.
 *
 *  @param commomCommandObject The command object written to all the FCCOB registers.
 *  @return bool - TRUE if Flash command has no errors in FSTAT register, FALSE if error is returned from ANDing masks.
 *  @note Assumes Flash has been initialized.
 */
static bool LaunchCommand(TFCCOB * commomCommandObject) {

  bool hasError = false;
  FTFE_FSTAT = FTFE_FSTAT_ACCERR_MASK;
  FTFE_FSTAT = FTFE_FSTAT_FPVIOL_MASK;
  FTFE_FSTAT = FTFE_FSTAT_RDCOLERR_MASK;

  FTFE_FCCOB0 =  (*commomCommandObject).FCCOB0;
  FTFE_FCCOB1 =  (*commomCommandObject).FCCOB1;
  FTFE_FCCOB2 =  (*commomCommandObject).FCCOB2;
  FTFE_FCCOB3 =  (*commomCommandObject).FCCOB3;
  FTFE_FCCOB4 =  (*commomCommandObject).FCCOB4;
  FTFE_FCCOB5 =  (*commomCommandObject).FCCOB5;
  FTFE_FCCOB6 =  (*commomCommandObject).FCCOB6;
  FTFE_FCCOB7 =  (*commomCommandObject).FCCOB7;
  FTFE_FCCOB8 =  (*commomCommandObject).FCCOB8;
  FTFE_FCCOB9 =  (*commomCommandObject).FCCOB9;
  FTFE_FCCOBA =  (*commomCommandObject).FCCOBA;
  FTFE_FCCOBB =  (*commomCommandObject).FCCOBB;

  FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK;

  while (~FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK) {}


  hasError = (FTFE_FSTAT & FTFE_FSTAT_ACCERR_MASK) == FTFE_FSTAT_ACCERR_MASK;

  hasError = (FTFE_FSTAT & FTFE_FSTAT_FPVIOL_MASK) == FTFE_FSTAT_FPVIOL_MASK;

  hasError = (FTFE_FSTAT & FTFE_FSTAT_RDCOLERR_MASK) == FTFE_FSTAT_RDCOLERR_MASK;

  return !hasError;
}

/*! @brief Writes modified phrase to entire Flash sector by constructing command object representation in big endian notation.
 *
 *  @param address The address of start of flash sector memory to be overwritten.
 *  @return bool - TRUE if Flash was overwritten successfully through LaunchCommand returning no errors, FALSE if error in LaunchCommand.
 *  @note Assumes Flash has been initialized.
 */
static bool WritePhrase(const uint32_t address, const uint64union_t phrase) {
  TFCCOB commandObject;
  commandObject.FCCOB0 = FLASH_WRITE_PHRASE_COMMAND;
  commandObject.FCCOB1 = address >> 16;
  commandObject.FCCOB2 = address >> 8;
  commandObject.FCCOB3 = address;

  commandObject.FCCOB4 =  phrase.s.Lo >> 24;
  commandObject.FCCOB5 =  phrase.s.Lo >> 16;
  commandObject.FCCOB6 =  phrase.s.Lo >> 8;
  commandObject.FCCOB7 =  phrase.s.Lo;

  commandObject.FCCOB8 =  phrase.s.Hi >> 24;
  commandObject.FCCOB9 =  phrase.s.Hi >> 16;
  commandObject.FCCOBA =  phrase.s.Hi >> 8;
  commandObject.FCCOBB =  phrase.s.Hi;

  return LaunchCommand(&commandObject);
}

/*! @brief Erases a sector of flash through passing erase command to LaunchCommand.
 *
 *  @param address The address of start of flash sector to be erased.
 *  @return bool - TRUE if Flash was erased successfully through LaunchCommand returning no errors, FALSE if error in LaunchCommand.
 *  @note Assumes Flash has been initialized.
 */
static bool EraseSector(const uint32_t address) {

  TFCCOB commandObject;
  commandObject.FCCOB0 = FLASH_ERASE_SECTOR_COMMAND;
  commandObject.FCCOB1 = address >> 16;
  commandObject.FCCOB2 = address >> 8;
  commandObject.FCCOB3 = address;

  return LaunchCommand(&commandObject);
}

/*! @brief Modifies a sector of flash.
 *
 *  @param address The address of start of flash sector to be modified.
 *  @return bool - TRUE if Flash was modified successfully through WritePhase, FALSE if error in WritePhrase.
 *  @note Assumes Flash has been initialized.
 */
static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase) {
  if (!EraseSector(address))  {
      return false;
  }
  return WritePhrase(address, phrase);
}

/*! @brief Allocates space for a non-volatile variable in the Flash memory.
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in Flash memory.
 *         The pointer will be allocated to a relevant address:
 *         If the variable is a byte, then any address.
 *         If the variable is a half-word, then an even address.
 *         If the variable is a word, then an address divisible by 4.
 *         This allows the resulting variable to be used with the relevant Flash_Write function which assumes a certain memory address.
 *         e.g. a 16-bit variable will be on an even address
 *  @param size The size, in bytes, of the variable that is to be allocated space in the Flash memory. Valid values are 1, 2 and 4.
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 *  @note Assumes Flash has been initialized.
 */

static uint8_t countSetBits(int n)
{
    unsigned int count = 0;
    while (n)
    {
      n &= (n-1) ;
      count++;
    }
    // Thankyou Brian K
    return count;
}

bool Flash_AllocateVar(volatile void** variable, const uint8_t size) {

  static uint8_t fullMask = 0b00000000;

  uint8_t offset = countSetBits(fullMask);

  switch(size) {
    // get a byte's worth of address
    case 1:
      if (offset > 7) {
	  return false;
      }
      fullMask |= 1 << 7 - offset;
      break;
    // get a half word's worth of address
    case 2:
      if (offset > 6) {
	  return false;
      }
      fullMask |= 3 << 6 - offset;
      break;
    case 4:
      // get a wordsworth (the poet) of address;
      if (offset > 4) {
	  return false;
      }
      fullMask |= 15 << 4 - offset;
      break;
     // get the entire phrase bro
    case 8:
      if (offset > 0) {
	  return false;
      }
      fullMask |= 255;
      break;
    default:
      break;
  }

  *variable = FLASH_DATA_START + (offset);


  return true;

}

/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write32(volatile uint32_t* const address, const uint32_t data) {
  uint8_t offset = (uint8_t)(address - FLASH_DATA_START);
  uint64union_t phrase;
  uint32_t newAddress;

   if (offset > 3) {
       phrase.s.Lo = _FW(address - 1);
       phrase.s.Hi = data;
       newAddress = (uint32_t) (address - 1);
   } else {
       phrase.s.Lo = data;
       phrase.s.Hi = _FW(address + 1);
       newAddress = (uint32_t) (address);
   }

  return ModifyPhrase(newAddress, phrase);
}

/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write16(volatile uint16_t* const address, const uint16_t data) {

  uint8_t offset = (uint8_t)(address - FLASH_DATA_START);
  uint32union_t word;
  uint32_t newAddress;

  if (offset % 4 == 0) {
      word.s.Lo = data;
      word.s.Hi = _FH(address + 1);
      newAddress = address;
  } else if (offset % 2 == 0) {
      word.s.Lo = _FH(address - 1);
      word.s.Hi = data;
      newAddress = address - 1;
  } else {
      // cant happen;
      return false;
  }

  return Flash_Write32((uint32_t *)newAddress, word.l);

}

/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write8(volatile uint8_t* const address, const uint8_t data) {
   uint8_t offset = (uint8_t)(address - FLASH_DATA_START);
   uint16union_t halfWord;
   uint32_t newAddress;

   if (offset % 2 == 0) {
       halfWord.s.Lo = data;
       halfWord.s.Hi = _FB(address + 1);
       newAddress = address;
   } else {
       halfWord.s.Lo = _FB(address - 1);
       halfWord.s.Hi = data;
       newAddress =  (address - 1);
   }

  return Flash_Write16((uint16_t *) newAddress, halfWord.l);
}

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void) {
  return EraseSector(FLASH_DATA_START);
}
