#include "eeprom_emulation.h"

// define constants
__attribute__ ((section (".constants"), used)) const uint8_t FW_VERSION[] = "1.0";
__attribute__ ((section (".constants"), used)) const uint8_t SERIAL_NUMBER[8] = {'1','2','3','4','5','6','7','8'};
__attribute__ ((section (".constants"), used)) const uint32_t DFU_SIGNATURE = 0x3DC23DC2;

static uint32_t pageSectors[PAGES_NUM] = {PAGE_0_SECTOR, PAGE_1_SECTOR};
static uint32_t pageAddress[PAGES_NUM] = {ADDR_FLASH_SECTOR_10, ADDR_FLASH_SECTOR_11};
static uint16_t varIdList[VAR_NUM] = {JOYSTICK_LEFT_H_AXIS_MIN_KEY, JOYSTICK_LEFT_H_AXIS_MAX_KEY, JOYSTICK_LEFT_H_AXIS_ZERO_KEY,
									  JOYSTICK_LEFT_V_AXIS_MIN_KEY, JOYSTICK_LEFT_V_AXIS_MAX_KEY, JOYSTICK_LEFT_V_AXIS_ZERO_KEY,
									  JOYSTICK_RIGHT_H_AXIS_MIN_KEY, JOYSTICK_RIGHT_H_AXIS_MAX_KEY, JOYSTICK_RIGHT_H_AXIS_ZERO_KEY,
									  JOYSTICK_RIGHT_V_AXIS_MIN_KEY, JOYSTICK_RIGHT_V_AXIS_MAX_KEY, JOYSTICK_RIGHT_V_AXIS_ZERO_KEY};
/* Functions -----------------------------------------------------------------*/
// driver functions
static EepromResult EEPROM_Init(void);
static EepromResult EEPROM_Read(uint16_t varId, uint16_t *varValue);
static EepromResult EEPROM_Write(uint16_t varId, uint16_t varValue);
static EepromResult EEPROM_ResetDfuSignature(void);
static EepromResult EEPROM_GetSerialNumber(uint8_t* sn_buf, uint8_t length);
static EepromResult EEPROM_GetFwVersion(uint8_t* fw_ver_buf, uint8_t length);
// inner functions
static uint32_t FLASH_Read(uint32_t address);
static PageState EEPROM_ReadPageState(PageIdx idx);
static EepromResult EEPROM_SetPageState(PageIdx idx, PageState state);
static EepromResult EEPROM_ClearPage(PageIdx idx);
static EepromResult EEPROM_Format();
static EepromResult EEPROM_GetActivePageIdx(PageIdx *idx);
static EepromResult EEPROM_CopyPageData(PageIdx oldPage, PageIdx newPage);
static EepromResult EEPROM_WriteData(uint32_t address, uint16_t varId, uint16_t varValue);
static EepromResult EEPROM_PageTransfer(PageIdx activePage, uint16_t varId, uint16_t varValue);

EepromDrv eeprom = {EEPROM_Init, EEPROM_Read, EEPROM_Write, EEPROM_ResetDfuSignature, EEPROM_GetSerialNumber, EEPROM_GetFwVersion};
EepromDrv* eeprom_drv = &eeprom;

/**
 * @brief Init EEPROM emulation module
 * @param: None
 * @return: None
 */
EepromResult EEPROM_Init()
{
  EepromResult res = EEPROM_OK;
  PageState pageStates[PAGES_NUM];

  for (uint8_t i = 0; i < PAGES_NUM; i++)
  {
    pageStates[i] = EEPROM_ReadPageState((PageIdx)i);
  }

  if (((pageStates[PAGE_0] == PAGE_CLEARED) && (pageStates[PAGE_1] == PAGE_CLEARED)) ||
      ((pageStates[PAGE_0] == PAGE_ACTIVE) && (pageStates[PAGE_1] == PAGE_ACTIVE)) ||
      ((pageStates[PAGE_0] == PAGE_RECEIVING_DATA) && (pageStates[PAGE_1] == PAGE_RECEIVING_DATA)))
  {
    res = EEPROM_Format();

    if (res != EEPROM_OK)
    {
      return res;
    }

    res = EEPROM_SetPageState(PAGE_0, PAGE_ACTIVE);
  }

  if ((pageStates[PAGE_0] == PAGE_RECEIVING_DATA) && (pageStates[PAGE_1] == PAGE_CLEARED))
  {
    res = EEPROM_SetPageState(PAGE_0, PAGE_ACTIVE);
  }

  if ((pageStates[PAGE_0] == PAGE_CLEARED) && (pageStates[PAGE_1] == PAGE_RECEIVING_DATA))
  {
    res = EEPROM_SetPageState(PAGE_1, PAGE_ACTIVE);
  }

  if ((pageStates[PAGE_0] == PAGE_RECEIVING_DATA) && (pageStates[PAGE_1] == PAGE_ACTIVE))
  {
    res = EEPROM_CopyPageData(PAGE_1, PAGE_0);
  }

  if ((pageStates[PAGE_0] == PAGE_ACTIVE) && (pageStates[PAGE_1] == PAGE_RECEIVING_DATA))
  {
    res = EEPROM_CopyPageData(PAGE_0, PAGE_1);
  }

  return res;
}

/**
 * @brief Read value from emulated EEPROM
 * @param: varId - EEPROM variable key (defined in "varIdList")
 * @param: varValue - variable value read from emulated EEPROM
 * @return: EEPROM operation result: OK or ERROR
 */
EepromResult EEPROM_Read(uint16_t varId, uint16_t *varValue)
{
  EepromResult res = EEPROM_OK;

  PageIdx activePage = PAGE_0;
  res = EEPROM_GetActivePageIdx(&activePage);

  if (res != EEPROM_OK)
  {
    return res;
  }

  uint32_t startAddr = pageAddress[activePage] + PAGE_DATA_OFFSET;
  uint32_t endAddr = pageAddress[activePage] + PAGE_SIZE - PAGE_DATA_SIZE;
  uint32_t addr = startAddr;

  uint8_t dataFound = 0;

  while (addr <= endAddr)
  {
    uint32_t temp = FLASH_Read(addr);
    uint16_t idData = temp>>16;
    if (idData == varId)
    {
      dataFound++; // increase found instances
      *varValue = (uint16_t)(temp & 0xFFFF);
      addr += PAGE_DATA_SIZE;
    }
    else if(temp == 0xFFFFFFFF)
    {
    	break; // we've reached free space of page, stop cycle
    }
    else
    {
    	addr += PAGE_DATA_SIZE;
    }
  }

  if (dataFound == 0)
  {
    res = EEPROM_ERROR;
  }

  return res;
}

/**
 * @brief Write value into emulated EEPROM
 * @param: varId - EEPROM variable key (defined in "varIdList")
 * @param: varValue - variable value
 * @return: EEPROM operation result: OK or ERROR
 */
EepromResult EEPROM_Write(uint16_t varId, uint16_t varValue)
{
  EepromResult res = EEPROM_OK;

  uint8_t validId = 0;
  for (uint16_t curVar = 0; curVar < VAR_NUM; curVar++)
  {
    if (varIdList[curVar] == varId)
    {
      validId = 1;
      break;
    }
  }

  if (validId == 0)
  {
    res = EEPROM_ERROR;
    return res;
  }

  PageIdx activePage = PAGE_0;
  res = EEPROM_GetActivePageIdx(&activePage);

  if (res != EEPROM_OK)
  {
    return res;
  }

  uint32_t startAddr = pageAddress[activePage] + PAGE_DATA_OFFSET;
  uint32_t endAddr = pageAddress[activePage] + PAGE_SIZE - PAGE_DATA_SIZE;
  uint32_t addr = startAddr;

  uint8_t freeSpaceFound = 0;

  while (addr <= endAddr)
  {
    uint32_t temp = FLASH_Read(addr);
    if (temp == 0xFFFFFFFF)
    {
      freeSpaceFound = 1;
      break;
    }
    else
    {
      addr += PAGE_DATA_SIZE;
    }
  }

  if (freeSpaceFound == 1)
  {
    res = EEPROM_WriteData(addr, varId, varValue);
  }
  else
  {
    res = EEPROM_PageTransfer(activePage, varId, varValue);
  }

  return res;
}

/**
 * @brief Reset DFU signature for starting DFU bootloader after system rest
 * @param: None
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_ResetDfuSignature(void)
{
	EepromResult res = EEPROM_OK;
	FLASH_EraseInitTypeDef erase;
	HAL_StatusTypeDef flashRes = HAL_OK;
	uint32_t pageError = 0;
	uint32_t sector_data[4] = {0};

	// store serial number and firmware version to temp buffer
	for(uint8_t i = 0; i < 4; i++)
	{
		sector_data[i] = *(__IO uint32_t*)(SERIAL_NUMBER_ADDRESS + 4*i);
	}

	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.Banks = FLASH_BANK_1;
	erase.Sector = CONSTANTS_SECTOR;
	erase.NbSectors = 1;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	flashRes = HAL_FLASHEx_Erase(&erase, &pageError);
	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	if (flashRes != HAL_OK)
	{
		res = EEPROM_ERROR;
	}
	else
	{
		// restore serial number and firmware version into flash memory
		for(uint8_t i = 0; i < 4; i++)
		{
			flashRes = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(SERIAL_NUMBER_ADDRESS+4*i), sector_data[i]);
			if(flashRes != HAL_OK)
			{
				res = EEPROM_ERROR;
				break;
			}
		}
	}

	HAL_FLASH_Lock();

	return res;
}

/**
 * @brief Read device serial number
 * @param: sn_buf - data buffer for serial number
 * @param: length - data buffer length
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_GetSerialNumber(uint8_t* sn_buf, uint8_t length)
{
	for(uint8_t i = 0; i < length; i++)
	{
		sn_buf[i] = *(__IO uint8_t*)(SERIAL_NUMBER_ADDRESS+i);
	}

	return EEPROM_OK;
}

/**
 * @brief Read firmware version
 * @param: fw_ver_buf - data buffer for firmware version
 * @param: length - data buffer length
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_GetFwVersion(uint8_t* fw_ver_buf, uint8_t length)
{
	for(uint8_t i = 0; i < length; i++)
	{
		fw_ver_buf[i] = *(__IO uint8_t*)(FW_VERSION_ADDRESS+i);
		// if we've reached null-symbol, stop read
		if(fw_ver_buf[i] == '\0') break;
	}

	return EEPROM_OK;
}

/**
 * @brief Inner function for write data into emulated EEPROM
 * @param: address - address in flash memory, where data will be write
 * @param: varId - EEPROM variable key (defined in "varIdList")
 * @param: varValue - variable value
 * @return: EEPROM operation result: OK or ERROR
 */
EepromResult EEPROM_WriteData(uint32_t address, uint16_t varId, uint16_t varValue)
{
  EepromResult res = EEPROM_OK;
  HAL_StatusTypeDef flashRes = HAL_OK;
  uint32_t fullData = ((uint32_t)(varId << 16)) | (uint32_t)varValue;

  HAL_FLASH_Unlock();
  flashRes = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, fullData);
  HAL_FLASH_Lock();

  if (flashRes != HAL_OK)
  {
    res = EEPROM_ERROR;
  }

  return res;
}

/**
 * @brief Inner function for copying actual variables data into next flash memory page
 * @param: activePage - active flash memory page index
 * @param: varId - EEPROM variable key (defined in "varIdList")
 * @param: varValue - variable value
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_PageTransfer(PageIdx activePage, uint16_t varId, uint16_t varValue)
{
  EepromResult res = EEPROM_OK;
  PageIdx oldPage, newPage;

  if (activePage == PAGE_0)
  {
    oldPage = PAGE_0;
    newPage = PAGE_1;
  }
  else
  {
    if (activePage == PAGE_1)
    {
      oldPage = PAGE_1;
      newPage = PAGE_0;
    }
  }

  res = EEPROM_SetPageState(newPage, PAGE_RECEIVING_DATA);

  if (res != EEPROM_OK)
  {
    return res;
  }

  uint32_t curAddr = pageAddress[newPage] + PAGE_DATA_OFFSET;
  res = EEPROM_WriteData(curAddr, varId, varValue);

  if (res != EEPROM_OK)
  {
    return res;
  }

  curAddr += PAGE_DATA_SIZE;

  for (uint16_t curVar = 0; curVar < VAR_NUM; curVar++)
  {
    if (varIdList[curVar] != varId)
    {
      uint16_t curVarValue = 0;
      res = EEPROM_Read(varIdList[curVar], &curVarValue);

      if (res == EEPROM_OK)
      {
        res = EEPROM_WriteData(curAddr, varIdList[curVar], curVarValue);

        if (res != EEPROM_OK)
        {
          return res;
        }

        curAddr += PAGE_DATA_SIZE;
      }
    }
  }

  res = EEPROM_ClearPage(oldPage);

  if (res != EEPROM_OK)
  {
    return res;
  }

  res = EEPROM_SetPageState(newPage, PAGE_ACTIVE);

  return res;
}

/**
 * @brief Inner function for copying one flash page data into other page
 * @param: oldPage - source flash memory page index
 * @param: newPage - destination flash memory page index
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_CopyPageData(PageIdx oldPage, PageIdx newPage)
{
  EepromResult res = EEPROM_OK;

  uint32_t curAddr = pageAddress[newPage] + PAGE_DATA_OFFSET;

  for (uint32_t curVar = 0; curVar < VAR_NUM; curVar++)
  {
    uint16_t curVarValue = 0;
    res = EEPROM_Read(varIdList[curVar], &curVarValue);

    if (res == EEPROM_OK)
    {
      res = EEPROM_WriteData(curAddr, varIdList[curVar], curVarValue);

      if (res != EEPROM_OK)
      {
        return res;
      }

      curAddr += PAGE_DATA_SIZE;
    }
  }

  res = EEPROM_SetPageState(newPage, PAGE_ACTIVE);

  if (res != EEPROM_OK)
  {
    return res;
  }

  res = EEPROM_ClearPage(oldPage);

  return res;
}

/**
 * @brief Read data from flash memory
 * @param: address - target memory address
 * @return: taget memory address data
 */
static uint32_t FLASH_Read(uint32_t address)
{
  return (*(__IO uint32_t*)address);
}

/**
 * @brief Inner function for reading flash memory page state
 * @param: idx - flash memory page index
 * @return: page state
 */
static PageState EEPROM_ReadPageState(PageIdx idx)
{
  PageState pageState;
  pageState = (PageState)FLASH_Read(pageAddress[idx]);
  return pageState;
}

/**
 * @brief Inner function for setting flash memory page state
 * @param: idx - flash memory page index
 * @param: state - required page state
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_SetPageState(PageIdx idx, PageState state)
{
  EepromResult res = EEPROM_OK;
  HAL_StatusTypeDef flashRes = HAL_OK;

  HAL_FLASH_Unlock();
  flashRes = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAddress[idx], state);
  HAL_FLASH_Lock();

  if (flashRes != HAL_OK)
  {
    res = EEPROM_ERROR;
  }

  return res;
}

/**
 * @brief Inner function for clear flash memory page
 * @param: idx - flash memory page index
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_ClearPage(PageIdx idx)
{
  EepromResult res = EEPROM_OK;

  FLASH_EraseInitTypeDef erase;

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_1;
  erase.Sector = pageSectors[idx];
  erase.NbSectors = 1;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_StatusTypeDef flashRes = HAL_OK;
  uint32_t pageError = 0;

  HAL_FLASH_Unlock();
  flashRes = HAL_FLASHEx_Erase(&erase, &pageError);
  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
  HAL_FLASH_Lock();

  if (flashRes != HAL_OK)
  {
    res = EEPROM_ERROR;
    return res;
  }

  res = EEPROM_SetPageState(idx, PAGE_CLEARED);

  return res;
}

/**
 * @brief Emulated EEPROM format
 * @param: None
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_Format()
{
  EepromResult res = EEPROM_OK;

  for (uint8_t i = 0; i < PAGES_NUM; i++)
  {
    res = EEPROM_ClearPage((PageIdx)i);

    if (res != EEPROM_OK)
    {
      return res;
    }
  }

  return res;
}

/**
 * @brief Get active flash memory page index
 * @param: idx - active flash memory page index value
 * @return: EEPROM operation result: OK or ERROR
 */
static EepromResult EEPROM_GetActivePageIdx(PageIdx *idx)
{
  EepromResult res = EEPROM_OK;
  PageState pageStates[PAGES_NUM];

  for (uint8_t i = 0; i < PAGES_NUM; i++)
  {
    pageStates[i] = EEPROM_ReadPageState((PageIdx)i);
  }

  if(pageStates[PAGE_0] == PAGE_CLEARED && pageStates[PAGE_1] == PAGE_CLEARED)
  {
	  *idx = PAGE_0;
	  return res;
  }

  if ((pageStates[PAGE_0] == PAGE_ACTIVE) && (pageStates[PAGE_1] != PAGE_ACTIVE))
  {
    *idx = PAGE_0;
  }
  else
  {
    if ((pageStates[PAGE_1] == PAGE_ACTIVE) && (pageStates[PAGE_0] != PAGE_ACTIVE))
    {
      *idx = PAGE_1;
    }
    else
    {
      res = EEPROM_ERROR;
    }
  }

  return res;
}

