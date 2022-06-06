#include "eeprom_emulation.h"


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

EepromDrv eeprom = {EEPROM_Init, EEPROM_Read, EEPROM_Write};
EepromDrv* eeprom_drv = &eeprom;

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

/******************************************************************************/
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

/******************************************************************************/
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

/******************************************************************************/
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

/******************************************************************************/
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



/******************************************************************************/
static uint32_t FLASH_Read(uint32_t address)
{
  return (*(__IO uint32_t*)address);
}



/******************************************************************************/
static PageState EEPROM_ReadPageState(PageIdx idx)
{
  PageState pageState;
  pageState = (PageState)FLASH_Read(pageAddress[idx]);
  return pageState;
}



/******************************************************************************/
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



/******************************************************************************/
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



/******************************************************************************/
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



/******************************************************************************/
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
