/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "stm32mp1xx_hal_sd.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

static volatile DSTATUS stat = STA_NOINIT;
SD_HandleTypeDef hsdmmc1 = {0};

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	int result;

	stat = STA_NOINIT;
	if(HAL_SD_GetCardState(&hsdmmc1) == HAL_SD_CARD_TRANSFER) {
		stat = RES_OK;
	}
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	stat = STA_NOINIT;

	HAL_SD_CardCIDTypedef pCID;
	HAL_SD_CardCSDTypedef pCSD;

	hsdmmc1.Instance = SDMMC1;
	HAL_SD_DeInit(&hsdmmc1);

	hsdmmc1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsdmmc1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsdmmc1.Init.BusWide = SDMMC_BUS_WIDE_4B;
	hsdmmc1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	hsdmmc1.Init.ClockDiv = 2;

	if(HAL_SD_Init(&hsdmmc1) != HAL_OK) {
		return stat;
	}

	HAL_SD_GetCardCID(&hsdmmc1, &pCID);
	HAL_SD_GetCardCSD(&hsdmmc1, &pCSD);

/*
	if(MP1_SDMMC_Init() == HAL_OK) {
		stat = disk_status(pdrv);
	}
*/	return disk_status(pdrv);
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res = RES_ERROR;

	if(HAL_SD_ReadBlocks(&hsdmmc1, (uint8_t *)buff, sector, count, 100) == HAL_OK) {
		return RES_OK;
	}

	return res;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res = RES_ERROR;

	if(HAL_SD_WriteBlocks(&hsdmmc1, (uint8_t *)buff, sector, count, 100) == HAL_OK) {
		res = RES_OK;
	}

	return res;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res = RES_ERROR;
	int result;

	  if (stat & STA_NOINIT) return RES_NOTRDY;

	  switch (cmd)
	  {
	  /* Make sure that no pending write process */
	  case CTRL_SYNC :
	    res = RES_OK;
	    break;

	  /* Get number of sectors on the disk (DWORD) */
	  case GET_SECTOR_COUNT :
	    *(DWORD*)buff = hsdmmc1.SdCard.LogBlockNbr;;
	    res = RES_OK;
	    break;

	  /* Get R/W sector size (WORD) */
	  case GET_SECTOR_SIZE :
	    *(WORD*)buff = hsdmmc1.SdCard.LogBlockSize;
	    res = RES_OK;
	    break;

	  /* Get erase block size in unit of sector (DWORD) */
	  case GET_BLOCK_SIZE :
	    *(DWORD*)buff = hsdmmc1.SdCard.LogBlockSize / 512;
	  res = RES_OK;
	    break;

	  default:
	    res = RES_PARERR;
	  }

	  return res;
}

