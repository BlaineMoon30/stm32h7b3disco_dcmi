/**
  ******************************************************************************
  * @file    camera.c
  * @author  MCD Application Team 
  * @brief        
  *      
  @verbatim
  ==============================================================================
                     ##### <camera.c> features #####
  ==============================================================================
  [..]
    Sample
    (+) Sample
        (++) Samle

                     ##### How to use this <camera.c> #####
  ==============================================================================
  [..]
    Sample
    (+) Sample
        (++) Samle   

                     ##### <camera.c> Limitations #####
  ==============================================================================
  [..]
    Sample
    (+) Sample
        (++) Samle   

  @endverbatim 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "camera.h"
#include "i2c.h"
#include "dcmi.h"
#include "image_st_logo.h"
#include "sdram.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define CAMERA_VGA_RES_X          640
#define CAMERA_VGA_RES_Y          480
#define CAMERA_480x272_RES_X      480
#define CAMERA_480x272_RES_Y      272

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32H747I_DISCO_CAMERA_Exported_Variables Exported Variables
  * @{
  */
void                *Camera_CompObj = NULL;
DCMI_HandleTypeDef  hcamera_dcmi;
CAMERA_Ctx_t        Camera_Ctx[CAMERA_INSTANCES_NBR];
/**
  * @}
  */

/** @defgroup STM32H747I_DISCO_CAMERA_Private_Variables Private Variables
  * @{
  */
static CAMERA_Drv_t *Camera_Drv = NULL;
static CAMERA_Capabilities_t Camera_Cap;
static uint32_t HSPolarity = DCMI_HSPOLARITY_LOW;
static uint32_t CameraId;
static uint32_t Capture_Start = 0;
/**
  * @}
  */


/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
static int32_t OV9655_Probe(uint32_t Resolution, uint32_t PixelFormat);
static int32_t I2C4_ReadReg(uint16_t DevAddr, uint16_t Reg, uint16_t MemAddSize, uint8_t *pData, uint16_t Length);
static int32_t I2C4_WriteReg(uint16_t DevAddr, uint16_t Reg, uint16_t MemAddSize, uint8_t *pData, uint16_t Length);

/* External functions --------------------------------------------------------*/
int32_t CAMERA_I2C4_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t CAMERA_I2C4_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t CAMERA_GetTick(void);
int32_t CAMERA_HwReset(uint32_t Instance);

/**
  * @brief  Get the capture size in pixels unit.
  * @param  Resolution  the current resolution.
  * @param  PixelFormat Camera pixel format
  * @retval capture size in 32-bit words.
  */
static int32_t GetSize(uint32_t Resolution, uint32_t PixelFormat)
{
  uint32_t size = 0;
  uint32_t pf_div;
  if(PixelFormat == CAMERA_PF_RGB888)
  {
    pf_div = 3; /* each pixel on 3 bytes so 3/4 words */
  }
  else
  {
    pf_div = 2; /* each pixel on 2 bytes so 1/2 words*/
  }
  /* Get capture size */
  switch (Resolution)
  {
  case CAMERA_R160x120:
    size =  ((uint32_t)(160*120)*pf_div)/4U;
    break;
  case CAMERA_R320x240:
    size =  ((uint32_t)(320*240)*pf_div)/4U;
    break;
  case CAMERA_R480x272:
    size =  ((uint32_t)(480*272)*pf_div)/4U;
    break;
  case CAMERA_R640x480:
    size =  ((uint32_t)(640*480)*pf_div)/4U;
    break;
  case CAMERA_R800x480:
    size =  ((uint32_t)(800*480)*pf_div)/4U;
    break;
  default:
    break;
  }

  return (int32_t)size;
}

/**
  * @brief  Initializes the camera.
  * @param  Instance    Camera instance.
  * @param  Resolution  Camera sensor requested resolution (x, y) : standard resolution
  *         naming QQVGA, QVGA, VGA ...
  * @param  PixelFormat Capture pixel format
  * @retval BSP status
  */

int32_t CAMERA_Init(uint32_t Instance, uint32_t Resolution, uint32_t PixelFormat)
{
  int32_t ret = BSP_ERROR_NONE;
  CAMERA_HwReset(0);
  ret = OV9655_Probe(Resolution, PixelFormat);
  //CAMERA_HwReset(0);

  for (uint32_t i = 0; i < 0x10000; i++)
  {
    *(__IO uint32_t*) (CAMERA_FRAME_BUFFER + i) = 0xAA;
    *(__IO uint32_t*) (LCD_FRAME_BUFFER + i) = 0xAA;
  }


  // Test ..
  if (HAL_DCMI_Start_DMA(&hdcmi, CAMERA_MODE_CONTINUOUS, (uint32_t)CAMERA_FRAME_BUFFER, (uint32_t)GetSize(Resolution/*Camera_Ctx[Instance].Resolution*/,PixelFormat /*Camera_Ctx[Instance].PixelFormat*/)) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  return ret;
}

/**
  * @brief  Copy the Captured Picture to the display Frame buffer.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Pointer to destination buffer
  * @param  xsize: Picture X size
  * @param  ysize: Picture Y Size
  * @retval None
  */
static uint32_t   LcdResX    = 480;
static uint32_t   LcdResY    = 272;
#define ARGB8888_BYTE_PER_PIXEL  4
static void DMA2D_ConvertFrameToARGB8888(void *pSrc, void *pDst, uint32_t xsize, uint32_t ysize)
{
  uint32_t xPos, yPos, destination;

  /* Calculate the destination transfer address */
#if 0
  xPos = 0;//(LcdResX  - xsize)/2;
  yPos = 0;//(LcdResY  - ysize)/2;

  destination = (uint32_t)pDst + (yPos * LcdResX + xPos) * ARGB8888_BYTE_PER_PIXEL;
#else
  destination = (uint32_t)pDst;
#endif
  //dma2d_pending_copy = 1;
  /* Starts the DMA2D transfer */
  if(HAL_DMA2D_Start_IT(&hdcmi, (uint32_t)pSrc, destination, xsize, ysize) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  DMA2D Transfer completed callback
 * @param  hdma2d: DMA2D handle.
 * @retval None
 */
static void DMA2D_TransferCompleteCallback(DMA2D_HandleTypeDef *hdma2d)
{
  //dma2d_pending_copy = 0;
  //Display_StartRefresh();
  //SDRAM_LCD_REFRESH();
}

void CAMERA_Capture_Start(uint32_t param)
{
  Capture_Start = param;
}


/**
  * @brief  Line event callback
  * @param  hdcmi  pointer to the DCMI handle
  * @retval None
  */
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdcmi);
  //printf("nn");
  //BSP_CAMERA_LineEventCallback(0);
}

/**
  * @brief  Frame event callback
  * @param  hdcmi pointer to the DCMI handle
  * @retval None
  */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdcmi);
  //HAL_DCMI_Suspend(hdcmi);
  //if(cameraState == CAMERA_STATE_CAPTURE_ONGOING)
  {
    //cameraState = CAMERA_STATE_DISPLAY_ONGOING;
    /* Convert captured frame to ARGB8888 and copy it to LCD FRAME BUFFER */
    //DMA2D_ConvertFrameToARGB8888((uint32_t *)(CAMERA_FRAME_BUFFER), (uint32_t *)(LCD_FRAME_BUFFER), /*CameraResX[0]*/320, /*CameraResY[0]*/240);
  }
  //SDRAM_LCD_REFRESH();
  //HAL_DCMI_Resume(hdcmi);
  //printf("nn");
  //BSP_CAMERA_FrameEventCallback(0);

  if (Capture_Start == 1)
  {
    HAL_DCMI_Suspend(hdcmi);

#if 0
    /* Copy CAMERA_FRAME_BUFFER memory to LCD_FRAME_BUFFER */
    for (uint32_t i = 0; i < (640 * 480) * 2; i++)
    {
      *(__IO uint32_t*)(LCD_FRAME_BUFFER + i) = *(__IO uint32_t*)(CAMERA_FRAME_BUFFER + i);
    }

    HAL_DCMI_Resume(hdcmi);

    CAMERA_Capture_Start(0);
#endif
  }
}

/**
  * @brief  Vsync event callback
  * @param  hdcmi pointer to the DCMI handle
  * @retval None
  */
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdcmi);
  //printf("nn");
  //BSP_CAMERA_VsyncEventCallback(0);
}

/**
  * @brief  Error callback
  * @param  hdcmi pointer to the DCMI handle
  * @retval None
  */
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdcmi);
  //printf("nn");
  //BSP_CAMERA_ErrorCallback(0);
}


/**
  * @brief  Register Bus IOs if component ID is OK
  * @retval error status
  */
static int32_t OV9655_Probe(uint32_t Resolution, uint32_t PixelFormat)
{
  int32_t ret;
  OV9655_IO_t IOCtx;
  static OV9655_Object_t OV9655Obj;

  /* Configure the audio driver */
  IOCtx.Address = CAMERA_OV9655_ADDRESS;
  IOCtx.Init = MX_I2C4_Init;
  //IOCtx.DeInit = BSP_I2C4_DeInit;
  IOCtx.ReadReg = CAMERA_I2C4_ReadReg;
  IOCtx.WriteReg = CAMERA_I2C4_WriteReg;
  IOCtx.GetTick = CAMERA_GetTick;

  if(OV9655_RegisterBusIO (&OV9655Obj, &IOCtx) != OV9655_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if (OV9655_ReadID(&OV9655Obj, &CameraId) != OV9655_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if ((CameraId != OV9655_ID) && (CameraId != OV9655_ID_2))
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    Camera_Drv = (CAMERA_Drv_t*)&OV9655_CAMERA_Driver;
    Camera_CompObj = &OV9655Obj;
    if (Camera_Drv->Init(Camera_CompObj, Resolution, PixelFormat) != OV9655_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if (Camera_Drv->GetCapabilities(Camera_CompObj, &Camera_Cap) != OV9655_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  return ret;
}

/**
  * @brief  Delay function
  * @retval Tick value
  */
int32_t CAMERA_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}


/**
  * @brief  Read a register of the device through BUS
  * @param  DevAddr    Device address on BUS
  * @param  MemAddSize Size of internal memory address
  * @param  Reg        The target register address to read
  * @param  pData      The target register value to be read
  * @param  Length     data length in bytes
  * @retval BSP status
  */
static int32_t I2C4_ReadReg(uint16_t DevAddr, uint16_t Reg, uint16_t MemAddSize, uint8_t *pData, uint16_t Length)
{
  if (HAL_I2C_Mem_Read(&hi2c4, DevAddr, Reg, MemAddSize, pData, Length, 1000) == HAL_OK)
  {
    return BSP_ERROR_NONE;
  }

  return BSP_ERROR_BUS_FAILURE;
}

/**
  * @brief  Read a 8bit register of the device through BUS
  * @param  DevAddr Device address on BUS
  * @param  Reg     The target register address to read
  * @param  pData   Pointer to data buffer
  * @param  Length  Length of the data
  * @retval BSP status
  */
int32_t CAMERA_I2C4_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;
#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif
  if(I2C4_ReadReg(DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length) == 0)
  {
    ret = BSP_ERROR_NONE;
  }
  else
  {
    if( HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif
  return ret;
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr    Device address on Bus.
  * @param  MemAddSize Size of internal memory address
  * @param  Reg        The target register address to write
  * @param  pData      The target register value to be written
  * @param  Length     data length in bytes
  * @retval BSP status
  */
static int32_t I2C4_WriteReg(uint16_t DevAddr, uint16_t Reg, uint16_t MemAddSize, uint8_t *pData, uint16_t Length)
{
  if(HAL_I2C_Mem_Write(&hi2c4, DevAddr, Reg, MemAddSize, pData, Length, 1000) == HAL_OK)
  {
    return BSP_ERROR_NONE;
  }

  return BSP_ERROR_BUS_FAILURE;
}

/**
  * @brief  Write a 8bit value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write
  * @param  pData  The target register value to be written
  * @param  Length buffer size to be written
  * @retval BSP status
  */
int32_t CAMERA_I2C4_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret;
#if defined(BSP_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(BspI2cSemaphore, osWaitForever);
#endif
  if(I2C4_WriteReg(DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length) == 0)
  {
    ret = BSP_ERROR_NONE;
  }
  else
  {
    if( HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
#if defined(BSP_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(BspI2cSemaphore);
#endif
  return ret;
}

/**
  * @brief  CAMERA hardware reset
  * @param  Instance Camera instance.
  * @retval BSP status
  */
int32_t CAMERA_HwReset(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  GPIO_InitTypeDef gpio_init_structure;

  /* Init DCMI PWR_ENABLE Pin */
  /* Enable GPIO clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio_init_structure.Pin = GPIO_PIN_7;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init_structure);

  /* De-assert the camera POWER_DOWN pin (active high) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

  HAL_Delay(100); /* POWER_DOWN de-asserted during 100 ms */

  /* Assert the camera POWER_DOWN pin (active high) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(20);

  return ret;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
