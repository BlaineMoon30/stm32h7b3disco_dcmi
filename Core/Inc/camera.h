/**
  ******************************************************************************
  * @file    camera.h
  * @author  MCD Application Team
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INC_CAMERA_H_
#define __INC_CAMERA_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ov9655.h"

/* Private includes ----------------------------------------------------------*/


/* Exported defines ----------------------------------------------------------*/

/** @defgroup STM32H747I_DISCO_CAMERA_Exported_Constants Exported Constants
 * @{
 */
/* Camera instance number */
#define CAMERA_INSTANCES_NBR           1U

#define CAMERA_MODE_CONTINUOUS         DCMI_MODE_CONTINUOUS
#define CAMERA_MODE_SNAPSHOT           DCMI_MODE_SNAPSHOT

/* Camera resolutions */
#define CAMERA_R160x120                 0U     /* QQVGA Resolution            */
#define CAMERA_R320x240                 1U     /* QVGA Resolution             */
#define CAMERA_R480x272                 2U     /* 480x272 Resolution          */
#define CAMERA_R640x480                 3U     /* VGA Resolution              */
#define CAMERA_R800x480                 4U     /* WVGA Resolution             */

/* Camera Pixel Format */
#define CAMERA_PF_RGB565                0U     /* Pixel Format RGB565         */
#define CAMERA_PF_RGB888                1U     /* Pixel Format RGB888         */
#define CAMERA_PF_YUV422                2U     /* Pixel Format YUV422         */

/* Brightness */
#define CAMERA_BRIGHTNESS_MIN          -4
#define CAMERA_BRIGHTNESS_MAX           4

/* Saturation */
#define CAMERA_SATURATION_MIN          -4
#define CAMERA_SATURATION_MAX           4

/* Contrast */
#define CAMERA_CONTRAST_MIN            -4
#define CAMERA_CONTRAST_MAX             4

/* Hue Control */
#define CAMERA_HUEDEGREE_MIN           -6
#define CAMERA_HUEDEGREE_MAX            5

/* Mirror/Flip */
#define CAMERA_MIRRORFLIP_NONE          0x00U   /* Set camera normal mode     */
#define CAMERA_MIRRORFLIP_FLIP          0x01U   /* Set camera flip config     */
#define CAMERA_MIRRORFLIP_MIRROR        0x02U   /* Set camera mirror config   */

/* Zoom */
#define CAMERA_ZOOM_x8                  0x00U   /* Set zoom to x8             */
#define CAMERA_ZOOM_x4                  0x11U   /* Set zoom to x4             */
#define CAMERA_ZOOM_x2                  0x22U   /* Set zoom to x2             */
#define CAMERA_ZOOM_x1                  0x44U   /* Set zoom to x1             */

/* Color Effect */
#define CAMERA_COLOR_EFFECT_NONE        0x00U   /* No effect                  */
#define CAMERA_COLOR_EFFECT_BLUE        0x01U   /* Blue effect                */
#define CAMERA_COLOR_EFFECT_RED         0x02U   /* Red effect                 */
#define CAMERA_COLOR_EFFECT_GREEN       0x04U   /* Green effect               */
#define CAMERA_COLOR_EFFECT_BW          0x08U   /* Black and White effect     */
#define CAMERA_COLOR_EFFECT_SEPIA       0x10U   /* Sepia effect               */
#define CAMERA_COLOR_EFFECT_NEGATIVE    0x20U   /* Negative effect            */

/* Light Mode */
#define CAMERA_LIGHT_AUTO               0x00U   /* Light Mode Auto            */
#define CAMERA_LIGHT_SUNNY              0x01U   /* Light Mode Sunny           */
#define CAMERA_LIGHT_OFFICE             0x02U   /* Light Mode Office          */
#define CAMERA_LIGHT_HOME               0x04U   /* Light Mode Home            */
#define CAMERA_LIGHT_CLOUDY             0x08U   /* Light Mode Claudy          */

/* Night Mode */
#define CAMERA_NIGHT_MODE_SET           0x00U   /* Disable night mode         */
#define CAMERA_NIGHT_MODE_RESET         0x01U   /* Enable night mode          */

#define CAMERA_IRQHandler               DCMI_IRQHandler
#define CAMERA_DMA_IRQHandler           DMA2_Stream3_IRQHandler

#define CAMERA_OV9655_ADDRESS           0x60U
#define CAMERA_OV5640_ADDRESS           0x78U

/* Common Error codes */
#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED  -11

/* BSP OSPI error codes */
#define BSP_ERROR_QSPI_ASSIGN_FAILURE     -24
#define BSP_ERROR_QSPI_SETUP_FAILURE      -25
#define BSP_ERROR_QSPI_MMP_LOCK_FAILURE   -26
#define BSP_ERROR_QSPI_MMP_UNLOCK_FAILURE -27

/* BSP TS error code */
#define BSP_ERROR_TS_TOUCH_NOT_DETECTED   -30

/* BSP BUS error codes */
#define BSP_ERROR_BUS_TRANSACTION_FAILURE    -100
#define BSP_ERROR_BUS_ARBITRATION_LOSS       -101
#define BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE    -102
#define BSP_ERROR_BUS_PROTOCOL_FAILURE       -103

#define BSP_ERROR_BUS_MODE_FAULT             -104
#define BSP_ERROR_BUS_FRAME_ERROR            -105
#define BSP_ERROR_BUS_CRC_ERROR              -106
#define BSP_ERROR_BUS_DMA_FAILURE            -107

#define LCD_FRAME_BUFFER                0xD0100000//0x240A0000
#define CAMERA_FRAME_BUFFER             0xD0000000//0x24040000//0x30000000//0xD0177000

/* Exported types ------------------------------------------------------------*/

/** @defgroup CAMERA_Driver_structure  Camera Driver structure
 * @{
 */

typedef struct
{
  int32_t (*Init)(void*, uint32_t, uint32_t);
  int32_t (*DeInit)(void*);
  int32_t (*ReadID)(void*, uint32_t*);
  int32_t (*GetCapabilities)(void*, void*);
  int32_t (*SetLightMode)(void*, uint32_t);
  int32_t (*SetColorEffect)(void*, uint32_t);
  int32_t (*SetBrightness)(void*, int32_t);
  int32_t (*SetSaturation)(void*, int32_t);
  int32_t (*SetContrast)(void*, int32_t);
  int32_t (*SetHueDegree)(void*, int32_t);
  int32_t (*MirrorFlipConfig)(void*, uint32_t);
  int32_t (*ZoomConfig)(void*, uint32_t);
  int32_t (*SetResolution)(void*, uint32_t);
  int32_t (*GetResolution)(void*, uint32_t*);
  int32_t (*SetPixelFormat)(void*, uint32_t);
  int32_t (*GetPixelFormat)(void*, uint32_t*);
  int32_t (*NightModeConfig)(void*, uint32_t);
} CAMERA_Drv_t;

/** @defgroup STM32H747I_DISCO_CAMERA_Exported_Types Exported Types
  * @{
  */
typedef struct
{
  uint32_t CameraId;
  uint32_t Resolution;
  uint32_t PixelFormat;
  uint32_t LightMode;
  uint32_t ColorEffect;
  int32_t Brightness;
  int32_t Saturation;
  int32_t Contrast;
  int32_t HueDegree;
  uint32_t MirrorFlip;
  uint32_t Zoom;
  uint32_t NightMode;
  uint32_t IsMspCallbacksValid;
} CAMERA_Ctx_t;

typedef struct
{
  uint32_t Resolution;
  uint32_t LightMode;
  uint32_t ColorEffect;
  uint32_t Brightness;
  uint32_t Saturation;
  uint32_t Contrast;
  uint32_t HueDegree;
  uint32_t MirrorFlip;
  uint32_t Zoom;
  uint32_t NightMode;
} CAMERA_Capabilities_t;

#if (USE_HAL_DCMI_REGISTER_CALLBACKS == 1)
typedef struct
{
  void (* pMspInitCb)(DCMI_HandleTypeDef *);
  void (* pMspDeInitCb)(DCMI_HandleTypeDef *);
}BSP_CAMERA_Cb_t;
#endif /* (USE_HAL_DCMI_REGISTER_CALLBACKS == 1) */


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
extern int32_t CAMERA_Init(uint32_t Instance, uint32_t Resolution, uint32_t PixelFormat);
extern void CAMERA_Capture_Start(uint32_t param);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif /* __INC_CAMERA_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
