/**
  ******************************************************************************
  * @file    lcd_rk043fn48h.h
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
#ifndef __INC_LCD_RK043FN48H_H_
#define __INC_LCD_RK043FN48H_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/


/* Exported defines ----------------------------------------------------------*/
/**
 * @brief LCD special pins
 */
/* LCD Display control pin */
#define LCD_DISP_CTRL_PIN                     GPIO_PIN_2
#define LCD_DISP_CTRL_PULL                    GPIO_NOPULL
#define LCD_DISP_CTRL_GPIO_PORT               GPIOA
#define LCD_DISP_CTRL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_DISP_CTRL_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOA_CLK_DISABLE()

/* LCD Display enable pin */
#define LCD_DISP_EN_PIN                      GPIO_PIN_7
#define LCD_DISP_EN_PULL                     GPIO_NOPULL
#define LCD_DISP_EN_GPIO_PORT                GPIOK
#define LCD_DISP_EN_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOK_CLK_ENABLE()
#define LCD_DISP_EN_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOK_CLK_DISABLE()

/* Back-light control pin */
#define LCD_BL_CTRL_PIN                       GPIO_PIN_1
#define LCD_BL_CTRL_GPIO_PORT                 GPIOA
#define LCD_BL_CTRL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_BL_CTRL_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()

/**
 * @brief  RK043FN48H Size
 */
#define  RK043FN48H_WIDTH    ((uint16_t)480)          /* LCD PIXEL WIDTH            */
#define  RK043FN48H_HEIGHT   ((uint16_t)272)          /* LCD PIXEL HEIGHT           */

/**
 * @brief  RK043FN48H Timing
 */
#define  RK043FN48H_HSYNC            ((uint16_t)41)   /* Horizontal synchronization */
#define  RK043FN48H_HBP              ((uint16_t)13)   /* Horizontal back porch      */
#define  RK043FN48H_HFP              ((uint16_t)32)   /* Horizontal front porch     */
#define  RK043FN48H_VSYNC            ((uint16_t)10)   /* Vertical synchronization   */
#define  RK043FN48H_VBP              ((uint16_t)2)    /* Vertical back porch        */
#define  RK043FN48H_VFP              ((uint16_t)2)    /* Vertical front porch       */


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
extern void LCD_InitSequence(void);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif /* __INC_LCD_RK043FN48H_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
