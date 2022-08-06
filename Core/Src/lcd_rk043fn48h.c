/**
  ******************************************************************************
  * @file    lcd_rk043fn48h.c
  * @author  MCD Application Team 
  * @brief        
  *      
  @verbatim
  ==============================================================================
                     ##### <lcd_rk043fn48h.c> features #####
  ==============================================================================
  [..]
    Sample
    (+) Sample
        (++) Samle

                     ##### How to use this <lcd_rk043fn48h.c> #####
  ==============================================================================
  [..]
    Sample
    (+) Sample
        (++) Samle   

                     ##### <lcd_rk043fn48h.c> Limitations #####
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
#include "lcd_rk043fn48h.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/


/* External functions --------------------------------------------------------*/

/**
 * @brief  LCD Init Sequence
 *         Configure the display pinout.
 * @retval None
 */
void LCD_InitSequence(void)
{
  GPIO_InitTypeDef gpio_init_structure;
  /* LCD_DISP GPIO configuration */
  LCD_DISP_EN_GPIO_CLK_ENABLE();

  gpio_init_structure.Pin = LCD_DISP_EN_PIN;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DISP_EN_GPIO_PORT, &gpio_init_structure);
  /* Assert LCD_DISP  pin */
  HAL_GPIO_WritePin(LCD_DISP_EN_GPIO_PORT, LCD_DISP_EN_PIN, GPIO_PIN_SET);

  /* LCD_DISP_CTRL GPIO configuration */
  LCD_BL_CTRL_GPIO_CLK_ENABLE();
  gpio_init_structure.Pin = LCD_DISP_CTRL_PIN;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(LCD_DISP_CTRL_GPIO_PORT, &gpio_init_structure);

  /* LCD_BL_CTRL GPIO configuration */
  LCD_BL_CTRL_GPIO_CLK_ENABLE();

  gpio_init_structure.Pin = LCD_BL_CTRL_PIN;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &gpio_init_structure);

  /* De-assert display enable LCD_DISP_EN pin */
  HAL_GPIO_WritePin(LCD_DISP_EN_GPIO_PORT, LCD_DISP_EN_PIN, GPIO_PIN_RESET);

  /* Assert display enable LCD_DISP_CTRL pin */
  HAL_GPIO_WritePin(LCD_DISP_CTRL_GPIO_PORT, LCD_DISP_CTRL_PIN, GPIO_PIN_SET);

  /* Assert backlight LCD_BL_CTRL pin */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
