//=============================================================================
//		8-bit parallel LCD R61581 / no touch for LVGL version 8.3.10
//=============================================================================

look for other examples:
https://github.com/fa1ke5/STM32F407_LVGL_PORT

https://github.com/RobertoBenjami/stm32_hal_graphics_display_drivers

https://github.com/Alex2269/stm32_tft/tree/master/stm32f1-R61581-8bit/stm32f1-R61581-8bits-oscil_128mhz


 LVGL version 8.3.10

/** FSMC GPIO Configuration
  
  	FSMC_NE1(PD7) -> LCD_CS

	FSMC_NWE(PD5) -> LCD_WR

	FSMC_NOE(PD4) -> LCD_RD

	FSMC_A18(PD13) -> LCD_RS

	FSMC_D0(PD14) -> LCD_D0

	FSMC_D1(PD15) -> LCD_D1

	FSMC_D2(PD0) -> LCD_D2

	FSMC_D3(PD1) -> LCD_D3

	FSMC_D4(PE7) -> LCD_D4

	FSMC_D5(PE8) -> LCD_D5

	FSMC_D6(PE9) -> LCD_D6

	FSMC_D7(PE10) -> LCD_D7

	PD12 -> LCD_RST
**/
  
  PE7   ------> FSMC_D4
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE10   ------> FSMC_D7
  PD13   ------> FSMC_A18
  PD14   ------> FSMC_D0
  PD15   ------> FSMC_D1
  PD0   ------> FSMC_D2
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD5   ------> FSMC_NWE
  PD7   ------> FSMC_NE1
  PD12  ------> LCD_RST
  */
