/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max
*******************************************************************************/
//=============================================================================
//		8-bit parallel LCD R61581 / no touch for LVGL version 8.3.10
//=============================================================================

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
//=============================================================================


/* Information section */

/*
 * 8 bit paralell LCD FSMC driver
 * 5 control pins (CS, RS, WR, RD, RST) + 8 data pins
 * FSMC_NE1..NE4<-LCD_CS, FSMC_NOE<-LCD_RD, FSMC_NWE<-LCD_WR, FSMC_Ax<-LCD_RS
 * FSMC_D0<-LCD_D0, FSMC_D1<-LCD_D1, FSMC_D2<-LCD_D2, FSMC_D3<-LCD_D3
 * FSMC_D4<-LCD_D4, FSMC_D5<-LCD_D5, FSMC_D6<-LCD_D6, FSMC_D7<-LCD_D7
 */

/* Settings in CUBEIDE or CUBEMX
   FMC / FSMC
   - select the bank
   - select Chip Select
   - select Memory type: LCD Interface
   - select LCD Register Select: which pin we connected to the LCD RS pin
   - select Data: 8 bits
   - NOR/PSRAM Write operation: Enabled
   - NOR/PSRAM Extended mode: Disabled
   - NOR/PSRAM timings: first give a higher value, later if it works well, reduce it until it works well
   GPIO
   - Lcd reset pin (only when connected)
     - output level: High
     - mode: Output Push Pull
     - Pull-up/Pull-down: No pull-up and no pull-down
     - Max output speed: Low
     - User Label: LCD_RST

   Settings in main.h:
   - If you use freertos, add this line the main.h file
     #include "cmsis_os.h"
     (note: then the driver will also use the rtos signal to wait for the end of the dma transaction)
*/
#ifndef __LCD_LVGL_H
#define __LCD_LVGL_H

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
/* Memory address
  - Bank1 (NE1) 0x60000000
  - Bank2 (NE2) 0x64000000
  - Bank3 (NE3) 0x68000000
  - Bank4 (NE4) 0x6C000000
  - LCD_REGSELECT_BIT: to which address wire the LCD_RS pin is connected (if the LCD Register Select: A18 -> 18) */
#define LCD_ADDR_BASE         0x60000000
#define LCD_REGSELECT_BIT     18
#define LCD_ADDR_DATA         (LCD_ADDR_BASE + (1 << LCD_REGSELECT_BIT))

/* use DMA */
#define USE_LCD_DMA

/* Orientation
   - 0: 320x480 portrait 0'
   - 1: 480x320 landscape 90'
   - 2: 320x480 portrait 180'
   - 3: 480x320 landscape 270'
*/
#define LCD_ORIENTATION       1

#define LCD_WIDTH		480
#define LCD_HEIGHT		320
#define LCD_BUF_HEIGHT	20

void LVGL_setup(void);


#ifdef __cplusplus
}
#endif

#endif /* __LCD_LVGL_H */
