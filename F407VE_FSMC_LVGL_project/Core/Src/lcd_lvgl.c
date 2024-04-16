/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max
*******************************************************************************/#include "main.h"
#include "lcd_lvgl.h"
#include "../lvgl/lvgl.h"

//=============================================================================
#define  DMA_MAXSIZE       0xFFFE
/* note:
   - DMA_MAXSIZE: if the transacion Size > DMA_MAXSIZE -> multiple DMA transactions (because DMA transaction size register is 16bit) */

//=============================================================================
/* Memory to memory DMA handle name (see in main.c) */
#define LCD_DMA_HANDLE        hdma_memtomem_dma2_stream0

extern DMA_HandleTypeDef      LCD_DMA_HANDLE;

//=============================================================================

struct
{
  uint32_t size;              /* all transactions data size */
  uint32_t tsize;            /* actual DMA transaction data size */
  uint32_t ptr;               /* data pointer for DMA */
  lv_disp_drv_t * disp_drv;
}dma_status;


//=============================================================================
void lcd_delay(uint32_t Delay)
{
  #ifndef  osCMSIS
  HAL_Delay(Delay);
  #else
  osDelay(Delay);
  #endif
}
//=============================================================================
void lcd_reset(void)
{
  #if defined(LCD_RST_GPIO_Port) && defined (LCD_RST_Pin)
	lcd_delay(10);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	lcd_delay(10);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  #endif
	lcd_delay(10);
}
//=============================================================================
void LCD_Write_Command(uint8_t data)
{
	*(volatile uint8_t *)LCD_ADDR_BASE = data;
}
//=============================================================================
void LCD_Write_Data(uint8_t data)
{
	*(volatile uint8_t *)LCD_ADDR_DATA = data;
}
//=============================================================================
void lcd_set_address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
	LCD_Write_Command(0x2A);
	LCD_Write_Data(X1>>8);
	LCD_Write_Data(X1);
	LCD_Write_Data(X2>>8);
	LCD_Write_Data(X2);

	LCD_Write_Command(0x2B);
	LCD_Write_Data(Y1>>8);
	LCD_Write_Data(Y1);
	LCD_Write_Data(Y2>>8);
	LCD_Write_Data(Y2);

	LCD_Write_Command(0x2C);
}
//=============================================================================
void lcd_init(void)
{
	lcd_reset();
	LCD_Write_Command(0x01); // soft reset
	lcd_delay(150);

	LCD_Write_Command(0x28); 	// display off

	LCD_Write_Command(0x3A);	//Pixel read=565, write=565.
	LCD_Write_Data(0x55);

	LCD_Write_Command(0xB0);	// unlocks E0, F0
	LCD_Write_Data(0x00);

	LCD_Write_Command(0xB3);	//Frame Memory, interface [02 00 00 00]
	LCD_Write_Data(0x02);
	LCD_Write_Data(0x00);
	LCD_Write_Data(0x00);
	LCD_Write_Data(0x00);

	LCD_Write_Command(0xB4);	// Frame mode [00]
	LCD_Write_Data(0x00);

	LCD_Write_Command(0xD0);	// Set Power [00 43 18] x1.00, x6, x3
	LCD_Write_Data(0x07);
	LCD_Write_Data(0x42);
	LCD_Write_Data(0x18);

	LCD_Write_Command(0xD1);	// Set VCOM [00 00 00] x0.72, x1.02
	LCD_Write_Data(0x00);
	LCD_Write_Data(0x07);
	LCD_Write_Data(0x10);

	LCD_Write_Command(0xD2);	// Set Power for Normal Mode [01 22]
	LCD_Write_Data(0x01);
	LCD_Write_Data(0x02);

	LCD_Write_Command(0xD3);	// Set Power for Partial Mode [01 22]
	LCD_Write_Data(0x01);
	LCD_Write_Data(0x02);

	LCD_Write_Command(0xD4); 	// Set Power for Idle Mode [01 22]
	LCD_Write_Data(0x01);
	LCD_Write_Data(0x02);

	LCD_Write_Command(0xC0);	//Panel Driving BGR for 1581 [10 3B 00 02 11]
	LCD_Write_Data(0x12);
	LCD_Write_Data(0x3B);
	LCD_Write_Data(0x00);
	LCD_Write_Data(0x02);
	LCD_Write_Data(0x11);

	LCD_Write_Command(0xC5);	//Frame Rate [03]
	LCD_Write_Data(0x03);



	#if (LCD_ORIENTATION == 0)
		LCD_Write_Command(0x36);
		LCD_Write_Data(0x40);
		LCD_Write_Data(0x08);
	#elif (LCD_ORIENTATION == 1)
		LCD_Write_Command(0x36);
		LCD_Write_Data(0x20);
		LCD_Write_Data(0x08);
	#elif (LCD_ORIENTATION == 2)
		LCD_Write_Command(0x36);
		LCD_Write_Data(0x80);
		LCD_Write_Data(0x08);
	#elif (LCD_ORIENTATION == 3)
		LCD_Write_Command(0x36);
		LCD_Write_Data(0x40);
		LCD_Write_Data(0x80);
		LCD_Write_Data(0x20);
		LCD_Write_Data(0x08);
	#endif

	LCD_Write_Command(0x11);	//Sleep Out
	lcd_delay(150);

	LCD_Write_Command(0x29);	//Display On
	lcd_delay(10);

}
//==================================================================================================
/*
 * 			LVGL
 */
//==================================================================================================
static void flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
	int16_t w = area->x2 - area->x1 + 1;
	int16_t h = area->y2 - area->y1 + 1;
	lcd_set_address(area->x1, area->y1, area->x2, area->y2);

#ifdef USE_LCD_DMA
	    dma_status.disp_drv = disp_drv;
	    dma_status.size = w * h;
	    if(dma_status.size > DMA_MAXSIZE)
	          dma_status.tsize = DMA_MAXSIZE;
	    else /* the transaction can be performed with one DMA operation */
	          dma_status.tsize = dma_status.size;
	    dma_status.ptr = (uint32_t)color_p;
	    HAL_DMA_Start_IT(&LCD_DMA_HANDLE, dma_status.ptr, LCD_ADDR_DATA, dma_status.tsize);
#else
	int32_t i, size = w * h;
	    while(size--)
	    {
			*(volatile uint8_t *)LCD_ADDR_DATA = color_p->full;
			*(volatile uint8_t *)LCD_ADDR_DATA =color_p->full >> 8;
			color_p ++;
	    }
	lv_disp_flush_ready(disp_drv);

#endif

}
//==================================================================================================
static void dma_transferComplete(DMA_HandleTypeDef * hdma)
{
	  if(hdma == &hdma_memtomem_dma2_stream0)
	  {
	    if(dma_status.size > dma_status.tsize)
	    { /* dma operation is still required */

	      dma_status.ptr += dma_status.tsize;        /* 8bit multidata */

	      dma_status.size -= dma_status.tsize;
	      if(dma_status.size <= DMA_MAXSIZE)
	        dma_status.tsize = dma_status.size;

	      HAL_DMA_Start_IT(&LCD_DMA_HANDLE, dma_status.ptr, LCD_ADDR_DATA, dma_status.tsize);
	    }
	    else
	    { /* dma operations have ended */
	    	lv_disp_flush_ready(dma_status.disp_drv);
	    }
	  }


}
//==================================================================================================
void LVGL_setup(void)
{
	static lv_color_t disp_draw_buf[LCD_WIDTH * LCD_BUF_HEIGHT];
	static lv_color_t disp_draw_buf2[LCD_WIDTH * LCD_BUF_HEIGHT];
	static lv_disp_draw_buf_t disp_buf;

	lcd_init();
	lv_init(); // Initialize lvgl

	lv_disp_draw_buf_init(&disp_buf, disp_draw_buf, disp_draw_buf2, LCD_WIDTH * LCD_BUF_HEIGHT);

	static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LCD_WIDTH;                 /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LCD_HEIGHT;                 /*Set the vertical resolution in pixels*/

	lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
#ifdef USE_LCD_DMA
	HAL_DMA_RegisterCallback(&LCD_DMA_HANDLE, HAL_DMA_XFER_CPLT_CB_ID, dma_transferComplete);
#endif
}
//==================================================================================================


