/*
 * drivers/video/sunxi/disp2/disp/lcd/default_panel.c
 *
 * Copyright (c) 2007-2019 Allwinnertech Co., Ltd.
 * Author: zhengxiaobin <zhengxiaobin@allwinnertech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "default_panel.h"

static void LCD_power_on(u32 sel);
static void LCD_power_off(u32 sel);
static void LCD_bl_open(u32 sel);
static void LCD_bl_close(u32 sel);

static void LCD_panel_init(u32 sel);
static void LCD_panel_exit(u32 sel);
static void ili9488_init(void);

static void LCD_cfg_panel_info(panel_extend_para * info)
{
	u32 i = 0, j=0;
	u32 items;
	u8 lcd_gamma_tbl[][2] =
	{
		//{input value, corrected value}
		{0, 0},
		{15, 15},
		{30, 30},
		{45, 45},
		{60, 60},
		{75, 75},
		{90, 90},
		{105, 105},
		{120, 120},
		{135, 135},
		{150, 150},
		{165, 165},
		{180, 180},
		{195, 195},
		{210, 210},
		{225, 225},
		{240, 240},
		{255, 255},
	};

	u32 lcd_cmap_tbl[2][3][4] = {
	{
		{LCD_CMAP_G0,LCD_CMAP_B1,LCD_CMAP_G2,LCD_CMAP_B3},
		{LCD_CMAP_B0,LCD_CMAP_R1,LCD_CMAP_B2,LCD_CMAP_R3},
		{LCD_CMAP_R0,LCD_CMAP_G1,LCD_CMAP_R2,LCD_CMAP_G3},
		},
		{
		{LCD_CMAP_B3,LCD_CMAP_G2,LCD_CMAP_B1,LCD_CMAP_G0},
		{LCD_CMAP_R3,LCD_CMAP_B2,LCD_CMAP_R1,LCD_CMAP_B0},
		{LCD_CMAP_G3,LCD_CMAP_R2,LCD_CMAP_G1,LCD_CMAP_R0},
		},
	};

	items = sizeof(lcd_gamma_tbl)/2;
	for (i=0; i<items-1; i++) {
		u32 num = lcd_gamma_tbl[i+1][0] - lcd_gamma_tbl[i][0];

		for (j=0; j<num; j++) {
			u32 value = 0;

			value = lcd_gamma_tbl[i][1] + ((lcd_gamma_tbl[i+1][1] - lcd_gamma_tbl[i][1]) * j)/num;
			info->lcd_gamma_tbl[lcd_gamma_tbl[i][0] + j] = (value<<16) + (value<<8) + value;
		}
	}
	info->lcd_gamma_tbl[255] = (lcd_gamma_tbl[items-1][1]<<16) + (lcd_gamma_tbl[items-1][1]<<8) + lcd_gamma_tbl[items-1][1];

	memcpy(info->lcd_cmap_tbl, lcd_cmap_tbl, sizeof(lcd_cmap_tbl));

}

static s32 LCD_open_flow(u32 sel)
{
	LCD_OPEN_FUNC(sel, LCD_power_on, 30);   //open lcd power, and delay 50ms
	LCD_OPEN_FUNC(sel, LCD_panel_init, 50);   //open lcd power, than delay 200ms
	LCD_OPEN_FUNC(sel, sunxi_lcd_tcon_enable, 100);     //open lcd controller, and delay 100ms
	LCD_OPEN_FUNC(sel, LCD_bl_open, 0);     //open lcd backlight, and delay 0ms

	return 0;
}

static s32 LCD_close_flow(u32 sel)
{
	LCD_CLOSE_FUNC(sel, LCD_bl_close, 0);       //close lcd backlight, and delay 0ms
	LCD_CLOSE_FUNC(sel, sunxi_lcd_tcon_disable, 0);         //close lcd controller, and delay 0ms
	LCD_CLOSE_FUNC(sel, LCD_panel_exit,	200);   //open lcd power, than delay 200ms
	LCD_CLOSE_FUNC(sel, LCD_power_off, 500);   //close lcd power, and delay 500ms

	return 0;
}

static void LCD_power_on(u32 sel)
{
	sunxi_lcd_power_enable(sel, 0);//config lcd_power pin to open lcd power0
	sunxi_lcd_pin_cfg(sel, 1);
}

static void LCD_power_off(u32 sel)
{
	sunxi_lcd_pin_cfg(sel, 0);
	sunxi_lcd_power_disable(sel, 0);//config lcd_power pin to close lcd power0
}

static void LCD_bl_open(u32 sel)
{
	sunxi_lcd_pwm_enable(sel);
	sunxi_lcd_backlight_enable(sel);//config lcd_bl_en pin to open lcd backlight
}

static void LCD_bl_close(u32 sel)
{
	sunxi_lcd_backlight_disable(sel);//config lcd_bl_en pin to close lcd backlight
	sunxi_lcd_pwm_disable(sel);
}

static void LCD_panel_init(u32 sel)
{
	ili9488_init();
	return;
}

static void LCD_panel_exit(u32 sel)
{
	return ;
}

//sel: 0:lcd0; 1:lcd1
static s32 LCD_user_defined_func(u32 sel, u32 para1, u32 para2, u32 para3)
{
	return 0;
}

__lcd_panel_t default_panel = {
	/* panel driver name, must mach the name of lcd_drv_name in sys_config.fex */
	.name = "default_lcd",
	.func = {
		.cfg_panel_info = LCD_cfg_panel_info,
		.cfg_open_flow = LCD_open_flow,
		.cfg_close_flow = LCD_close_flow,
		.lcd_user_defined_func = LCD_user_defined_func,
	},
};

#include <asm/arch/gpio.h>

static int sunxi_gpio_output2(u32 pin, u32 val)
{
	u32 dat;
	u32 bank = GPIO_BANK(pin);
	u32 num = GPIO_NUM(pin);
	struct sunxi_gpio *pio = BANK_TO_GPIO(bank);

	dat = readl(&pio->dat);
	if (val)
		dat |= 0x1 << num;
	else
		dat &= ~(0x1 << num);

	writel(dat, &pio->dat);

	return 0;
}

#define	LCD_CS_SET  	{sunxi_gpio_output2(SUNXI_GPE(11),1);}    	//片选端口1	
#define	LCD_SDA_SET		{sunxi_gpio_output2(SUNXI_GPE(10),1);}    	//写数据1    	   
#define	LCD_SCLK_SET	{sunxi_gpio_output2(SUNXI_GPE(8),1);}     	//写脉冲1
								    
#define	LCD_CS_CLR  	{sunxi_gpio_output2(SUNXI_GPE(11),0);}     	//片选端口0
#define	LCD_SDA_CLR		{sunxi_gpio_output2(SUNXI_GPE(10),0);}     	//写数据0     	   
#define	LCD_SCLK_CLR	{sunxi_gpio_output2(SUNXI_GPE(8),0);}     	//写脉冲0

static void write_cmd(uint8_t cmd)
{
	uint8_t i = 0;
	uint16_t command = 0;

	LCD_CS_CLR;
	udelay(30);
	command |= cmd;

	for(i=0;i<9;i++)
	{
		LCD_SCLK_CLR;
		if(command&0x0100)
		{	
			LCD_SDA_SET;
		}
		else
		{	
			LCD_SDA_CLR;
		}
		udelay(30);
		LCD_SCLK_SET;
		udelay(30);
		command <<= 1;
	}
	LCD_CS_SET;
	udelay(30);
}

static void write_dat(uint8_t data)
{
	uint8_t i = 0;
	uint16_t my_data = 0x0100;

	LCD_CS_CLR;
	udelay(30);
	my_data |= data;

	for(i=0;i<9;i++)
	{
		LCD_SCLK_CLR;
		if(my_data&0x0100)
		{	
			LCD_SDA_SET;
		}
		else
		{	
			LCD_SDA_CLR;
		}
		udelay(30);
		LCD_SCLK_SET;
		udelay(30);
		my_data <<= 1;
	}
	LCD_CS_SET;
	udelay(30);
}

static void ili9488_init(void)
{
#define LCD_XSIZE 	640
#define LCD_YSIZE 	480
	sunxi_gpio_set_cfgpin(SUNXI_GPE(11), SUNXI_GPIO_OUTPUT);	//cs	pe11
	sunxi_gpio_set_cfgpin(SUNXI_GPE(10), SUNXI_GPIO_OUTPUT);	//sda	pe10
	sunxi_gpio_set_cfgpin(SUNXI_GPE(8), SUNXI_GPIO_OUTPUT);		//sclk	pe8
	sunxi_gpio_set_cfgpin(SUNXI_GPD(22), SUNXI_GPIO_OUTPUT);	//blk	pd22

	sunxi_gpio_set_pull(SUNXI_GPE(11), SUNXI_GPIO_PULL_UP);
	sunxi_gpio_set_pull(SUNXI_GPE(10), SUNXI_GPIO_PULL_UP);
	sunxi_gpio_set_pull(SUNXI_GPE(8), SUNXI_GPIO_PULL_UP);
	sunxi_gpio_set_pull(SUNXI_GPD(22), SUNXI_GPIO_PULL_UP);

	sunxi_gpio_set_drv(SUNXI_GPE(11), 3);	
	sunxi_gpio_set_drv(SUNXI_GPE(10), 3);
	sunxi_gpio_set_drv(SUNXI_GPE(8), 3);
	sunxi_gpio_set_drv(SUNXI_GPD(22), 3);

	sunxi_gpio_set_cfgpin(SUNXI_GPB(7), SUNXI_GPIO_OUTPUT);
	//sunxi_gpio_set_pull(SUNXI_GPB(7), SUNXI_GPIO_PULL_UP);
	//sunxi_gpio_set_drv(SUNXI_GPB(7), 3);
	sunxi_gpio_output2(SUNXI_GPB(7),1);

	printk("wincao uboot ili9488 3line spi init...\n");
#if 0
	// VCI=2.8V 
	//************* Reset LCD Driver ****************// 
	LCD_nRESET = 1; 
	Delayms(1); // Delay 1ms 
	LCD_nRESET = 0; 
	Delayms(10); // Delay 10ms // This delay time is necessary 
	LCD_nRESET = 1; 
	Delayms(120); // Delay 120 ms 
#endif	
	//************* Start Initial Sequence **********// 
	
	write_cmd(0xFF);
	write_dat(0x30); 
	write_cmd(0xFF);
	write_dat(0x52); 
	write_cmd(0xFF);
	write_dat(0x01); 
	write_cmd(0xE3); 
	write_dat(0x00); 
	write_cmd(0x0A); 
	write_dat(0x01); 
	write_cmd(0x23); 
	write_dat(0xA2); 
	write_cmd(0x24); 
	write_dat(0x0C); 
	write_cmd(0x25); 
	write_dat(0x06); 
	write_cmd(0x26); 
	write_dat(0x14); 
	
	write_cmd(0X27);      //N-Gamma 
	write_dat(0x14); 
	write_cmd(0x30); 
	write_dat(0x68); 
	write_cmd(0x38); 
	write_dat(0x9C); 
	write_cmd(0x39); 
	write_dat(0xA7); 
	write_cmd(0x3A); 
	write_dat(0x4C); 
	write_cmd(0x28); 
	write_dat(0x40); 
	write_cmd(0x29); 
	write_dat(0x01); 
	write_cmd(0x2A); 
	write_dat(0xdF); 
	
	write_cmd(0X49);    //Power Control 1 
	write_dat(0x3C);    //Vreg1out 
	write_cmd(0x91);    //Verg2out 
	
	write_dat(0x77);    //Power Control 2 
	write_cmd(0x92);    //VGH,VGL 
	
	write_dat(0x77);    //Power Control 3 
	write_cmd(0x98); 
	write_dat(0x46);    //Vcom 
	write_cmd(0xA0);

	write_dat(0X55); 
	write_cmd(0xA1);
	write_dat(0x50);
	write_cmd(0xA4);
	write_dat(0x9C);	//479

	write_cmd(0XA7); 
	write_dat(0x02);
	write_cmd(0xA8);
	write_dat(0x01);
	write_cmd(0xA9);	//319

	
	write_dat(0x01);      //Memory Access
	write_cmd(0xAA); 	  //0x28
	
	write_dat(0xFC);      // Interface Pixel Format 
	write_cmd(0xAB);    //18bit 
	
	write_dat(0X28);      // Interface Mode Control 
	write_cmd(0xAC);     
	
	write_dat(0x06);      //Frame rate 
	write_cmd(0xAD);    //60Hz 
	
	write_dat(0x06);      //Display Inversion Control 
	write_cmd(0xAE);    //2-dot 
	
	write_dat(0X06);      //RGB/MCU Interface Control 
	write_cmd(0xAF);    //MCU:02; RGB:32/22 
	write_dat(0x03);    //Source,Gate scan dieection 
	
	
	write_cmd(0XB0);      // Set Image Function   
	write_dat(0x08);    //disable 24 bit data input 
	
	write_cmd(0xB1);     //   A d j u s t   C o n t r o l 
	write_dat(0x26);     
	write_cmd(0xB2);     
	write_dat(0x28);     
	write_cmd(0xB3);   //   D 7   s t r e a m ,   l o o s e  
	
	write_dat(0x28);      //Normal Black 
	
	write_cmd(0xB4);
	write_dat(0x03);  
	write_cmd(0xB5);
	write_dat(0x08);  
	write_cmd(0xB6);
	write_dat(0x26);  
	write_cmd(0xB7);
	write_dat(0x08);  
	write_cmd(0xB8);
	write_dat(0x26); 
	write_cmd(0xFF);
	write_dat(0x30);
	write_cmd(0xFF);
	write_dat(0x52);
	write_cmd(0xFF);
	write_dat(0x02);
	write_cmd(0xB0);
	write_dat(0x0B);
	write_cmd(0xB1);
	write_dat(0x16);
	write_cmd(0xB2);
	write_dat(0x17); 
	write_cmd(0xB3);
	write_dat(0x2C); 
	write_cmd(0xB4);
	write_dat(0x32);  
	write_cmd(0xB5);
	write_dat(0x3B);  
	write_cmd(0xB6);
	write_dat(0x29); 
	write_cmd(0xB7);
	write_dat(0x40);   
	write_cmd(0xB8);
	write_dat(0x0d);
	write_cmd(0xB9);
	write_dat(0x05);
	write_cmd(0xBA);
	write_dat(0x12);
	write_cmd(0xBB);
	write_dat(0x10);
	write_cmd(0xBC);
	write_dat(0x12);
	write_cmd(0xBD);
	write_dat(0x15);
	write_cmd(0xBE);
	write_dat(0x19);              
	write_cmd(0xBF);
	write_dat(0x0E);
	write_cmd(0xC0);
	write_dat(0x16);  
	write_cmd(0xC1);
	write_dat(0x0A);
	write_cmd(0xD0);
	write_dat(0x0C);
	write_cmd(0xD1);
	write_dat(0x17);
	write_cmd(0xD2);
	write_dat(0x14);
	write_cmd(0xD3);
	write_dat(0x2E);   
	write_cmd(0xD4);
	write_dat(0x32);   
	write_cmd(0xD5);
	write_dat(0x3C);  
	write_cmd(0xD6);
	write_dat(0x22);
	write_cmd(0xD7);
	write_dat(0x3D);
	write_cmd(0xD8);
	write_dat(0x0D);
	write_cmd(0xD9);
	write_dat(0x07);
	write_cmd(0xDA);
	write_dat(0x13);
	write_cmd(0xDB);
	write_dat(0x13);
	write_cmd(0xDC);
	write_dat(0x11);
	write_cmd(0xDD);
	write_dat(0x15);
	write_cmd(0xDE);
	write_dat(0x19);                   
	write_cmd(0xDF);
	write_dat(0x10);
	write_cmd(0xE0);
	write_dat(0x17);    
	write_cmd(0xE1);
	write_dat(0x0A);
	write_cmd(0xFF);
	write_dat(0x30);
	write_cmd(0xFF);
	write_dat(0x52);
	write_cmd(0xFF);
	write_dat(0x03);   
	write_cmd(0x00);
	write_dat(0x2A);
	write_cmd(0x01);
	write_dat(0x2A);
	write_cmd(0x02);
	write_dat(0x2A);
	write_cmd(0x03);
	write_dat(0x2A);
	write_cmd(0x04);
	write_dat(0x61);  
	write_cmd(0x05);
	write_dat(0x80);   
	write_cmd(0x06);
	write_dat(0xc7);   
	write_cmd(0x07);
	write_dat(0x01);  
	write_cmd(0x08);
	write_dat(0x03); 
	write_cmd(0x09);
	write_dat(0x04);
	write_cmd(0x70);
	write_dat(0x22);
	write_cmd(0x71);
	write_dat(0x80);
	write_cmd(0x30);
	write_dat(0x2A);
	write_cmd(0x31);
	write_dat(0x2A);
	write_cmd(0x32);
	write_dat(0x2A);
	write_cmd(0x33);
	write_dat(0x2A);
	write_cmd(0x34);
	write_dat(0x61);
	write_cmd(0x35);
	write_dat(0xc5);
	write_cmd(0x36);
	write_dat(0x80);
	write_cmd(0x37);
	write_dat(0x23);
	write_cmd(0x40);
	write_dat(0x03); 
	write_cmd(0x41);
	write_dat(0x04); 
	write_cmd(0x42);
	write_dat(0x05); 
	write_cmd(0x43);
	write_dat(0x06); 
	write_cmd(0x44);
	write_dat(0x11); 
	write_cmd(0x45);
	write_dat(0xe8); 
	write_cmd(0x46);
	write_dat(0xe9); 
	write_cmd(0x47);
	write_dat(0x11);
	write_cmd(0x48);
	write_dat(0xea); 
	write_cmd(0x49);
	write_dat(0xeb);
	write_cmd(0x50);
	write_dat(0x07); 
	write_cmd(0x51);
	write_dat(0x08); 
	write_cmd(0x52);
	write_dat(0x09); 
	write_cmd(0x53);
	write_dat(0x0a); 
	write_cmd(0x54);
	write_dat(0x11); 
	write_cmd(0x55);
	write_dat(0xec); 
	write_cmd(0x56);
	write_dat(0xed); 
	write_cmd(0x57);
	write_dat(0x11); 
	write_cmd(0x58);
	write_dat(0xef); 
	write_cmd(0x59);
	write_dat(0xf0); 
	write_cmd(0xB1);
	write_dat(0x01); 
	write_cmd(0xB4);
	write_dat(0x15); 
	write_cmd(0xB5);
	write_dat(0x16); 
	write_cmd(0xB6);
	write_dat(0x09); 
	write_cmd(0xB7);
	write_dat(0x0f); 
	write_cmd(0xB8);
	write_dat(0x0d); 
	write_cmd(0xB9);
	write_dat(0x0b); 
	write_cmd(0xBA);
	write_dat(0x1D); 
	write_cmd(0xC7);
	write_dat(0x02); 
	write_cmd(0xCA);
	write_dat(0x17); 
	write_cmd(0xCB);
	write_dat(0x18); 
	write_cmd(0xCC);
	write_dat(0x0a); 
	write_cmd(0xCD);
	write_dat(0x10); 
	write_cmd(0xCE);
	write_dat(0x0e); 
	write_cmd(0xCF);
	write_dat(0x0c); 
	write_cmd(0xD0);
	write_dat(0x00); 
	write_cmd(0x81);
	write_dat(0x1D);
	write_cmd(0x84);
	write_dat(0x15); 
	write_cmd(0x85);
	write_dat(0x16); 
	write_cmd(0x86);
	write_dat(0x10); 
	write_cmd(0x87);
	write_dat(0x0a); 
	write_cmd(0x88);
	write_dat(0x0c); 
	write_cmd(0x89);
	write_dat(0x0e);
	write_cmd(0x8A);
	write_dat(0x02); 
	write_cmd(0x97);
	write_dat(0x1D); 
	write_cmd(0x9A);
	write_dat(0x17); 
	write_cmd(0x9B);
	write_dat(0x18);
	write_cmd(0x9C);
	write_dat(0x0f);
	write_cmd(0x9D);
	write_dat(0x09); 
	write_cmd(0x9E);
	write_dat(0x0b); 
	write_cmd(0x9F);
	write_dat(0x0d); 
	write_cmd(0xA0);
	write_dat(0x01); 
	write_cmd(0xFF);
	write_dat(0x30);
	write_cmd(0xFF);
	write_dat(0x52); 
	write_cmd(0xFF);
	write_dat(0x00);   
	write_cmd(0x36);
	write_dat(0x02);//0A
	write_cmd(0x3A);
	write_dat(0x70);
	write_cmd(0x11);     //Sleep out 
	mdelay(200); 
	write_cmd(0x29);    //Display on 
	printk("ili9488 init ok\n");
}
