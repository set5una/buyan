#pragma once

// Board level I2C pin def
#define I2C_SCL                 (GPIO_NUM_7)
#define I2C_SDA                 (GPIO_NUM_15)

// LCD resolution
#define LCD_H_RES 320
#define LCD_V_RES 820

// LCD aux/cfg SPI channel
#define LCD_SPI_CS              (GPIO_NUM_0)
#define LCD_SPI_SCK             (GPIO_NUM_2)
#define LCD_SPI_SDO             (GPIO_NUM_1)

// LCD control
#define LCD_RGB_DE              (GPIO_NUM_40)
#define LCD_RGB_PCLK            (GPIO_NUM_41)
#define LCD_RGB_VSYNC           (GPIO_NUM_39)
#define LCD_RGB_HSYNC           (GPIO_NUM_38)
#define LCD_RGB_BLPWM           (GPIO_NUM_6)
#define LCD_RGB_RESET           (GPIO_NUM_16)

// LCD data
#define LCD_RGB_R0              (GPIO_NUM_17)
#define LCD_RGB_R1              (GPIO_NUM_46)
#define LCD_RGB_R2              (GPIO_NUM_3)
#define LCD_RGB_R3              (GPIO_NUM_8)
#define LCD_RGB_R4              (GPIO_NUM_18)

#define LCD_RGB_G0              (GPIO_NUM_14)
#define LCD_RGB_G1              (GPIO_NUM_13)
#define LCD_RGB_G2              (GPIO_NUM_12)
#define LCD_RGB_G3              (GPIO_NUM_11)
#define LCD_RGB_G4              (GPIO_NUM_10)
#define LCD_RGB_G5              (GPIO_NUM_9)

#define LCD_RGB_B0              (GPIO_NUM_21)
#define LCD_RGB_B1              (GPIO_NUM_5)
#define LCD_RGB_B2              (GPIO_NUM_45)
#define LCD_RGB_B3              (GPIO_NUM_48)
#define LCD_RGB_B4              (GPIO_NUM_47)
