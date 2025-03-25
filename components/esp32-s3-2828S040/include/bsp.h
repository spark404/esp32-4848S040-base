/*
* SPDX-FileCopyrightText: 2025 Hugo Trippaers
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BSP_H
#define BSP_H

#define GPIO_LCD_BL 38

#define GPIO_LCD_SPI_CS 39
#define GPIO_LCD_SPI_SCK 48
#define GPIO_LCD_SPI_MOSI 47

#define GPIO_LCD_DE 18
#define GPIO_LCD_HSYNC 16
#define GPIO_LCD_VSYNC 17
#define GPIO_LCD_PCLK 21
#define GPIO_LCD_DISP GPIO_NUM_NC

#define GPIO_LCD_R0 11
#define GPIO_LCD_R1 12
#define GPIO_LCD_R2 13
#define GPIO_LCD_R3 14
#define GPIO_LCD_R4 0

#define GPIO_LCD_G0 8
#define GPIO_LCD_G1 20
#define GPIO_LCD_G2 3
#define GPIO_LCD_G3 46
#define GPIO_LCD_G4 9
#define GPIO_LCD_G5 10

#define GPIO_LCD_B0 4
#define GPIO_LCD_B1 5
#define GPIO_LCD_B2 6
#define GPIO_LCD_B3 7
#define GPIO_LCD_B4 15

#define GPIO_LCD_RST GPIO_NUM_NC

#define GPIO_TP_SCL       45
#define GPIO_TP_SDA       19
#define GPIO_TP_INT       GPIO_NUM_NC // Not connected
#define GPIO_TP_RST       GPIO_NUM_NC // Not connected

#define LCD_BIT_PER_PIXEL 18
#define LCD_H_RES 480
#define LCD_V_RES 480

#define GPIO_RELAY_1 40
#define GPIO_RELAY_2 2
#define GPIO_RELAY_3 1


#endif //BSP_H