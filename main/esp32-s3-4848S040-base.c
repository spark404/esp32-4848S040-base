#include <esp_lcd_panel_io_additions.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st7701.h>
#include <esp_lcd_touch_gt911.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>

#include <portmacro.h>
#include <stdio.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_private/esp_task_wdt.h>
#include <soc/gpio_num.h>

#include "bsp.h"

#define TAG "app_main"

extern lv_image_dsc_t anime_480x480;
extern lv_image_dsc_t rgb_large;
extern lv_image_dsc_t RGB_24bits_palette_color_test_chart;

static lv_disp_t * disp_handle = NULL;
static lv_indev_t* touch_handle = NULL;

void app_main(void) {
    esp_task_wdt_stop();

    const gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_LCD_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LCD_BL, 0));

    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO,
        .cs_gpio_num = GPIO_LCD_SPI_CS,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = GPIO_LCD_SPI_SCK,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = GPIO_LCD_SPI_MOSI,
        .io_expander = NULL,
    };
    esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));


    ESP_LOGI(TAG, "Install ST7701 panel driver");
    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .psram_trans_align = 64,
        .data_width = 16,
        .bits_per_pixel = 16,
        .de_gpio_num = GPIO_LCD_DE,
        .pclk_gpio_num = GPIO_LCD_PCLK,
        .vsync_gpio_num = GPIO_LCD_VSYNC,
        .hsync_gpio_num = GPIO_LCD_HSYNC,
        .disp_gpio_num = GPIO_LCD_DISP,
        .data_gpio_nums = {
            GPIO_LCD_R0,
            GPIO_LCD_R1,
            GPIO_LCD_R2,
            GPIO_LCD_R3,
            GPIO_LCD_R4,
            GPIO_LCD_G0,
            GPIO_LCD_G1,
            GPIO_LCD_G2,
            GPIO_LCD_G3,
            GPIO_LCD_G4,
            GPIO_LCD_G5,
            GPIO_LCD_B0,
            GPIO_LCD_B1,
            GPIO_LCD_B2,
            GPIO_LCD_B3,
            GPIO_LCD_B4,
        },
        .timings = ST7701_480_480_PANEL_60HZ_RGB_TIMING(),
        .flags = {
            .fb_in_psram = true
        },
        .num_fbs = 2,
        .bounce_buffer_size_px = LCD_H_RES * 10
    };

    st7701_lcd_init_cmd_t lcd_init_cmds[] = {
        //   cmd   data        data_size  delay_ms
        {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
        {0xC0, (uint8_t []){0x3B, 0x00}, 2, 0},
        {0xC1, (uint8_t []){0x0D, 0x02}, 2, 0},
        {0xC2, (uint8_t []){0x31, 0x05}, 2, 0},
        {
            0xB0,
            (uint8_t []){
                0x00, 0x11, 0x18, 0x0E, 0x11, 0x06, 0x07, 0x08, 0x07, 0x22, 0x04, 0x12, 0x0F, 0xAA, 0x31, 0x18
            },
            16, 0
        },
        {
            0xB1,
            (uint8_t []){
                0x00, 0x11, 0x19, 0x0E, 0x12, 0x07, 0x08, 0x08, 0x08, 0x22, 0x04, 0x11, 0x11, 0xA9, 0x31, 0x18
            },
            16, 0
        },
        {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
        {0xB0, (uint8_t []){0x60}, 1, 0}, // Vop=4.7375v
        {0xB1, (uint8_t []){0x32}, 1, 0}, // VCOM=32
        {0xB2, (uint8_t []){0x07}, 1, 0}, // VGH=15v
        {0xB3, (uint8_t []){0x80}, 1, 0},
        {0xB5, (uint8_t []){0x49}, 1, 0}, // VGL=-10.17v
        {0xB7, (uint8_t []){0x85}, 1, 0},
        {0xB8, (uint8_t []){0x21}, 1, 0}, // AVDD=6.6 & AVCL=-4.6
        {0xC1, (uint8_t []){0x78}, 1, 0},
        {0xC2, (uint8_t []){0x78}, 1, 0},
        {0xE0, (uint8_t []){0x00, 0x1B, 0x02}, 3, 0},
        {0xE1, (uint8_t []){0x08, 0xA0, 0x00, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x44, 0x44}, 11, 0},
        {0xE2, (uint8_t []){0x11, 0x11, 0x44, 0x44, 0xED, 0xA0, 0x00, 0x00, 0xEC, 0xA0, 0x00, 0x00}, 12, 0},
        {0xE3, (uint8_t []){0x00, 0x00, 0x11, 0x11}, 4, 0},
        {0xE4, (uint8_t []){0x44, 0x44}, 2, 0},
        {
            0xE5,
            (uint8_t []){
                0x0A, 0xE9, 0xD8, 0xA0, 0x0C, 0xEB, 0xD8, 0xA0, 0x0E, 0xED, 0xD8, 0xA0, 0x10, 0xEF, 0xD8, 0xA0
            },
            16, 0
        },
        {0xE6, (uint8_t []){0x00, 0x00, 0x11, 0x11}, 4, 0},
        {0xE7, (uint8_t []){0x44, 0x44}, 2, 0},
        {
            0xE8,
            (uint8_t []){
                0x09, 0xE8, 0xD8, 0xA0, 0x0B, 0xEA, 0xD8, 0xA0, 0x0D, 0xEC, 0xD8, 0xA0, 0x0F, 0xEE, 0xD8, 0xA0
            },
            16, 0
        },
        {0xEB, (uint8_t []){0x02, 0x00, 0xE4, 0xE4, 0x88, 0x00, 0x40}, 7, 0},
        {0xEC, (uint8_t []){0x3C, 0x00}, 2, 0},
        {
            0xED,
            (uint8_t []){
                0xAB, 0x89, 0x76, 0x54, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x20, 0x45, 0x67, 0x98, 0xBA
            },
            16, 0
        },

        //-----------VAP & VAN---------------
        {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
        {0xE5, (uint8_t []){0xE4}, 1, 0},
        {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},

        // This is configured by esp_lcd_panel_dev_config_t
        // {0x3A, (uint8_t []){0x60}, 1, 10}, // 0x70 RGB888, 0x60 RGB666, 0x50 RGB565

        {0x11, NULL, 0, 120},
        {0x29, NULL, 0, 0}, // Display On

    };

    st7701_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .init_cmds = lcd_init_cmds, // Uncomment these line if use custom initialization commands
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7701_lcd_init_cmd_t),
        .flags = {
            .mirror_by_cmd = 1, // Only work when `enable_io_multiplex` is set to 0
            .enable_io_multiplex = 0, /**
                                             * Set to 1 if panel IO is no longer needed after LCD initialization.
                                             * If the panel IO pins are sharing other pins of the RGB interface to save GPIOs,
                                             * Please set it to 1 to release the pins.
                                             */
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_LCD_RST, // Set to -1 if not use
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR, // Implemented by LCD command `36h`
        .bits_per_pixel = LCD_BIT_PER_PIXEL, // Implemented by LCD command `3Ah` (16/18/24)
        .vendor_config = &vendor_config,
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(io_handle, &panel_config, &panel_handle)); /**
                                                                                             * Only create RGB when `enable_io_multiplex` is set to 0,
                                                                                             * or initialize st7701 meanwhile                                                                                   */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    // Only reset RGB when `enable_io_multiplex` is set to 1, or reset st7701 meanwhile
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // Only initialize RGB when `enable_io_multiplex` is set to 1, or initialize st7701 meanwhile
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LCD_BL, 1));

    // Initialize lvgl
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    // Add the screen
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .double_buffer = true,
        .buffer_size = LCD_H_RES * LCD_V_RES,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = false,
            .buff_spiram = true,
            .full_refresh = false,
            .direct_mode = true
        }
    };

    const lvgl_port_display_rgb_cfg_t rgb_cfg =
        {
        .flags =
        {
            .bb_mode = true,
            .avoid_tearing = true,
        }
        };

    disp_handle = lvgl_port_add_disp_rgb(&disp_cfg, &rgb_cfg);
    if (disp_handle == NULL) {
        ESP_LOGE(TAG, "Unable to setup display");
    }
    i2c_master_bus_handle_t tp_bus_handle = NULL;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = GPIO_TP_SCL,
        .sda_io_num = GPIO_TP_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &tp_bus_handle));
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = 100000;

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(tp_bus_handle, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_io_gt911_config_t tp_gt911_config = {
        .dev_addr = tp_io_config.dev_addr,
    };

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_V_RES,
        .y_max = LCD_H_RES,
        .rst_gpio_num = GPIO_TP_RST,
        .int_gpio_num = GPIO_TP_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .driver_data = &tp_gt911_config
    };

    esp_lcd_touch_handle_t tp = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp_handle,
        .handle = tp,
    };
    touch_handle = lvgl_port_add_touch(&touch_cfg);
    if (touch_handle == NULL) {
        ESP_LOGE(TAG, "Unable to setup touchpad");
    }


    lv_obj_t *scr = lv_scr_act();

    lvgl_port_lock(0);
    lv_obj_t *background = lv_img_create(scr);
    lv_img_set_src(background, &anime_480x480);
    lv_obj_align(background, LV_ALIGN_TOP_LEFT, 0, 0);

    // lv_obj_t *img_logo = lv_img_create(scr);
    // lv_img_set_src(img_logo, &RGB_24bits_palette_color_test_chart);
    // lv_obj_align(img_logo, LV_ALIGN_CENTER, 0, 0);

    lvgl_port_unlock();

    while (true) {
        vTaskDelay(100 /portTICK_PERIOD_MS);
    }
}
