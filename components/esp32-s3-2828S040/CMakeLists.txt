#
# SPDX-FileCopyrightText: 2025 Hugo Trippaers
#
# SPDX-License-Identifier: Apache-2.0
#

idf_component_register(
        INCLUDE_DIRS "include"
        PRIV_INCLUDE_DIRS "priv_include"
        REQUIRES driver spiffs
        PRIV_REQUIRES fatfs esp_timer esp_lcd esp_lcd_touch esp_lcd_st7701
)