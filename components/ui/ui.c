// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 9.2.2
// Project name: SquareLine_Project

#include "ui.h"

#include <misc/lv_event_private.h>

#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
extern void wifi_start_scan(void *context);

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);

lv_obj_t *ui_Screen1;
lv_obj_t *ui_Panel1;
lv_obj_t *ui_Container1;
lv_obj_t *ui_Dropdown1;
lv_obj_t *ui_Button1;
lv_obj_t *ui_Label2;

void ui_event_TextArea2(lv_event_t *e);

lv_obj_t *ui_TextArea2;
lv_obj_t *ui_Label3;
lv_obj_t *ui_Button2;
lv_obj_t *ui_Label4;
lv_obj_t *ui_Button3;
lv_obj_t *ui_Label5;

void ui_event_Keyboard1(lv_event_t *e);

lv_obj_t *ui_Keyboard1;
// CUSTOM VARIABLES

// EVENTS
lv_obj_t *ui____initial_actions0;

// IMAGES AND IMAGE SETS

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_TextArea2(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_FOCUSED) {
        _ui_keyboard_set_target(ui_Keyboard1, ui_TextArea2);
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
    if (event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
    if (event_code == LV_EVENT_DEFOCUSED) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

void ui_event_Keyboard1(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_READY) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

void ui_event_Dropdown1(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_VALUE_CHANGED) {
        lv_dropdown_set_text(ui_Dropdown1, NULL);
    }
}

void ui_event_Button1(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_CLICKED) {
        wifi_start_scan(e->user_data);
    }

}

///////////////////// SCREENS ////////////////////

void ui_init(void) {
    lv_disp_t *dispp = lv_display_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                              false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Screen1);
}
