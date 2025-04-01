// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 9.2.2
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Screen1_screen_init(void)
{
ui_Screen1 = lv_obj_create(NULL);
lv_obj_remove_flag( ui_Screen1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_image_src( ui_Screen1, &ui_img_anime_480x480_png, LV_PART_MAIN | LV_STATE_DEFAULT );

ui_Panel1 = lv_obj_create(ui_Screen1);
lv_obj_set_width( ui_Panel1, 440);
lv_obj_set_height( ui_Panel1, 440);
lv_obj_set_align( ui_Panel1, LV_ALIGN_CENTER );
lv_obj_remove_flag( ui_Panel1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_blend_mode(ui_Panel1, LV_BLEND_MODE_SUBTRACTIVE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_opa(ui_Panel1, 192, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Container1 = lv_obj_create(ui_Screen1);
lv_obj_remove_style_all(ui_Container1);
lv_obj_set_width( ui_Container1, 430);
lv_obj_set_height( ui_Container1, 430);
lv_obj_set_align( ui_Container1, LV_ALIGN_CENTER );
lv_obj_remove_flag( ui_Container1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_blend_mode(ui_Container1, LV_BLEND_MODE_NORMAL, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_opa(ui_Container1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Dropdown1 = lv_dropdown_create(ui_Container1);
    lv_dropdown_set_text(ui_Dropdown1, "Select WifI Network...");
lv_dropdown_set_options( ui_Dropdown1, "SparkNet\nKoffieNet" );
lv_obj_set_width( ui_Dropdown1, 227);
lv_obj_set_height( ui_Dropdown1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Dropdown1, 20 );
lv_obj_set_y( ui_Dropdown1, 45 );
lv_obj_add_flag( ui_Dropdown1, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags

ui_Button1 = lv_button_create(ui_Container1);
lv_obj_set_width( ui_Button1, 100);
lv_obj_set_height( ui_Button1, 34);
lv_obj_set_x( ui_Button1, 297 );
lv_obj_set_y( ui_Button1, 47 );
lv_obj_add_flag( ui_Button1, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_remove_flag( ui_Button1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Label2 = lv_label_create(ui_Button1);
lv_obj_set_width( ui_Label2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label2,"Scan");

ui_TextArea2 = lv_textarea_create(ui_Container1);
lv_obj_set_width( ui_TextArea2, 374);
lv_obj_set_height( ui_TextArea2, LV_SIZE_CONTENT);   /// 70
lv_obj_set_x( ui_TextArea2, 20 );
lv_obj_set_y( ui_TextArea2, 124 );
lv_textarea_set_placeholder_text(ui_TextArea2,"Placeholder...");
lv_textarea_set_one_line(ui_TextArea2,true);
lv_textarea_set_password_mode(ui_TextArea2, true);
lv_obj_remove_flag( ui_TextArea2, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_TextArea2, LV_SCROLLBAR_MODE_OFF);

ui_Label3 = lv_label_create(ui_Container1);
lv_obj_set_width( ui_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label3, 20 );
lv_obj_set_y( ui_Label3, 100 );
lv_label_set_text(ui_Label3,"WiFI Password");
lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Button2 = lv_button_create(ui_Container1);
lv_obj_set_width( ui_Button2, 100);
lv_obj_set_height( ui_Button2, 50);
lv_obj_set_x( ui_Button2, 149 );
lv_obj_set_y( ui_Button2, 179 );
lv_obj_set_align( ui_Button2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button2, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_remove_flag( ui_Button2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Label4 = lv_label_create(ui_Button2);
lv_obj_set_width( ui_Label4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label4, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label4,"Connect");

ui_Button3 = lv_button_create(ui_Container1);
lv_obj_set_width( ui_Button3, 100);
lv_obj_set_height( ui_Button3, 50);
lv_obj_set_x( ui_Button3, 37 );
lv_obj_set_y( ui_Button3, 178 );
lv_obj_set_align( ui_Button3, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button3, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_remove_flag( ui_Button3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Label5 = lv_label_create(ui_Button3);
lv_obj_set_width( ui_Label5, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label5, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label5,"Cancel");

ui_Keyboard1 = lv_keyboard_create(ui_Screen1);
lv_obj_set_width( ui_Keyboard1, 480);
lv_obj_set_height( ui_Keyboard1, 200);
lv_obj_set_x( ui_Keyboard1, -1 );
lv_obj_set_y( ui_Keyboard1, 120 );
lv_obj_set_align( ui_Keyboard1, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Keyboard1, LV_OBJ_FLAG_HIDDEN );   /// Flags

lv_obj_add_event_cb(ui_TextArea2, ui_event_TextArea2, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Keyboard1, ui_event_Keyboard1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Dropdown1, ui_event_Dropdown1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Button1, ui_event_Button1, LV_EVENT_CLICKED, NULL);
}
