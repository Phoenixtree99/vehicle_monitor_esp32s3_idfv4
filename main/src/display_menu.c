#include "display_menu.h"
#include <stdio.h>

#include "lvgl/demos/lv_demos.h"
#include "lvgl_helpers.h"
#include "lvgl/src/hal/lv_hal_tick.h"

/*-------------------- GLOABLE VERIABLE START --------------------*/

lv_obj_t *cont_temperature;         // temperature cont容器
lv_obj_t *label_temperature;        // temperature lable标签

lv_obj_t *cont_humidity;            // humidity cont容器
lv_obj_t *label_humidity;           // humidity lable标签

lv_obj_t *cont_position;            // latitude cont容器
lv_obj_t *label_position;           // latitude lable标签

lv_obj_t *cont_position_status;     // longitude cont容器
lv_obj_t *label_position_status;    // longitude lable标签

lv_obj_t *cont_speed;               // speed cont容器
lv_obj_t *label_speed;              // speed lable标签

lv_obj_t *cont_sub_beidounum;       // sub_beidounum cont容器
lv_obj_t *label_sub_beidounum;      // sub_beidounum lable标签

lv_obj_t *cont_sub_gpsnum;          // sub_gpsnum cont容器
lv_obj_t *label_sub_gpsnum;         // sub_gpsnum lable标签

lv_coord_t ui_Chart2_series_1_array[] = {0,0,0,0,0,0,0,0,0,0};
lv_coord_t ui_Chart2_series_2_array[] = {0,0,0,0,0,0,0,0,0,0};

lv_chart_series_t* ui_Chart2_series_1;
lv_chart_series_t* ui_Chart2_series_2;

lv_obj_t *ui_Chart2;                // chart对象

extern char show_str[64];

extern int temperature;
extern int humidity;

/*--------------------- GLOABLE VERIABLE END ---------------------*/

void menu_init(void)
{
    lv_obj_t *cont;  // cont容器
    lv_obj_t *label; // lable标签

    /* ------------------------------------------------------------------------- */
    
    /*1-1 Create a menu object*/
    lv_obj_t *menu = lv_menu_create(lv_scr_act());                               // 创建菜单对象
    lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL)); // 设置大小
    lv_obj_center(menu);                                                         // 居中显示
    
    /* ------------------------------------------------------------------------- */

    /* 2-1 温湿度详情 sub pages */

    lv_obj_t *sub_1_page = lv_menu_page_create(menu, "Temperature & Humidity"); // 创建Page子菜单界面

    cont = lv_menu_cont_create(sub_1_page);
    ui_Chart2 = lv_chart_create(sub_1_page);
    lv_obj_set_width( ui_Chart2, 230);
    lv_obj_set_height( ui_Chart2, 140);
    // lv_obj_set_x( ui_Chart2, 0 );
    // lv_obj_set_y( ui_Chart2, 12 );
    lv_obj_set_align( ui_Chart2, LV_ALIGN_CENTER );
    lv_chart_set_type( ui_Chart2, LV_CHART_TYPE_LINE);
    lv_chart_set_range( ui_Chart2, LV_CHART_AXIS_PRIMARY_Y, -15, 45);
    lv_chart_set_axis_tick( ui_Chart2, LV_CHART_AXIS_PRIMARY_X, 10, 10, 10, 1, true, 50);
    lv_chart_set_axis_tick( ui_Chart2, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 5, 5, true, 50);
    lv_chart_set_axis_tick( ui_Chart2, LV_CHART_AXIS_SECONDARY_Y, 10, 5, 5, 5, true, 25);
    ui_Chart2_series_1 = lv_chart_add_series(ui_Chart2, lv_color_hex(0xFF0000), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_ext_y_array(ui_Chart2, ui_Chart2_series_1, ui_Chart2_series_1_array);
    ui_Chart2_series_2 = lv_chart_add_series(ui_Chart2, lv_color_hex(0x0000FF), LV_CHART_AXIS_SECONDARY_Y);
    lv_chart_set_ext_y_array(ui_Chart2, ui_Chart2_series_2, ui_Chart2_series_2_array);

    /* ------------------------------------------------------------------------- */

    /*2-2 BDS-GPS可见卫星数 sub pages */
    lv_obj_t *sub_2_page = lv_menu_page_create(menu, "Visible Satellites");

    cont = lv_menu_cont_create(sub_2_page);
    label = lv_label_create(cont);
    cont_sub_beidounum = cont;
    label_sub_beidounum = label;
    lv_label_set_text(label, "Beidou : --");

    cont = lv_menu_cont_create(sub_2_page);
    label = lv_label_create(cont);
    cont_sub_gpsnum = cont;
    label_sub_gpsnum = label;
    lv_label_set_text(label, "GPS : --");

    /* ------------------------------------------------------------------------- */

    /*2-3 Create sub pages*/
    lv_obj_t *sub_3_page = lv_menu_page_create(menu, "aaaa");

    cont = lv_menu_cont_create(sub_3_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Speed:0");

    /* ------------------------------------------------------------------------- */

    /*3-1 Modify the header*/
    lv_obj_t *back_btn = lv_menu_get_main_header_back_btn(menu); // 获取菜单头部返回键
    lv_obj_t *back_btn_label = lv_label_create(back_btn);        // 在返回键上创建label
    lv_label_set_text(back_btn_label, "Back");                   // 设置label显示内容

    /* ------------------------------------------------------------------------- */

    // /*Create a main page*/
    // cont = lv_menu_cont_create(main_page);               // 创建菜单主界面
    // label = lv_label_create(cont);                       // 创建菜单cont容器对象
    // lv_label_set_text(label, "Speed:0");      // 设置label显示内容
    // lv_menu_set_load_page_event(menu, cont, sub_3_page); // 加载cont到menu,设置跳转界面

    /* ------------------------------------------------------------------------- */

    /* 4-1 温度栏 */
    lv_obj_t *main_page = lv_menu_page_create(menu, NULL); // 创建菜单主界面

    cont = lv_menu_cont_create(main_page);               // 创建菜单cont容器对象
    label = lv_label_create(cont);                       // 创建label
    cont_temperature = cont;
    label_temperature = label;
    memset(show_str, '\0', sizeof(show_str));
    sprintf(show_str, "Temperature : %d", temperature);
    lv_label_set_text(label, show_str);
    lv_menu_set_load_page_event(menu, cont, sub_1_page); // 加载cont到menu,设置跳转界面

    /* ------------------------------------------------------------------------- */

    /* 4-2 湿度栏 */
    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    cont_humidity = cont;
    label_humidity = label;
    memset(show_str, '\0', sizeof(show_str));
    sprintf(show_str, "Huminity : %d", humidity);
    lv_label_set_text(label, show_str);
    lv_menu_set_load_page_event(menu, cont, sub_1_page);

    /* ------------------------------------------------------------------------- */

    /* 4-3 定位状态栏 */
    cont = lv_menu_cont_create(main_page);               // 创建菜单主界面
    label = lv_label_create(cont);                       // 创建菜单cont容器对象
    cont_position_status = cont;
    label_position_status = label;
    memset(show_str, '\0', sizeof(show_str));
    sprintf(show_str, "Positioning status : %s", "invalid");
    lv_label_set_text(label, show_str);
    lv_menu_set_load_page_event(menu, cont, sub_2_page); // 加载cont到menu,设置跳转界面

    /* ------------------------------------------------------------------------- */

    /* 4-4 定位栏 */
    cont = lv_menu_cont_create(main_page);               // 创建菜单主界面
    label = lv_label_create(cont);                       // 创建菜单cont容器对象
    cont_position = cont;
    label_position = label;
    memset(show_str, '\0', sizeof(show_str));
    sprintf(show_str, "Position : %3.5f, %3.5f", 000.00000, 000.00000);
    lv_label_set_text(label, show_str);
    // lv_menu_set_load_page_event(menu, cont, sub_3_page); // 加载cont到menu,设置跳转界面

    /* ------------------------------------------------------------------------- */

    /* 4-5 速度栏 */
    cont = lv_menu_cont_create(main_page);               // 创建菜单主界面
    label = lv_label_create(cont);                       // 创建菜单cont容器对象
    cont_speed = cont;
    label_speed = label;
    memset(show_str, '\0', sizeof(show_str));
    sprintf(show_str, "Speed : %s", "--");
    lv_label_set_text(label, show_str);
    // lv_menu_set_load_page_event(menu, cont, sub_3_page); // 加载cont到menu,设置跳转界面

    /* ------------------------------------------------------------------------- */

    /* 5-1 设置菜单主界面 */
    lv_menu_set_page(menu, main_page);

    /* ------------------------------------------------------------------------- */
}