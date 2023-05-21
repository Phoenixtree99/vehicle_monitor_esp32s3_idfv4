#include "display_menu.h"
#include <stdio.h>

#include "lvgl/demos/lv_demos.h"
#include "lvgl_helpers.h"
#include "lvgl/src/hal/lv_hal_tick.h"

/*-------------------- GLOABLE VERIABLE START --------------------*/

lv_obj_t *cont_temperature;  // temperature cont容器
lv_obj_t *label_temperature; // temperature lable标签

lv_obj_t *cont_humidity;  // humidity cont容器
lv_obj_t *label_humidity; // humidity lable标签

lv_obj_t *cont_position;  // latitude cont容器
lv_obj_t *label_position; // latitude lable标签

lv_obj_t *cont_position_status;  // longitude cont容器
lv_obj_t *label_position_status; // longitude lable标签

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
    label = lv_label_create(cont);
    lv_label_set_recolor(label, true);							// 使能字符命令重新对字符上色
    //lv_obj_set_align(label, LV_FLEX_ALIGN_CENTER);			// 内容居中对齐
    lv_label_set_text(label, "#ff0000 Temperature : 23#");
    lv_obj_set_height(label, 20);
    //lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);		// 对齐到中心偏上

    cont = lv_menu_cont_create(sub_1_page);
    label = lv_label_create(cont);
    lv_label_set_recolor(label, true);
    lv_label_set_text(label, "#0000ff Humidity : 65#");

    /* ------------------------------------------------------------------------- */

    /*2-2 BDS-GPS可见卫星数 sub pages */
    lv_obj_t *sub_2_page = lv_menu_page_create(menu, "Visible Satellites");

    cont = lv_menu_cont_create(sub_2_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Beidou : 12");

    cont = lv_menu_cont_create(sub_2_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "GPS : 14");

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
    sprintf(show_str, "Pos: %3.5f, %3.5f",-999.99999, -999.99999);
    lv_label_set_text(label, show_str);
    lv_menu_set_load_page_event(menu, cont, sub_3_page); // 加载cont到menu,设置跳转界面

    /* ------------------------------------------------------------------------- */

    /* 5-1 设置菜单主界面 */
    lv_menu_set_page(menu, main_page);

    /* ------------------------------------------------------------------------- */
}