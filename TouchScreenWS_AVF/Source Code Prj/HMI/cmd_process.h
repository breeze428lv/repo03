/*! 
*  \file hmi_driver.h
*  \brief 指令分类处理
*  \version 1.0
*  \date 2012-2018
*  \copyright 广州大彩光电科技有限公司
*/


#ifndef _CMD_PROCESS_H
#define _CMD_PROCESS_H
#include "hmi_driver.h"
#include "stm32f10x.h"

#define NOTIFY_TOUCH_PRESS         0X01  //触摸屏按下通知
#define NOTIFY_TOUCH_RELEASE       0X03  //触摸屏松开通知
#define NOTIFY_WRITE_FLASH_OK      0X0C  //写FLASH成功
#define NOTIFY_WRITE_FLASH_FAILD   0X0D  //写FLASH失败
#define NOTIFY_READ_FLASH_OK       0X0B  //读FLASH成功
#define NOTIFY_READ_FLASH_FAILD    0X0F  //读FLASH失败
#define NOTIFY_MENU                0X14  //菜单事件通知
#define NOTIFY_TIMER               0X43  //定时器超时通知
#define NOTIFY_CONTROL             0XB1  //控件更新通知
#define NOTIFY_READ_RTC            0XF7  //读取RTC时间
#define MSG_GET_CURRENT_SCREEN     0X01  //画面ID变化通知
#define MSG_GET_DATA               0X11  //控件数据通知
#define NOTIFY_HandShake           0X55  //握手通知

#define PTR2U16(PTR) ((((uint8 *)(PTR))[0]<<8)|((uint8 *)(PTR))[1])  //从缓冲区取16位数据
#define PTR2U32(PTR) ((((uint8 *)(PTR))[0]<<24)|(((uint8 *)(PTR))[1]<<16)|(((uint8 *)(PTR))[2]<<8)|((uint8 *)(PTR))[3])  //从缓冲区取32位数据


/*
//#define SCREEN_START              0
#define SCREEN_MAIN               1
#define SCREEN_MANUAL_CTR         14

#define SCREEN_START                                             0
#define SCREEN_MAIN_1                                            1
#define SCREEN_MANUAL_CTR_2                                      2
#define SCREEN_USER_SETTINGS_2                                   3
#define SCREEN_SYS_SETTINGS_2                                    4
#define SCREEN_FAILURE_INQUIRY_2                                 5
#define SCREEN_OPERATION_MANUAL                                  6
#define SCREEN_GROUP_SETTING_3                                   7
#define SCREEN_TEMP_SETTING_3                                    8
#define SCREEN_TIME_CTR_3                                        9
#define SCREEN_VALVE_SETTING_3                                   10
#define SCREEN_PUMP_SETTING_3                                    11
#define SCREEN_TIME_SETTING_3                                    12
#define SCREEN_PIN_SETTING_3                                     13
#define SCREEN_SCREEN_SETTING                                    14
#define SCREEN_SUPPLIER_INFO_3                                   15
#define SCREEN_MAIN_ANIMATION                                    16
#define SCREEN_PIN_CONTROL                                       17
#define SCREEN_PIN_CONTROL_FACTORY                               18
#define SCREEN_BALANCE_WARNING                                   19
#define SCREEN_PRESSURE_SETTING_3                                20

#define SCREEN_USER_PIN_MANAGER                                  22

#define SCREEN_PUMP_GROUPING                                     24
#define SCREEN_OUTLET_SENSOR                                     25
#define SCREEN_ENTRANCE_SENSOR                                   26
#define SCREEN_PUMP_SWITCH_CONDTION                              27
#define SCREEN_SLEEP_SETTING                                     28
#define SCREEN_POWER_UP_SETTING                                  29
#define SCREEN_VALVE_CONTROL                                     30
#define SCREEN_PID_SETTING                                       31


	   */
#define SCREEN_START                                             0
#define SCREEN_MAIN_1                                            1
#define SCREEN_MANUAL_CTR_2                                      2
#define SCREEN_USER_SETTINGS_2                                   3
#define SCREEN_SYS_SETTINGS_2                                    4
#define SCREEN_FAILURE_INQUIRY_2                                 5
#define SCREEN_OPERATION_MANUAL                                  6
#define SCREEN_TIME_CTR_3                                        7
#define SCREEN_TIME_SETTING_3                                    8
#define SCREEN_PIN_SETTING_3                                     9
#define SCREEN_SCREEN_SETTING                                    10
#define SCREEN_SUPPLIER_INFO_3                                   11
#define SCREEN_PIN_CONTROL                                       12
#define SCREEN_PIN_CONTROL_FACTORY                               13
#define SCREEN_BALANCE_WARNING                                   14
#define SCREEN_USER_PIN_MANAGER                           		 15
#define SCREEN_PUMP_GROUPING                                     16
#define SCREEN_OUTLET_SENSOR                                     17
#define SCREEN_ENTRANCE_SENSOR                                   18
#define SCREEN_PUMP_SWITCH_CONDTION                              19
#define SCREEN_SLEEP_SETTING                                     20
#define SCREEN_POWER_UP_SETTING                                  21
#define SCREEN_VALVE_CONTROL                                     22
#define SCREEN_PID_SETTING                                       23
#define SCREEN_CONTROL_CURVES                                    24
#define SCREEN_ENTRANCE_SENSOR0                                  25
#define SCREEN_ENTRANCE_SENSOR2                                  26
#define SCREEN_VALVE_CONTROL1                                    27
#define SCREEN_STOP_CONFIRM1                                     29
#define SCREEN_RTC_SETTING                                       30
#define SCREEN_PASSWORD_SELECT                                   31
#define SCREEN_TRAIL_DATE_SETTING                                32
#define SCREEN_FUNC_DBG_SWITCHES                                 34
#define SCREEN_PUMP_TEM_CONTROL                                  38
#define SCREEN_DEVICE_ADDRESS                                    40
#define SCREEN_MANUAL_CTR_AVF                                    41
#define SCREEN_OUTLET_CALI                                       42
#define SCREEN_ENTRANCE_CALI                                     43







enum CtrlType
{
    kCtrlUnknown=0x0,
    kCtrlButton=0x10,                     //按钮
    kCtrlText,                            //文本
    kCtrlProgress,                        //进度条
    kCtrlSlider,                          //滑动条
    kCtrlMeter,                            //仪表
    kCtrlDropList,                        //下拉列表
    kCtrlAnimation,                       //动画
    kCtrlRTC,                             //时间显示
    kCtrlGraph,                           //曲线图控件
    kCtrlTable,                           //表格控件
    kCtrlMenu,                            //菜单控件
    kCtrlSelector,                        //选择控件
    kCtrlQRCode,                          //二维码
};

#pragma pack(push)
#pragma pack(1)                           //按字节对齐

typedef struct
{
    uint8    cmd_head;                    //帧头

    uint8    cmd_type;                    //命令类型(UPDATE_CONTROL)    
    uint8    ctrl_msg;                    //CtrlMsgType-指示消息的类型
    uint16   screen_id;                   //产生消息的画面ID
    uint16   control_id;                  //产生消息的控件ID
    uint8    control_type;                //控件类型

    uint8    param[256];                  //可变长度参数，最多256个字节

    uint8  cmd_tail[4];                   //帧尾
}CTRL_MSG,*PCTRL_MSG;

#pragma pack(pop)

/*! 
*  \brief  握手通知
*/
void NOTIFYHandShake(void);

/*! 
*  \brief  消息处理流程
*  \param msg 待处理消息
*  \param size 消息长度
*/
void ProcessMessage( PCTRL_MSG msg, uint16 size );
/*! 
*  \brief  画面切换通知
*  \details  当前画面改变时(或调用GetScreen)，执行此函数
*  \param screen_id 当前画面ID
*/
void NotifyScreen(uint16 screen_id);
/*! 
*  \brief  触摸坐标事件响应
*  \param press 1按下触摸屏，3松开触摸屏
*  \param x x坐标
*  \param y y坐标
*/
void NotifyTouchXY(uint8 press,uint16 x,uint16 y);

/*! 
*  \brief  按钮控件通知
*  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param state 按钮状态：0弹起，1按下
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8 state);
/*! 
*  \brief  文本控件通知
*  \details  当文本通过键盘更新(或调用GetControlValue)时，执行此函数
*  \details  文本控件的内容以字符串形式下发到MCU，如果文本控件内容是浮点值，
*  \details  则需要在此函数中将下发字符串重新转回浮点值。
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param str 文本控件内容
*/
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str);
/*!                                                                              
*  \brief  进度条控件通知                                                       
*  \details  调用GetControlValue时，执行此函数                                  
*  \param screen_id 画面ID                                                      
*  \param control_id 控件ID                                                     
*  \param value 值                                                              
*/   
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value);
/*!                                                                              
*  \brief  滑动条控件通知                                                       
*  \details  当滑动条改变(或调用GetControlValue)时，执行此函数                  
*  \param screen_id 画面ID                                                      
*  \param control_id 控件ID                                                     
*  \param value 值                                                              
*/    
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value);
/*! 
*  \brief  仪表控件通知
*  \details  调用GetControlValue时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param value 值
*/
void NotifyMeter(uint16 screen_id, uint16 control_id, uint32 value);
/*! 
*  \brief  菜单控件通知
*  \details  当菜单项按下或松开时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param item 菜单项索引
*  \param state 按钮状态：0松开，1按下
*/
void NotifyMenu(uint16 screen_id, uint16 control_id, uint8  item, uint8  state);

/*! 
*  \brief  选择控件通知
*  \details  当选择控件变化时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param item 当前选项
*/
void NotifySelector(uint16 screen_id, uint16 control_id, uint8  item);
/*! 
*  \brief  定时器超时通知处理
*  \param screen_id 画面ID
*  \param control_id 控件ID
*/
void NotifyTimer(uint16 screen_id, uint16 control_id);
/*! 
*  \brief  读取用户FLASH状态返回
*  \param status 0失败，1成功
*  \param _data 返回数据
*  \param length 数据长度
*/
void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length);

/*! 
*  \brief  写用户FLASH状态返回
*  \param status 0失败，1成功
*/
void NotifyWriteFlash(uint8 status);
/*! 
*  \brief  读取RTC时间，注意返回的是BCD码
*  \param year 年（BCD）
*  \param month 月（BCD）
*  \param week 星期（BCD）
*  \param day 日（BCD）
*  \param hour 时（BCD）
*  \param minute 分（BCD）
*  \param second 秒（BCD）
*/
void NotifyReadRTC(uint8 year,uint8 month,uint8 week,uint8 day,uint8 hour,uint8 minute,uint8 second);

#endif
