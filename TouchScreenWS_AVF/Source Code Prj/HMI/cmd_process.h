/*! 
*  \file hmi_driver.h
*  \brief ָ����ദ��
*  \version 1.0
*  \date 2012-2018
*  \copyright ���ݴ�ʹ��Ƽ����޹�˾
*/


#ifndef _CMD_PROCESS_H
#define _CMD_PROCESS_H
#include "hmi_driver.h"
#include "stm32f10x.h"

#define NOTIFY_TOUCH_PRESS         0X01  //����������֪ͨ
#define NOTIFY_TOUCH_RELEASE       0X03  //�������ɿ�֪ͨ
#define NOTIFY_WRITE_FLASH_OK      0X0C  //дFLASH�ɹ�
#define NOTIFY_WRITE_FLASH_FAILD   0X0D  //дFLASHʧ��
#define NOTIFY_READ_FLASH_OK       0X0B  //��FLASH�ɹ�
#define NOTIFY_READ_FLASH_FAILD    0X0F  //��FLASHʧ��
#define NOTIFY_MENU                0X14  //�˵��¼�֪ͨ
#define NOTIFY_TIMER               0X43  //��ʱ����ʱ֪ͨ
#define NOTIFY_CONTROL             0XB1  //�ؼ�����֪ͨ
#define NOTIFY_READ_RTC            0XF7  //��ȡRTCʱ��
#define MSG_GET_CURRENT_SCREEN     0X01  //����ID�仯֪ͨ
#define MSG_GET_DATA               0X11  //�ؼ�����֪ͨ
#define NOTIFY_HandShake           0X55  //����֪ͨ

#define PTR2U16(PTR) ((((uint8 *)(PTR))[0]<<8)|((uint8 *)(PTR))[1])  //�ӻ�����ȡ16λ����
#define PTR2U32(PTR) ((((uint8 *)(PTR))[0]<<24)|(((uint8 *)(PTR))[1]<<16)|(((uint8 *)(PTR))[2]<<8)|((uint8 *)(PTR))[3])  //�ӻ�����ȡ32λ����


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
    kCtrlButton=0x10,                     //��ť
    kCtrlText,                            //�ı�
    kCtrlProgress,                        //������
    kCtrlSlider,                          //������
    kCtrlMeter,                            //�Ǳ�
    kCtrlDropList,                        //�����б�
    kCtrlAnimation,                       //����
    kCtrlRTC,                             //ʱ����ʾ
    kCtrlGraph,                           //����ͼ�ؼ�
    kCtrlTable,                           //���ؼ�
    kCtrlMenu,                            //�˵��ؼ�
    kCtrlSelector,                        //ѡ��ؼ�
    kCtrlQRCode,                          //��ά��
};

#pragma pack(push)
#pragma pack(1)                           //���ֽڶ���

typedef struct
{
    uint8    cmd_head;                    //֡ͷ

    uint8    cmd_type;                    //��������(UPDATE_CONTROL)    
    uint8    ctrl_msg;                    //CtrlMsgType-ָʾ��Ϣ������
    uint16   screen_id;                   //������Ϣ�Ļ���ID
    uint16   control_id;                  //������Ϣ�Ŀؼ�ID
    uint8    control_type;                //�ؼ�����

    uint8    param[256];                  //�ɱ䳤�Ȳ��������256���ֽ�

    uint8  cmd_tail[4];                   //֡β
}CTRL_MSG,*PCTRL_MSG;

#pragma pack(pop)

/*! 
*  \brief  ����֪ͨ
*/
void NOTIFYHandShake(void);

/*! 
*  \brief  ��Ϣ��������
*  \param msg ��������Ϣ
*  \param size ��Ϣ����
*/
void ProcessMessage( PCTRL_MSG msg, uint16 size );
/*! 
*  \brief  �����л�֪ͨ
*  \details  ��ǰ����ı�ʱ(�����GetScreen)��ִ�д˺���
*  \param screen_id ��ǰ����ID
*/
void NotifyScreen(uint16 screen_id);
/*! 
*  \brief  ���������¼���Ӧ
*  \param press 1���´�������3�ɿ�������
*  \param x x����
*  \param y y����
*/
void NotifyTouchXY(uint8 press,uint16 x,uint16 y);

/*! 
*  \brief  ��ť�ؼ�֪ͨ
*  \details  ����ť״̬�ı�(�����GetControlValue)ʱ��ִ�д˺���
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*  \param state ��ť״̬��0����1����
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8 state);
/*! 
*  \brief  �ı��ؼ�֪ͨ
*  \details  ���ı�ͨ�����̸���(�����GetControlValue)ʱ��ִ�д˺���
*  \details  �ı��ؼ����������ַ�����ʽ�·���MCU������ı��ؼ������Ǹ���ֵ��
*  \details  ����Ҫ�ڴ˺����н��·��ַ�������ת�ظ���ֵ��
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*  \param str �ı��ؼ�����
*/
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str);
/*!                                                                              
*  \brief  �������ؼ�֪ͨ                                                       
*  \details  ����GetControlValueʱ��ִ�д˺���                                  
*  \param screen_id ����ID                                                      
*  \param control_id �ؼ�ID                                                     
*  \param value ֵ                                                              
*/   
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value);
/*!                                                                              
*  \brief  �������ؼ�֪ͨ                                                       
*  \details  ���������ı�(�����GetControlValue)ʱ��ִ�д˺���                  
*  \param screen_id ����ID                                                      
*  \param control_id �ؼ�ID                                                     
*  \param value ֵ                                                              
*/    
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value);
/*! 
*  \brief  �Ǳ�ؼ�֪ͨ
*  \details  ����GetControlValueʱ��ִ�д˺���
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*  \param value ֵ
*/
void NotifyMeter(uint16 screen_id, uint16 control_id, uint32 value);
/*! 
*  \brief  �˵��ؼ�֪ͨ
*  \details  ���˵���»��ɿ�ʱ��ִ�д˺���
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*  \param item �˵�������
*  \param state ��ť״̬��0�ɿ���1����
*/
void NotifyMenu(uint16 screen_id, uint16 control_id, uint8  item, uint8  state);

/*! 
*  \brief  ѡ��ؼ�֪ͨ
*  \details  ��ѡ��ؼ��仯ʱ��ִ�д˺���
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*  \param item ��ǰѡ��
*/
void NotifySelector(uint16 screen_id, uint16 control_id, uint8  item);
/*! 
*  \brief  ��ʱ����ʱ֪ͨ����
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*/
void NotifyTimer(uint16 screen_id, uint16 control_id);
/*! 
*  \brief  ��ȡ�û�FLASH״̬����
*  \param status 0ʧ�ܣ�1�ɹ�
*  \param _data ��������
*  \param length ���ݳ���
*/
void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length);

/*! 
*  \brief  д�û�FLASH״̬����
*  \param status 0ʧ�ܣ�1�ɹ�
*/
void NotifyWriteFlash(uint8 status);
/*! 
*  \brief  ��ȡRTCʱ�䣬ע�ⷵ�ص���BCD��
*  \param year �꣨BCD��
*  \param month �£�BCD��
*  \param week ���ڣ�BCD��
*  \param day �գ�BCD��
*  \param hour ʱ��BCD��
*  \param minute �֣�BCD��
*  \param second �루BCD��
*/
void NotifyReadRTC(uint8 year,uint8 month,uint8 week,uint8 day,uint8 hour,uint8 minute,uint8 second);

#endif
