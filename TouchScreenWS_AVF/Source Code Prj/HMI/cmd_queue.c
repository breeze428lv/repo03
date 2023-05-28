/************************************版权申明********************************************
**                             广州大彩光电科技有限公司
**                             http://www.gz-dc.com
**-----------------------------------文件信息--------------------------------------------
** 文件名称:   cmd_queue.c
** 修改时间:   2018-05-18
** 文件说明:   用户MCU串口驱动函数库
** 技术支持：  Tel: 020-82186683  Email: hmi@gz-dc.com Web:www.gz-dc.com
--------------------------------------------------------------------------------------

--------------------------------------------------------------------------------------
使用必读
cmd_queue.c中共5个函数：清空指令数据queue_reset()、从串口添加指令数据queue_push()、
从队列中取一个数据queue_pop().获取队列中有效数据个数queue_size()、从指令队列中取出一条完整的指令queue_find_cmd（）
若移植到其他平台，需要修改底层寄存器设置,但禁止修改函数名称，否则无法与HMI驱动库(hmi_driver.c)匹配。
--------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------*/
#include "cmd_queue.h"
#include "ulitity.h"
#include "macros.h"
#include "hmi_driver.h"
#define CMD_HEAD 0XEE                                                  //帧头
#define CMD_TAIL 0XFFFCFFFF                                            //帧尾

typedef struct _QUEUE                                             
{                                                                 
    qsize _head;                                                       //队列头
    qsize _tail;                                                       //队列尾
    qdata _data[QUEUE_MAX_SIZE];                                       //队列数据缓存区
}QUEUE;                                                           

static QUEUE que = {0,0,0};                                            //指令队列
static uint32 cmd_state = 0;                                           //队列帧尾检测状态
static qsize cmd_pos = 0;                                              //当前指令指针位置


#ifdef USE_MODBUS

extern uint8 ModBusTrafficTimer;
#endif
/*! 
*  \brief  清空指令数据
*/
void queue_reset()
{
    que._head = que._tail = 0;
    cmd_pos = cmd_state = 0;
}
/*! 
* \brief  添加指令数据
* \detial 串口接收的数据，通过此函数放入指令队列 
*  \param  _data 指令数据
*/
void queue_push(qdata _data)
{
    qsize pos = (que._head+1)%QUEUE_MAX_SIZE;
    if(pos!=que._tail)                                                //非满状态
    {
        que._data[que._head] = _data;
        que._head = pos;
    }
}

//从队列中取一个数据
static void queue_pop(qdata* _data)
{
    if(que._tail!=que._head)                                          //非空状态
    {
        *_data = que._data[que._tail];
        que._tail = (que._tail+1)%QUEUE_MAX_SIZE;
    }
}

//获取队列中有效数据个数
static qsize queue_size()
{
    return ((que._head+QUEUE_MAX_SIZE-que._tail)%QUEUE_MAX_SIZE);
}
/*! 
*  \brief  从指令队列中取出一条完整的指令
*  \ cmd_state is a 4 byte detection window to capture cmd tail
*  \param  cmd 指令接收缓存区
*  \param  buf_len 指令接收缓存区大小
*  \return  指令长度，0表示队列中无完整指令
*/
qsize queue_find_cmd(qdata *buffer,qsize buf_len)
{
    qsize cmd_size = 0;
    qdata _data = 0;

    while(queue_size()>0)
    {
        //pop 1 byte from queue to _data
        queue_pop(&_data);

        if(cmd_pos == 0&&_data != CMD_HEAD)                               //指令第一个字节必须是帧头，否则跳过
        {
// 		 printf("----**************************WRONG HEAD!!!!!!\n");
            continue;
        }
        //    LED2_ON;
        if(cmd_pos < buf_len)                                           //防止缓冲区溢出
            buffer[cmd_pos++] = _data;
///////////////OVERFLOW!!!///////////////////////////////
		else
		{
//		       printf("----**************************OVERFLOW!!!!\n");

			cmd_state = 0;                                            //重新检测帧尾巴
            cmd_pos = 0;                                              //复位指令指针
			return 0;
		}
			
////////////////////////////////////////////////

        cmd_state = ((cmd_state<<8)|_data);                           //shift 1 byte in 拼接最后4个字节，组成一个32位整数

        //最后4个字节与帧尾匹配，得到完整帧
        if(cmd_state==CMD_TAIL)									 //whenever find a match for CMD_TAIL, return
        {
            //LED2_ON;
            cmd_size = cmd_pos;                                       //指令字节长度
            cmd_state = 0;                                            //重新检测帧尾巴
            cmd_pos = 0;                                              //复位指令指针

#if(CRC16_ENABLE)
            //去掉指令头尾EE，尾FFFCFFFF共计5个字节，只计算数据部分CRC
            if(!CheckCRC16(buffer+1,cmd_size-5))                      //CRC校验
                return 0;

            cmd_size -= 2;                                            //去掉CRC16（2字节）
#endif
            return cmd_size;
        }
    }
    return 0;                                                         //没有形成完整的一帧
}
