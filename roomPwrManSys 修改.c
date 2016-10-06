#include "roomPwrManSys.h"
#include "hal_io.h"
#include <string.h>
#include "hal_adc.h"
/********************************/
/* 协调器代码                   */
/********************************/
#if defined(ZDO_COORDINATOR)
uint8 descPkg[] = {
    0x03, DevIRPers, 0
};

static uint16 nodeNwkAddr[Devmax];
static uint8 nodeEndPoint[Devmax];

static uint8 irPersStatus = 0;
static uint8 illumStatus = 0;
static uint8 controlStatus = 0;
void roomPwrManSys_StaChgRt(struct ep_info_t *ep);
void roomPwrManSys_StaChgRt(struct ep_info_t *ep)
{
    // 寻找人体红外节点 
    descPkg[1] = DevIRPers;
    SendData(ep->ep, descPkg, 0xFFFF, CONTROL_ENDPOINT, sizeof(descPkg));
}
void roomPwrManSys_IncmRt(struct ep_info_t *ep, uint16 addr, uint8 endPoint, afMSGCommandFormat_t *msg);
void roomPwrManSys_IncmRt(struct ep_info_t *ep, uint16 addr, uint8 endPoint, afMSGCommandFormat_t *msg)
{
    //msg->Data[], msg->DataLength, msg->TransSeqNumber
    if((endPoint == CONTROL_ENDPOINT) && (msg->Data[0] == 0x03))
    {
        // endPoint: msg->Data[1], rCycle: msg->Data[2]
        // 将上一次寻找的节点地址和端点号保存起来
        nodeNwkAddr[descPkg[1]] = addr;
        nodeEndPoint[descPkg[1]] = msg->Data[1];
        // 准备寻找下一个节点
        descPkg[1] = descPkg[1] + 1;
        // 所有节点是否都已经寻找完毕?
        if(descPkg[1] < Devmax)
            SendData(ep->ep, descPkg, 0xFFFF, CONTROL_ENDPOINT, sizeof(descPkg));
    }
    else
    {
        if(addr == nodeNwkAddr[DevIllum])
        {
            // 接收到光照度传感器数据
            uint16 i = 0;
            memcpy(&i, msg->Data, 2);
            illumStatus = i > 800;
            HalUARTWrite(HAL_UART_PORT_0, msg->Data, 2);
        }
        else if(addr == nodeNwkAddr[DevIRPers])
        {
            // 接收到人体红外传感器数据
            irPersStatus = !!(msg->Data[0]);
        }
        if(nodeNwkAddr[DevExecuter] != 0xFFFF)
        {
            // 如果执行节点存在
            uint8 ctrl = 0;
            if(irPersStatus && illumStatus)
            
                ctrl = 1;
            // 如果灯光的当前状态与需要设置的状态不一样则发送数据
            if(controlStatus != ctrl)
                SendData(ep->ep, &ctrl, nodeNwkAddr[DevExecuter], nodeEndPoint[DevExecuter], 1);
            controlStatus = ctrl;
        }
    }
}
void roomPwrManSys_ToRt(struct ep_info_t *ep);
void roomPwrManSys_ToRt(struct ep_info_t *ep)
{
    // 超时函数,用于检查节点搜索是否完成
    // 如果没有完成,则继续搜索
    if(descPkg[1] < Devmax)
    {
        SendData(ep->ep, descPkg, 0xFFFF, CONTROL_ENDPOINT, sizeof(descPkg));
    }
}
void roomPwrManSys_ResAvbRt(struct ep_info_t *ep, RES_TYPE type, void *res);
void roomPwrManSys_ResAvbRt(struct ep_info_t *ep, RES_TYPE type, void *res)
{
    switch(type)
    {
    case ResInit:
        memset(nodeNwkAddr, 0xFF, sizeof(nodeNwkAddr));
        memset(nodeEndPoint, 0xFF, sizeof(nodeEndPoint));
        break;
    case ResUserTimer:
        break;
    case ResControlPkg:
        break;
    }
}
#else
/********************************/
/* 人体红外节点代码             */
/********************************/
#if defined(IRPERS_NODE)
#define SAFTY_IO_GROUP      0
#define SAFTY_IO_BIT        0
void sensorIRPersResAvailable(struct ep_info_t *ep, RES_TYPE type, void *res);
void sensorIRPersResAvailable(struct ep_info_t *ep, RES_TYPE type, void *res)
{
    if(type == ResInit)
    {
        HalIOSetInput(SAFTY_IO_GROUP, SAFTY_IO_BIT, Pull_Up);
        HalIOIntSet(ep->ep, SAFTY_IO_GROUP, SAFTY_IO_BIT, IOInt_Rising, 0);
    }
    //IO端口中断触发，中断源检测
    if(type == ResIOInt)
    {
        uint8 IRPersValue = 1;
        SendData(ep->ep, &IRPersValue, 0x0000, TRANSFER_ENDPOINT, sizeof(IRPersValue));
    }
}
void sensorIRPersTimeout(struct ep_info_t *ep);
void sensorIRPersTimeout(struct ep_info_t *ep)
{
    uint8 value = HalIOGetLevel(SAFTY_IO_GROUP, SAFTY_IO_BIT);
    SendData(ep->ep, &value, 0x0000, TRANSFER_ENDPOINT, sizeof(value)); 
}
#endif
/********************************/
/* 光照度节点代码               */
/********************************/
#if defined(ILLUM_NODE)
#define IllInit()  do { P0DIR &= ~0x02; P1DIR |= 0x40; P1_6 = 1; }while(0)
void sensorILLumResAvailable(struct ep_info_t *ep, RES_TYPE type, void *res);
void sensorILLumResAvailable(struct ep_info_t *ep, RES_TYPE type, void *res)
{
    if(type == ResInit)
    { IllInit();}   
}
void sensorILLumTimeout(struct ep_info_t *ep);
void sensorILLumTimeout(struct ep_info_t *ep)
{
    //uint16 LightValue = 256 - (HalAdcRead(0, HAL_ADC_RESOLUTION_14) >> 3);
    // 将AD值变换为光照度的100倍
    //LightValue = LightValue * 39;// * 10000 / 256;
    uint16 LightValue = 0 ;
    LightValue = HalAdcRead(1, HAL_ADC_RESOLUTION_8) * 39;
    if (LightValue > 800)
    { CLRBIT(P1, 6);}
    else SETBIT(P1, 6);
    SendData(ep->ep, &LightValue, 0x0000, TRANSFER_ENDPOINT, sizeof(LightValue));
}
#endif
/********************************/
/* 执行节点代码                 */
/********************************/
#if defined(EXECUTER_NODE)
#define ControlInit()   do { HalIOSetOutput(1,4);Control(0); } while(0)
#define Control(mask)   do { HalIOSetLevel(1,4,mask&0x01);} while(0)
void OutputExecuteBResAvailable(struct ep_info_t *ep, RES_TYPE type, void *res);
void OutputExecuteBResAvailable(struct ep_info_t *ep, RES_TYPE type, void *res)
{
    if(type == ResInit)
        ControlInit();
}
void outputExecuteB(struct ep_info_t *ep, uint16 addr, uint8 endPoint, afMSGCommandFormat_t *msg);
void outputExecuteB(struct ep_info_t *ep, uint16 addr, uint8 endPoint, afMSGCommandFormat_t *msg)
{
    //msg->Data[], msg->DataLength, msg->TransSeqNumber
    Control(msg->Data[0]);
    SendData(ep->ep, &msg->Data[0], 0x0000, TRANSFER_ENDPOINT, 1);
}
void outputExecuteBTimeout(struct ep_info_t *ep);
void outputExecuteBTimeout(struct ep_info_t *ep)
{
    uint8 value = P1 >> 4;
    SendData(ep->ep, &value, 0x0000, TRANSFER_ENDPOINT, sizeof(value));
}
#endif
#endif

struct ep_info_t funcList[] = {
#if defined(ZDO_COORDINATOR)
    {
        roomPwrManSys_StaChgRt,
        roomPwrManSys_IncmRt,
        roomPwrManSys_ToRt,
        roomPwrManSys_ResAvbRt,
        { DevPwrmanSys, 0, 3 },
    },
#else
# if defined(IRPERS_NODE)
    {
        NULL, NULL, sensorIRPersTimeout, sensorIRPersResAvailable,
        { DevIRPers, 0, 2 },                // type, id, refresh cycle
    },
# elif defined(ILLUM_NODE)
    {
        NULL, NULL, sensorILLumTimeout, sensorILLumResAvailable,
        { DevIllum, 0, 5 },                // type, id, refresh cycle
    },
# elif defined(EXECUTER_NODE)
    {
        NULL, outputExecuteB, outputExecuteBTimeout, OutputExecuteBResAvailable,
        { DevExecuter, 0, 7 },              // type, id, refresh cycle
    },
# else
#  error You must define one device
# endif
#endif
};

// 不能修改下面的内容!!!
const uint8 funcCount = sizeof(funcList) / sizeof(funcList[0]);
