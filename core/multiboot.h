/*
 * MultiBoot.h
 *
 *  Created on: 2022-11-1
 *      Author: duxingyu
 */

#ifndef MULTIBOOT_H_
#define MULTIBOOT_H_
typedef unsigned int u32;
typedef unsigned char u8;
typedef unsigned short u16;

#define FUNC_FLAG_DSP_CONTROL 0xD4
#define CMD_TYPE_RESTRUCT_BEGIN 0x0E
#define CORE_NUM_6678     8
#define TOTAL_NODE_NUM 4

#define  IPCGR_0_REGS            (*((volatile unsigned int *)(0x02620240)))
#define  IPCGR_1_REGS            (*((volatile unsigned int *)(0x02620244)))
#define  IPCGR_2_REGS            (*((volatile unsigned int *)(0x02620248)))
#define  IPCGR_3_REGS            (*((volatile unsigned int *)(0x0262024C)))
#define  IPCGR_4_REGS            (*((volatile unsigned int *)(0x02620250)))
#define  IPCGR_5_REGS            (*((volatile unsigned int *)(0x02620254)))
#define  IPCGR_6_REGS            (*((volatile unsigned int *)(0x02620258)))
#define  IPCGR_7_REGS            (*((volatile unsigned int *)(0x0262025C)))

#define CORE_0_MAGIC_ADDR          (*((volatile unsigned int *)(0X1087FFFC)))
#define CORE_1_MAGIC_ADDR       (*((volatile unsigned int *)(0X1187FFFC)))
#define CORE_2_MAGIC_ADDR          (*((volatile unsigned int *)(0X1287FFFC)))
#define CORE_3_MAGIC_ADDR          (*((volatile unsigned int *)(0X1387FFFC)))
#define CORE_4_MAGIC_ADDR          (*((volatile unsigned int *)(0X1487FFFC)))
#define CORE_5_MAGIC_ADDR          (*((volatile unsigned int *)(0X1587FFFC)))
#define CORE_6_MAGIC_ADDR          (*((volatile unsigned int *)(0X1687FFFC)))
#define CORE_7_MAGIC_ADDR          (*((volatile unsigned int *)(0X1787FFFC)))


#define CORE0_C_INI00_ADDR 0x80566C40
#define CORE1_C_INI00_ADDR 0xA0004C00
#define CORE2_C_INI00_ADDR 0xA8004C00
#define CORE3_C_INI00_ADDR 0xB0004C00
#define CORE4_C_INI00_ADDR 0xC0004C00
#define CORE5_C_INI00_ADDR 0xD0004C00
#define CORE6_C_INI00_ADDR 0xE0004C00
#define CORE7_C_INI00_ADDR 0xE8004C00

#define KICK0   (*((volatile unsigned int *)(0x02620038)))
#define KICK1   (*((volatile unsigned int *)(0x0262003C)))

#define KICK0_UNLOCK     (0x83E70B13)
#define KICK1_UNLOCK     (0x95A4F1E0)
#define KICK_LOCK             (0x1)

#define  RESTRUCT_START_CMD 0x90000000


#define SYNC_HEADER 0xEB
#define DEVICE_CODE_REPLY 0x90
#define DEVICE_CODE_DEMO  0x9A //demonstration node
#define DEVICE_CODE_CMD_CROSS_DEVICE 0x5E
#define DEVICE_CODE_CMD_CROSS_DSP 0x6A


#define DATA_LENGTH_DSP_STATUS 14

#define DATA_LENGTH_ALL_CORE_WORKS 71


#define DATA_LENGTH_DSP_STATUS_WORK_FEED 8

#define DEVICE_FLAG_MANAGER01  0x01
#define DEVICE_FLAG_MANAGER02  0x02
#define DEVICE_FLAG_WORK_NODE1 0xA1
#define DEVICE_FLAG_WORK_NODE2 0xA2

#define NODE_NUM_MANAGER01  0x01
#define NODE_NUM_MANAGER02  0x02
#define NODE_NUM_WORK_NODE1 0x03
#define NODE_NUM_WORK_NODE2 0x04


#define DEVICE_STATUS_SUSPEND  0
#define DEVICE_STATUS_RUNNING  0x1
#define DEVICE_STATUS_RESETING 0x2
#define DEVICE_STATUS_FAULT    0x3


#define FUNC_FLAG_RUNNING_STATUS 0xD1
#define FUNC_FLAG_TASK_FEAD_BACK 0xD2

#define FUNC_FLAG_PIC_DEAL_SOBEL     0xD3
#define FUNC_FLAG_RELOAD_NAVIGATION  0xD4 //
#define FUNC_FLAG_RELOAD_PROGRAME    0xD5
#define FUNC_FLAG_PIC_DEAL_THRESHOLD 0xD6

#define FUNC_FLAG_CMD_TAKEOFF 0xC0
#define FUNC_FLAG_CMD_RESET   0xC1
#define FUNC_FLAG_CMD_START   0xC2
#define FUNC_FLAG_CMD_CLOSE   0xC3

#define FUNC_FLAG_CMD_PERCEPTION_NODE_BUG 0xC4 //ganzhi node (pic)
#define FUNC_FLAG_CMD_PROCESSING_NODE_BUG 0xC5 //
#define FUNC_FLAG_CMD_MANAGER_NODE_BUG     0xC7

#define FUNC_FLAG_CMD_WAKEUP 0xC8


#define FUNC_FLAG_CMD_REALTIME  0xC9
#define FUNC_FLAG_CMD_USER_SPEC 0xCA
#define FUNC_FLAG_CMD_DEVICE_OFFLINE 0xCC

#define FUNC_FLAG_CMD_BUG_INPUT 0xCF

#define FUNC_FLAG_ORIGIN_PICTURE  0x3D
#define FUNC_FLAG_NAVIGATION_DATA 0x4D
#define FUNC_FLAG_RESTRUCT_DATA  0x5D

#define FUNC_FLAG_CMD_FLAG 0xC0

#define PACKAGE_FLAG_FISRT_PACKAGE   0xA5
#define PACKAGE_FLAG_MID_PACKAGE     0xAB
#define PACKAGE_FLAG_LAST_PACKAGE    0xAF
#define PACKAGE_FLAG_SINGLE_PACKAGE  0xAA
#define PACKAGE_DATA_LENGTH 0x400 // 1024 bytes


#define WORK_NODE_TASK_DATA_LENGTH 22
struct ARGS_S {
    unsigned int Entry_adress;
    unsigned int WriteAddr;
    unsigned char corenum;
    unsigned char dspNum;
    unsigned char boardNum;
    unsigned char reserved;
};

typedef struct StrucSendCmd {
    u8 syncHeader;
    u8 deviceID;
    u16 cmdLength;
    u8 deviceFlag; //DSP2 0x8E
    u8 funcFlag; // DSP control
    u8 coreNum;
    u8 cmdType; //0E restruct begin,
    u32 entranceAddress;
    u32 crcValue;
} SendCmd;


typedef union struct_DSPRunStatus {
    u32 dspStatusValue[2];
    struct {
        /* dsp[0]*/
        /* Manage dsp 0 */
        u32 dsp0Core0 : 2;
        u32 dsp0Core1 : 2;
        u32 dsp0Core2 : 2;
        u32 dsp0Core3 : 2;
        
        u32 dsp0Core4 : 2;
        u32 dsp0Core5 : 2;
        u32 dsp0Core6 : 2;
        u32 dsp0Core7 : 2;
        
        /* Manage dsp 1 */
        u32 dsp1Core0 : 2;
        u32 dsp1Core1 : 2;
        u32 dsp1Core2 : 2;
        u32 dsp1Core3 : 2;
        
        u32 dsp1Core4 : 2;
        u32 dsp1Core5 : 2;
        u32 dsp1Core6 : 2;
        u32 dsp1Core7 : 2;
        
        /* dsp[1]*/
        /* Work dsp 1 */
        u32 work1DspCore0 : 2;
        u32 work1DspCore1 : 2;
        u32 work1DspCore2 : 2;
        u32 work1DspCore3 : 2;
        
        u32 work1DspCore4 : 2;
        u32 work1DspCore5 : 2;
        u32 work1DspCore6 : 2;
        u32 work1DspCore7 : 2;
        
        /* Work dsp 2 */
        u32 work2DspCore0 : 2;
        u32 work2DspCore1 : 2;
        u32 work2DspCore2 : 2;
        u32 work2DspCore3 : 2;
        
        u32 work2DspCore4 : 2;
        u32 work2DspCore5 : 2;
        u32 work2DspCore6 : 2;
        u32 work2DspCore7 : 2;        
    } BitValue;
} DSPRunStatus;


typedef union struct_WorkNodeRunStatus {
    u32 dspStatusValue;
    struct {
        /* dsp[1]*/
        /* Work dsp 1 */
        u32 work1DspCore0 : 2;
        u32 work1DspCore1 : 2;
        u32 work1DspCore2 : 2;
        u32 work1DspCore3 : 2;
        
        u32 work1DspCore4 : 2;
        u32 work1DspCore5 : 2;
        u32 work1DspCore6 : 2;
        u32 work1DspCore7 : 2;
        
        /* Work dsp 2 */
        u32 work2DspCore0 : 2;
        u32 work2DspCore1 : 2;
        u32 work2DspCore2 : 2;
        u32 work2DspCore3 : 2;
        
        u32 work2DspCore4 : 2;
        u32 work2DspCore5 : 2;
        u32 work2DspCore6 : 2;
        u32 work2DspCore7 : 2;
    } BitValue;
} WorkNodeRunStatus;

typedef union struct_RunStatusOneNode {
    u16 dspStatusValue;
    struct {
        u16 workStatusCore0 : 2;
        u16 workStatusCore1 : 2;
        u16 workStatusCore2 : 2;
        u16 workStatusCore3 : 2;
        
        u16 workStatusCore4 : 2;
        u16 workStatusCore5 : 2;
        u16 workStatusCore6 : 2;
        u16 workStatusCore7 : 2;
    } BitValue;
} RunStatusOneNode;


typedef union struct_TaskStatuesPerCore {
    u16 taskStatus;
    struct {
        u8 dspNum  : 4;
        u8 nodeNum : 4;
        u8 taskNum;
    } BitValue;
} TaskStatuesPerCore;

#pragma pack(1)

typedef struct struct_StatusRet {
    u8 syncHeader;
    u8 deviceID;
    u16 cmdLength;
    u8 deviceFlag;
    u8 funcFlag;
    DSPRunStatus dspStatus;
    u32 crcValue;
} StatusRet;


typedef struct struct_WorkNodeStatusRet {
    u8 syncHeader;
    u8 deviceID;
    u16 cmdLength;
    u8 deviceFlag;
    u8 funcFlag;
    RunStatusOneNode dspStatus;
    u32 crcValue;
} WorkNodeStatusRet;

typedef struct struct_WorkNodeTaskStsRet {
    u8 syncHeader;
    u8 deviceID;
    u16 cmdLength;
    u8 deviceFlag;
    u8 funcFlag;
    TaskStatuesPerCore taskStatus[8];
    u32 crcValue;
} WorkNodeTaskStsRet;

typedef struct struct_AllNodeTaskStsRet {
    u8 syncHeader;
    u8 deviceID;
    u16 cmdLength;
    u8 deviceFlag;
    u8 funcFlag;
    u8 taskNum;
    TaskStatuesPerCore taskStatus[32];
    u32 crcValue;
} AllNodeTaskStsRet;

#pragma pack()

void Load_Core_app_Start(unsigned int EntryAddr, int corenum);
void CoreStart(unsigned int EntryAddr, int corenum);
void ResetCore(u32 corenum);

#endif /* MULTIBOOT_H_ */
