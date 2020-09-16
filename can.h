#ifndef __CAN_H
#define __CAN_H

/* DSP库 */
#include "math.h"
#include "arm_math.h"

#include "stm32h7xx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

/*********************************** CANopen从站字典与部分全局参数 **************************************************************************************/
/*
 * RPDO1 1600H-1h 映射对象：60FFH-0h 电机速度（编码器单位）4个字节
 * RPDO2 1601H-1h 映射对象：607EH-0h 电机极性 1个字节
 * RPDO3 1602H-1h 映射对象：6083H-0h 电机加速度（编码器单位）4个字节
 * RPDO3 1602H-2h 映射对象：6084H-0h 电机减速度（编码器单位）4个字节
 * RPDO4 1603H-1h 映射对象：607FH-0h 电机最大速度限制（编码器单位）4个字节
 *
 * TPDO1 1A00H-1h 映射对象：606CH-0h 电机速度反馈（编码器单位）4个字节
 * TPDO1 1A00H-2h 映射对象：6063H-0h 电机位置反馈（编码器单位）4个字节
 * TPDO2 1A01H-1h 映射对象：6040H-0h 控制字 2个字节
 * TPDO2 1A01H-2h 映射对象：6041H-0h 状态字 2个字节
 * TPDO2 1A01H-3h 映射对象：4080H-0h 母线电压反馈（0.1V单位）2个字节
 * TPDO2 1A01H-4h 映射对象：6078H-0h 电流反馈（0.1A单位）2个字节
 */
/*************一些重要的全局变量和宏定义*******************/
/* 舵轮相关的重要参数 */
#define AGV_MOTOR_NUMBER				4
#define DRIVE_RESOLUTION  				((uint32_t)(2500*4))		//驱动电机分辨率：2500线*4 = 10000
#define ROTATE_RESOLUTION      			((uint32_t)(2500*4))        //转向电机分辨率：2500线*4 = 10000
#define DRIVE_REDUCTION_RAT 			((float)1/21)				//驱动轮减速比
#define ROTATE_REDUCTION_RAT 			((float)1/19.5)				//转向轮减速比
#define ROTATE_GEAR_RAT					((float)20/110)				//转向齿轮比
/* 舵轮电机加、减速度 */
#define FRONT_DRIVE_ACCELERATED			((uint32_t)0x4E20)			//前舵轮驱动电机加速度
#define FRONT_DRIVE_DECELERATION		((uint32_t)0x9C40)
#define FRONT_ROTATE_ACCELERATED		((uint32_t)0x105D74)		//TODO: 【lyh-crj】前舵轮转向电机加速度:360°/s2
#define FRONT_ROTATE_DECELERATION		((uint32_t)0x105D74)		//DEBUG:0xBA3 0x4175D 0x5747C 0x6D19B 0x82EBA  0x98BD9 0xAE8F8 0x105D74 0x00
#define BACK_DRIVE_ACCELERATED			((uint32_t)0x4E20)
#define BACK_DRIVE_DECELERATION			((uint32_t)0x9C40)
#define BACK_ROTATE_ACCELERATED			((uint32_t)0x4E20)
#define BACK_ROTATE_DECELERATION		((uint32_t)0x9C40)
#define MAXSPEED						((uint32_t)0x7A120)			//电机的速度上限是3000r/min	对应的就是500000cnt/s
#define RESET_SPEED_RAD					((float)PI/9)				//上电复位速度
/* 速度环增益相关宏定义 */
#define SPEED_GAIN_PROPORTION1	 		((uint16_t)0x0320)			//速度环比例增益1:8.00
#define SPEED_GAIN_INTEGRAL1 	 		((uint16_t)0x0064)			//速度环积分常数1:10.0
#define SPEED_GAIN_PROPORTION2	 		((uint16_t)0x03E8)			//速度环比例增益2:10.00
#define SPEED_GAIN_INTEGRAL2	 		((uint16_t)0x0064)			//速度环积分常数1:10.0
#define GAIN_CUT_DELAY	 				((uint16_t)0x0004)			//速度环增益切换延迟时间:4ms
#define GAIN_CUT_TIME	 				((uint16_t)0x0004)			//速度环增益切换过程时间:4ms
#define GAIN_CUT_SPEED	 				((uint16_t)0x000A)			//速度环增益切换速度:10rpm
#define GAIN_CUT_ERR_RANGE	 			((uint16_t)0x0002)			//速度环增益切换速度回差:2rpm

#pragma  pack(push)  	//保存对齐状态
#pragma  pack(1)		//改为1字节对齐，防止内存对齐导致的错误

/* 舵轮反馈参数相关全局变量 */
typedef struct
{
	int frontDriveSite;			//前驱动电机绝对位置(编码器单位)
	int frontDriveSpeed;		//前驱动电机编码器速度
	int frontRotateSite;		//前转向电机绝对位置(编码器单位)
	int frontRotateSpeed;		//前转向电机编码器速度
	int backDriveSite;			//后驱动电机绝对位置(编码器单位)
	int backDriveSpeed;			//后驱动电机编码器速度
	int backRotateSite;			//后转向电机绝对位置(编码器单位)
	int backRotateSpeed;		//后转向电机编码器速度
	uint16_t controlFlag[4];	//控制字  1-4号电机按：前驱动、前转向、后驱动、后转向排
	uint16_t stateFlag[4];		//状态字
	uint16_t busVoltage[4];		//母线电压
	uint16_t busCurrent[4];		//母线电流
}SteeringWheelDataStruct;

typedef struct
{
	uint8_t is_exist;			//0x00：未启用/故障  --0x01：启用
	float frontDriveSite;		//前驱动电机绝对位置(编码器单位)
	float frontDriveSpeed;		//前驱动电机编码器速度
	float frontRotateSite;		//前转向电机绝对位置(编码器单位)
	float frontRotateSpeed;		//前转向电机编码器速度
	float backDriveSite;		//后驱动电机绝对位置(编码器单位)
	float backDriveSpeed;		//后驱动电机编码器速度
	float backRotateSite;		//后转向电机绝对位置(编码器单位)
	float backRotateSpeed;		//后转向电机编码器速度
	uint16_t controlFlag[4];	//控制字  1-4号电机按：前驱动、前转向、后驱动、后转向排
	uint16_t stateFlag[4];		//状态字
	uint16_t busVoltage[4];		//母线电压
	uint16_t busCurrent[4];		//母线电流
}UDPWheelDataStruct;


#pragma  pack(pop)		//恢复对齐状态


/* NMT上线报文格式：ID:COB_ID 数据：data(1byte：状态) */
/* NMT管理报文格式：ID:00H 数据：data(2bytes：NMT_CMD + NODE_ID) */
/* 节点守护报文格式：ID:COB_ID 数据：无（远程帧） */
/* 节点守护应答报文格式：ID:COB_ID 数据：NMT_STATE首位翻转（1byte） */
/* PDO通信参数SDO设置帧结构：ID:COB_ID 数据：CMD + INDEX + Sub_Index + data（8bit—32bit）*/
/* PDO映射参数SDO设置帧结构：ID:COB_ID 数据：CMD + INDEX + Sub_Index + data（32bit：映射对象长度 + 映射对象子索引 + 映射对象索引）*/
/* PDO帧结构：ID:COB_ID 数据：data（8bytes） */
/* 定义标准报文的11比特标识符中高4比特为功能码，后7比特为节点号，重命名为通讯对象标识符（COB-ID）*/
/* COB_ID = FC_CODE + NODE_ID */
/* 枚举CANopen的功能码 */
typedef union
{
	uint32_t data;
	uint16_t data16;
	uint8_t dataArr[4];
}Data;

enum FC_CODE
{
	NMT_ID   			= 0x000,		//网络管理报文
	SYNC_ID  			= 0x080,		//同步对象
	EMERGENCY_ID 		= 0x080,		//紧急报文
	TIME 	 			= 0x100,		//主站发出，用于同步所有从站的内部时钟
	RPDO1_ID 			= 0x200,		//4个接收PDO，一般用于发送舵轮控制指令
	RPDO2_ID 			= 0x300,
	RPDO3_ID 			= 0x400,
	RPDO4_ID 			= 0x500,
	TPDO1_ID 			= 0x180,		//4个发送PDO，一般用于从站反馈信息
	TPDO2_ID 			= 0x280,
	TPDO3_ID 			= 0x380,
	TPDO4_ID 			= 0x480,
	TSDO_ID  			= 0x580,		//发送SDO，一般用于从站反馈配置结果
	RSDO_ID  			= 0x600,		//接收SDO，一般用于配置从站PDO对象参数
	NMT_ERR_CONTROL_ID 	= 0x700			//错误控制，如上线报文、心跳包、节点守护
};
/* 枚举舵轮CAN节点 */
enum NODE_ID
{
	SYNC_NODE 			= 0x00,			//同步报文用,非节点号！
	FRONT_DRIVE 		= 0x01,			//前驱动电机
	FRONT_ROTATE 		= 0x02,			//前转向电机
	BACK_DRIVE 			= 0x03,			//后驱动电机
	BACK_ROTATE 		= 0x04			//后转向电机
};

/*读写对象字典的命令字*/
enum CMD
{
	READ_DICT 			= 0x40,			//读字典的值
	REPLY_READ_32BIT 	= 0x43,			//应答读字典：32位数据
	REPLY_READ_16BIT 	= 0x4B,			//应答读字典：16位数据
	REPLY_READ_8BIT 	= 0x4F,			//应答读字典：8位数据
	WRITE_DICT_32BIT 	= 0x23,			//写字典：32位数据
	WRITE_DICT_16BIT 	= 0x2B,			//写字典：16位数据
	WRITE_DICT_8BIT 	= 0x2F,			//写字典：8位数据
	REPLY_WRITE_OK 		= 0x60,			//正常应答写字典
	REPLY_ERR 			= 0x80			//错误应答读写字典
};
/*裁剪对象字典，只包含常用的字典索引*/
enum INDEX
{
	/* CANopen通信对象组裁剪（不可映射） */
	AUTO_OPERATION 		= 0x1004,		//写入0x55AA,状态机上电自动进入操作模式
	SAVE_PARAMETER 		= 0x1010,		//保存所有参数
	RST_PARAMETER 		= 0x1011,		//重置所有参数
	RPDO1_COMMUNICATION = 0x1400,		//RPDO通信参数
	RPDO2_COMMUNICATION = 0x1401,
	RPDO3_COMMUNICATION = 0x1402,
	RPDO4_COMMUNICATION = 0x1403,
	RPDO1_MAP 			= 0x1600,		//RPDO映射参数
	RPDO2_MAP 			= 0x1601,
	RPDO3_MAP 			= 0x1602,
	RPDO4_MAP 			= 0x1603,
	TPDO1_COMMUNICATION = 0x1800,		//TPDO通信参数
	TPDO2_COMMUNICATION = 0x1801,
	TPDO3_COMMUNICATION = 0x1802,
	TPDO4_COMMUNICATION = 0x1803,
	TPDO1_MAP 			= 0x1A00,		//TPDO映射参数
	TPDO2_MAP 			= 0x1A01,
	TPDO3_MAP 			= 0x1A02,
	TPDO4_MAP 			= 0x1A03,

	/* 和利时提供的对象组裁剪（2000H-3FFFH） */
	/* 关于速度环增益切换的描述：
	 * 伺服中可以保持两种增益，可以根据预设的速度切换点进行增益切换
	 * 1.配置0x2019:设定为2-增益切换
	 * 2.配置切换速度，即达到某个转速之后触发增益的切换
	 * 3.设定切换延迟时间，即达到速度后延迟多久开始平滑切换
	 * 4.设定增益切换时间，即平滑过渡到另一个增益的时间
	 * 5.设定速度回差值，避免切换时候因为速度波动而增益频繁来回切换
	 * 6.判断条件：速度反馈值<=切换速度-回差 或 速度反馈值>=切换速度+回差*/
	GAIN_SET 			= 0x2019,		//速度增益设置：0-固定1增益；1-固定2增益；-1-关闭第1增益； 2-增益切换；-4-IO控制；其他-开启第2增益
	SPEED_GAIN_P1 		= 0x2087,		//速度环第1比例增益		设定单位是0.01，16进制的1代表0.01，即8.00等于800 = 0x320
	SPEED_GAIN_I1 		= 0x2088,		//速度环第1积分时间常数	设定单位是0.1ms，16进制的1代表0.1，即10.0等于100 = 0x64
	SPEED_GAIN_P2 		= 0x2089,		//速度环第2比例增益		设定单位是0.01，16进制的1代表0.01，即10.00等于800 = 0x3E8
	SPEED_GAIN_I2 		= 0x208A,		//速度环第2积分时间常数	设定单位是0.1ms，16进制的1代表0.1，即10.0等于100 = 0x64
	GAIN_SW_DELAY 		= 0x20B0,		//增益延迟时间			设定单位是ms	默认值4
	GAIN_SW_TIME 		= 0x20B1,		//增益切换时间      		设定单位是ms	默认值4
	GAIN_SW_SPEED 		= 0x20B2,		//增益切换速度分界点		设定单位是rpm	默认值10
	GAIN_SW_ERR_RANGE 	= 0x20B3,		//增益切换速度回差		设定单位是rpm	默认值2
//	SPEED_BACK 			= 0x400E,		//速度反馈值
//	DRIVE_SITE_BACK 	= 0x401D,		//驱动电机绝对位置
//	ROTATE_SITE_BACK 	= 0x401E,		//转向电机绝对位置
	TEMPERATURE 		= 0x4025,		//驱动器温度
	BUS_VOLTAGE 		= 0x4080,		//直流母线电压，0.1V为单位

	/* DIA 402 子协议对象组裁剪 (6000H-)*/
	ERR_CODE 			= 0x603F,		//伺服错误码
	CONTROL_FLAG 		= 0x6040,		//控制字，可切换状态机状态，使能伺服
	STATE_FLAG 			= 0x6041,		//状态字，可用于监控
	STOP_QUITE 			= 0x605A,		//快速停机，默认是0，自由停机
	MODE 				= 0x6060,		//模式选择,速度模式为3
	SITE_BACK 			= 0x6063,		//电机位置反馈，编码器单位
	SPEED_BACK 			= 0x606C,		//电机速度反馈，编码器单位
	BUS_CURRENT 		= 0x6078,		//电流反馈，0.1A为单位
	MOTOR_POLARITY 		= 0x607E,		//电机极性，控制电机正反转
	SPEED_LIMIT 		= 0x607F,		//最大速度限制
	ACCELERATED 		= 0x6083,		//轮廓加速度
	DECELERATION 		= 0x6084,		//轮廓减速度
	STOP_DECELERATION 	= 0x6085,		//停机轮廓减速度
	SPEED 				= 0x60FF		//目标速度
};

/* 枚举保存参数对象的子索引（数据为：0x65766173） */
enum SAVE_PARAMETER_SubIdex
{
	SAVE_ALL 			= 0x01,			//保存所有对象参数
	SAVE_COMMUNICATION 	= 0x02,			//保存通信对象参数
	SAVE_DIA402 		= 0x03			//保存子协议区对象参数
};

/* 枚举重置参数对象的子索引（数据为：0x64616F6C） */
enum RST_PARAMETER_SubIdex
{
	RST_ALL 			= 0x01,			//恢复所有对象参数
	RST_COMMUNICATION 	= 0x02,			//恢复通信对象参数
	RST_DIA402			= 0x03			//恢复子协议区对象参数
};

/* 枚举PDO通信参数子索引 */
enum PDO_COMMUNICATION_SubIndex
{
	ENTRIES_COUNT 		= 0x00,			//有效条目数量
	COB_ID 				= 0x01,			//发送/接收这个PDO的帧ID
	TRANSMISSION_TYPE 	= 0x02,			//发送类型
	LIMIT_TIME 			= 0x03,			//生产禁止约束时间（0.1ms），约束PDO的最小发送间隔，减少总线负担
	EVENT_TIME 			= 0x05,			//事件定时触发时间（ms），定时发送PDO，若为默认0，则为事件改变发送
	SYNC_START_VALUE 	= 0x06			//同步起始值，收到几个同步对象才可用触发PDO，我们的控制方案置为0
};
/* 枚举上述PDO通信的发送类型 */
enum PDO_TRANSMISSION_TYPE
{
	NOCYCSYNC 			= 0x00,			//非循环同步
	CYCSYNC 			= 0x01,			//循环同步（从1-240表示经过几个同步对象才发送TPDO或更新应用RPDO）
	LONG_SYNC 			= 0xFC,			//远程同步
	LONG_ASYNC 			= 0xFD,			//远程异步
	USER_ASYNC 			= 0xFE,			//异步，制造商特定事件（映射的数据发生改变时才发送或更新应用）
	DIA402_ASYNC 		= 0xFF			//异步，设备子协议特定事件（定时时间到发送）
};
/* 枚举PDO映射参数子索引 */
enum PDO_MAP_SubIndex
{
	OBJ_COUNT 			= 0x00,			//映射的对象个数，一个PDO可用映射8个对象
	OBJ1,								//映射对象1-8
	OBJ2,
	OBJ3,
	OBJ4,
	OBJ5,
	OBJ6,
	OBJ7,
	OBJ8
};
/* 枚举增益切换设置格式 */
enum GAIN_SET_TYPE
{
	CLOSE_GAIN1 		= -1,
	USE_GAIN1 			= 0x00,	//固定增益1
	USE_GAIN2 			= 0x01,	//固定增益2
	GAIN_SW				= 0x02	//增益切换
};
/* CIA402 状态机的切换：初始化->伺服无故障（上电自动到这步）->准备好->等待打开使能->运行 */
/* 枚举6040H控制字 */
enum CONTROL_FLAG_SubIndex
{
	CONTROL_NOERR 		= 0x00,			//伺服无故障
	CONTROL_STOP_QUITE 	= 0x02,			//快速停机
	CONTROL_READY 		= 0x06,			//伺服准备好
	CONTROL_WAIT_RUN 	= 0x07,			//等待打开伺服使能
	CONTROL_RUN 		= 0x0F,			//伺服运行
	CONTROL_RST_ERR 	= 0x80			//故障->伺服无故障
};
/* 枚举6041H状态字 */
enum STATE_FLAG_SubIndex
{
	STATE_INIT 			= 0x00,
	STATE_NOERR 		= 0x50,			//伺服无故障
	STATE_STOP_QUITE 	= 0x17,			//快速停机
	STATE_READY 		= 0x31,			//伺服准备好
	STATE_WAIT_RUN 		= 0x33,			//等待打开伺服使能
	STATE_RUN 			= 0x37,			//伺服运行
	STATE_ERR 			= 0x18,			//故障
	STATE_STOP_ERR 		= 0x1F			//故障停机
};
/* 枚举快速停机的类型 */
enum STOP_QUITE
{
	STOP_FREE 			= 0,			//自由停机，保持自由运行状态（默认为0）
	STOP_6084_FREE,						//以6084h设定的减速度斜坡停机，后自由运行
	STOP_6085_FREE,						//以6085h设定的减速度斜坡停机，后自由运行
	STOP_2007H_10h_FREE,				//转矩模式，不做参考
	STOP_6084 = 5,						//以6084h设定的减速度斜坡停机，后保持位置锁定
	STOP_6085,							//以6085h设定的减速度斜坡停机，后保持位置锁定
	STOP_2007H_10h						//转矩模式，不做参考
};
/* 模式选择，和利时的CANopen只支持速度模式 */
enum MODE
{
	PLACE_MODE 			= 0x01,			//轮廓位置模式
	SPEED_MODE 			= 0x03,			//轮廓速度模式（和利时支持）
	BACKZERO_MODE 		= 0x06,			//回零模式
	INTERPOLATION_MODE 	= 0x07			//插补模式
};
/* 枚举电机极性，用于控制电机正反转 */
enum POLARITY
{
	SPIN_POSITIVE 		= 0x00,			//电机正转（默认值）
	SPIN_NEGATIVE 		= 0x40,			//电机反转
};


/* 枚举节点守护设备状态 */
enum NMT_STATE
{
	BOOTUP 				= 0x00,			//启动状态
	STOPPED 			= 0x04,			//停止
	OPERATIONAL 		= 0x05,			//可操作
	PRE_OPERATIONAL 	= 0x7F			//预操作（正常上电自动切换到预操作）
};

/* 枚举NMT管理报文命令字 */
enum NMT_CMD
{
	NMT_CMD_OPERATIONAL 		= 0x01,			//可操作状态,管理后可使用SDO PDO
	NMT_CMD_STOP 				= 0x02,			//停止状态,管理后只能使用SDO
	NMT_CMD_PRE_OPERATIONAL 	= 0x80,			//预操作状态,管理后只能使用SDO
	NMT_CMD_NODE_RST 			= 0x81,			//重置节点
	NMT_CMD_COMMUNICATION_RST 	= 0x82			//重置通信
};

/* NMT管理报文 */
typedef struct
{
	enum NMT_CMD cmd;
	enum NODE_ID node_Id;
}NMTStruct;


/* 节点守护 (远程帧)*/
typedef struct
{
	enum FC_CODE id;
	enum NODE_ID node_Id;
}nodeStruct;

typedef union
{
	enum INDEX index;
	uint8_t indexArr[2];
}Index;

/* PDO对象通信参数设置报文格式（SDO） */
typedef struct
{
	enum CMD cmd;
	Index index;
	enum PDO_COMMUNICATION_SubIndex subIndex;
	Data data;
}pdoCommunicationStruct;

/* PDO映射参数设置报文数据部分 */
typedef struct
{
	Index index;
	uint8_t subIndex;
	uint8_t dataLen;
}pdoMapData;
/* PDO对象映射参数设置报文格式（SDO） */
typedef struct
{
	enum CMD cmd;
	Index index;
	enum PDO_COMMUNICATION_SubIndex subIndex;
	pdoMapData data;
}pdoMapStruct;

/* PDO通信数据部分格式 */
typedef union
{
	uint64_t data;
	uint32_t obj32[2];
	uint16_t obj16[4];
	uint8_t obj8[8];
}pdoData;
/* PDO通信报文格式（PDO）*/
typedef struct
{
	pdoData data;
}pdoStruct;

/* SDO通信数据部分格式 */
typedef union
{
	uint64_t data;
	uint32_t obj32[2];
	uint16_t obj16[4];
	uint8_t obj8[8];
}sdoData;

/* 枚举舵轮类型 */
enum WHEELTYPE
{
	FRONT_WHEEL = 0,		//前舵轮
	BACK_WHEEL				//后舵轮
};


/* 舵轮电机角度结构体 */
// 枚举有无角度偏移
enum WORK
{
	WORKDISABLE = 0x00,
	WORKENABLE = 0x01
};

//电机位置控制状态结构体
typedef struct
{
	int targetVal;		 	 //目标位置（脉冲），设置角度时记录
	float angleRad;			 //目标位置（角度）
	float speed;			 //电机当前速度（弧度制），设置速度时记录
	float agv_acceleration;	 // 加速度
	float agv_deceleration;  // 减速度
	enum NODE_ID motorNum;	 //发生动作的电机号
//	enum WORK speedLive;	 //速度控制使能标志
	enum WORK angelLive;	 //角度控制使能标志
	enum POLARITY direction; //方向，设置角度时记录
}WheelControlStruct;

/******************************************** 全局变量/函数声明 *****************************************/
/* 中断打开标志 */
#define FDCAN1_RX0_INT_ENABLE	1

/* CAN收发数据部分格式 */
typedef union
{
	uint64_t data64;
	int data32[2];
	uint16_t data16[4];
	uint8_t data8[8];
}canData;

typedef union
{
	uint16_t id16;
	uint8_t id8[2];
}canID;

//CAN句柄和数据缓冲
extern FDCAN_HandleTypeDef FDCAN1_Handler;
extern uint8_t can1RxRTR;			//CAN远程帧标志缓存
extern canID can1RxID;			    //CAN帧ID缓存
extern canData can1RxData;			//CAN数据帧缓存
extern canData can1TxData;		    //CAN数据帧缓存

extern FDCAN_HandleTypeDef FDCAN2_Handler;
extern uint8_t can21RxRTR;			//CAN远程帧标志缓存
extern canID can2RxID;			    //CAN帧ID缓存
extern canData can2RxData;			//CAN数据帧缓存
extern canData can2TxData;		    //CAN数据帧缓存

/* CAN初始化、收发函数声明 */
uint8_t FDCAN1_Mode_Init(uint16_t presc, uint8_t ntsjw, uint16_t ntsg1, uint8_t ntsg2, uint32_t mode);
uint8_t CAN1_Filter_Init(FDCAN_HandleTypeDef *hfdcan, uint32_t FilterIndex, uint32_t FilterID1, uint32_t FilterID2);
uint8_t FDCAN1_Send_Msg(uint8_t *msg, uint32_t len, uint32_t id);
uint8_t FDCAN1_Receive_Msg(uint8_t *buf);
uint8_t EmergencyDirective1(uint32_t id);
uint8_t FDCAN2_Mode_Init(uint16_t presc, uint8_t ntsjw, uint16_t ntsg1, uint8_t ntsg2, uint32_t mode);
uint8_t CAN2_Filter_Init(FDCAN_HandleTypeDef *hfdcan, uint32_t FilterIndex, uint32_t FilterID1, uint32_t FilterID2);
uint8_t FDCAN2_Send_Msg(uint8_t *msg, uint32_t len, uint32_t id);
uint8_t FDCAN2_Receive_Msg(uint8_t *buf);
uint8_t EmergencyDirective2(uint32_t id);

/* 舵轮控制相关重要变量 */
extern WheelControlStruct wheelControl[AGV_MOTOR_NUMBER];	/* 用于舵轮转向控制 */
extern SteeringWheelDataStruct sheerWheelData;	/* 电机反馈数据 */
extern int frontRotateFirstSite; /* 前轮上电复位记录位置 */
extern int backRotateFirstSite;/* 后轮上电复位记录位置 */

/* 过程信号量 */
extern SemaphoreHandle_t WheelInitSem_Handle;	/* 二值信号量句柄,用于同步舵轮上电与初始化映射 */
extern SemaphoreHandle_t WriteDictSem_Handle;	/* 创建二值信号量，用于SDO写字典完成同步 */
extern SemaphoreHandle_t WheelResetSem_Handle;	/* 创建二值信号量 */
extern SemaphoreHandle_t WheelControlSem_Handle;	/* 创建普通互斥量，用于舵轮控制权的保护，保证同一时刻只有一方线程可控 */


uint8_t SYSC1_Msg(void);
uint8_t SYSC2_Msg(void);
uint8_t NMT_Manage_Msg(enum NMT_CMD cmd, enum NODE_ID nodeId);
uint8_t Write_Dictionary(enum FC_CODE fcCode, enum NODE_ID nodeId, enum CMD cmd, enum INDEX index, uint8_t subIndex, uint32_t data);
uint8_t Read_Dictionary(enum FC_CODE fcCode, enum NODE_ID nodeId,enum INDEX index, uint8_t subIndex);
uint8_t Map_Dictionary(enum FC_CODE fcCode, enum NODE_ID nodeId, enum CMD cmd,enum INDEX index,enum PDO_MAP_SubIndex subIndex,pdoMapData data);
uint8_t SendPdo(enum FC_CODE fcCode, enum NODE_ID nodeId, pdoStruct data);
uint8_t WheelInit(void);
void PDO1_DataManage (canID *canRxID);
void PDO2_DataManage (canID *canRxID);
uint8_t NMT_Manage_Msg(enum NMT_CMD cmd, enum NODE_ID nodeId);
uint8_t Node_Guard_Msg(enum NODE_ID nodeId);
float SpeedToRad(enum NODE_ID motorNum, int speed);
int RadToSpeed(enum NODE_ID motorNum, float speedRad);
float AngelToRad(enum NODE_ID motorNum, int angel);
int RadToAngel(enum NODE_ID motorNum, float angelRad);
float RadToLinespeed(float speedRad);
float LinespeedToRad(float lineSpeed);
float Rotate_Speed_Contorl(float angel_rotate, float angel_aim);
uint8_t SetDrive(enum NODE_ID motorNum, float speedRad);
uint8_t SetDriveLineV(enum NODE_ID motorNum, float lineSpeed);
uint8_t SetRotate(enum NODE_ID motorNum, float speedRad, float angelRad);
uint8_t SetPolarity(enum NODE_ID motorNum, enum POLARITY polarity);
uint8_t MaxSpeed(enum NODE_ID motorNum, uint32_t speedVal);
uint8_t Acceleratrat(float acceleration);
uint8_t Decelerat(float deceleration);
uint8_t AcceAndDece(float acc);
//uint8_t AcceleratAndDecelerat(enum NODE_ID motorNum, uint32_t acceleration, uint32_t deceleration);
uint8_t SteeringWheelSet(enum WHEELTYPE wheel_choice, float power_w, float steering_w, float steering_rad);
uint8_t SteeringWheelReset(void);
uint8_t SteeringWheelReset_IO(void);
uint8_t SteeringWheelStop(void);
float RotateWheelRAD(enum WHEELTYPE wheel_choice);
float DriveWheelRAD(enum WHEELTYPE wheel_choice);

#endif
