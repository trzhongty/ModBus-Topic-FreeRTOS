/*======================================================================================*
 * 文件名   ：modbus.h
 * 描述     ：
 * 日期     ：2019年6月27日16:36:10
 * 作者     ：trzhongty@163.com

 * 修改记录 :
 *--------------------------------------------------------------------------------------
 * 版本		日期										作者								修改内容
 * V0.01  2019年6月27日16:38:07   trzhongty@163.com   1) 初次创建 
 * v0.02	2019年7月2日01:31:50		trzhongty@163.com		1) 对TxPort、RxPort进行了修改，增加
																											超时参数，进一步完善了从机超时机制
 *=====================================================================================*/

#ifndef __MODBUS_H__
#define __MODBUS_H__
#if defined(__cplusplus)
extern "C" {
#endif

/*======================================== 文件包含 =====================================*/
#include <string.h>

/*======================================== 移植相关 =====================================*/
//#include "main.h"
#include "freertos.h"
#include "semphr.h"
#include "task.h"

#define MODBUS_SLAVETASK_STACK_SIZE			512u		//从机任务分配堆栈大小
#define MODBUS_SLAVETASK_PRIORITY_NUM		3u			//任务优先级
//#define MODBUS_SLAVETASK_BUFF_SIZE			512u

////当处理完毕，需要回复响应时接口
//void __wifi_funreply( uint8_t * pdata, uint8_t num );
//#define ModBus_HAL_ReplyData( _replyData_, _replyLen_)		__wifi_funreply( _replyData_, _replyLen_)
////当数据包无法处理或者数据格式有问题，需要抛弃时回调
//#define ModBus_HAL_PackDump()								do{__wifi_packclear(); hwififun.state |= WIFI_FUN_STATE_ENDING_DONE;}while(0)							
//#define ModBus_HAL_FunParseStart()							FunWork_ModBus_FunParseStart()

/*======================================== 宏定义  ======================================*/
#define MODBUS_DIS_VALUE_ON			((uint16_t)(0xFF00u))
#define MODBUS_DIS_VALUE_OFF		((uint16_t)(0x0000u))

#define MODBUS_RTU_PACKLEN_MIN					8u	 	//RTU至少8字节
#define MODBUS_RTU_REPLY_PACKLEN_MIN		6u		//RTU回应字节最少6u
#define MODBUS_RTU_PACKLEN_READ_SIZE		8u		//RTU 读取指令，全部为8字节
#define MODBUS_RTU_PACKLEN_WRITE_SIZE		8u		//RTU 写单个指令，全部为8字节
#define MODBUS_RTU_REPLY_PACKLEN_ERRORACK_SIZE		5u

#define MODBUS_PACK_VERSION_DEF		((uint16_t)(0x0000u))		//第2、3个字节协议版本号
#define MODBUS_PACK_VERSION_DEF_H	( (uint8_t)(MODBUS_PACK_VERSION_DEF >> 8u) )
#define MODBUS_PACK_VERSION_DEF_L	( (uint8_t)(MODBUS_PACK_VERSION_DEF) )


#define MODBUS_PACKLEN_MIN		8u		//7个字节首，和1个字节功能码为最低要求，但小于12字节
#define MODBUS_FUN_READ_REMAINDERLEN_DEF		6u	//读取类，默认6个字节剩余长度
#define MODBUS_FUN_WRITE_REMAINDERLEN_DEF		6u	//单个写入类，默认6个字节剩余长度

#define MODBUS_PACK_NUM_PACKID_H			0u
#define MODBUS_PACK_NUM_PACKID_L			1u
#define MODBUS_PACK_NUM_VERSION_H		2u
#define MODBUS_PACK_NUM_VERSION_L		3u
#define MODBUS_PACK_NUM_REMAINDERLEN_H	4u
#define MODBUS_PACK_NUM_REMAINDERLEN_L	5u
#define MODBUS_PACK_NUM_SLAVEID			6u
#define MODBUS_PACK_NUM_FUNCODE			7u


#define MODBUS_DIS_DATA_ON_DEF			((uint16_t)0xFF00u)		//需要转换才能和大端顺序一致
#define MODBUS_DIS_DATA_OFF_DEF			((uint16_t)0x0000u)

/*======================================== 类型定义  ====================================*/

//支持的功能码枚举
typedef enum
{
	MODBUS_FUN_READ_DISOUT 				= 0x01u,
	MODBUS_FUN_READ_DISIN 				= 0x02u,
	MODBUS_FUN_READ_HOLD 					= 0x03u,
	MODBUS_FUN_READ_INPUT 				= 0x04u,
	MODBUS_FUN_WRITE_DISOUT 			= 0x05u,
	MODBUS_FUN_WRITE_HOLD 				= 0x06u,
	MODBUS_FUN_WRITE_DISOUT_MORE 	= 0x0Fu,
	MODBUS_FUN_WRITE_HOLD_MORE		= 0X10u,
	
	MODBUS_EXFUN_REPORT_DISOUT_MORE		= 0x45u,	//主动上报离散输出、离散输入、保持寄存器、输入寄存器
	MODBUS_EXFUN_REPORT_DISIN_MORE		= 0x42u,
	MODBUS_EXFUN_REPORT_HOLD_MORE			= 0x46u,
	MODBUS_EXFUN_REPORT_INPUT_MORE		= 0x44u
}ModBus_Fun_enum;

//不正常响应错误列表
typedef enum
{
	MODBUS_REACK_ERROR_NULL			= 0x00u,	//无错误
	MODBUS_REACK_ERROR_FUN			= 0x01u,
	MODBUS_REACK_ERROR_REG			= 0x02u,
	MODBUS_REACK_ERROR_DATA			= 0x03u,
	MODBUS_REACK_ERROR_DAMAGE		= 0x04u,
	MODBUS_REACK_ERROR_ACK			= 0x05u,
	MODBUS_REACK_ERROR_BUSY			= 0x06u,
	MODBUS_REACK_ERROR_NO				= 0x07u,
	MODBUS_REACK_ERROR_PARITY		= 0x08u
}ModBus_ErrorAck_enum;

//状态枚举
typedef enum
{
	ModBus_STATE_NULL	= 0,
	ModBus_STATE_READY	= 1,
	ModBus_STATE_BUSY	= 2,
}ModBus_state_enum;

//通讯格式类型枚举
typedef enum
{
	ModBus_TYPE_RTU 	= 0,
	ModBus_TYPE_ASCII = 1,
	ModBus_TYPE_TCP 	= 2,
}ModBus_Type_enum;

//数据点类型
typedef enum
{
	MODBUS_DATA_TPYE_NULL		= 0,	//未知类型
	MODBUS_DATA_TPYE_DISOUT = 1,
	MODBUS_DATA_TPYE_DISIN 	= 2,
	MODBUS_DATA_TPYE_HOLD 	= 3,
	MODBUS_DATA_TPYE_INPUT 	= 4,
}ModBus_DataType_enum;

//存储解析之后的数据
typedef struct
{
	uint16_t	*pValue;			//对于单个写入命令，所存储数值信息
	uint16_t	tcpId;				//数据包的前两个字节,通讯标号
	uint16_t	remainderLen;	//数据包剩余字节数
	uint16_t	regAddrStart;	//如果功能码为寄存器读写类，则将寄存器起始地址存放该处
	uint16_t	regLen;				//如果功能码为寄存器读写类，则将读写数量存放该处
	uint8_t		byteLen;			//对于多写入命令，所存储的数值占用字节数
	uint8_t		slaveId;			//数据包从机设备标识，从机地址
	uint8_t		funCode;			//功能码
	ModBus_DataType_enum	dataType;		//数据点类型
	ModBus_ErrorAck_enum	errorCode;	//异常代码
}ModBus_Parse_TypeDef;

//单个通讯Modbus结构体，
typedef struct
{
	uint8_t	*	packData;				//指向来源数据帧
	TickType_t	getTimeTick;	//获取数据时刻
	uint16_t	packLen;				//响应包总长度
	ModBus_Parse_TypeDef 		parse;
}ModBus_Msg_TypeDef;

//数据点读写模式
typedef enum
{
	MODBUS_DATA_MODE_ALL 			= 0,	//任意，只要有订阅范围内即可
	MODBUS_DATA_MODE_STRICT 	= 1,	//严格模式，必须是订阅限定的起始寄存器及其长度
	MODBUS_DATA_MODE_CONTAIN 	= 2,	//必须要指令寄存器范围大于订阅范围
}ModBus_DataMode_enum;

//数据点订阅链表
typedef struct ModBus_TopicLink_struct
{
	struct ModBus_TopicLink_struct *pPext;
	ModBus_ErrorAck_enum (*pWriteFun)(	const struct ModBus_TopicLink_struct *pTopicLink,
																			uint16_t	 *pValue,
																			uint16_t 		regAddrStart,
																			uint16_t 		regLen );
	ModBus_ErrorAck_enum (*pReadFun)(		const struct ModBus_TopicLink_struct *pTopicLink,
																			uint16_t	 *pValue,
																			uint16_t 		regAddrStart,
																			uint16_t 		regLen );
	uint16_t 		regAddrStart;
	uint16_t 		regLen;
	uint8_t			slaveId;
	ModBus_DataType_enum	dataType;
	ModBus_DataMode_enum	dataMode;
}ModBus_TopicLink_TypeDef;

//数据点写入与读取函数指针定义
//pValue由Modbus分配函数创建空间，对于离散型，其本质为uint8_t*，对于uint16_t类型，分配函数将会自动进行大小端转换
//离散写入和读取，pValue指向的值应该为 0xff00或者 0x0000。
typedef ModBus_ErrorAck_enum ( *ModBus_TopicWriteFun)( 	const struct ModBus_TopicLink_struct *pTopicLink,
																												uint16_t	*pValue,
																												uint16_t 		regAddrStart,
																												uint16_t 		regLen );
																			
typedef ModBus_ErrorAck_enum ( *ModBus_TopicReadFun)( const struct ModBus_TopicLink_struct *pTopicLink,
																											uint16_t	 *pValue,
																											uint16_t 		regAddrStart,
																											uint16_t 		regLen );

//硬件接口层，实现回应数据
typedef int(*ModBus_TxPort_TypeDef)(uint8_t *pData, uint16_t size, TickType_t xTicksToWait);
//硬件接口层，实现接收数据
typedef uint8_t *(*ModBus_RxPort_TypeDef)(uint16_t *packLen, TickType_t xTicksToWait);

//创建一个从机模式任务，传入参数结构体
typedef struct
{
	ModBus_RxPort_TypeDef		fRxPort;		//读取数据包接口，由该函数创建空间，并返回长度
	ModBus_TxPort_TypeDef		fTxPort;		//硬件接口层，用于回复
	TickType_t	timeOutTick;			//限定处理超时时间
	ModBus_Type_enum msgType;			//消息格式
	
}ModBus_SlaveInit_TypeDef;	

//从机模式，功能控制主结构体														
typedef struct
{
	SemaphoreHandle_t topicLinkMutex;						//链表互斥锁
	ModBus_TopicLink_TypeDef *pTopicLinkHand;		//链表头指针
	ModBus_SlaveInit_TypeDef init;
	ModBus_state_enum				state;
}ModBus_Slave_TypeDef;


/*======================================== 函数声明 =====================================*/

//创建从机 任务，控制接收、解析、分发、回复
ModBus_Slave_TypeDef * ModBus_CreateSlave( ModBus_SlaveInit_TypeDef *hSlaveInit );
//对指定从机任务，添加订阅链表信息
int ModBus_AddTopic( ModBus_Slave_TypeDef *hSlave, ModBus_TopicLink_TypeDef *hTopicLink );


//将多个离散型寄存器数量转化成所需要的数据字节长度
#define __ModBus_DisRegLenToByteSize( _regLen_ )	(( (_regLen_) >> 3u ) + (((_regLen_) & 0x07u)?( 1u ):( 0u )))

//大小端转换函数
void __ModBus_U16SmallToBig( uint16_t u16SmallData, uint8_t * pU16BigData );	
uint16_t __ModBus_U16BigToSmall( uint8_t * pU16BigData );

//主机形式，下发指令
int ModBus_MainTxCom( ModBus_SlaveInit_TypeDef *hSlaveInit,ModBus_Msg_TypeDef *hModBusMsg);


/*======================================== 变量声明 =====================================*/


#if defined(__cplusplus)
}
#endif
#endif	/* __ModBus_H__ */
