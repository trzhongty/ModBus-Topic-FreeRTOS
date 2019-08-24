# ModBus-Topic-FreeRTOS
基于FreeRTOS，订阅分发式ModBus协议栈，支持RTU/ASCII/TCP，多格式，采用函数指针来对接，灵活多变。

#设置
在modbus.h //移植相关//中，可以设定ModBus从机任务占用的堆栈空间、任务优先级、接收缓冲长度
#define MODBUS_SLAVETASK_STACK_SIZE			512u
#define MODBUS_SLAVETASK_PRIORITY_NUM		3u
#define MODBUS_SLAVETASK_BUFF_SIZE			512u

#ModBus规则
所有读写通讯分为4种寄存器：
输入寄存器     //只读
保持寄存器     //读写
离散输入       //开关型、线圈类制度
离散输出       //开关型、线圈类读写

这四种分别对应着不同的ModBus功能码读写指令，像

#作为从机使用
通过以下接口来创建ModBus从机，该函数会自动创建一个任务，用于检测指定收发函数下的ModBus数据包并作出响应，分发至订阅列表
//创建从机 任务，控制接收、解析、分发、回复
ModBus_Slave_TypeDef * ModBus_CreateSlave( ModBus_SlaveInit_TypeDef *hSlaveInit );

以下为传入结构体定义：
//创建一个从机模式任务，传入参数结构体
typedef struct
{
	ModBus_RxPort_TypeDef		fRxPort;		//读取数据包接口，由该函数创建空间，并返回长度
	ModBus_TxPort_TypeDef		fTxPort;		//硬件接口层，用于回复
	TickType_t	timeOutTick;			//限定处理超时时间
	ModBus_Type_enum msgType;			//消息格式
	
}ModBus_SlaveInit_TypeDef;
输入参数结构体指针所指向的内容，需要包含ModBus数据包接收函数、发送函数、一次通讯超时时间、以及消息格式（初版只有RTU）

返回ModBus_Slave_TypeDef 结构体指针，即ModBus从机句柄。

为了实现具体通讯内容，还必须订阅具体的地址、寄存器类型、寄存器地址、长度等信息。才能使ModBus从机正常分发任务。
以下为添加订阅接口：
//对指定从机任务，添加订阅链表信息
int ModBus_AddTopic( ModBus_Slave_TypeDef *hSlave, ModBus_TopicLink_TypeDef *hTopicLink );

具体示例如下：
以下示例创建了一个从机，并分别订阅了4个输入寄存器，1个保持寄存器。这5个寄存器读写函数是同一个，4个输入寄存器地址通过for循环累加循环订阅。
		//初始化流程
		//ModBusRTU 从机初始化
		hSlaveInit.fRxPort = &JCZ_GM3RxModBusPack;
		hSlaveInit.fTxPort = &JCZ_GM3Tx;
		hSlaveInit.msgType = ModBus_TYPE_RTU;
		hSlaveInit.timeOutTick = JCZ_MODBUS_NET_OUTTIME_TICK;
		hJCZ.modbusSlave = ModBus_CreateSlave( &hSlaveInit );
		if( hJCZ.modbusSlave == NULL )
			break;
		
		//注册数据点
		//输入类型
		hTopicLink.dataMode = MODBUS_DATA_MODE_CONTAIN;
		hTopicLink.dataType = MODBUS_DATA_TPYE_INPUT;
		hTopicLink.pReadFun = &JCZ_DataPointRead;
		hTopicLink.pWriteFun = NULL;
		hTopicLink.regLen = 1;
		hTopicLink.slaveId = 1;
		hTopicLink.pPext = NULL;
		for( hTopicLink.regAddrStart = 0; hTopicLink.regAddrStart < 4; ++(hTopicLink.regAddrStart) )
		{
			ModBus_AddTopic( hJCZ.modbusSlave, &hTopicLink );
		}
		//保持类型
		hTopicLink.dataType = MODBUS_DATA_TPYE_HOLD;
		hTopicLink.pWriteFun = &JCZ_DataPointWrite;
		hTopicLink.regAddrStart = 0;
		ModBus_AddTopic( hJCZ.modbusSlave, &hTopicLink );
    
订阅类型不仅局限于单个寄存器，也可以是连续多个寄存器，并且可以设定访问模式
//数据点读写模式
typedef enum
{
	MODBUS_DATA_MODE_ALL 			= 0,	//任意，只要有订阅范围内即可
	MODBUS_DATA_MODE_STRICT 	= 1,	//严格模式，必须是订阅限定的起始寄存器及其长度
	MODBUS_DATA_MODE_CONTAIN 	= 2,	//必须要指令寄存器范围大于订阅范围
}ModBus_DataMode_enum;
按照寄存器实际功能选择模式，从而避免跨功能指令的错误响应。
对于示例连续的只读寄存器，有对应相同内存序列的，可以用同一组读写函数订阅，通过数组访问的形式统一操作。

#作为主机使用
作为主机，无需创建从机任务。仅通过一个函数即可实现整个通讯任务
//主机形式，下发指令
int ModBus_MainTxCom( ModBus_SlaveInit_TypeDef *hSlaveInit,ModBus_Msg_TypeDef *hModBusMsg);

需要事先准备好参数，包括读写接口函数、超市时间的 ModBus_SlaveInit_TypeDef。
准备好指令信息，包括从机地址、功能码、寄存器地址、长度、数据等等，
包含在hModBusMsg->parse中。

示例如下：	
ModBus_Msg_TypeDef hModBusMsg = 
{
  .packData = NULL,
  .packLen = NULL,
  .parse = (0),
};
ModBus_SlaveInit_TypeDef hInitRs485 = 
{
  .fRxPort = &RS485_RxPack,
  .fTxPort = &RS485_TxPack,
  .timeOutTick = JCZ_MODBUS_RS485_OUTTIME_TICK,
  .msgType = ModBus_TYPE_RTU,
};
//访问地址为02，输入寄存器，从00~02的数据，
hModBusMsg.parse.slaveId = 0x02u;
hModBusMsg.parse.funCode = MODBUS_FUN_READ_INPUT;
hModBusMsg.parse.regAddrStart = 0x00u;
hModBusMsg.parse.regLen = 0x02u;

RS485_CleanRxData();    //这是输入输出函数接口驱动函数，与本协议框架无关。清空接收，防止干扰
result = ModBus_MainTxCom( &hInitRs485, &hModBusMsg );

上述示例，通过从机读取指定寄存器信息，最终结果会存储在：
hModBusMsg.parse.pValue 中。

可以对hModBusMsg，结构体纹丝不动，或稍加修改，再次通过ModBus_MainTxCom接口发送到其他ModBus通讯接口上。
实现解析转发、逐级上报的效果。
