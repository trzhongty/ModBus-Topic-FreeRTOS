/*======================================================================================*
 * 文件名   ：modbus.c
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

/*======================================== 文件包含 ===================================*/
#include "modbus.h"

/*======================================== 移植相关 ===================================*/

/*======================================== 私有宏定义  ================================*/

/*======================================== 私有类型定义  ==============================*/

/*======================================== 私有函数声明 ===============================*/


/*======================================== 私有变量定义 ===============================*/


/*======================================== 接口变量定义 ===============================*/

/*======================================== 函数定义 ===================================*/
//以下为ModBusRTU CRC校验算法
/* CRC 高位字节值表 */
const uint8_t TblCRCHi[] = 
{
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
/* CRC低位字节值表*/
const uint8_t TblCRCLo[] = 
{
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

//生成的CRC高
static uint16_t CRC16( uint8_t *pMsg, uint8_t DataLen)
{
	uint16_t Index;        /* CRC循环中的索引  */
	uint8_t	CRCHi = 0xFF; /* 高CRC字节初始化  */
	uint8_t CRCLo = 0xFF; /* 低CRC字节初始化 */

	while (DataLen--)  /* 传输消息缓冲区 */
	{
		Index = CRCHi ^ *pMsg++ ;   /* 计算CRC */
		CRCHi = CRCLo ^ TblCRCHi[Index] ;
		CRCLo = TblCRCLo[Index] ;
	}
	return (((uint16_t)(CRCHi) << 8) | CRCLo);
}

/*************************************************
函数名称:__ModBus_U16BigToSmall

功能描述:将两个字节的字节流大端格式U16转换为小端u16

调用函数清单: 

本函数被调来源: 

输入参数说明: pU16BigData，大端格式两字节首地址，即高字节

输出参数说明: 

返回值说明: 小端格式u16

备注:
*************************************************/
uint16_t __ModBus_U16BigToSmall( uint8_t * pU16BigData )
{
	return ((( uint16_t )( *pU16BigData )) << 8u ) | (*(pU16BigData + 1));
}


/*************************************************
函数名称:__ModBus_U16SmallToBig

功能描述:将u16小端格式数值 转换为大端格式存储字节流

调用函数清单: 

本函数被调来源: 

输入参数说明: u16SmallData，小端格式数值

输出参数说明: pU16BigData，大端格式字节流首地址，不得为空

返回值说明: 小端格式u16

备注:
*************************************************/
void __ModBus_U16SmallToBig( uint16_t u16SmallData, uint8_t * pU16BigData )
{
	if( pU16BigData == NULL )
		return;
	*pU16BigData = (uint8_t)(u16SmallData >> 8u);
	*( ++pU16BigData) = (uint8_t)(u16SmallData);
}

/*************************************************
函数名称:ModBus_RtuAddCrc

功能描述:在输入字节流中结尾，按照ModBusRTU增加两个CRC字节.

调用函数清单: 

本函数被调来源: 

输入参数说明: uint8_t *pData
								输入字节流指针
							uint16_t usefulDataLen
								字节流中有效数据字节数，字节流实际存储长度应该比该值大2个字节

输出参数说明: 

返回值说明: 

备注:
*************************************************/
void ModBus_RtuAddCrc( uint8_t *pData, uint16_t usefulDataLen  )
{
	__ModBus_U16SmallToBig( CRC16( pData, usefulDataLen ), pData + usefulDataLen );
}


/*************************************************
函数名称:ModBus_CreatErrorAckPack

功能描述:根据消息结构体解析中的errorCode等信息，生成标准异常响应信息字节流返回

调用函数清单: 

本函数被调来源: 

输入参数说明:	ModBus_Msg_TypeDef * hModBusMsg, 
								包含功能码、异常码等信息
							ModBus_Type_enum msgType,
								消息格式
输出参数说明: uint16_t * packLen
								返回字节流的长度
返回值说明: 返回生成的消息字节流指针，需要由接受者自行释放

备注:
*************************************************/
uint8_t * ModBus_CreatErrorAckPack( ModBus_Msg_TypeDef * hModBusMsg, ModBus_Type_enum msgType, uint16_t * packLen)
{
	uint8_t *pData = NULL;
	switch( msgType )
	{
	case ModBus_TYPE_RTU:
		pData = pvPortMalloc( 5 );
		if( NULL == pData )
			break;
		pData[0] = hModBusMsg->parse.slaveId;
		pData[1] = hModBusMsg->parse.funCode | 0x80u;
		pData[2] = hModBusMsg->parse.errorCode;
		ModBus_RtuAddCrc( pData, 3 );
		*packLen = 5;
		break;
	case ModBus_TYPE_ASCII:
		break;
	case ModBus_TYPE_TCP:
		break;
	}
	return pData;
}


/*************************************************
函数名称:ModBus_Parse

功能描述:得到一个完整的ModBus数据帧，输入处理，对数据区有效性进行判断。
		无效格式数据将会直接抛弃。格式正确，但是功能码或数据有误的将会进行
		异常响应。

调用函数清单: 

本函数被调来源: 

输入参数说明:	ModBus_Msg_TypeDef * hModBusMsg 
								必须包含：packData指向数据内容，packLen数据长度，type数据类型
							ModBus_Type_enum msgType
								包含信息类型
							
输出参数说明: hModBusMsg->parse存储解析后的数据

返回值说明: 返回0表示解析成功，非零异常，解析失败，如有失败功能码，将在hModBusMsg->parse.errorCode中

备注:
*************************************************/
int ModBus_Parse( ModBus_Msg_TypeDef * hModBusMsg, ModBus_Type_enum msgType )
{
	int result = -1;
	uint16_t i;
	
	switch( msgType )
	{
	case ModBus_TYPE_RTU:
		//最小长度判断
		if( hModBusMsg->packLen < MODBUS_RTU_PACKLEN_MIN )
			break;
		//CRC校验
		if( 0 != CRC16( hModBusMsg->packData, hModBusMsg->packLen ))
			break;
		//开始分析
		hModBusMsg->parse.slaveId = (hModBusMsg->packData)[0];
		hModBusMsg->parse.funCode = (hModBusMsg->packData)[1];
		hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_NULL;		//数据类型重置为未知
		//匹配功能码
		switch( hModBusMsg->parse.funCode )
		{
		case MODBUS_FUN_READ_DISOUT:
			hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_DISOUT;
		case MODBUS_FUN_READ_DISIN:
			if( MODBUS_DATA_TPYE_NULL == hModBusMsg->parse.dataType )
				hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_DISIN;
		case MODBUS_FUN_READ_HOLD:
			if( MODBUS_DATA_TPYE_NULL == hModBusMsg->parse.dataType )
				hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_HOLD;
		case MODBUS_FUN_READ_INPUT:
			if( MODBUS_DATA_TPYE_NULL == hModBusMsg->parse.dataType )
				hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_INPUT;
			//长度判断
			if( MODBUS_RTU_PACKLEN_READ_SIZE != hModBusMsg->packLen )
				break;
			hModBusMsg->parse.regAddrStart = __ModBus_U16BigToSmall( (hModBusMsg->packData) + 2 );
			hModBusMsg->parse.regLen = __ModBus_U16BigToSmall( (hModBusMsg->packData) + 4 );
			//离散与寄存器数据宽度不同
			if( ( MODBUS_FUN_READ_DISOUT == hModBusMsg->parse.funCode ) || ( MODBUS_FUN_READ_DISIN == hModBusMsg->parse.funCode) )
			{
				hModBusMsg->parse.byteLen = __ModBus_DisRegLenToByteSize( hModBusMsg->parse.regLen );
			}
			else
			{
				hModBusMsg->parse.byteLen = hModBusMsg->parse.regLen * 2u;
			}
			//创建存储空间
			if( hModBusMsg->parse.pValue != NULL )
				vPortFree( hModBusMsg->parse.pValue );
			hModBusMsg->parse.pValue = pvPortMalloc( hModBusMsg->parse.byteLen );
			if( hModBusMsg->parse.pValue == NULL )
				break;		//待完善
			memset( hModBusMsg->parse.pValue, 0, hModBusMsg->parse.byteLen );
			
			result = 0;
			break;
			
		case MODBUS_FUN_WRITE_DISOUT:
			hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_DISOUT;
		case MODBUS_FUN_WRITE_HOLD:
			if( MODBUS_DATA_TPYE_NULL == hModBusMsg->parse.dataType )
				hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_HOLD;
			//长度判断
			if( MODBUS_RTU_PACKLEN_READ_SIZE != hModBusMsg->packLen )
				break;
			//创建数值存储区，长度为2个字节
			if( hModBusMsg->parse.pValue != NULL )
				vPortFree( hModBusMsg->parse.pValue );
			hModBusMsg->parse.pValue = pvPortMalloc( sizeof(uint16_t) );
			if( hModBusMsg->parse.pValue == NULL )
				break;		//待完善
			*(hModBusMsg->parse.pValue) = __ModBus_U16BigToSmall( (hModBusMsg->packData) + 4 );
			hModBusMsg->parse.regLen = 1;
			if( MODBUS_FUN_WRITE_DISOUT == hModBusMsg->parse.funCode )		//对于离散输出型，写入值有限定
			{
				if( ( *(hModBusMsg->parse.pValue) != MODBUS_DIS_VALUE_ON ) && ( *(hModBusMsg->parse.pValue) != MODBUS_DIS_VALUE_OFF ))
				{
					hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_DATA;		//数据范围异常响应
					break;
				}
			}
			
			result = 0;
			break;
			
		case MODBUS_FUN_WRITE_DISOUT_MORE:
			hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_DISOUT;
			hModBusMsg->parse.regLen = __ModBus_U16BigToSmall( (hModBusMsg->packData) + 4 );
			hModBusMsg->parse.byteLen = (hModBusMsg->packData)[ 6 ];
			if(	( hModBusMsg->parse.byteLen != 0 ) ||\
					((((int16_t)( hModBusMsg->parse.byteLen )) * 8u) < hModBusMsg->parse.regLen) ||\
					((((int16_t)( hModBusMsg->parse.byteLen - 1 )) * 8u) >= hModBusMsg->parse.regLen ))
			{
				//数据Byte区值，与对应数据长度不对。格式错误，忽视
				break;
			}
			//创建数值存储区，长度为byteLen个字节
			if( hModBusMsg->parse.pValue != NULL )
				vPortFree( hModBusMsg->parse.pValue );
			hModBusMsg->parse.pValue = pvPortMalloc( hModBusMsg->parse.byteLen );
			if( hModBusMsg->parse.pValue == NULL )
				break;		//待完善
			memcpy( hModBusMsg->parse.pValue, (hModBusMsg->packData) + 7, hModBusMsg->parse.byteLen );
			
			result = 0;
			break;
			
		case MODBUS_FUN_WRITE_HOLD_MORE:
			hModBusMsg->parse.dataType = MODBUS_DATA_TPYE_HOLD;
			hModBusMsg->parse.regLen = __ModBus_U16BigToSmall( (hModBusMsg->packData) + 4 );
			hModBusMsg->parse.byteLen = (hModBusMsg->packData)[ 6 ];
			if(	( hModBusMsg->parse.byteLen != 0 ) ||\
					( ( hModBusMsg->parse.byteLen ) & 0x01u ) ||\
					( ( ( (int16_t)( hModBusMsg->parse.byteLen )) >> 1u) != hModBusMsg->parse.regLen) )
			{
				//数据Byte区值，与对应数据长度不对。格式错误，忽视
				break;
			}
			//创建数值存储区，长度为byteLen个字节
			if( hModBusMsg->parse.pValue != NULL )
				vPortFree( hModBusMsg->parse.pValue );
			hModBusMsg->parse.pValue = pvPortMalloc( sizeof(uint16_t) * (hModBusMsg->parse.byteLen >> 0x01u) );
			if( hModBusMsg->parse.pValue == NULL )
				break;		//待完善
			for( i = 0; i < hModBusMsg->parse.regLen; ++i )
			{
				((uint16_t *)(hModBusMsg->parse.pValue))[i] = __ModBus_U16BigToSmall( (hModBusMsg->packData) + 7 + 2 * i );
			}
			
			result = 0;
			break;
		
		default:
			hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_FUN;		//不在支持的功能码列表内
			break;
		}
		break;
		
	case ModBus_TYPE_ASCII:
		break;
	case ModBus_TYPE_TCP:
		break;
	}
	
	return result;
}




/*************************************************
函数名称:ModBus_CreateReplyPack

功能描述:当ModBus指令被正常解析、处理之后，通过该函数生成正确的回复数据包，而后进行回复

调用函数清单: 

本函数被调来源: 

输入参数说明:	ModBus_Msg_TypeDef * hModBusMsg 
								必须包含：parse已解析数据，parse.pValue数值等
							ModBus_Type_enum msgType
								打包信息类型
							
输出参数说明: uint16_t * packLen
								返回字节流的长度

返回值说明: 返回生成的消息字节流指针，需要由接受者自行释放

备注:
*************************************************/
uint8_t *ModBus_CreateReplyPack( ModBus_Msg_TypeDef * hModBusMsg, ModBus_Type_enum msgType, uint16_t *packLen)
{
	uint8_t *pack = NULL;
	uint16_t i;
	*packLen = 0;
	
	switch( msgType )
	{
	case ModBus_TYPE_RTU:
		//匹配功能码
		switch( hModBusMsg->parse.funCode )
		{
		case MODBUS_FUN_READ_DISOUT:
		case MODBUS_FUN_READ_DISIN:
		case MODBUS_FUN_READ_HOLD:
		case MODBUS_FUN_READ_INPUT:
			//读取类结构类似
			*packLen = 5 + hModBusMsg->parse.byteLen;
			pack = pvPortMalloc( *packLen );
			if( pack == NULL )
				break;
			pack[ 0 ] = hModBusMsg->parse.slaveId;
			pack[ 1 ] = hModBusMsg->parse.funCode;
			pack[ 2 ] = hModBusMsg->parse.byteLen;
			if( hModBusMsg->parse.funCode == MODBUS_FUN_READ_DISOUT || hModBusMsg->parse.funCode == MODBUS_FUN_READ_DISIN )
			{
				memcpy( pack + 3, hModBusMsg->parse.pValue, hModBusMsg->parse.byteLen );
			}
			else
			{
				//对于寄存器数据，需要大小端转换
				for( i = 0; i < hModBusMsg->parse.byteLen; i += 2)
				{
					__ModBus_U16SmallToBig( hModBusMsg->parse.pValue[ i >> 1u ], pack + 3 + i );
				}
			}
			ModBus_RtuAddCrc( pack, *packLen - 2 );
		
			break;
			
		case MODBUS_FUN_WRITE_DISOUT:
		case MODBUS_FUN_WRITE_HOLD:
			//写单个结构体类似
			*packLen = 8;
			pack = pvPortMalloc( *packLen );
			if( pack == NULL )
				break;
			pack[ 0 ] = hModBusMsg->parse.slaveId;
			pack[ 1 ] = hModBusMsg->parse.funCode;
			__ModBus_U16SmallToBig( hModBusMsg->parse.regAddrStart , pack + 2 );
			__ModBus_U16SmallToBig( *(hModBusMsg->parse.pValue), pack + 4 );
			ModBus_RtuAddCrc( pack, *packLen - 2 );
		
			break;
			
		case MODBUS_FUN_WRITE_DISOUT_MORE:
		case MODBUS_FUN_WRITE_HOLD_MORE:
			//写多个结构类似
			*packLen = 8;
			pack = pvPortMalloc( *packLen );
			if( pack == NULL )
				break;
			pack[ 0 ] = hModBusMsg->parse.slaveId;
			pack[ 1 ] = hModBusMsg->parse.funCode;
			__ModBus_U16SmallToBig( hModBusMsg->parse.regAddrStart , pack + 2 );
			__ModBus_U16SmallToBig( hModBusMsg->parse.regLen, pack + 4 );
			ModBus_RtuAddCrc( pack, *packLen - 2 );
			break;
		
		default:
			//不在支持的功能码列表内
			break;
		}
	
		break;
	case ModBus_TYPE_ASCII:
		break;
	case ModBus_TYPE_TCP:
		break;
	}
	return pack;
}



/*************************************************
函数名称:ModBus_Distribution

功能描述:当modbus数据通过解析之后，调用该函数，进行读写请求分发，根据pSlave中的topicLink
	订阅链表进行分发。并统一结果，进行回复

调用函数清单: 

本函数被调来源: 

输入参数说明:	ModBus_Slave_TypeDef * hSlave
								功能主结构体，包含链表、链表互斥量等信息
							ModBus_Msg_TypeDef * hModBusMsg 
								消息结构体，包含这次消息
输出参数说明: hModBusMsg->parse.errorCode，记录失败时的异常码

返回值说明: 返回0表示成功，非零表示异常

备注://待完善，未完善超时机制
*************************************************/
int ModBus_Distribution( ModBus_Slave_TypeDef * hSlave, ModBus_Msg_TypeDef * hModBusMsg )
{
	int result = -1;
	uint16_t i, j;
	ModBus_TopicLink_TypeDef *pTopicLink;
	uint16_t regAddr = hModBusMsg->parse.regAddrStart;
	uint16_t regLen = 1;
	uint16_t regAddrEnd = hModBusMsg->parse.regAddrStart + hModBusMsg->parse.regLen;
	uint16_t byteSize;
	uint8_t findTopic = 0;
	uint16_t *pValue = NULL;
	
	uint8_t isWrite = 0;	//是否为写入指令
	uint8_t isDis = 0;		//是否为离散寄存器
	
	//先判断读写类型
	switch( hModBusMsg->parse.funCode )
	{
	case MODBUS_FUN_WRITE_DISOUT:
	case MODBUS_FUN_WRITE_HOLD:
	case MODBUS_FUN_WRITE_DISOUT_MORE:
	case MODBUS_FUN_WRITE_HOLD_MORE:
		isWrite = 1;
	}
	//判断读数据是否为离散
	switch( hModBusMsg->parse.funCode )
	{
	case MODBUS_FUN_READ_DISOUT:
	case MODBUS_FUN_READ_DISIN:
	case MODBUS_FUN_WRITE_DISOUT:
	case MODBUS_FUN_WRITE_DISOUT_MORE:
		isDis = 1;
	}
	
	if( ( xTaskGetTickCount() < hSlave->init.timeOutTick + hModBusMsg->getTimeTick ) &&\
			( pdTRUE == xSemaphoreTake( hSlave->topicLinkMutex, hSlave->init.timeOutTick + hModBusMsg->getTimeTick - xTaskGetTickCount())))
	{
		while( regAddr < regAddrEnd )		//regAddr递增，直至结尾
		{
			//寻找匹配的数据点
			for( pTopicLink = hSlave->pTopicLinkHand, i = 0, findTopic = 0; pTopicLink != NULL ; ++i, pTopicLink = pTopicLink->pPext)
			{
				if( ( pTopicLink->slaveId == hModBusMsg->parse.slaveId ) &&\
						( pTopicLink->dataType == hModBusMsg->parse.dataType ))
				{
					//从机地址、数据类型匹配之后，进一步匹配地址
					switch( pTopicLink->dataMode )
					{
					case MODBUS_DATA_MODE_ALL:
						if( ( pTopicLink->regAddrStart <= regAddr ) &&\
							  ( pTopicLink->regLen + pTopicLink->regAddrStart > regAddr ))
						{
							//任意模式下，只要有交集，即regAddr在起始和结尾之间
							findTopic = 1;
						}
						break;
					
					case MODBUS_DATA_MODE_STRICT:
						if(	( regAddr == hModBusMsg->parse.regAddrStart	) &&\
								( pTopicLink->regAddrStart == regAddr ) &&\
							  ( pTopicLink->regLen == hModBusMsg->parse.regLen ))
						{
							//严格模式下，必须单条指令，完全对应
							findTopic = 1;
						}
						break;
					
					case MODBUS_DATA_MODE_CONTAIN:
						if( ( pTopicLink->regAddrStart == regAddr ) &&\
							  ( pTopicLink->regLen + pTopicLink->regAddrStart <= regAddrEnd ))
						{
							//包含模式下，当前起始地址必须相等，且结尾地址应该小于等于指令范围
							findTopic = 1;
						}
						break;
					}
				}
				if( findTopic )
					break;	//提前结束循环，已找到匹配订阅
			}
			if( findTopic )
			{
				//已找到当前寄存器地址匹配订阅，进行分发调用前准备
				//计算实际要用的寄存器长度
				if( regAddrEnd >= pTopicLink->regLen + pTopicLink->regAddrStart )
					regLen = pTopicLink->regLen + pTopicLink->regAddrStart - regAddr;
				else
					regLen = regAddrEnd - regAddr;
				//准备数据或者数据空间
				if( pValue != NULL )
				{
					vPortFree( pValue );
					pValue = NULL;
				}
				if( isDis )
				{
					//离散型数据准备
					byteSize = __ModBus_DisRegLenToByteSize( regLen );
					pValue = pvPortMalloc( byteSize );	
					if( pValue != NULL )
						break;		//待完善
					memset( pValue, 0, byteSize  );
					if( isWrite )
					{
						//将离散值从msg转写到pValue
						for( i = 0, j = regAddr - hModBusMsg->parse.regAddrStart; i < regLen ; ++i)
						{
							if( (((uint8_t *)(hModBusMsg->parse.pValue))[ j / 8u ] >> ( j % 8u)) & 0x01u )
								((uint8_t *)(pValue))[ i / 8u ] |= 0x01u << i % 8u;
						}
					}
				}
				else
				{
					//U16类型数据准备
					byteSize = regLen * 2u;
					pValue = pvPortMalloc( byteSize );
					if( pValue == NULL )
						break;		//待完善
					memset( pValue, 0, byteSize  );
					if( isWrite )
					{
						//将值从msg转写到pValue
						for( i = 0, j = regAddr - hModBusMsg->parse.regAddrStart; i < regLen ; ++i)
						{
							pValue[ i ] = (hModBusMsg->parse.pValue)[j];
						}
					}
				}
				//数据及空间等参数准备就绪，进行调用
				if( isWrite )
				{
					hModBusMsg->parse.errorCode = (pTopicLink->pWriteFun)( pTopicLink, pValue, regAddr, regLen);
				}
				else
				{
					hModBusMsg->parse.errorCode = (pTopicLink->pReadFun)( pTopicLink, pValue, regAddr, regLen);
				}
				if( hModBusMsg->parse.errorCode != MODBUS_REACK_ERROR_NULL )
					break;
				//调用正常结束
				//如果是读取类，应当把数据存储到对应空间中
				if( !isWrite )
				{
					if( isDis )
					{
						//将离散值从pValue转写到msg
						for( i = 0, j = regAddr - hModBusMsg->parse.regAddrStart; i < regLen ; ++i)
						{
							if( (((uint8_t *)(pValue))[ i / 8u ] >> ( j % 8u)) & 0x01u )
								((uint8_t *)(hModBusMsg->parse.pValue))[ j / 8u ] |= 0x01u << i % 8u;
						}
					}
					else
					{
						//将值从pValue转写到msg
						for( i = 0, j = regAddr - hModBusMsg->parse.regAddrStart; i < regLen ; ++i)
						{
							(hModBusMsg->parse.pValue)[j] = pValue[ i ];
						}
					}
				}
				//regAddr递增
				regAddr += regLen;
				//开始新的订阅匹配搜寻
			}
			else
			{
				//查遍所有订阅链表均未发现匹配。
				hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_REG;		//寄存器地址错误
				break;	//退出分发
			}
			
		}
		if( regAddr >= regAddrEnd  )		//地址表已经到达或超过结束，表示正常完成分发
			result = 0;
		
		xSemaphoreGive( hSlave->topicLinkMutex);
	}
	
	if( pValue != NULL )
	{
		vPortFree( pValue );
		pValue = NULL;
	}
	return result;
}



/*************************************************
函数名称：ModBus_FreeTopicLink

功能描述：析构TopicLink链表，包含其后续的链表结构体。

调用函数清单: 

本函数被调来源: 

输入参数说明: ModBus_TopicLink_TypeDef *hTopicLink
								需要析构的链表首指针

输出参数说明: 

返回值说明: 

备注://待完善，并没有多任务安全保护,严禁析构同时进行添加等写入操作
*************************************************/
void ModBus_FreeTopicLink( ModBus_TopicLink_TypeDef *hTopicLink )
{
	if( hTopicLink == NULL )
		return;
	if( hTopicLink->pPext == NULL )
	{
		vPortFree( hTopicLink );
	}
	else
	{
		ModBus_FreeTopicLink( hTopicLink->pPext );		//递归调用析构
	}
}


/*************************************************
函数名称：ModBus_FreeSlave

功能描述：析构Slave结构体，包含内部指针等。

调用函数清单: 

本函数被调来源: 

输入参数说明: ModBus_Slave_TypeDef *hSlave
								需要析构的结构体指针

输出参数说明: 

返回值说明: 

备注://待完善，并没有多任务安全保护,严禁析构同时进行添加等写入操作
*************************************************/
void ModBus_FreeSlave( ModBus_Slave_TypeDef *hSlave )
{
	if( hSlave == NULL )
		return;
	if( hSlave->topicLinkMutex != NULL )
	{
		vPortFree( hSlave->topicLinkMutex );
		hSlave->topicLinkMutex = NULL;
	}
	if( hSlave->pTopicLinkHand != NULL )
	{
		ModBus_FreeTopicLink( hSlave->pTopicLinkHand );
		hSlave->pTopicLinkHand = NULL;
	}
	vPortFree( hSlave );
}

/*************************************************
函数名称：ModBus_SlaveTask

功能描述：管控Modbus数据接收、解析、分发处理、回复的整个流程主任务函数。

调用函数清单: 

本函数被调来源: 

输入参数说明: ModBus_SlaveInit_TypeDef *hSlaveInit
								初始化参数

输出参数说明: 

返回值说明: 

备注://待完善，未完善超时机制
*************************************************/
void ModBus_SlaveTask( ModBus_Slave_TypeDef *hSlave )
{
	ModBus_Msg_TypeDef hModBusMsg;
	memset( &hModBusMsg, 0, sizeof( hModBusMsg ) );
	
	uint8_t * pReplyData = NULL;
	uint16_t replyLen = 0;
	
	while(1)
	{
		//获取ModBus数据包
		hModBusMsg.packData = hSlave->init.fRxPort( &(hModBusMsg.packLen), portMAX_DELAY );
		if( hModBusMsg.packData == NULL )
			continue;
		hModBusMsg.getTimeTick = xTaskGetTickCount();
		hModBusMsg.parse.errorCode = MODBUS_REACK_ERROR_NULL;
		//调用解析
		if( 0 != ModBus_Parse( &hModBusMsg, hSlave->init.msgType ) )
		{
			if( MODBUS_REACK_ERROR_NULL != hModBusMsg.parse.errorCode )
			{
				pReplyData = ModBus_CreatErrorAckPack( &hModBusMsg, hSlave->init.msgType, &replyLen);
			}
		}
		else
		{
			//解析通过。进行分发
			if( 0 != ModBus_Distribution( hSlave, &hModBusMsg ))
			{
				//分发过程中出现异常
				if( MODBUS_REACK_ERROR_NULL == hModBusMsg.parse.errorCode )
				{
					hModBusMsg.parse.errorCode = MODBUS_REACK_ERROR_DAMAGE;		//未知的设备故障
				}
				pReplyData = ModBus_CreatErrorAckPack( &hModBusMsg, hSlave->init.msgType, &replyLen);
			}
			else
			{
				//分发成功完成，进行生成响应信息进行回复
				pReplyData = ModBus_CreateReplyPack( &hModBusMsg, hSlave->init.msgType, &replyLen);
			}
		}
		if( ( pReplyData != NULL ) &&\
				(	hSlave->init.timeOutTick + hModBusMsg.getTimeTick > xTaskGetTickCount() ))
		{
			//进行回复
			hSlave->init.fTxPort( pReplyData, replyLen, xTaskGetTickCount() - hModBusMsg.getTimeTick - hSlave->init.timeOutTick);
			vPortFree( pReplyData );
			pReplyData = NULL;
		}
		if( hModBusMsg.parse.pValue != NULL )
		{
			vPortFree( hModBusMsg.parse.pValue );
			hModBusMsg.parse.pValue = NULL;
			hModBusMsg.parse.byteLen = 0;
		}
		if( hModBusMsg.packData != NULL )
		{
			vPortFree( hModBusMsg.packData );
			hModBusMsg.packData = NULL;
			hModBusMsg.packLen = 0;
		}
	}
}


/*************************************************
函数名称：ModBus_CreateSlave

功能描述：创建SlaveTask任务，分配参数。返回主结构指针

调用函数清单: ModBus_SlaveTask

本函数被调来源: 

输入参数说明: ModBus_SlaveInit_TypeDef *hSlaveInit
								初始化参数

输出参数说明: 

返回值说明: ModBus_Slave_TypeDef *hSlave

备注:
*************************************************/
ModBus_Slave_TypeDef * ModBus_CreateSlave( ModBus_SlaveInit_TypeDef *hSlaveInit )
{
	ModBus_Slave_TypeDef *hSlave = NULL;
	int8_t result = -1;
	
	hSlave = pvPortMalloc( sizeof(ModBus_Slave_TypeDef) );
	if( hSlave == NULL )
		return hSlave;
	
	do
	{
		memset( hSlave, 0, sizeof(ModBus_Slave_TypeDef) );
		//初始化参数
		hSlave->topicLinkMutex = xSemaphoreCreateMutex();
		if( hSlave->topicLinkMutex == NULL )
			break;
		hSlave->pTopicLinkHand = NULL;
		hSlave->state = ModBus_STATE_READY;
		hSlave->init.msgType = hSlaveInit->msgType;
		hSlave->init.fRxPort = hSlaveInit->fRxPort;
		hSlave->init.fTxPort = hSlaveInit->fTxPort;
		hSlave->init.timeOutTick = hSlaveInit->timeOutTick;
		
		if( pdPASS != xTaskCreate(	(void (*)(void *))&ModBus_SlaveTask, 
																"ModBusSlave", 
																MODBUS_SLAVETASK_STACK_SIZE, 
																hSlave, 
																MODBUS_SLAVETASK_PRIORITY_NUM, 
																NULL ))
		{
			break;	
		}
		
		result = 0;
	}while(0);
	
	if( result != 0 )
	{
		if( hSlave != NULL )
		{
			ModBus_FreeSlave( hSlave );
			hSlave = NULL;
		}
	}
	return hSlave;
}

/*************************************************
函数名称：ModBus_AddTopic

功能描述：对指定SlaveTask任务中，新增数据点订阅链表

调用函数清单: 

本函数被调来源: 

输入参数说明: ModBus_Slave_TypeDef *hSlave
								指定的SlaveTask，包含订阅链表头、订阅链表互斥锁
							ModBus_TopicLink_TypeDef *hTopicLink
								所需要订阅的数据点信息，不考虑pNext
								
输出参数说明: 

返回值说明:0成功，非零失败

备注://待完善，需要去重确保互斥。
*************************************************/
int ModBus_AddTopic( ModBus_Slave_TypeDef *hSlave, ModBus_TopicLink_TypeDef *hTopicLink )
{
	int result = -1;
	ModBus_TopicLink_TypeDef *pTopicLinkTmp = NULL;
	
	do
	{
		if( hSlave == NULL || hTopicLink == NULL || hSlave->topicLinkMutex == NULL)
			break;
		if( pdTRUE == xSemaphoreTake( hSlave->topicLinkMutex, portMAX_DELAY ) )
		{
			do
			{
				pTopicLinkTmp = hSlave->pTopicLinkHand;
				if( pTopicLinkTmp != NULL )
				{
					for( ; pTopicLinkTmp->pPext != NULL ; pTopicLinkTmp = pTopicLinkTmp->pPext)
					{
					} 
					pTopicLinkTmp->pPext = pvPortMalloc( sizeof( ModBus_TopicLink_TypeDef ) );
					if( pTopicLinkTmp->pPext == NULL )
						break;
					pTopicLinkTmp = pTopicLinkTmp->pPext;
				}
				else
				{
					pTopicLinkTmp = hSlave->pTopicLinkHand = pvPortMalloc( sizeof( ModBus_TopicLink_TypeDef ) );
					if( pTopicLinkTmp == NULL )
						break;
				}
				pTopicLinkTmp->pWriteFun = hTopicLink->pWriteFun;
				pTopicLinkTmp->pReadFun = hTopicLink->pReadFun;
				pTopicLinkTmp->regAddrStart = hTopicLink->regAddrStart;
				pTopicLinkTmp->regLen = hTopicLink->regLen;
				pTopicLinkTmp->slaveId = hTopicLink->slaveId;
				pTopicLinkTmp->dataType = hTopicLink->dataType;
				pTopicLinkTmp->dataMode = hTopicLink->dataMode;
				pTopicLinkTmp->pPext = NULL;
				
				result = 0;
			}while(0);
			xSemaphoreGive( hSlave->topicLinkMutex );
		}		
	}while(0);
		
	return result;
}



/*************************************************
函数名称:ModBus_CreateMainPack

功能描述:根据msg.parse中的信息，进行ModBus协议主机指令数据包生成并返回该指针。同时创建读指令所需要的pValue空间

调用函数清单: 

本函数被调来源: 

输入参数说明:	ModBus_Msg_TypeDef * hModBusMsg 
								包含下发必须的内容:从机地址、功能码、寄存器起始地址、寄存器长度(可选)、值(可选)
							ModBus_Type_enum msgType
								信息类型
							
输出参数说明: uint16_t * packLen
								返回字节流的长度

返回值说明: 返回生成的消息字节流指针，需要由接受者自行释放，如果为NULL可能因为无法创建空间或者数据有误

备注:
*************************************************/
uint8_t *ModBus_CreateMainPack( ModBus_Msg_TypeDef * hModBusMsg, ModBus_Type_enum msgType, uint16_t *packLen)
{
	uint8_t *pack = NULL;
	uint16_t i;
	uint8_t isOtherCase = 0;
	
	hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_DATA;
	
	switch( msgType )
	{
	case ModBus_TYPE_RTU:
		hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_NULL;
	
		//匹配功能码
		switch( hModBusMsg->parse.funCode )
		{
		case MODBUS_FUN_READ_DISOUT:
		case MODBUS_FUN_READ_DISIN:
			hModBusMsg->parse.byteLen = __ModBus_DisRegLenToByteSize( hModBusMsg->parse.regLen );
			isOtherCase = 1;
		case MODBUS_FUN_READ_HOLD:
		case MODBUS_FUN_READ_INPUT:
			//离散与寄存器数据宽度不同
			if( !isOtherCase )
			{
				hModBusMsg->parse.byteLen = hModBusMsg->parse.regLen * 2u;
			}
			//创建存储空间
			if( hModBusMsg->parse.pValue != NULL )
				vPortFree( hModBusMsg->parse.pValue );
			hModBusMsg->parse.pValue = pvPortMalloc( hModBusMsg->parse.byteLen );
			if( hModBusMsg->parse.pValue == NULL )
				break;
			memset( hModBusMsg->parse.pValue, 0 , hModBusMsg->parse.byteLen );
			//读取类结构类似
			*packLen = 8;
			pack = pvPortMalloc( *packLen );
			if( pack == NULL )
				break;
			pack[ 0 ] = hModBusMsg->parse.slaveId;
			pack[ 1 ] = hModBusMsg->parse.funCode;
			__ModBus_U16SmallToBig( hModBusMsg->parse.regAddrStart, pack + 2 );
			__ModBus_U16SmallToBig( hModBusMsg->parse.regLen, pack + 4 );
			ModBus_RtuAddCrc( pack, *packLen - 2 );
			
			break;
			
		case MODBUS_FUN_WRITE_DISOUT:
			if( hModBusMsg->parse.pValue == NULL || \
				  (	*(hModBusMsg->parse.pValue) != MODBUS_DIS_VALUE_ON && \
						*(hModBusMsg->parse.pValue) != MODBUS_DIS_VALUE_OFF ))
				break;
			isOtherCase = 1;
		case MODBUS_FUN_WRITE_HOLD:
			if( !isOtherCase )
				if( hModBusMsg->parse.pValue == NULL )
					break;
			*packLen = 8;
			pack = pvPortMalloc( *packLen );
			if( pack == NULL )
				break;
			pack[ 0 ] = hModBusMsg->parse.slaveId;
			pack[ 1 ] = hModBusMsg->parse.funCode;
			__ModBus_U16SmallToBig( hModBusMsg->parse.regAddrStart , pack + 2 );
			__ModBus_U16SmallToBig( *(hModBusMsg->parse.pValue), pack + 4 );
			ModBus_RtuAddCrc( pack, *packLen - 2 );
		
			break;
			
		case MODBUS_FUN_WRITE_DISOUT_MORE:
		case MODBUS_EXFUN_REPORT_DISOUT_MORE:
		case MODBUS_EXFUN_REPORT_DISIN_MORE:
			if( hModBusMsg->parse.pValue == NULL || \
				  hModBusMsg->parse.byteLen == 0 || \
					hModBusMsg->parse.byteLen != __ModBus_DisRegLenToByteSize( hModBusMsg->parse.regLen ))
				break;
			isOtherCase = 1;
			
		case MODBUS_FUN_WRITE_HOLD_MORE:
		case MODBUS_EXFUN_REPORT_HOLD_MORE:
		case MODBUS_EXFUN_REPORT_INPUT_MORE:
			//写多个结构类似
			if( !isOtherCase )
			{
				if( hModBusMsg->parse.pValue == NULL || \
				  hModBusMsg->parse.byteLen == 0 ||\
					hModBusMsg->parse.byteLen != 2u * hModBusMsg->parse.regLen)
					break;
			}
			//写单个结构体类似
			*packLen = 9 + hModBusMsg->parse.byteLen;
			pack = pvPortMalloc( *packLen );
			if( pack == NULL )
				break;
			pack[ 0 ] = hModBusMsg->parse.slaveId;
			pack[ 1 ] = hModBusMsg->parse.funCode;
			__ModBus_U16SmallToBig( hModBusMsg->parse.regAddrStart , pack + 2 );
			__ModBus_U16SmallToBig( hModBusMsg->parse.regLen, pack + 4 );
			pack[ 6 ] = hModBusMsg->parse.byteLen;
			
			if( isOtherCase )
			{
				memcpy( pack + 7, hModBusMsg->parse.pValue, hModBusMsg->parse.byteLen );
			}
			else
			{
				//对于寄存器数据，需要大小端转换
				for( i = 0; i < hModBusMsg->parse.byteLen; i += 2)
				{
					__ModBus_U16SmallToBig( hModBusMsg->parse.pValue[ i >> 1u ], pack + 7 + i );
				}
			}
			ModBus_RtuAddCrc( pack, *packLen - 2 );
			hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_NULL;
		
			break;
		
		default:
			//不在支持的功能码列表内
			hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_REG;
			break;
		}
	
		break;
	case ModBus_TYPE_ASCII:
		break;
	case ModBus_TYPE_TCP:
		break;
	}
	return pack;
}


/*************************************************
函数名称:ModBus_ParseReplyPack()

功能描述:解析主机模式下返回数据包

调用函数清单: 

本函数被调来源: 

输入参数说明:	ModBus_Msg_TypeDef * hModBusMsg 
								包含:从机地址、功能码、寄存器起始地址、寄存器长度(可选)、值(可选)
								包含：packData指向回应数据内容，packLen数据长度
							ModBus_Type_enum msgType
								信息类型
							
输出参数说明: uint16_t * packLen
								返回字节流的长度

返回值说明: 返回生成的消息字节流指针，需要由接受者自行释放，如果为NULL可能因为无法创建空间或者数据有误

备注:
*************************************************/
ModBus_ErrorAck_enum ModBus_ParseReplyPack( ModBus_Msg_TypeDef * hModBusMsg, ModBus_Type_enum msgType)
{
	int result = -1;
	uint16_t i;
	
	switch( msgType )
	{
	case ModBus_TYPE_RTU:
		//最小长度判断
		if( hModBusMsg->packLen < MODBUS_RTU_REPLY_PACKLEN_MIN )
			break;
		//CRC校验
		if( 0 != CRC16( hModBusMsg->packData, hModBusMsg->packLen ))
			break;
		//开始分析
		
		if( ( hModBusMsg->parse.slaveId != (hModBusMsg->packData)[0] ) ||\
				( ( hModBusMsg->parse.funCode != ( (hModBusMsg->packData)[1] & 0x7Fu ) ) && \
					!( 	( hModBusMsg->parse.funCode == MODBUS_EXFUN_REPORT_INPUT_MORE) && \
							( ( hModBusMsg->packData)[1] & 0x7Fu ) == MODBUS_EXFUN_REPORT_HOLD_MORE ) ) )	//加入了有人扩展码BUG
		{
			break;
		}
		if( (hModBusMsg->packData)[1] & 0x80u )
		{
			//错误码返回
			if( hModBusMsg->packLen != MODBUS_RTU_REPLY_PACKLEN_ERRORACK_SIZE )
				break;
			hModBusMsg->parse.errorCode = (hModBusMsg->packData)[2];
			result = 0;		//虽然是错误码，但是也算是正常返回数据了
			break;
		}
		hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_NULL;
		//匹配功能码
		switch( hModBusMsg->parse.funCode )
		{
		case MODBUS_FUN_READ_DISOUT:
		case MODBUS_FUN_READ_DISIN:
		case MODBUS_FUN_READ_HOLD:
		case MODBUS_FUN_READ_INPUT:
			//长度判断 从机+功能码+字节数+数据+CRC
			if( (hModBusMsg->packData)[2] != hModBusMsg->parse.byteLen )
				break;
			if( hModBusMsg->parse.pValue == NULL )
			{
				hModBusMsg->parse.pValue = pvPortMalloc( (hModBusMsg->packData)[2] );
				if( hModBusMsg->parse.pValue == NULL )
					break;
			}
			
			if( hModBusMsg->parse.funCode == MODBUS_FUN_READ_DISOUT || hModBusMsg->parse.funCode == MODBUS_FUN_READ_DISIN )
			{
				memcpy( hModBusMsg->parse.pValue, hModBusMsg->packData + 3, hModBusMsg->parse.byteLen );
			}
			else
			{
				//对于寄存器数据，需要大小端转换
				for( i = 0; i < hModBusMsg->parse.byteLen; i += 2)
				{
					(hModBusMsg->parse.pValue)[ i >> 1u] = __ModBus_U16BigToSmall( hModBusMsg->packData + 3 + i );
				}
			}
			
			result = 0;
			break;
			
		case MODBUS_FUN_WRITE_DISOUT:
		case MODBUS_FUN_WRITE_HOLD:
			//写单个，写指令与回复指令一模一样
			if( ( MODBUS_RTU_PACKLEN_WRITE_SIZE != hModBusMsg->packLen ) ||\
					( __ModBus_U16BigToSmall( hModBusMsg->packData + 2 ) != hModBusMsg->parse.regAddrStart ) ||\
					( __ModBus_U16BigToSmall( hModBusMsg->packData + 4 ) != *(hModBusMsg->parse.pValue) ))
				break;
			
			result = 0;
			break;
			
		case MODBUS_FUN_WRITE_DISOUT_MORE:
		case MODBUS_FUN_WRITE_HOLD_MORE:
		case MODBUS_EXFUN_REPORT_DISOUT_MORE:
		case MODBUS_EXFUN_REPORT_DISIN_MORE:
		case MODBUS_EXFUN_REPORT_HOLD_MORE:
		case MODBUS_EXFUN_REPORT_INPUT_MORE:
			if( ( 8 != hModBusMsg->packLen ) ||\
					( __ModBus_U16BigToSmall( hModBusMsg->packData + 2 ) != hModBusMsg->parse.regAddrStart ) ||\
					( __ModBus_U16BigToSmall( hModBusMsg->packData + 4 ) != hModBusMsg->parse.regLen ))
				break;
			
			result = 0;
			break;
		
		default:
			hModBusMsg->parse.errorCode = MODBUS_REACK_ERROR_FUN;		//不在支持的功能码列表内
			break;
		}
		break;
		
	case ModBus_TYPE_ASCII:
		break;
	case ModBus_TYPE_TCP:
		break;
	}
	
	return result;
}


/*************************************************
函数名称：ModBus_MainTxCom

功能描述：主机形式，下发指令

调用函数清单: 

本函数被调来源: 

输入参数说明: ModBus_SlaveInit_TypeDef *hSlaveInit
								包含RX TX 函数接口，通讯时间限定、消息格式内容
							ModBus_Msg_TypeDef *hModBusMsg
								包含下发必须的内容:从机地址、功能码、寄存器起始地址、寄存器长度(可选)、值(可选)
								
输出参数说明: ModBus_Msg_TypeDef *hModBusMsg
								将会包含读指令的pValue值。

返回值说明: 0成功通讯，如果异常hModBusMsg->parse.errorCode存有异常码，非0收发失败。

备注:
*************************************************/
int ModBus_MainTxCom( ModBus_SlaveInit_TypeDef *hSlaveInit, ModBus_Msg_TypeDef *hModBusMsg)
{
	int result = -1;
	uint8_t *pack = NULL;
	uint16_t packLen = 0;
	TickType_t beginTick = xTaskGetTickCount();
	
	do
	{
		if( hModBusMsg->packData != NULL )
		{
			vPortFree( hModBusMsg->packData );
			hModBusMsg->packData = NULL;
		}
		hModBusMsg->packLen = 0;
		
		pack = ModBus_CreateMainPack( hModBusMsg, hSlaveInit->msgType, &packLen );
		if( pack == NULL )
			break;
		if( 0 != hSlaveInit->fTxPort( pack, packLen, hSlaveInit->timeOutTick + beginTick - xTaskGetTickCount() ))
			break;
		
		if( hSlaveInit->fRxPort != NULL )
		{
			while( hSlaveInit->timeOutTick + beginTick > xTaskGetTickCount() )
			{
				hModBusMsg->packData = hSlaveInit->fRxPort( &(hModBusMsg->packLen), hSlaveInit->timeOutTick + beginTick - xTaskGetTickCount());
				if( hModBusMsg->packData != NULL )
				{
					if( 0 != ModBus_ParseReplyPack( hModBusMsg, hSlaveInit->msgType ) && \
							hModBusMsg->parse.errorCode == MODBUS_REACK_ERROR_NULL)
						continue;		//没有成功解析返回数据包，有可能该包不是发给本通讯程序的，丢弃该包，继续接收
					result = 0;
					break;
				}
			}
		}
		else
		{
			//如果rxPort接收函数指针为空，认为不需要接收返回，直接返回发送成功
			result = 0;
		}
	}
	while(0);
	
	if( hModBusMsg->packData != NULL )
	{
		vPortFree( hModBusMsg->packData );
		hModBusMsg->packData = NULL;
	}
	if( pack != NULL )
		vPortFree( pack );
	
	return result;
}


/*===================================== End of File ===================================*/
