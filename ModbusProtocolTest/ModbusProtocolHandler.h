//*********************************************************************************************************//
//MODBUS protocol implementation in C++
//MODBUS protocol handler header file. MODBUS master/slave based on ModbusRegisterMap.
//Created 01.06.2020
//Created by Novikov Dmitry
//*********************************************************************************************************//

#ifndef MODBUS_PROTOCOL_HANDLER
#define MODBUS_PROTOCOL_HANDLER

#include <stdint.h>
#include <vector>
#include <algorithm>
#include <functional>
#include <atomic>
#include <iostream>
#include <windows.h>
#include "ModbusRegisterMap.h"

using std::cout;
using std::wcout;
using std::endl;
using std::function;
using std::vector;

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* config block */

//debug modes enable
//#define MODBUS_SLAVE_DEBUG
#define MODBUS_ENABLE_DEBUG_MESSAGES_TO_CONSOLE
//#define MODBUS_ENABLE_DEBUG_MESSAGES_TO_WINDLG

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* platform specified user defined functions for count timeout */
UINT StartTimeoutTimer(int timeout, function <void()> callbackFunc, UINT timerID);
bool StopTimeoutTimer(UINT timerID);
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* debug interface for output errors - platform specified */
inline void outputErrorMessageA(const char* text)
{
	#ifdef MODBUS_ENABLE_DEBUG_MESSAGES_TO_CONSOLE
		cout << text << endl;
	#endif
	#ifdef MODBUS_ENABLE_DEBUG_MESSAGES_TO_WINDLG
		MessageBoxA(NULL, text, "Error", MB_RETRYCANCEL);
	#endif
}

inline void outputErrorMessageW(const wchar_t* text)
{
	#ifdef MODBUS_ENABLE_DEBUG_MESSAGES_TO_CONSOLE
		wcout << text << endl;
	#endif
	#ifdef MODBUS_ENABLE_DEBUG_MESSAGES_TO_WINDLG
		MessageBoxW(NULL, text, L"Error", MB_RETRYCANCEL);
	#endif
}

#define outputErrorMessage(text) outputErrorMessageA(text);
/*-----------------------------------------------------------------------------------------------------------------------------*/

/* calculate modbus crc16 */
uint16_t ModbusCRC16(const uint8_t* inputData, uint16_t dataLength);

/*-----------------------------------------------------------------------------------------------------------------------------*/
//base class for modbus protocol parser
class ModbusProtocolBase
{
	//type for send data callback
	typedef function <bool(uint8_t*, size_t)> SendDataFuncObj;

	public:
		/* constructor */
		ModbusProtocolBase()
		{
			inputDataBuffer.clear();
			outputDataBuffer.clear();

			//try reserve memory for input and output buffers
			try
			{
				inputDataBuffer.reserve(inputBufferMaxSize);
				outputDataBuffer.reserve(outputBufferMaxSize);
			}
			catch (...)
			{
				//fail
				throw "Fail reserve memory (RAM) for data buffers.";
			}
		};

		/* destructor*/
		virtual ~ModbusProtocolBase()
		{
			inputDataBuffer.clear();
			outputDataBuffer.clear();
		};

		/* set send data calback */
		bool SetSendDataFunc(SendDataFuncObj sendFunc)
		{
			if (sendFunc)
			{
				this->sendDataFunc = sendFunc;
				return true;
			}
			return false;
		}

		/* set modbus registers map */
		bool SetRegisterMap(ModbusRegMap* map)
		{
			if (map)
			{
				this->modbusRegisterMap = map;
				return true;
			}
			return false;
		}

		/* set modbus device address */
		bool SetDeviceAddress(uint8_t address)
		{
			this->deviceAddress = address;
			return true;
		}

		//input and output buffers max queue size in bytes
		size_t inputBufferMaxSize = 1024;
		size_t outputBufferMaxSize = 1024;

	protected:
		/* structures for modbus input packet */
		#pragma pack(push, 1)
		// struct for modbus input packet - functions 0x01 to 0x04
		struct inputPackTemplateF01F04
		{
			uint8_t address;
			uint8_t funcCode;
			uint16_t regAddress;
			uint16_t regsCount;
			uint16_t packetCRC;
		};
		// struct for modbus input packet - functions 0x05 and 0x06
		struct inputPackTemplateF05F06
		{
			uint8_t address;
			uint8_t funcCode;
			uint16_t regAddress;
			uint16_t regValue;
			uint16_t packetCRC;
		};
		// struct for modbus input packet - functions 0x0F(15) and 0x10(16)
		struct inputPackTemplateF15F16
		{
			uint8_t address;
			uint8_t funcCode;
			uint16_t startRegAddress;
			uint16_t regsCount;
			uint8_t bytesCount;
		};
		// struct for modbus output packet - functions 0x01 to 0x04
		struct outputPackTemplateF01F04
		{
			uint8_t address;
			uint8_t funcCode;
			uint8_t byteCount;
		};
		// struct for modbus output packet - functions 0x05 and 0x06
		typedef inputPackTemplateF05F06 outputPackTemplateF05F06;
		// struct for modbus output packet - functions 0x0F(15) and 0x10(16)
		typedef inputPackTemplateF01F04 outputPackTemplateF15F16;
		// struct for modbus error packet
		struct outputPackTemplateError
		{
			uint8_t address;
			uint8_t errorCode;
			uint8_t exceptionCode;
			uint16_t packetCRC;
		};
		#pragma pack(pop)
		const size_t inputPackTemplateF01F04_Size = sizeof(inputPackTemplateF01F04);
		const size_t inputPackTemplateF05F06_Size = sizeof(inputPackTemplateF05F06);
		const size_t inputPackTemplateF15F16_Size = sizeof(inputPackTemplateF15F16);
		const size_t outputPackTemplateF01F04_Size = sizeof(outputPackTemplateF01F04);
		const size_t outputPackTemplateF05F06_Size = sizeof(outputPackTemplateF05F06);
		const size_t outputPackTemplateF15F16_Size = sizeof(outputPackTemplateF15F16);
		const size_t outputPackTemplateError_Size = sizeof(outputPackTemplateError);

		/* constants for modbus exceptions */
		enum modbusExceptionCode
		{
			ILLEGAL_FUNCTION = 0x01,
			ILLEGAL_DATA_ADDRESS,
			ILLEGAL_DATA_VALUE,
			SERVER_DEVICE_FAILURE,
			ACKNOWLEDGE,
			SERVER_DEVICE_BUSY,
			MEMORY_PARITY_ERROR = 0x08,
			GATEWAY_PATH_UNAVAILABLE = 0x0A,
			GATEWAY_TARGET_RESPOND_FAILED
		};

		//input & output buffers
		vector <uint8_t> inputDataBuffer;
		vector <uint8_t> outputDataBuffer;

		//send data callback
		SendDataFuncObj sendDataFunc = nullptr;

		//device modbus address
		uint8_t deviceAddress = 1;

		//modbus register map
		ModbusRegMap* modbusRegisterMap = nullptr;
};
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
//modbus protocol master class
class ModbusProtocolMaster: public ModbusProtocolBase
{
	public:
		/* constructor */
		ModbusProtocolMaster()
		{

		}

		/* destructor */
		~ModbusProtocolMaster()
		{

		}

		/* parse input packet (in buffer) */
		void inputPacketParse(uint8_t* inputBuffer, size_t inputLen);

		/* read all registers to register map */
		bool readAllRegisters(void);

	private:
		/* modbus master current states */
		enum modbusMasterCurrentStates
		{
			MM_STATE_FREE = 0,
			MM_STATE_BUSY,
			MM_STATE_RESPONSE_READY
		};

		// modbus master current state
		std::atomic_int masterCurrentState = MM_STATE_FREE;

		//data of last request
		struct
		{
			uint8_t functionCode;
			uint16_t bytesCount;
			int attemptsCount;
			UINT timerIdentifier;
			void Reset(void)
			{
				this->functionCode = 0;
				this->bytesCount = 0;
				this->attemptsCount = 0;
				this->timerIdentifier = 0;
			}
		} lastRequestInfo = {};

		//timeout, ms
		int responseTimeout = 2000;
		//number of attempts if request error
		int numberOfAttempts = 3;

		/*  */
		void timeoutExpired(void);

		/* private utils functions */
		bool requestFunc_01_02_03_04(uint8_t functionCode, uint16_t startingAddress, uint16_t quantityOfData);
		int parsingAnswerFunc_01_02(vector <uint8_t>& inputBuffer);
		int parsingAnswerFunc_03_04(vector <uint8_t>& inputBuffer);
		bool requestFunc_05_06(uint8_t functionCode, uint16_t outputAddress);
		int parsingAnswerFunc_05_06(vector <uint8_t>& inputBuffer);
		bool requestFunc_15_16(uint8_t functionCode, uint16_t startingAddress, uint16_t quantityOfData);
		int parsingAnswerFunc_15_16(vector <uint8_t>& inputBuffer);
};
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
//modbus protocol slave class
class ModbusProtocolSlave: public ModbusProtocolBase
{
	public:
		/* constructor */
		ModbusProtocolSlave()
		{

		}

		/* destructor */
		~ModbusProtocolSlave()
		{

		}

		/* parse input packet (in buffer) */
		void inputPacketParse(uint8_t* inputBuffer, size_t inputLen);

	private:

		/* modbus functions processing */
		int processingFunc01_02(inputPackTemplateF01F04* inputPacket);
		int processingFunc03_04(inputPackTemplateF01F04* inputPacket);
		int processingFunc05(inputPackTemplateF05F06* inputPacket);
		int processingFunc06(inputPackTemplateF05F06* inputPacket);
		int processingFunc15(inputPackTemplateF15F16* inputPacket);
		int processingFunc16(inputPackTemplateF15F16* inputPacket);
		int processingExceptionResponse(inputPackTemplateF01F04* inputPacket, modbusExceptionCode excepCode);
};
/*-----------------------------------------------------------------------------------------------------------------------------*/

#endif
