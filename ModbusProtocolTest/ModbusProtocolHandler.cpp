//*********************************************************************************************************//
//MODBUS protocol implementation in C++
//MODBUS protocol handler source file. MODBUS master/slave based on ModbusRegisterMap.
//Created 01.06.2020
//Created by Novikov Dmitry
//*********************************************************************************************************//

#include "ModbusProtocolHandler.h"

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* platform specified user defined functions for count timeout */
UINT StartTimeoutTimer(int timeout, function <void()> callbackFunc, UINT timerID)
{
	return SetTimer(0, timerID, timeout, (TIMERPROC)&callbackFunc);
}

bool StopTimeoutTimer(UINT timerID)
{
	return (bool)KillTimer(0, timerID);
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* private util function: not check this object state and port state, not check this object config */
bool ModbusProtocolMaster::requestFunc_01_02_03_04(uint8_t functionCode, uint16_t startingAddress, uint16_t quantityOfData)
{
	//reset data of last request
	this->lastRequestInfo.Reset();

	//fill new request
	this->inputDataBuffer[0] = this->deviceAddress; // 1: address of device
	this->inputDataBuffer[1] = functionCode; // 2: current function code
	this->inputDataBuffer[2] = (uint8_t)(startingAddress >> 8); // 3.2: address of first bit (coil) / register - MSB
	this->inputDataBuffer[3] = (uint8_t)(startingAddress); // 3.1: address of first bit (coil) / register - LSB
	this->inputDataBuffer[4] = (uint8_t)(quantityOfData >> 8); // 4.2: quantity of bits (coils) / registers - MSB
	this->inputDataBuffer[5] = (uint8_t)(quantityOfData); // 4.1: quantity of bits (coils) / registers - LSB
	uint16_t crcVal = ModbusCRC16(&(this->inputDataBuffer[0]), 6); // 5: CRC
	this->inputDataBuffer[6] = (uint8_t)(crcVal >> 8); // 5.2: modbus CRC - MSB
	this->inputDataBuffer[7] = (uint8_t)(crcVal); // 5.1: modbus CRC - LSB

	//send request
 	if (!this->sendDataFunc(&(this->inputDataBuffer[0]), 8))
	{
		return false;
	}

	//save information of request
	this->lastRequestInfo.functionCode = functionCode;
	this->lastRequestInfo.bytesCount = 8;
	this->lastRequestInfo.attemptsCount = this->numberOfAttempts;

	//start timeout timer
	this->lastRequestInfo.timerIdentifier = StartTimeoutTimer(this->responseTimeout,
		std::bind(&ModbusProtocolMaster::timeoutExpired, this),
		this->lastRequestInfo.timerIdentifier);
	if (!this->lastRequestInfo.timerIdentifier)
	{
		this->lastRequestInfo.Reset();
		return false;
	}

	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* private util function: not check this object state and port state */
int ModbusProtocolMaster::parsingAnswerFunc_01_02(vector <uint8_t>& inputBuffer)
{
	//get request info
	int startingAddress = this->inputDataBuffer[2] << 8 | this->inputDataBuffer[3];
	int quantityOfBits = this->inputDataBuffer[4] << 8 | this->inputDataBuffer[5];
	//get packet header
	outputPackTemplateF01F04* packHeader = (outputPackTemplateF01F04*)&inputBuffer[0];

	//validate request and response bits count
	if (quantityOfBits / 8 + (quantityOfBits % 8 != 0) != packHeader->byteCount)
	{
		return -1;
	}

	//parsing packet
	for (int i = 0; i < quantityOfBits; i++)
	{
		uint8_t val = inputBuffer[this->inputPackTemplateF01F04_Size + i / 8] >> (i % 8) & 0x01;
		if (!this->modbusRegisterMap->SetElementValue(packHeader->funcCode, startingAddress + i, &val, 1))
		{
			return -1;
		}
	}
	
	//return packet size in bytes
	return this->inputPackTemplateF01F04_Size + packHeader->byteCount + 2;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* private util function: not check this object state and port state */
int ModbusProtocolMaster::parsingAnswerFunc_03_04(vector <uint8_t>& inputBuffer)
{
	//get request info
	int startingAddress = this->inputDataBuffer[2] << 8 | this->inputDataBuffer[3];
	int quantityOfRegisters = this->inputDataBuffer[4] << 8 | this->inputDataBuffer[5];
	//get packet header
	outputPackTemplateF01F04* packHeader = (outputPackTemplateF01F04*)&inputBuffer[0];

	//validate request and response bytes count
	if (quantityOfRegisters * 2 != packHeader->byteCount)
	{
		return -1;
	}

	//parsing packet
	for (int i = 0; i < quantityOfRegisters; i++)
	{
		uint16_t val = inputBuffer[this->inputPackTemplateF01F04_Size + i*2] << 8 | inputBuffer[this->inputPackTemplateF01F04_Size + i*2 + 1];
		if (!this->modbusRegisterMap->SetElementValue(packHeader->funcCode, startingAddress + i, (uint8_t*)&val, 2))
		{
			return -1;
		}
	}

	//return packet size in bytes
	return this->inputPackTemplateF01F04_Size + packHeader->byteCount + 2;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* private util function: not check this object state and port state, not check this object config */
bool ModbusProtocolMaster::requestFunc_05_06(uint8_t functionCode, uint16_t outputAddress)
{
	//reset data of last request
	this->lastRequestInfo.Reset();

	//read value from database
	uint16_t val{};
	uint16_t valBytesCount{};
	if (!this->modbusRegisterMap->GetElementValue(functionCode, outputAddress, (uint8_t*)&val, 2, &valBytesCount) && valBytesCount != 2)
	{
		return false;
	}

	//if function 0x05 - write single coil
	if (functionCode == 0x05 && val)
	{
		//assign 0xFF00 value if logic "1"
		val = 0xFF00;
	}

	//fill new request
	this->inputDataBuffer[0] = this->deviceAddress;           // 1: address of device
	this->inputDataBuffer[1] = functionCode;                  // 2: current function code
	this->inputDataBuffer[2] = (uint8_t)(outputAddress >> 8); // 3.2: address of coil / register - MSB
	this->inputDataBuffer[3] = (uint8_t)(outputAddress);      // 3.1: address of coil / register - LSB
	this->inputDataBuffer[4] = (uint8_t)(val >> 8);           // 4.2: data of coil / register - MSB
	this->inputDataBuffer[5] = (uint8_t)(val);                // 4.1: data of coil / register - LSB
	uint16_t crcVal = ModbusCRC16(&(this->inputDataBuffer[0]), 6); // 5: CRC
	this->inputDataBuffer[6] = (uint8_t)(crcVal >> 8);        // 5.2: modbus CRC - MSB
	this->inputDataBuffer[7] = (uint8_t)(crcVal);             // 5.1: modbus CRC - LSB

	//send request
	if (!this->sendDataFunc(&(this->inputDataBuffer[0]), 8))
	{
		return false;
	}

	//save information of request
	this->lastRequestInfo.functionCode = functionCode;
	this->lastRequestInfo.bytesCount = 8;
	this->lastRequestInfo.attemptsCount = this->numberOfAttempts;

	//start timeout timer
	this->lastRequestInfo.timerIdentifier = StartTimeoutTimer(this->responseTimeout,
		std::bind(&ModbusProtocolMaster::timeoutExpired, this),
		this->lastRequestInfo.timerIdentifier);
	if (!this->lastRequestInfo.timerIdentifier)
	{
		this->lastRequestInfo.Reset();
		return false;
	}

	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* private util function: not check this object state and port state */
int ModbusProtocolMaster::parsingAnswerFunc_05_06(vector <uint8_t>& inputBuffer)
{
	//check response = request (echo) or not
	for (size_t i = 0; i < this->outputPackTemplateF05F06_Size; i++)
	{
		if (this->inputDataBuffer[i] != inputBuffer[i])
		{
			return -1;
		}
	}
	
	//return packet size in bytes
	return this->outputPackTemplateF05F06_Size;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* private util function: not check this object state and port state, not check this object config */
bool ModbusProtocolMaster::requestFunc_15_16(uint8_t functionCode, uint16_t startingAddress, uint16_t quantityOfData)
{
	//reset data of last request
	this->lastRequestInfo.Reset();

	//fill new request
	this->inputDataBuffer[0] = this->deviceAddress;             // 1: address of device
	this->inputDataBuffer[1] = functionCode;                    // 2: current function code
	this->inputDataBuffer[2] = (uint8_t)(startingAddress >> 8); // 3.2: address of first coil / register - MSB
	this->inputDataBuffer[3] = (uint8_t)(startingAddress);      // 3.1: address of first coil / register - LSB
	this->inputDataBuffer[4] = (uint8_t)(quantityOfData >> 8);  // 4.2: count coils / registers - MSB
	this->inputDataBuffer[5] = (uint8_t)(quantityOfData);       // 4.1: count coils / registers - LSB

	//variant for modbus function 0x0F - write multiple coils
	if (functionCode == 0x0F)
	{
		uint8_t dataByte{};
		uint8_t dataBit{};
		uint16_t valBytesCount{};
		for (int i = 0; i < quantityOfData; i++)
		{
			//get coil value
			if (!this->modbusRegisterMap->GetElementValue(functionCode, startingAddress + i, (uint8_t*)&dataBit, 1, &valBytesCount) && valBytesCount != 1)
			{
				return false;
			}
			valBytesCount = i % 8;
			dataByte |= (dataBit & 0x01) << valBytesCount;
			//save byte to request packet if ready
			if (valBytesCount == 7)
			{
				this->inputDataBuffer[7 + i / 8] = dataByte;
				dataByte = 0;
			}
		}
		this->inputDataBuffer[6] = quantityOfData / 8 + 1; // 5: data bytes count
	}

	//variant for modbus function 0x10 - write multiple registers
	if (functionCode == 0x10)
	{
		uint16_t dataReg{};
		uint16_t valBytesCount{};
		for (int i = 0; i < quantityOfData; i++)
		{
			//get register value
			if (!this->modbusRegisterMap->GetElementValue(functionCode, startingAddress + i, (uint8_t*)&dataReg, 2, &valBytesCount) && valBytesCount != 2)
			{
				return false;
			}
			//save register to request packet
			this->inputDataBuffer[7 + i * 2] = (uint8_t)(dataReg >> 8); // MSB
			this->inputDataBuffer[7 + i * 2 + 1] = (uint8_t)dataReg; // LSB
		}
		this->inputDataBuffer[6] = quantityOfData * 2; // 5: data bytes count
	}

	uint16_t crcVal = ModbusCRC16(&(this->inputDataBuffer[0]), 7 + this->inputDataBuffer[6]); // 5: CRC
	this->inputDataBuffer[7 + this->inputDataBuffer[6]] = (uint8_t)(crcVal >> 8); // 5.2: modbus CRC - MSB
	this->inputDataBuffer[8 + this->inputDataBuffer[6]] = (uint8_t)(crcVal);      // 5.1: modbus CRC - LSB

	//save packet bytes count in temporary variable
	uint16_t* packetBytesCount = &crcVal;
	*packetBytesCount = 9 + this->inputDataBuffer[6];

	//send request
	if (!this->sendDataFunc(&(this->inputDataBuffer[0]), *packetBytesCount))
	{
		return false;
	}

	//save information of request
	this->lastRequestInfo.functionCode = functionCode;
	this->lastRequestInfo.bytesCount = *packetBytesCount;
	this->lastRequestInfo.attemptsCount = this->numberOfAttempts;

	//start timeout timer
	this->lastRequestInfo.timerIdentifier = StartTimeoutTimer(this->responseTimeout,
		std::bind(&ModbusProtocolMaster::timeoutExpired, this),
		this->lastRequestInfo.timerIdentifier);
	if (!this->lastRequestInfo.timerIdentifier)
	{
		this->lastRequestInfo.Reset();
		return false;
	}

	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* private util function: not check this object state and port state */
int ModbusProtocolMaster::parsingAnswerFunc_15_16(vector <uint8_t>& inputBuffer)
{
	//get request data
	uint16_t startingAddress = (uint16_t)this->inputDataBuffer[2] << 8 | this->inputDataBuffer[3];
	uint16_t quantityOfData = (uint16_t)this->inputDataBuffer[4] << 8 | this->inputDataBuffer[5];
	//response data
	outputPackTemplateF15F16* responseData = (outputPackTemplateF15F16*)&inputBuffer[0];

	//check echo data
	if (startingAddress != responseData->regAddress || quantityOfData != responseData->regsCount)
	{
		return -1;
	}

	//return packet size in bytes
	return 9 + this->inputDataBuffer[6];
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/*  */
void ModbusProtocolMaster::timeoutExpired(void)
{
	//check last request function code
	if (!this->lastRequestInfo.functionCode)
	{
		this->masterCurrentState = MM_STATE_FREE;
		return;
	}

	//message
	outputErrorMessage("Last request timeout expired.");

	//if need repeat request
	if (this->lastRequestInfo.attemptsCount)
	{
		//try send request
		if (!this->sendDataFunc(&(this->inputDataBuffer[0]), this->lastRequestInfo.bytesCount))
		{
			outputErrorMessage("Fail repeat last request. Send data error.");
			this->lastRequestInfo.Reset();
			this->masterCurrentState = MM_STATE_FREE;
			return;
		}
		//start timeout timer
		if (!StartTimeoutTimer(this->responseTimeout,
			std::bind(&ModbusProtocolMaster::timeoutExpired, this),
			this->lastRequestInfo.timerIdentifier))
		{
			outputErrorMessage("Fail set request timeout.");
			this->lastRequestInfo.Reset();
			this->masterCurrentState = MM_STATE_FREE;
			return;
		}
		outputErrorMessage("Repeated request sent.");
		this->lastRequestInfo.attemptsCount--;
	}
	else
	{
		this->lastRequestInfo.Reset();
		this->masterCurrentState = MM_STATE_FREE;
	}
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* modbus master - parse input packet */
void ModbusProtocolMaster::inputPacketParse(uint8_t* inputBuffer, size_t inputLen)
{
	//check request info
	if (!this->lastRequestInfo.functionCode || this->masterCurrentState == MM_STATE_FREE)
	{
		//reset and exit
		this->lastRequestInfo.Reset();
		this->masterCurrentState = MM_STATE_FREE;
		return;
	}
	//check input data
	if (!inputBuffer || !inputLen || inputLen > inputBufferMaxSize)
	{
		return;
	}
	//check modbus database access
	if (!this->modbusRegisterMap)
	{
		//reset and exit
		this->lastRequestInfo.Reset();
		this->masterCurrentState = MM_STATE_FREE;
		outputErrorMessage("Modbus master: not set valid Register Map.");
		return;
	}
	//check input buffer free space, erase data if need
	if (inputDataBuffer.size() + inputLen > inputBufferMaxSize)
	{
		if (inputDataBuffer.size() + inputLen - inputBufferMaxSize == 1)
		{
			inputDataBuffer.erase(inputDataBuffer.begin());
		}
		else
		{
			inputDataBuffer.erase(inputDataBuffer.begin(), inputDataBuffer.begin() + (inputDataBuffer.size() + inputLen - inputBufferMaxSize));
		}
	}
	//copy input data to buffer
	inputDataBuffer.insert(inputDataBuffer.end(), inputBuffer, inputBuffer + inputLen);
	//if inputDataBuffer size < [inputPackTemplate_Size] bytes, exit; modbus request never less min base size = inputPackTemplate_Size bytes
	if (inputDataBuffer.size() < this->outputPackTemplateError_Size)
	{
		return;
	}

	//local lambda for check modbus response for functions 01...04
	auto checkModbusResponsePos_F01ToF04 = [this](uint8_t& i) -> bool
	{
		outputPackTemplateF01F04* packHeader = (outputPackTemplateF01F04*)&i;
		if (packHeader->funcCode < 0x01 || packHeader->funcCode > 0x04)
		{
			return false;
		}
		uint16_t* packetCRC = (uint16_t*)(&i + (uint16_t)this->outputPackTemplateF01F04_Size + packHeader->byteCount);
		return ModbusCRC16((uint8_t*)&i, (uint16_t)this->outputPackTemplateF01F04_Size + packHeader->byteCount) == *packetCRC;
	};
	//local lambda for check modbus response for functions 05...06
	auto checkModbusResponsePos_F05ToF06 = [this](uint8_t& i) -> bool
	{
		outputPackTemplateF05F06* packHeader = (outputPackTemplateF05F06*)&i;
		if (packHeader->funcCode != 0x05 && packHeader->funcCode != 0x06)
		{
			return false;
		}
		return ModbusCRC16((uint8_t*)&i, (uint16_t)this->outputPackTemplateF05F06_Size - 2) == packHeader->packetCRC;
	};
	//local lambda for check modbus response for functions 15...16
	auto checkModbusResponsePos_F15ToF16 = [this](uint8_t& i) -> bool
	{
		outputPackTemplateF15F16* packHeader = (outputPackTemplateF15F16*)&i;
		if (packHeader->funcCode != 0x0F && packHeader->funcCode != 0x10)
		{
			return false;
		}
		return ModbusCRC16((uint8_t*)&i, (uint16_t)this->outputPackTemplateF15F16_Size - 2) == packHeader->packetCRC;
	};
	//local lambda for check modbus exception (error) response
	auto checkModbusResponsePos_Error = [this](uint8_t& i) -> bool
	{
		outputPackTemplateError* packHeader = (outputPackTemplateError*)&i;
		return ModbusCRC16((uint8_t*)&i, (uint16_t)this->outputPackTemplateError_Size - 2) == packHeader->packetCRC;
	};
	//local lambda for check modbus response
	auto checkModbusRequestPos = [&](uint8_t& i) -> bool
	{
		return checkModbusResponsePos_F01ToF04(i) || checkModbusResponsePos_F05ToF06(i) ||
			checkModbusResponsePos_F15ToF16(i) || checkModbusResponsePos_Error(i);
	};

	//try find packet in buffer, from begin position to position [end - [this->outputPackTemplateF05F06_Size] byte]
	int packetPos = -1;
	vector<uint8_t>::iterator iterPos = find_if(inputDataBuffer.begin(), inputDataBuffer.end() - this->outputPackTemplateF05F06_Size, checkModbusRequestPos);
	if (iterPos == inputDataBuffer.end())
	{
		//no valid data - no answer
		return;
	}
	else
	{
		packetPos = iterPos - inputDataBuffer.begin();
	}

	//if finded pos > 0, delete excess bytes
	if (packetPos > 0)
	{
		inputDataBuffer.erase(inputDataBuffer.begin(), inputDataBuffer.begin() + packetPos);
		packetPos = 0;
	}

	//access to input packet data
	outputPackTemplateF01F04* outputPacket = (outputPackTemplateF01F04*)&inputDataBuffer[0];

	//count bytes to erase from input packet
	int countInputBytesToErase = 0;

	//check device address and function code
	if (outputPacket->address == this->deviceAddress && (outputPacket->funcCode & 0x7F) == this->lastRequestInfo.functionCode)
	{
		//stop timeout timer
		if (!StopTimeoutTimer(this->lastRequestInfo.timerIdentifier))
		{
			//timer stop error
			outputErrorMessage("Error: timeout timer stop function returned fail.");
			//reset and exit
			this->lastRequestInfo.Reset();
			this->masterCurrentState = MM_STATE_FREE;
			return;
		}
		try
		{
			//if device address match
			//check function code
			switch (outputPacket->funcCode)
			{
				case 0x01:
				case 0x02:
					//read coils - 0x01
					//or read discrete inputs - 0x02
					countInputBytesToErase = parsingAnswerFunc_01_02(inputDataBuffer);
					//if return error code
					if (countInputBytesToErase < 0)
					{
						//calc packet size
						countInputBytesToErase = this->outputPackTemplateF01F04_Size + outputPacket->byteCount + 2;
						//error message
						throw "Error: modbus function 0x01 (0x02) response parsing error.";
					}
				break;
				case 0x03:
				case 0x04:
					//read analog output - read holding registers (0x03)
					//or read analog input - read input registers (0x04)
					countInputBytesToErase = parsingAnswerFunc_03_04(inputDataBuffer);
					//if return error code
					if (countInputBytesToErase < 0)
					{
						//calc packet size
						countInputBytesToErase = this->outputPackTemplateF01F04_Size + outputPacket->byteCount + 2;
						//error message
						throw "Error: modbus function 0x03 (0x04) response parsing error.";
					}
				break;
				case 0x05:
				case 0x06:
					//write single coil - 0x05
					//or write single register - 0x06
					countInputBytesToErase = parsingAnswerFunc_05_06(inputDataBuffer);
					//if return error code
					if (countInputBytesToErase < 0)
					{
						//calc packet size
						countInputBytesToErase = this->outputPackTemplateF05F06_Size;
						//error message
						throw "Error: modbus function 0x05 (0x06) response parsing error.";
					}
				break;
				case 0x0F:
				case 0x10:
					//write multiple coils - 0x0F
					//or write multiple registers - 0x10
					countInputBytesToErase = parsingAnswerFunc_15_16(inputDataBuffer);
					//if return error code
					if (countInputBytesToErase < 0)
					{
						//calc packet size
						countInputBytesToErase = 9 + this->inputDataBuffer[6];
						//error message
						throw "Error: modbus function 0x0F (0x10) response parsing error.";
					}
				break;
				case 0x81:
				case 0x82:
				case 0x83:
				case 0x84:
				case 0x85:
				case 0x86:
				case 0x8F:
				case 0x90:
					{
						//modbus exception
						countInputBytesToErase = outputPackTemplateError_Size;
						outputPackTemplateError* packHeader = (outputPackTemplateError*)&this->inputDataBuffer[0];
						//return exception messages
						if (packHeader->exceptionCode == ILLEGAL_FUNCTION) throw "Error: modbus exception, illegal function.";
						if (packHeader->exceptionCode == ILLEGAL_DATA_ADDRESS) throw "Error: modbus exception, illegal data address.";
						if (packHeader->exceptionCode == ILLEGAL_DATA_VALUE) throw "Error: modbus exception, illegal data value.";
						if (packHeader->exceptionCode == SERVER_DEVICE_FAILURE) throw "Error: modbus exception, server device failure.";
						//unknown exception
						throw "Error: modbus exception, unknown error.";
					}
				break;
				default:
					//error - unknown function code
					//error message
					throw "Error: modbus response unknown function code.";
				break;
			}
		}
		catch (const char* str)
		{
			outputErrorMessage(str);
		}
	}
	else
	{
		//if not match device address
		return;
	}

	//erase prepared packet
	if (countInputBytesToErase)
	{
		inputDataBuffer.erase(inputDataBuffer.begin(), inputDataBuffer.begin() + countInputBytesToErase);
	}

	//reset and exit
	this->lastRequestInfo.Reset();
	this->masterCurrentState = MM_STATE_RESPONSE_READY;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* function for update all values of registers in modbus registers map */
bool ModbusProtocolMaster::readAllRegisters()
{
	//check access to this modbus master object
	if (this->masterCurrentState != MM_STATE_FREE)
	{
		return false;
	}

	//lock access
	this->masterCurrentState = MM_STATE_BUSY;

	//check modbus database access
	if (!this->modbusRegisterMap)
	{
		//unlock
		this->masterCurrentState = MM_STATE_FREE;
		return false;
	}

	for (ModbusElementBase* regMapElement = this->modbusRegisterMap->GetFirstElement();
		regMapElement != nullptr; regMapElement = this->modbusRegisterMap->GetNextElement())
	{
		switch (regMapElement->GetFunctionCode())
		{
			case 0x01:
			case 0x02:
			case 0x03:
			case 0x04:
			//case 0x15:
			//case 0x16:
				//request
				if (!requestFunc_01_02_03_04(regMapElement->GetFunctionCode(), regMapElement->GetRegisterAddress(), 1))
				{
					outputErrorMessage("Modbus master - update all registers function error. Create request fail.");
					//unlock access
					this->masterCurrentState = MM_STATE_FREE;
					//exit
					return false;
				}
				//wait response
				while (this->masterCurrentState == MM_STATE_BUSY) {};
				//check errors
				if (this->masterCurrentState == MM_STATE_FREE)
				{
					outputErrorMessage("Modbus master - update all registers function error.");
					//exit
					return false;
				}
			break;

			//case 0x05:
			//case 0x06:
			//	//request
			//	if (!requestFunc_05_06((&regMapElement)->GetFunctionCode(), (&regMapElement)->GetRegisterAddress()))
			//	{
			//		outputErrorMessage("Modbus master - update all registers function error. Create request fail.");
			//		//unlock access
			//		this->masterCurrentState = MM_STATE_FREE;
			//		//exit
			//		return false;
			//	}
			//	//wait response
			//	while (this->masterCurrentState == MM_STATE_BUSY) {};
			//	//check errors
			//	if (this->masterCurrentState == MM_STATE_FREE)
			//	{
			//		outputErrorMessage("Modbus master - update all registers function error.");
			//		//exit
			//		return false;
			//	}
			//break;
		}
	}

	//unlock access
	this->masterCurrentState = MM_STATE_FREE;
	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* parse input packet (in buffer) */
void ModbusProtocolSlave::inputPacketParse(uint8_t* inputBuffer, size_t inputLen)
{
	//check input data
	if (!inputBuffer || !inputLen || inputLen > inputBufferMaxSize)
	{
		return;
	}

	//check modbus database access
	if (!this->modbusRegisterMap)
	{
		return;
	}

	//check input buffer free space, erase data if need
	if (inputDataBuffer.size() + inputLen > inputBufferMaxSize)
	{
		if (inputDataBuffer.size() + inputLen - inputBufferMaxSize == 1)
		{
			inputDataBuffer.erase(inputDataBuffer.begin());
		}
		else
		{
			inputDataBuffer.erase(inputDataBuffer.begin(), inputDataBuffer.begin() + (inputDataBuffer.size() + inputLen - inputBufferMaxSize));
		}
	}
	//copy input data to buffer
	inputDataBuffer.insert(inputDataBuffer.end(), inputBuffer, inputBuffer + inputLen);
	//if inputDataBuffer size < [inputPackTemplate_Size] bytes, exit; modbus request never less min base size = inputPackTemplate_Size bytes
	if (inputDataBuffer.size() < this->inputPackTemplateF01F04_Size)
	{
		return;
	}

	//local lambda for check modbus request for functions 01...04
	auto checkModbusRequestPos_F01ToF04 = [this](uint8_t& i) -> bool
	{
		if (((inputPackTemplateF01F04*)&i)->funcCode < 0x01 || ((inputPackTemplateF01F04*)&i)->funcCode > 0x04)
		{
			return false;
		}
		return ModbusCRC16((uint8_t*)&i, (uint16_t)this->inputPackTemplateF01F04_Size - 2) == (((inputPackTemplateF01F04*)&i)->packetCRC);
	};
	//local lambda for check modbus request for functions 05...06
	auto checkModbusRequestPos_F05ToF06 = [this](uint8_t& i) -> bool
	{
		if (((inputPackTemplateF05F06*)&i)->funcCode != 0x05 && ((inputPackTemplateF05F06*)&i)->funcCode != 0x06)
		{
			return false;
		}
		return ModbusCRC16((uint8_t*)&i, (uint16_t)this->inputPackTemplateF05F06_Size - 2) == (((inputPackTemplateF01F04*)&i)->packetCRC);
	};
	//local lambda for check modbus request for functions 15...16 - NEED IMPROVEMENT!!!
	auto checkModbusRequestPos_F15ToF16 = [this](uint8_t& i) -> bool
	{
		if (((inputPackTemplateF15F16*)&i)->funcCode != 0x0F && ((inputPackTemplateF15F16*)&i)->funcCode != 0x10)
		{
			return false;
		}
		uint16_t requestLength = (uint16_t)((inputPackTemplateF15F16*)&i)->bytesCount + (uint16_t)this->inputPackTemplateF15F16_Size;
		return ModbusCRC16((uint8_t*)&i, requestLength) == *((uint16_t*)(&i + requestLength));
	};
	//local lambda for check modbus request
	auto checkModbusRequestPos = [&](uint8_t& i) -> bool
	{
		return checkModbusRequestPos_F01ToF04(i) || checkModbusRequestPos_F05ToF06(i) || checkModbusRequestPos_F15ToF16(i);
	};

	//try find packet in buffer, from begin position to position [end - [this->inputPackTemplate_Size] byte]
	int packetPos = -1;
	vector<uint8_t>::iterator iterPos = find_if(inputDataBuffer.begin(), inputDataBuffer.end() - this->inputPackTemplateF01F04_Size, checkModbusRequestPos);
	if (iterPos == inputDataBuffer.end())
	{
		//no valid data - no answer
		return;
	}
	else
	{
		packetPos = iterPos - inputDataBuffer.begin();
	}

	//if finded pos > 0, delete excess bytes
	if (packetPos > 0)
	{
		inputDataBuffer.erase(inputDataBuffer.begin(), inputDataBuffer.begin() + packetPos);
		packetPos = 0;
	}

	//manually rotate bytes - for all modbus functions type
	auto swapBytes = [&](uint8_t a, uint8_t b) { uint8_t c = inputDataBuffer[a]; inputDataBuffer[a] = inputDataBuffer[b]; inputDataBuffer[b] = c; };
	swapBytes(2, 3);
	swapBytes(4, 5);

	//access to input packet data
	inputPackTemplateF01F04* inputPacket = (inputPackTemplateF01F04*)&inputDataBuffer[0];

	//count bytes to erase from input packet
	int countInputBytesToErase = 0;

	//****** read question and create answer in output buffer
	//check device address
	if (inputPacket->address == this->deviceAddress || inputPacket->address == 0)
	{
		try
		{
			//if device address match
			//clear output data buffer
			outputDataBuffer.clear();
			//add device address to output buffer
			outputDataBuffer.push_back(inputPacket->address);
			//check function code
			switch (inputPacket->funcCode)
			{
				case 0x01:
				case 0x02:
					//read coils - 0x01
					//or read discrete inputs - 0x02
					countInputBytesToErase = processingFunc01_02(inputPacket);
				break;
				case 0x03:
				case 0x04:
					//read analog output - read holding registers (0x03)
					//or read analog input - read input registers (0x04)
					countInputBytesToErase = processingFunc03_04(inputPacket);
				break;
				case 0x05:
					//write single coil - 0x05
					countInputBytesToErase = processingFunc05((inputPackTemplateF05F06*)inputPacket);
				break;
				case 0x06:
					//write single register - 0x06
					countInputBytesToErase = processingFunc06((inputPackTemplateF05F06*)inputPacket);
				break;
				case 0x0F:
					//write multiple coils - 0x0F
					countInputBytesToErase = processingFunc15((inputPackTemplateF15F16*)inputPacket);
				break;
				case 0x10:
					//write multiple registers - 0x10
					countInputBytesToErase = processingFunc16((inputPackTemplateF15F16*)inputPacket);
				break;
				default:
					//error - unknown function code
					throw modbusExceptionCode::ILLEGAL_FUNCTION;
				break;
			}
		}
		catch (modbusExceptionCode excepCode)
		{
			countInputBytesToErase = processingExceptionResponse(inputPacket, excepCode);
		}
	}
	else
	{
		//if not match device address
		// - no answer
	}

	//send answer, if output buffer not empty, set send data function, device address not zero (not response for broadcast)
	if (outputDataBuffer.size() && this->sendDataFunc && inputPacket->address != 0)
	{
		//calc modbus CRC16
		uint16_t outputCRC = ModbusCRC16(&outputDataBuffer[0], (uint16_t)outputDataBuffer.size());
		//add CRC16
		outputDataBuffer.push_back((uint8_t)(outputCRC & 0x00FF));
		outputDataBuffer.push_back((uint8_t)((outputCRC & 0xFF00) >> 8));
		//send
		this->sendDataFunc(&outputDataBuffer[0], outputDataBuffer.size());
	}

	//erase prepared packet
	if (countInputBytesToErase)
	{
		inputDataBuffer.erase(inputDataBuffer.begin(), inputDataBuffer.begin() + countInputBytesToErase);
	}
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* modbus functions 01 and 02 processing - 0x01 Read Coils and 0x02 Read Discrete Inputs */
int ModbusProtocolSlave::processingFunc01_02(inputPackTemplateF01F04* inputPacket)
{
	//check quantity of coils
	if (!inputPacket->regsCount || inputPacket->regsCount > 0x07D0)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_VALUE;
	}
	//check address range
	if ((uint32_t)inputPacket->regAddress + (uint32_t)inputPacket->regsCount - 1 > 0x0000FFFF)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
	}
	//add data to output buffer
	//add function code
	outputDataBuffer.push_back(inputPacket->funcCode);	
	//add output data bytes count
	uint8_t oneDataByte = inputPacket->regsCount / 8 + (inputPacket->regsCount % 8 > 0);
	outputDataBuffer.push_back(oneDataByte);
	//add registers data
	uint8_t outputValueBuf;
	uint16_t outputValueBytesCount = 0;
	uint8_t bitNumber = 0;
	oneDataByte = 0;
	for (int i = inputPacket->regAddress; i < (inputPacket->regAddress + inputPacket->regsCount); i++)
	{
		//database request && check count of returned bytes
		if (this->modbusRegisterMap->GetElementValue(inputPacket->funcCode, i, &outputValueBuf, 1, &outputValueBytesCount) && outputValueBytesCount)
		{
			//add one bit to output byte
			oneDataByte |= (outputValueBuf & 0x01) << bitNumber++;
			//if bits count > 7, add to output buffer and clear
			if (bitNumber > 7)
			{
				outputDataBuffer.push_back(oneDataByte);
				oneDataByte = 0;
				bitNumber = 0;
			}
		}
		else
		{
			//error - unknown address of register
			throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
		}
	}
	//check last data byte, add to output buffer if not empty
	if (bitNumber)
	{
		outputDataBuffer.push_back(oneDataByte);
	}

	//return size of input packet
	return this->inputPackTemplateF01F04_Size;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* modbus function 03 and 04 processing - 0x03 Read Holding Registers and 0x04 Read Input Registers */
int ModbusProtocolSlave::processingFunc03_04(inputPackTemplateF01F04* inputPacket)
{
	//check quantity of registers
	if (!inputPacket->regsCount || inputPacket->regsCount > 0x007D)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_VALUE;
	}
	//check address range
	if ((uint32_t)inputPacket->regAddress + (uint32_t)inputPacket->regsCount - 1 > 0x0000FFFF)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
	}
	//add data to output buffer
	//add function code
	outputDataBuffer.push_back(inputPacket->funcCode);
#ifdef MODBUS_SLAVE_DEBUG
	//add bytes count
	outputDataBuffer.push_back(inputPacket->regsCount * 2);
	//add data registers
	for (int i = inputPacket->regAddress; i < (inputPacket->regAddress + inputPacket->regsCount); i++)
	{
		outputDataBuffer.push_back((uint8_t)((i & 0xFF00) >> 8));
		outputDataBuffer.push_back((uint8_t)(i & 0x00FF));
	}
#else
	uint8_t outputValueBuf[4];
	uint16_t outputValueBytesCount = 0;
	//add bytes count = 0
	outputDataBuffer.push_back(0);
	//add registers data
	for (uint16_t i = inputPacket->regAddress; i < (inputPacket->regAddress + inputPacket->regsCount); i++)
	{
		//database request && check count of returned bytes
		if (this->modbusRegisterMap->GetElementValue(inputPacket->funcCode, i, outputValueBuf, 4, &outputValueBytesCount) && outputValueBytesCount)
		{
			while (outputValueBytesCount--)
			{
				outputDataBuffer.push_back(outputValueBuf[outputValueBytesCount]);
			}
		}
		else
		{
			//error - unknown address of register
			throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
		}
	}
	//update bytes count
	outputDataBuffer[2] = (uint8_t)outputDataBuffer.size() - 3;
#endif

	//return size of input packet
	return this->inputPackTemplateF01F04_Size;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
// modbus function 05 processing - 0x05 Write Single Coil
int ModbusProtocolSlave::processingFunc05(inputPackTemplateF05F06* inputPacket)
{
	//check output value
	if (inputPacket->regValue != 0x0000 && inputPacket->regValue != 0xFF00)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_VALUE;
	}
	//try set new register value
	uint8_t newRegValue = (inputPacket->regValue == 0xFF00) ? 0x01 : 0x00;
	if (!this->modbusRegisterMap->SetElementValue(inputPacket->funcCode, inputPacket->regAddress, &newRegValue, 1))
	{
		throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
	}
	//add data to output buffer
	//add function code
	outputDataBuffer.push_back(inputPacket->funcCode);
	//add register address
	outputDataBuffer.push_back((uint8_t)((inputPacket->regAddress & 0xFF00) >> 8));
	outputDataBuffer.push_back((uint8_t)(inputPacket->regAddress & 0x00FF));
	//add register value
	outputDataBuffer.push_back((uint8_t)((inputPacket->regValue & 0xFF00) >> 8));
	outputDataBuffer.push_back((uint8_t)(inputPacket->regValue & 0x00FF));

	//return size of input packet
	return this->inputPackTemplateF05F06_Size;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
// modbus function 06 processing - 0x06 Write Single Register
int ModbusProtocolSlave::processingFunc06(inputPackTemplateF05F06* inputPacket)
{
	//try set new register value
	if (!this->modbusRegisterMap->SetElementValue(inputPacket->funcCode, inputPacket->regAddress, (uint8_t*)&inputPacket->regValue, 2))
	{
		throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
	}
	//add data to output buffer
	//add function code
	outputDataBuffer.push_back(inputPacket->funcCode);
	//add register address
	outputDataBuffer.push_back((uint8_t)((inputPacket->regAddress & 0xFF00) >> 8));
	outputDataBuffer.push_back((uint8_t)(inputPacket->regAddress & 0x00FF));
	//add register value
	outputDataBuffer.push_back((uint8_t)((inputPacket->regValue & 0xFF00) >> 8));
	outputDataBuffer.push_back((uint8_t)(inputPacket->regValue & 0x00FF));

	//return size of input packet
	return this->inputPackTemplateF05F06_Size;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
// modbus function 15 processing - 0x0F Write Multiple Coils
int ModbusProtocolSlave::processingFunc15(inputPackTemplateF15F16* inputPacket)
{
	//check quantity of coils
	if (!inputPacket->regsCount || inputPacket->regsCount > 0x07B0)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_VALUE;
	}
	//check bytes count
	uint8_t dataBytesCount = inputPacket->regsCount / 8 + (inputPacket->regsCount % 8 > 0);
	if (inputPacket->bytesCount != dataBytesCount)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_VALUE;
	}
	//check address range
	if ((uint32_t)inputPacket->startRegAddress + (uint32_t)inputPacket->regsCount - 1 > 0x0000FFFF)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
	}
	//write coils
	uint16_t currentReg = 0;
	for (int i = 0; i < inputPacket->bytesCount; i++)
	{
		uint8_t oneDataByte = *((uint8_t*)(inputPacket + this->inputPackTemplateF15F16_Size + i));
		for (int bitNumber = 0; bitNumber < 8 && currentReg < inputPacket->regsCount; bitNumber++, currentReg++)
		{
			uint8_t newRegValue = (oneDataByte & (0x01 << bitNumber)) != 0;
			if (!this->modbusRegisterMap->SetElementValue(inputPacket->funcCode, inputPacket->startRegAddress + currentReg, &newRegValue, 1))
			{
				throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
			}
		}
	}
	//add data to output buffer
	//add function code
	outputDataBuffer.push_back(inputPacket->funcCode);
	//add start register address
	outputDataBuffer.push_back((uint8_t)((inputPacket->startRegAddress & 0xFF00) >> 8));
	outputDataBuffer.push_back((uint8_t)(inputPacket->startRegAddress & 0x00FF));
	//add quantity of coils
	outputDataBuffer.push_back((uint8_t)((inputPacket->regsCount & 0xFF00) >> 8));
	outputDataBuffer.push_back((uint8_t)(inputPacket->regsCount & 0x00FF));

	//return size of input packet
	return (this->inputPackTemplateF15F16_Size + inputPacket->bytesCount);
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
// modbus function 16 processing - 0x10 Write Multiple Registers
int ModbusProtocolSlave::processingFunc16(inputPackTemplateF15F16* inputPacket)
{
	//check quantity of registers
	if (!inputPacket->regsCount || inputPacket->regsCount > 0x007B)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_VALUE;
	}
	//check bytes count
	if (inputPacket->bytesCount != inputPacket->regsCount * 2)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_VALUE;
	}
	//check address range
	if ((uint32_t)inputPacket->startRegAddress + (uint32_t)inputPacket->regsCount - 1 > 0x0000FFFF)
	{
		throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
	}
	//write registers
	for (uint16_t i = 0; i < inputPacket->regsCount; i++)
	{
		uint16_t newRegValue = *((uint8_t*)(inputPacket + this->inputPackTemplateF15F16_Size + i * 2));
		newRegValue = (newRegValue << 8) & 0xFF00;
		newRegValue += *((uint8_t*)(inputPacket + this->inputPackTemplateF15F16_Size + i * 2 + 1));
		if (!this->modbusRegisterMap->SetElementValue(inputPacket->funcCode, inputPacket->startRegAddress + i, (uint8_t*)&newRegValue, 2))
		{
			throw modbusExceptionCode::ILLEGAL_DATA_ADDRESS;
		}
	}
	//add data to output buffer
	//add function code
	outputDataBuffer.push_back(inputPacket->funcCode);
	//add start register address
	outputDataBuffer.push_back((uint8_t)((inputPacket->startRegAddress & 0xFF00) >> 8));
	outputDataBuffer.push_back((uint8_t)(inputPacket->startRegAddress & 0x00FF));
	//add quantity of registers
	outputDataBuffer.push_back((uint8_t)((inputPacket->regsCount & 0xFF00) >> 8));
	outputDataBuffer.push_back((uint8_t)(inputPacket->regsCount & 0x00FF));

	//return size of input packet
	return (this->inputPackTemplateF15F16_Size + inputPacket->bytesCount);
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* modbus exception processing */
int ModbusProtocolSlave::processingExceptionResponse(inputPackTemplateF01F04* inputPacket, modbusExceptionCode excepCode)
{
	//clear all already added data - clear output data buffer
	outputDataBuffer.clear();
	//add data to output buffer
	//add device address
	outputDataBuffer.push_back(inputPacket->address);
	//add function code with error bit
	outputDataBuffer.push_back(inputPacket->funcCode | 0x80);
	//add exception code
	outputDataBuffer.push_back((uint8_t)excepCode);

	//return size of input packet - now full size of input buffer
	return this->inputDataBuffer.size();
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* calculate modbus crc16 */
uint16_t ModbusCRC16(const uint8_t *inputData, uint16_t dataLength)
{
	static const uint16_t wCRCTable[] = {
	0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
	0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
	0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
	0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
	0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
	0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
	0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
	0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
	0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
	0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
	0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
	0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
	0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
	0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
	0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
	0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
	0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
	0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
	0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
	0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
	0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
	0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
	0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
	0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
	0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
	0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
	0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
	0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
	0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
	0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
	0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
	0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

	uint8_t nTemp;
	uint16_t wCRCWord = 0xFFFF;

	while (dataLength--)
	{
		nTemp = *inputData++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	}
	return wCRCWord;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
