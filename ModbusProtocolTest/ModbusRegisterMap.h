//*********************************************************************************************************//
//MODBUS protocol implementation in C++
//MODBUS register map header file. Create MODBUS registers map, load from/save to JSON.
//Created 01.05.2020
//Created by Novikov Dmitry
//*********************************************************************************************************//

#ifndef MODBUS_REGISTER_MAP
#define MODBUS_REGISTER_MAP

#include <stdint.h>
#include <map>
#include <string>
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filewritestream.h"

using std::string;
using std::map;

/* modbus data types enumeration */
//OneBit - discrete input/coil
//XXToFloat - integer with virtual decimal points as "DecimalPoints" JSON parameter
//FileRecord = uint16_t = one modbus register containing file number
enum ModbusDataType
{
	UnknownDataType = 0,
	OneBit,
	UInt16,
	SInt16,
	UInt32,
	SInt32,
	Float32,
	Char2Byte,
	Char4Byte,
	UInt16ToFloat,
	SInt16ToFloat,
	UInt32ToFloat,
	SInt32ToFloat,
	FileRecord,
	FirstDataType = UnknownDataType,
	LastDataType = FileRecord
};

/* ---------------------------------------------------------------------------------------------------------------------------- */
/* base modbus element class */
class ModbusElementBase
{
	public:
		/* constructor */
		ModbusElementBase(const char* registerName,
							uint8_t& functionCode,
							uint16_t& registerAddress,
							uint16_t& bytesCount,
							ModbusDataType& dataType,
							uint8_t& decimalPoints,
							const char* registerUnit)
			: FunctionCode(functionCode), RegisterAddress(registerAddress), BytesCount(bytesCount), DataType(dataType), DecimalPoints(decimalPoints)
		{
			//create and copy c-strings, if exist
			size_t strSize;
			if (registerName && (strSize = strlen(registerName)) )
			{
				this->RegisterName = new char[strSize + 1]();
				strcpy_s(this->RegisterName, strSize + 1, registerName);
			}
			if (registerUnit && (strSize = strlen(registerUnit)) )
			{
				this->RegisterUnit = new char[strSize + 1]();
				strcpy_s(this->RegisterUnit, strSize + 1, registerUnit);
			}
		}

		/* destructor */
		virtual ~ModbusElementBase()
		{
			//delete c-strings, if exist
			if (this->RegisterName)
			{
				delete this->RegisterName;
			}
			if (this->RegisterUnit)
			{
				delete this->RegisterUnit;
			}
		}

		/* get this object */
		virtual ModbusElementBase* GetModElObject()
		{
			return this;
		}

		/* get RegisterName */
		const char* GetRegisterName() const
		{
			return this->RegisterName;
		}

		/* set & get FunctionCode */
		void SetFunctionCode(uint8_t& functionCode)
		{
			this->FunctionCode = functionCode;
		}
		uint8_t GetFunctionCode() const
		{
			return this->FunctionCode;
		}

		/* set & get RegisterAddress */
		void SetRegisterAddress(uint16_t& registerAddress)
		{
			this->RegisterAddress = registerAddress;
		}
		uint16_t GetRegisterAddress() const
		{
			return this->RegisterAddress;
		}

		/* set & get BytesCount */
		void SetBytesCount(uint16_t& bytesCount)
		{
			this->BytesCount = bytesCount;
		}
		uint16_t GetBytesCount() const
		{
			return this->BytesCount;
		}

		/* set & get DataType */
		void SetDataType(ModbusDataType& dataType)
		{
			this->DataType = dataType;
		}
		ModbusDataType GetDataType() const
		{
			return this->DataType;
		}

		/* set & get DecimalPoints */
		void SetDecimalPoints(uint8_t& decimalPoints)
		{
			this->DecimalPoints = decimalPoints;
		}
		uint8_t GetDecimalPoints() const
		{
			return this->DecimalPoints;
		}

		/* get RegisterUnit */
		const char* GetRegisterUnit() const
		{
			return this->RegisterUnit;
		}

	private:
		char* RegisterName = nullptr;
		uint8_t FunctionCode;
		uint16_t RegisterAddress;
		uint16_t BytesCount;
		ModbusDataType DataType;
		uint8_t DecimalPoints;
		char* RegisterUnit = nullptr;
};
/* ---------------------------------------------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------------------------------------------- */
/* modbus registers map one element main class */
template <typename ModElType>
class ModbusElement : public ModbusElementBase
{
	public:
		/* constructor */
		ModbusElement(const char* registerName,
						uint8_t& functionCode,
						uint16_t& registerAddress,
						uint16_t& bytesCount,
						ModbusDataType& dataType,
						uint8_t& decimalPoints,
						ModElType& dataValue,
						ModElType& minDataValue,
						ModElType& maxDataValue,
						const char* registerUnit)
			: ModbusElementBase(registerName, functionCode, registerAddress, bytesCount, dataType, decimalPoints, registerUnit),
			DataValue(dataValue),
			MinDataValue(minDataValue),
			MaxDataValue(maxDataValue)
		{
		}

		/* destructor */
		~ModbusElement()
		{
		}

		/* get this object */
		virtual ModbusElement <ModElType>* GetModElObject() override
		{
			return (ModbusElement <ModElType>*)this;
		}

		/* set & get DataValue */
		void SetDataValue(ModElType& dataValue)
		{
			this->DataValue = dataValue;
		}
		const ModElType& GetDataValue() const
		{
  			return this->DataValue;
		}

		/* get min data value */
		const ModElType& GetMinDataValue() const
		{
			return this->MinDataValue;
		}

		/* get max data value */
		const ModElType& GetMaxDataValue() const
		{
			return this->MaxDataValue;
		}

	private:
		ModElType DataValue;
		ModElType MinDataValue;
		ModElType MaxDataValue;
};
/* ---------------------------------------------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------------------------------------------- */
/* modbus register map class */
class ModbusRegMap
{
	public:

		/* handler for update (synchronization) changed variable with extrenal data base */
		//result [copied bytes count or -1] = sendVarToExtDataBaseHandler(int varKey, void* varPointer, int varSize)
		//typedef int(*sendVarToExtDataBaseHandler)(int, void*, int);
		//pointer to process file function handler
		//bytesCount = processModbusFileRecord(fileNumber, startRecord, recordsCount, buffer, buffer size in bytes, flag[0 - read, 1 - write])
		//bytesCount <= 0 - error, else OK
		//typedef int(*processModbusFileRecord)(uint16_t, uint16_t, uint16_t, uint8_t*, uint32_t, uint8_t);


		//methods
		/* constructor & destructor */
		ModbusRegMap();
		~ModbusRegMap();
		/* clear register map */
		void Clear();
		/* add new element function */
		template <typename ModElType>
		bool AddNewElement(uint8_t functionCode, uint16_t registerAddress, ModbusDataType dataType, uint16_t bytesCount, const char* registerName,
			uint8_t decimalPoints, ModElType& value, ModElType& minDataValue, ModElType& maxDataValue, const char* registerUnit);
		/* util - get elements count */
		size_t ElementsCount();
		/* element exists check */
		bool ModbusElementExist(uint8_t functionCode, uint16_t registerAddress);
		/* get type of element */
		ModbusDataType GetElementType(uint8_t functionCode, uint16_t registerAddress);
		/* set and get element value */
			//overload #1 - Set
		template <typename ModElType>
		bool SetElementValue(uint8_t functionCode, uint16_t registerAddress, ModElType& value);
			//overload #2 - Set
		template <typename ModElType>
		bool SetElementValue(ModbusElementBase* modbusElementBase, ModElType& value);
			//overload #3 - Set
		bool SetElementValue(uint8_t functionCode, uint16_t registerAddress, uint8_t* buffer, uint16_t bytesCount);
			//overload #1 - Get
		template <typename ModElType>
		bool GetElementValue(uint8_t functionCode, uint16_t registerAddress, const ModElType** value);
			//overload #2 - Get
		template <typename ModElType>
		bool GetElementValue(ModbusElementBase* modbusElementBase, const ModElType** value);
			//overload #3 - Get RAW value
		bool GetElementValue(uint8_t functionCode, uint16_t registerAddress, uint8_t* buffer, uint8_t bufferLength, uint16_t* bytesCount);
		/* load register map from JSON file format */
		bool LoadFromFile(const string& sourceFilePath);
		/* save register map to JSON file format */
		bool SaveToFile(const string& sourceFilePath);

		/* set and get modbus protocol name */
		bool SetModbusProtocolName(string& protocolName)
		{
			this->ProtocolName = protocolName;
			return true;
		}
		const string& GetModbusProtocolName() const
		{
			return this->ProtocolName;
		}

		/* set and get modbus protocol version */
		bool SetModbusProtocolVersion(string& protocolVersion)
		{
			this->ProtocolVersion = protocolVersion;
			return true;
		}
		const string& GetModbusProtocolVersion() const
		{
			return this->ProtocolVersion;
		}

		/* get first register map element */
		ModbusElementBase* GetFirstElement()
		{
			currentElementIter = MainRegMap.begin();
			if (currentElementIter != MainRegMap.end())
			{
				return currentElementIter->second;
			}
			else
			{
				return nullptr;
			}
		}
		/* get next register map element */
		ModbusElementBase* GetNextElement()
		{
			if (currentElementIter == MainRegMap.end()) return nullptr;
			if (++currentElementIter != MainRegMap.end())
			{
				return currentElementIter->second;
			}
			else
			{
				return nullptr;
			}
		}

	private:
		//container with modbus map elements
		map <int, ModbusElementBase*> MainRegMap;
		//iterator for getting elements function
		map <int, ModbusElementBase*>::iterator currentElementIter = MainRegMap.begin();
		//modbus protocol name
		string ProtocolName = "";
		//modbus protocol version
		string ProtocolVersion = "";
		//function - handler for file data access modbus-><-external_file
		//processModbusFileRecord processFileRecordHandler = nullptr;

		//c-strings for access to json file format 
		const char* ModbusProtocolNameStr = "Protocol Name";
		const char* ModbusProtocolVersionStr = "Protocol Version";
		const char* ModbusProtocolRegMapStr = "Registers Map";
		const char* ModbusElFunctionCodeStr = "FuncCode";
		const char* ModbusElAddressStr = "Address";
		const char* ModbusElDataTypeStr = "DataType";
		const char* ModbusDataTypeStrings[ModbusDataType::LastDataType + 1] = { "unknown", "one_bit", "uint16_t", "sint16_t", "uint32_t", "sint32_t",
			"float32", "char[2]", "char[4]", "uint16_to_float", "sint16_to_float", "uint32_to_float", "sint32_to_float", "file_record" };
		const char* ModbusElBytesCountStr = "Bytes";
		const char* ModbusElRegName = "RegName";
		const char* ModbusElDefaultValueStr = "Default";
		const char* ModbusElMinValueStr = "Min";
		const char* ModbusElMaxValueStr = "Max";
		const char* ModbusElDecimalPointsStr = "DecimalPoints";
		const char* ModbusElUnitStr = "Unit";

		/* get modbus element function */
		map <int, ModbusElementBase*>::iterator getModbusElement(uint8_t functionCode, uint16_t registerAddress);
		/* helper function for add one new element to register map */
		template <typename ElDataType>
		bool addNewRegMapElement(rapidjson::Value::ValueIterator elIterator, ModbusDataType jDataType);
		/* helper function for copy element binary data to buffer data */
		template <typename ModElType>
		bool copyElementToRAWData(ModbusElementBase* modElBase, ModbusDataType dataType, uint8_t* buffer, uint8_t bufferLength, uint16_t* bytesCount);
		/* helper function for copy buffer data to element binary data */
		template <typename ModElType>
		bool copyRAWDataToElement(ModbusElementBase* modElBase, ModbusDataType dataType, uint8_t* buffer, uint16_t bytesCount);
		/* helper function for save modbus reg map to JSON */
		template <typename ElDataType>
		bool addDefMinMaxToJSON(ModbusElementBase* modbusElementBase, rapidjson::Value* jsonVal, rapidjson::Document* jsonDoc);
};
/* ---------------------------------------------------------------------------------------------------------------------------- */

#endif