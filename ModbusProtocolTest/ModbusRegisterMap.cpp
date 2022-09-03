//*********************************************************************************************************//
//MODBUS protocol implementation in C++
//MODBUS register map source file. Create MODBUS registers map, load from/save to JSON.
//Created 01.05.2020
//Created by Novikov Dmitry
//*********************************************************************************************************//

#include <fstream>
#include "ModbusRegisterMap.h"

using std::enable_if_t;
using std::is_same;
using std::is_integral;

//defines macro to check value(s) - only return false or return false and clear Obj
#define assertJsonCondition(condition) if ((!condition)) {return false;}
#define assertJsonTwoConditions(condition1, condition2) if (!(condition1 && condition2)) {return false;}
#define assertJsonConditionObj(condition) if ((!condition)) {this->Clear(); return false;}
#define assertJsonTwoConditionsObj(condition1, condition2) if (!(condition1 && condition2)) {this->Clear(); return false;}

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* constructor */
ModbusRegMap::ModbusRegMap()
{
	this->Clear();
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* destructor */
ModbusRegMap::~ModbusRegMap()
{
	this->Clear();
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* clear this object */
void ModbusRegMap::Clear()
{
	//clear map container
	if (this->MainRegMap.size())
	{
		for (map <int, ModbusElementBase*>::iterator iter = this->MainRegMap.begin(); iter != this->MainRegMap.end(); ++iter)
		{
			ModbusElementBase* modbusElement = iter->second->GetModElObject();
			if (modbusElement)
			{
				delete modbusElement;
			}
		}
		this->MainRegMap.clear();
	}
	//clear variables
	this->ProtocolName = "";
	this->ProtocolVersion = "";
	//reset iterator
	currentElementIter = this->MainRegMap.begin();
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* find and get element by function code and register address */
map <int, ModbusElementBase*>::iterator ModbusRegMap::getModbusElement(uint8_t functionCode, uint16_t registerAddress)
{
	//calc key value
	uint32_t key = ((uint32_t)functionCode << 16) | (uint32_t)registerAddress;
	//find element by key
	return this->MainRegMap.find(key);
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* add new element function */
template <typename ModElType>
bool ModbusRegMap::AddNewElement(uint8_t functionCode, uint16_t registerAddress, ModbusDataType dataType, uint16_t bytesCount, const char* registerName,
	uint8_t decimalPoints, ModElType& value, ModElType& minDataValue, ModElType& maxDataValue, const char* registerUnit)
{
	//find existing value
	if (getModbusElement(functionCode, registerAddress) != this->MainRegMap.end())
	{
		return false;
	}
	//check input parameters
	if (!bytesCount) return false;
	if (dataType < ModbusDataType::FirstDataType || dataType > ModbusDataType::LastDataType) return false;
	if (!registerName) return false;

	try
	{
		//new modbus element object
		ModbusElement <ModElType>* newModbusElement = new ModbusElement <ModElType>(registerName, functionCode,
			registerAddress, bytesCount, dataType, decimalPoints, value, minDataValue, maxDataValue, registerUnit);
		if (newModbusElement)
		{
			//calc key value
			uint32_t key = ((uint32_t)functionCode << 16) | (uint32_t)registerAddress;
			//insert new element
			this->MainRegMap.insert(std::pair<int, ModbusElementBase*>(key, (ModbusElementBase*)newModbusElement));
		}
		else
		{
			throw - 1;
		}
	}
	catch (...)
	{
		//cout << "Can't add new modbus object";
		return false;
	}
	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* get elements count */
size_t ModbusRegMap::ElementsCount()
{
	return this->MainRegMap.size();
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* find element by function code and register address, return exist or not */
bool ModbusRegMap::ModbusElementExist(uint8_t functionCode, uint16_t registerAddress)
{
	return this->getModbusElement(functionCode, registerAddress) != this->MainRegMap.end();
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* get element type */
ModbusDataType ModbusRegMap::GetElementType(uint8_t functionCode, uint16_t registerAddress)
{
	map <int, ModbusElementBase*>::iterator elementIterator = getModbusElement(functionCode, registerAddress);
	if (elementIterator == this->MainRegMap.end())
	{
		return ModbusDataType::UnknownDataType;
	}
	ModbusElementBase* modbusElement = elementIterator->second->GetModElObject();
	if (!modbusElement)
	{
		return ModbusDataType::UnknownDataType;
	}
	return modbusElement->GetDataType();
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* local template functions for check type match for json parsing */
//overload #1
template <typename unsignT, class = enable_if_t<is_same<unsignT, uint8_t>::value || is_same<unsignT, uint16_t>::value || is_same<unsignT, uint32_t>::value>, uint8_t = 0>
inline bool checkJsonElValueType(rapidjson::Value* testVal, const char* stringId)
{
	return (*testVal)[stringId].IsUint();
}
//overload #2
template <typename signT, class = enable_if_t<is_same<signT, int8_t>::value || is_same<signT, int16_t>::value || is_same<signT, int32_t>::value>, int8_t = 0>
inline bool checkJsonElValueType(rapidjson::Value* testVal, const char* stringId)
{
	return (*testVal)[stringId].IsInt();
}
//overload #3
template <typename floatT, class = enable_if_t<is_same<floatT, float>::value>, uint16_t = 0>
inline bool checkJsonElValueType(rapidjson::Value* testVal, const char* stringId)
{
	return (*testVal)[stringId].IsFloat();
}//overload #4
template <typename stringT, class = enable_if_t<is_same<stringT, string>::value>, int16_t = 0>
inline bool checkJsonElValueType(rapidjson::Value* testVal, const char* stringId)
{
	return (*testVal)[stringId].IsString();
}
//overload #5
template <typename T, class = enable_if_t<!is_integral<T>::value && !is_same<T, float>::value && !is_same<T, string>::value>, int32_t = 0>
inline bool checkJsonElValueType(rapidjson::Value* testVal, const char* stringId)
{
	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* local template functions to get value of type for json parsing */
//overload #1
template <typename unsignT, class = enable_if_t<is_same<unsignT, uint8_t>::value || is_same<unsignT, uint16_t>::value || is_same<unsignT, uint32_t>::value>, uint8_t = 0>
inline unsignT getJsonTypeValue(rapidjson::Value* testVal, const char* stringId)
{
	return (*testVal)[stringId].GetUint();
}
//overload #2
template <typename signT, class = enable_if_t<is_same<signT, int8_t>::value || is_same<signT, int16_t>::value || is_same<signT, int32_t>::value>, int8_t = 0>
inline signT getJsonTypeValue(rapidjson::Value* testVal, const char* stringId)
{
	return (*testVal)[stringId].GetInt();
}
//overload #3
template <typename floatT, class = enable_if_t<is_same<floatT, float>::value>, uint16_t = 0>
inline floatT getJsonTypeValue(rapidjson::Value* testVal, const char* stringId)
{
	return (*testVal)[stringId].GetFloat();
}
//overload #4
template <typename stringT, class = enable_if_t<is_same<stringT, string>::value>, int16_t = 0>
inline stringT getJsonTypeValue(rapidjson::Value* testVal, const char* stringId)
{
	return (*testVal)[stringId].GetString();
}
//overload #5
template <typename T, class = enable_if_t<!is_integral<T>::value && !is_same<T, float>::value && !is_same<T, string>::value>, int32_t = 0>
inline T getJsonTypeValue(rapidjson::Value* testVal, const char* stringId)
{
	T tVal;
	return tVal;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* local template functions for check min - def - max values */
//overload #1
template <typename intT, class = enable_if_t<is_integral<intT>::value>, int8_t = 0>
inline bool checkMinDefMax(intT def, intT min, intT max)
{
	return ( (def >= min) && (def <= max) );
}
//overload #2
template <typename floatT, class = enable_if_t<is_same<floatT, float>::value>, int16_t = 0>
inline bool checkMinDefMax(floatT def, floatT min, floatT max)
{
	const float precisVal = 1e-10f;
	return (((def > min) || (fabs(def - min) < precisVal)) && ((def < max) || (fabs(max - def) < precisVal)));
}
//overload #3
template <typename T, class = enable_if_t<!is_integral<T>::value && !is_same<T, float>::value>, int32_t = 0>
inline bool checkMinDefMax(T def, T min, T max)
{
	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* util private function for add one new element from json values to register map */
template <typename ElDataType>
bool ModbusRegMap::addNewRegMapElement(rapidjson::Value* elIterator, ModbusDataType jDataType)
{
	//1 [Modbus Function Code]
	assertJsonTwoConditions(elIterator->HasMember(ModbusElFunctionCodeStr), (*elIterator)[ModbusElFunctionCodeStr].IsUint());
	uint8_t jFunctionCode = (*elIterator)[ModbusElFunctionCodeStr].GetUint();

	//2 [Modbus Register Address]
	assertJsonTwoConditions(elIterator->HasMember(ModbusElAddressStr), (*elIterator)[ModbusElAddressStr].IsUint());
	uint16_t jRegisterAddress = (*elIterator)[ModbusElAddressStr].GetUint();

	//3 [Modbus Register Bytes Count]
	assertJsonTwoConditions(elIterator->HasMember(ModbusElBytesCountStr), (*elIterator)[ModbusElBytesCountStr].IsUint());
	uint16_t jBytesCount = (*elIterator)[ModbusElBytesCountStr].GetUint();

	//4 [Modbus Register Text Name]
	assertJsonTwoConditions(elIterator->HasMember(ModbusElRegName), (*elIterator)[ModbusElRegName].IsString());
	const char* jRegisterName = (*elIterator)[ModbusElRegName].GetString();

	//5 [Modbus Register Unit] - except FileRecord type
	const char* jRegisterUnit = nullptr;
	if (jDataType != ModbusDataType::FileRecord)
	{
		assertJsonTwoConditions(elIterator->HasMember(ModbusElUnitStr), (*elIterator)[ModbusElUnitStr].IsString());
		jRegisterUnit = (*elIterator)[ModbusElUnitStr].GetString();
	}

	//6 [Modbus Register Default Value] - except FileRecord type
	ElDataType defVal;
	if (jDataType != ModbusDataType::FileRecord)
	{
		assertJsonTwoConditions(elIterator->HasMember(ModbusElDefaultValueStr),
								checkJsonElValueType<ElDataType>(elIterator, ModbusElDefaultValueStr));
		defVal = getJsonTypeValue<ElDataType>(elIterator, ModbusElDefaultValueStr);
	}

	//7 [Modbus Register Min/Max Value] - except Char2Byte, Char4Byte, FileRecord
	ElDataType minVal;
	ElDataType maxVal;
	if (jDataType != ModbusDataType::Char2Byte && jDataType != ModbusDataType::Char4Byte && jDataType != ModbusDataType::FileRecord)
	{
		assertJsonTwoConditions(elIterator->HasMember(ModbusElMinValueStr),
								checkJsonElValueType<ElDataType>(elIterator, ModbusElMinValueStr));
		assertJsonTwoConditions(elIterator->HasMember(ModbusElMaxValueStr),
								checkJsonElValueType<ElDataType>(elIterator, ModbusElMaxValueStr));
		minVal = getJsonTypeValue<ElDataType>(elIterator, ModbusElMinValueStr);
		maxVal = getJsonTypeValue<ElDataType>(elIterator, ModbusElMaxValueStr);
		//check min-def-max range
		assertJsonCondition(checkMinDefMax<ElDataType>(defVal, minVal, maxVal));
	}

	//8 [Modbus Register Decimal Point Count] - for uint16_to_float, sint16_to_float, uint32_to_float, sint32_to_float
	uint8_t jDecimalPoints = 0;
	if (jDataType == ModbusDataType::UInt16ToFloat || jDataType == ModbusDataType::SInt16ToFloat ||
		jDataType == ModbusDataType::UInt32ToFloat || jDataType == ModbusDataType::SInt32ToFloat)
	{
		assertJsonCondition((*elIterator)[ModbusElDecimalPointsStr].IsUint());
		jDecimalPoints = (uint8_t)(*elIterator)[ModbusElDecimalPointsStr].GetUint();
	}

	//try create new reg map element
	return AddNewElement<ElDataType>(jFunctionCode, jRegisterAddress, jDataType, jBytesCount,
		jRegisterName, jDecimalPoints, defVal, minVal, maxVal, jRegisterUnit);

	return false;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* load modbus registers map from file */
bool ModbusRegMap::LoadFromFile(const string& sourceFilePath)
{
	using std::ifstream;
	using rapidjson::IStreamWrapper;
	using rapidjson::Document;
	using rapidjson::Value;

	//check input data
	if (sourceFilePath.size() < 7)
	{
		return false;
	}
	
	//try open file to file stream
	ifstream inputFileStream(sourceFilePath);
	if (!inputFileStream)
	{
		return false;
	}
	//input stream to rapidjson input stream wrapper
	IStreamWrapper inputFileWrapper(inputFileStream);

	//create json document, read to document, variables
	Document inputDoc;
	Value& jsonVal = inputDoc;
	//parsing input file
	inputDoc.ParseStream(inputFileWrapper);
	//check error
	if (inputDoc.HasParseError())
	{
		return false;
	}

	//clear modbus register map
	this->Clear();

	//read main protocol information
	assertJsonTwoConditionsObj(inputDoc.HasMember(ModbusProtocolNameStr), inputDoc[ModbusProtocolNameStr].IsString());
	this->ProtocolName = inputDoc[ModbusProtocolNameStr].GetString();
	assertJsonTwoConditionsObj(inputDoc.HasMember(ModbusProtocolVersionStr), inputDoc[ModbusProtocolVersionStr].IsString());
	this->ProtocolVersion = inputDoc[ModbusProtocolVersionStr].GetString();
	assertJsonTwoConditionsObj(inputDoc.HasMember(ModbusProtocolRegMapStr), inputDoc[ModbusProtocolRegMapStr].IsArray());
	jsonVal = inputDoc[ModbusProtocolRegMapStr];
	assertJsonConditionObj(jsonVal.GetArray().Capacity());

	//iteration elements and check data type
	int dataType;
	for (auto regMapIter = jsonVal.Begin(); regMapIter != jsonVal.End(); ++regMapIter)
	{
		//9 [Modbus Register Data Type]
		assertJsonTwoConditionsObj(regMapIter->HasMember(ModbusElDataTypeStr), (*regMapIter)[ModbusElDataTypeStr].IsString());
		//find c-string
		for (dataType = ModbusDataType::FirstDataType; dataType < (ModbusDataType::LastDataType + 1); dataType++)
		{
			if (!strcmp((*regMapIter)[ModbusElDataTypeStr].GetString(), ModbusDataTypeStrings[dataType]))
			{
				break;
			}
		}
		//exit, if no matches
		if (dataType == ModbusDataType::LastDataType + 1)
		{
			this->Clear();
			return false;
		}
		//switch, if exist matches
		switch (dataType)
		{
			case ModbusDataType::UnknownDataType:
				this->Clear();
				return false;
			break;
			case ModbusDataType::OneBit:
				assertJsonConditionObj(addNewRegMapElement<uint8_t>(regMapIter, (ModbusDataType)dataType));
			break;
			case ModbusDataType::UInt16:
			case ModbusDataType::UInt16ToFloat:
			case ModbusDataType::FileRecord:
				assertJsonConditionObj(addNewRegMapElement<uint16_t>(regMapIter, (ModbusDataType)dataType));
			break;
			case ModbusDataType::SInt16:
			case ModbusDataType::SInt16ToFloat:
				assertJsonConditionObj(addNewRegMapElement<int16_t>(regMapIter, (ModbusDataType)dataType));
			break;
			case ModbusDataType::UInt32:
			case ModbusDataType::UInt32ToFloat:
				assertJsonConditionObj(addNewRegMapElement<uint32_t>(regMapIter, (ModbusDataType)dataType));
			break;
			case ModbusDataType::SInt32:
			case ModbusDataType::SInt32ToFloat:
				assertJsonConditionObj(addNewRegMapElement<int32_t>(regMapIter, (ModbusDataType)dataType));
			break;
			case ModbusDataType::Float32:
				assertJsonConditionObj(addNewRegMapElement<float>(regMapIter, (ModbusDataType)dataType));
			break;
			case ModbusDataType::Char2Byte:
			case ModbusDataType::Char4Byte:
				assertJsonConditionObj(addNewRegMapElement<string>(regMapIter, (ModbusDataType)dataType));
			break;
		}
	}

	//read complete, return...
 	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
// helper function for save modbus reg map to JSON
template <typename ElDataType>
bool ModbusRegMap::addDefMinMaxToJSON(ModbusElementBase* modbusElementBase, rapidjson::Value* jsonVal, rapidjson::Document* jsonDoc)
{
	using rapidjson::StringRef;

	ElDataType* elValue;
	
	//get element value
	if (!GetElementValue<ElDataType>(modbusElementBase, (const ElDataType**)&elValue))
	{
		return false;
	}
	//add [Modbus Register Default Value]
	jsonVal->AddMember(StringRef(ModbusElDefaultValueStr), *elValue, jsonDoc->GetAllocator());
	
	//add [Modbus Register Min Value]
	jsonVal->AddMember(StringRef(ModbusElMinValueStr), ((ModbusElement <ElDataType>*)modbusElementBase)->GetMinDataValue(), jsonDoc->GetAllocator());

	//add [Modbus Register Max Value]
	jsonVal->AddMember(StringRef(ModbusElMaxValueStr), ((ModbusElement <ElDataType>*)modbusElementBase)->GetMaxDataValue(), jsonDoc->GetAllocator());

	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* save modbus registers map to file */
bool ModbusRegMap::SaveToFile(const string& sourceFilePath)
{
	using rapidjson::Document;
	using rapidjson::PrettyWriter;
	using rapidjson::FileWriteStream;
	using rapidjson::Value;
	using rapidjson::StringRef;

	//check input data
	if (sourceFilePath.size() < 7)
	{
		return false;
	}

	//try open file
	FILE* outputFile = nullptr;
	fopen_s(&outputFile, sourceFilePath.c_str(), "wb");
	if (outputFile == nullptr)
	{
		return false;
	}

	//create json document, writer to document, variables
	Document outputDoc;
	char bufferForOutputStream[1024];
	FileWriteStream outputFileStream(outputFile, bufferForOutputStream, sizeof(bufferForOutputStream));
	PrettyWriter <FileWriteStream> outputFileWriter(outputFileStream);
	outputDoc.SetObject();
	Value jsonVal;

	//function for write string
	auto writeString = [&outputDoc, &jsonVal] (const char* valueName, const string& valueData) mutable
	{
		jsonVal.SetString(StringRef(valueData.c_str()));
		outputDoc.AddMember(StringRef(valueName), jsonVal, outputDoc.GetAllocator());
	};

	//save main information to document
	writeString(ModbusProtocolNameStr, this->ProtocolName);
	writeString(ModbusProtocolVersionStr, this->ProtocolVersion);

	//save registers data to document
	Value jsonRegistersArray;
	jsonRegistersArray.SetArray();
	for (auto regMapIter = this->MainRegMap.cbegin(); regMapIter != this->MainRegMap.cend(); ++regMapIter)
	{
		try
		{
			//json val -> object
			jsonVal.SetObject();
			//write [Modbus Function Code]
			jsonVal.AddMember(StringRef(ModbusElFunctionCodeStr), regMapIter->second->GetFunctionCode(), outputDoc.GetAllocator());
			//write [Modbus Register Address]
			jsonVal.AddMember(StringRef(ModbusElAddressStr), regMapIter->second->GetRegisterAddress(), outputDoc.GetAllocator());
			//write [Modbus Register Data Type]
			jsonVal.AddMember(StringRef(ModbusElDataTypeStr), StringRef(ModbusDataTypeStrings[regMapIter->second->GetDataType()]), outputDoc.GetAllocator());
			//write [Modbus Register Bytes Count]
			jsonVal.AddMember(StringRef(ModbusElBytesCountStr), regMapIter->second->GetBytesCount(), outputDoc.GetAllocator());
			//write [Modbus Register Text Name]
			if (regMapIter->second->GetRegisterName() != nullptr)
			{
				jsonVal.AddMember(StringRef(ModbusElRegName), StringRef(regMapIter->second->GetRegisterName()), outputDoc.GetAllocator());
			}
			else
			{
				jsonVal.AddMember(StringRef(ModbusElRegName), "", outputDoc.GetAllocator());
			}
			
			//get data type and save to JSON depending on data type
			ModbusDataType valDataType = regMapIter->second->GetDataType();
			switch (valDataType)
			{
				case ModbusDataType::UnknownDataType:
					throw 0;
				break;
				case ModbusDataType::OneBit:
					if (!addDefMinMaxToJSON<uint8_t>(regMapIter->second, &jsonVal, &outputDoc)) { throw 0; };
				break;
				case ModbusDataType::UInt16:
				case ModbusDataType::UInt16ToFloat:
					if (!addDefMinMaxToJSON<uint16_t>(regMapIter->second, &jsonVal, &outputDoc)) { throw 0; };
				break;
				case ModbusDataType::SInt16:
				case ModbusDataType::SInt16ToFloat:
					if (!addDefMinMaxToJSON<int16_t>(regMapIter->second, &jsonVal, &outputDoc)) { throw 0; };
				break;
				case ModbusDataType::UInt32:
				case ModbusDataType::UInt32ToFloat:
					if (!addDefMinMaxToJSON<uint32_t>(regMapIter->second, &jsonVal, &outputDoc)) { throw 0; };
				break;
				case ModbusDataType::SInt32:
				case ModbusDataType::SInt32ToFloat:
					if (!addDefMinMaxToJSON<int32_t>(regMapIter->second, &jsonVal, &outputDoc)) { throw 0; };
				break;
				case ModbusDataType::Float32:
					if (!addDefMinMaxToJSON<float>(regMapIter->second, &jsonVal, &outputDoc)) { throw 0; };
				break;
				case ModbusDataType::Char2Byte:
				case ModbusDataType::Char4Byte:
					string* modbusElStr;
					if (!GetElementValue<string>(regMapIter->second, (const string**)&modbusElStr)) { throw 0; }
					else { jsonVal.AddMember(StringRef(ModbusElDefaultValueStr), StringRef(modbusElStr->c_str()), outputDoc.GetAllocator()); }
				break;
			}
			//separately types with additional decimal points
			if (valDataType == ModbusDataType::UInt16ToFloat || valDataType == ModbusDataType::SInt16ToFloat ||
				valDataType == ModbusDataType::UInt32ToFloat || valDataType == ModbusDataType::SInt32ToFloat)
			{
				jsonVal.AddMember(StringRef(ModbusElDecimalPointsStr), regMapIter->second->GetDecimalPoints(), outputDoc.GetAllocator());
			}
			//write [Modbus Register Unit]
			if (regMapIter->second->GetRegisterUnit() != nullptr)
			{
				jsonVal.AddMember(StringRef(ModbusElUnitStr), StringRef(regMapIter->second->GetRegisterUnit()), outputDoc.GetAllocator());
			}
			else
			{
				jsonVal.AddMember(StringRef(ModbusElUnitStr), "", outputDoc.GetAllocator());
			}
			//add json object to array
			jsonRegistersArray.PushBack(jsonVal, outputDoc.GetAllocator());
		}
		catch (...)
		{
			//message here...
			fclose(outputFile);
			return false;
		}
	}
	outputDoc.AddMember(StringRef(ModbusProtocolRegMapStr), jsonRegistersArray, outputDoc.GetAllocator());

	//save to file and close
	outputDoc.Accept(outputFileWriter);
	fclose(outputFile);

	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
//helper function for copy element binary data to buffer data
template <typename ModElType>
bool ModbusRegMap::copyElementToRAWData(ModbusElementBase* modElBase, ModbusDataType dataType, uint8_t* buffer, uint8_t bufferLength, uint16_t* bytesCount)
{
	//get modbus element
	ModbusElement <ModElType>* modbusElement = (ModbusElement <ModElType>*)modElBase->GetModElObject();
	if (!modbusElement)
	{
		return false;
	}
	//if data type = string, special algorithm
	if (dataType == ModbusDataType::Char2Byte)
	{
		if (bufferLength < 2)
		{
			return false;
		}
		ModbusElement <string>* modbusElStr = (ModbusElement <string>*)modElBase->GetModElObject();
		memcpy(buffer, modbusElStr->GetDataValue().c_str(), 2);
		*bytesCount = 2;
		return true;
	}
	else if (dataType == ModbusDataType::Char4Byte)
	{
		if (bufferLength < 4)
		{
			return false;
		}
		ModbusElement <string>* modbusElStr = (ModbusElement <string>*)modElBase->GetModElObject();
		memcpy(buffer, modbusElStr->GetDataValue().c_str(), 4);
		*bytesCount = 4;
		return true;
	}
	else
	{
		if (bufferLength < sizeof(ModElType))
		{
			return false;
		}
		memcpy(buffer, &(modbusElement->GetDataValue()), sizeof(ModElType));
		*bytesCount = sizeof(ModElType);
		return true;
	}
	return false;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
//helper function for copy buffer data to element binary data
template <typename ModElType>
bool ModbusRegMap::copyRAWDataToElement(ModbusElementBase* modElBase, ModbusDataType dataType, uint8_t* buffer, uint16_t bytesCount)
{
	//try access to inherited class
	ModbusElement <ModElType>* modbusElement = (ModbusElement <ModElType>*)modElBase->GetModElObject();
	if (!modbusElement)
	{
		return false;
	}
	//if data type = string, special algorithm
	if ( ((dataType == ModbusDataType::Char2Byte) && (bytesCount == 2)) ||
		 ((dataType == ModbusDataType::Char4Byte) && (bytesCount == 4)) )
	{
		string modbusElStr = string((const char*)buffer);
		((ModbusElement <string>*)modbusElement)->SetDataValue(modbusElStr);
		return true;
	} else
	{
		//standard algorithm for other data types
		//check data type size
		if (bytesCount != sizeof(ModElType))
		{
			return false;
		}
		//create variable for data type and copy data
		ModElType elVar;
		memcpy((void*)&elVar, buffer, bytesCount);
		//check min/max values
		if (!checkMinDefMax<ModElType>(elVar, modbusElement->GetMinDataValue(), modbusElement->GetMaxDataValue()))
		{
			return false;
		}
		//set new value
		modbusElement->SetDataValue(elVar);
		return true;
	}
	return false;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* set and get element value */
//overload #1 - Set
template <typename ModElType>
bool ModbusRegMap::SetElementValue(uint8_t functionCode, uint16_t registerAddress, ModElType& value)
{
	//find modbus reg map element
	map <int, ModbusElementBase*>::iterator elementIterator = getModbusElement(functionCode, registerAddress);
	if (elementIterator == this->MainRegMap.end())
	{
		return false;
	}
	//try access to inherited class
	ModbusElement <ModElType>* modbusElement = (ModbusElement <ModElType>*)elementIterator->second->GetModElObject();
	if (!modbusElement)
	{
		return false;
	}
	//check min/max values
	if (!checkMinDefMax<ModElType>(value, modbusElement->GetMinDataValue(), modbusElement->GetMaxDataValue()))
	{
		return false;
	}
	//set new value
	modbusElement->SetDataValue(value);
	return true;
}

//overload #2 - Set
template <typename ModElType>
bool ModbusRegMap::SetElementValue(ModbusElementBase* modbusElementBase, ModElType& value)
{
	//check input data
	if (modbusElementBase == nullptr)
	{
		return false;
	}
	//check min/max values
	if (!checkMinDefMax<ModElType>(value, ((ModbusElement <ModElType>*)modbusElementBase)->GetMinDataValue(), ((ModbusElement <ModElType>*)modbusElementBase)->GetMaxDataValue()))
	{
		return false;
	}
	//set new value
	((ModbusElement <ModElType>*)modbusElementBase)->SetDataValue(value);
	return true;
}

//overload #3 - Set RAW value
bool ModbusRegMap::SetElementValue(uint8_t functionCode, uint16_t registerAddress, uint8_t* buffer, uint16_t bytesCount)
{
	//check input data
	if (!buffer || !bytesCount)
	{
		return false;
	}

	//find modbus reg map element
	map <int, ModbusElementBase*>::iterator elementIterator = getModbusElement(functionCode, registerAddress);
	if (elementIterator == this->MainRegMap.end())
	{
		return false;
	}

	//parse element depending on type
	switch (elementIterator->second->GetDataType())
	{
	case ModbusDataType::UnknownDataType:
		//no data type
		break;
	case ModbusDataType::OneBit:
		return copyRAWDataToElement<uint8_t>(elementIterator->second, ModbusDataType::OneBit, buffer, bytesCount);
		break;
	case ModbusDataType::UInt16:
	case ModbusDataType::UInt16ToFloat:
	case ModbusDataType::FileRecord:
		return copyRAWDataToElement<uint16_t>(elementIterator->second, ModbusDataType::UInt16, buffer, bytesCount);
		break;
	case ModbusDataType::SInt16:
	case ModbusDataType::SInt16ToFloat:
		return copyRAWDataToElement<int16_t>(elementIterator->second, ModbusDataType::SInt16, buffer, bytesCount);
		break;
	case ModbusDataType::UInt32:
	case ModbusDataType::UInt32ToFloat:
		return copyRAWDataToElement<uint32_t>(elementIterator->second, ModbusDataType::UInt32, buffer, bytesCount);
		break;
	case ModbusDataType::SInt32:
	case ModbusDataType::SInt32ToFloat:
		return copyRAWDataToElement<int32_t>(elementIterator->second, ModbusDataType::SInt32, buffer, bytesCount);
		break;
	case ModbusDataType::Float32:
		return copyRAWDataToElement<float>(elementIterator->second, ModbusDataType::Float32, buffer, bytesCount);
		break;
	case ModbusDataType::Char2Byte:
		return copyRAWDataToElement<string>(elementIterator->second, ModbusDataType::Char2Byte, buffer, bytesCount);
		break;
	case ModbusDataType::Char4Byte:
		return copyRAWDataToElement<string>(elementIterator->second, ModbusDataType::Char4Byte, buffer, bytesCount);
		break;
	}
	return false;
}

//overload #1 - Get
template <typename ModElType>
bool ModbusRegMap::GetElementValue(uint8_t functionCode, uint16_t registerAddress, const ModElType** value)
{
	//check input data
	if (value == nullptr)
	{
		return false;
	}
	//find modbus reg map element
	map <int, ModbusElementBase*>::iterator elementIterator = getModbusElement(functionCode, registerAddress);
	if (elementIterator == this->MainRegMap.end())
	{
		return false;
	}
	//try access to inherited class
	ModbusElement <ModElType>* modbusElement = (ModbusElement <ModElType>*)elementIterator->second->GetModElObject();
	if (!modbusElement)
	{
		return false;
	}
	//get data value
	*value = &(modbusElement->GetDataValue());
	return true;
}

//overload #2 - Get
template <typename ModElType>
bool ModbusRegMap::GetElementValue(ModbusElementBase* modbusElementBase, const ModElType** value)
{
	//check input data
	if (value == nullptr)
	{
		return false;
	}
	if (modbusElementBase == nullptr)
	{
		return false;
	}
	//get data value
	*value = &(((ModbusElement <ModElType>*)modbusElementBase)->GetDataValue());
	return true;
}
//overload #3 - Get RAW value
bool ModbusRegMap::GetElementValue(uint8_t functionCode, uint16_t registerAddress, uint8_t* buffer, uint8_t bufferLength, uint16_t* bytesCount)
{
	//check input data
	if (!buffer || !bufferLength || !bytesCount)
	{
		return false;
	}

	//find modbus reg map element
	map <int, ModbusElementBase*>::iterator elementIterator = getModbusElement(functionCode, registerAddress);
	if (elementIterator == this->MainRegMap.end())
	{
		return false;
	}

	//parse element depending on type
	switch (elementIterator->second->GetDataType())
	{
		case ModbusDataType::UnknownDataType:
			//no data type
		break;
		case ModbusDataType::OneBit:
			return copyElementToRAWData<uint8_t>(elementIterator->second, ModbusDataType::OneBit, buffer, bufferLength, bytesCount);
		break;
		case ModbusDataType::UInt16:
		case ModbusDataType::UInt16ToFloat:
		case ModbusDataType::FileRecord:
			return copyElementToRAWData<uint16_t>(elementIterator->second, ModbusDataType::UInt16, buffer, bufferLength, bytesCount);
		break;
		case ModbusDataType::SInt16:
		case ModbusDataType::SInt16ToFloat:
			return copyElementToRAWData<int16_t>(elementIterator->second, ModbusDataType::SInt16, buffer, bufferLength, bytesCount);
		break;
		case ModbusDataType::UInt32:
		case ModbusDataType::UInt32ToFloat:
			return copyElementToRAWData<uint32_t>(elementIterator->second, ModbusDataType::UInt32, buffer, bufferLength, bytesCount);
		break;
		case ModbusDataType::SInt32:
		case ModbusDataType::SInt32ToFloat:
			return copyElementToRAWData<int32_t>(elementIterator->second, ModbusDataType::SInt32, buffer, bufferLength, bytesCount);
		break;
		case ModbusDataType::Float32:
			return copyElementToRAWData<float>(elementIterator->second, ModbusDataType::Float32, buffer, bufferLength, bytesCount);
		break;
		case ModbusDataType::Char2Byte:
			return copyElementToRAWData<string>(elementIterator->second, ModbusDataType::Char2Byte, buffer, bufferLength, bytesCount);
		break;
		case ModbusDataType::Char4Byte:
			return copyElementToRAWData<string>(elementIterator->second, ModbusDataType::Char4Byte, buffer, bufferLength, bytesCount);
		break;
	}
	return false;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
