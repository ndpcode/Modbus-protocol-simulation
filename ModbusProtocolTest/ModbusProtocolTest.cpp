//*********************************************************************************************************//
//Work with MODBUS example
//Using ModbusRegisterMap, ModbusProtocolHandler, IndustryDataStreamsAL libs
//Created 10.06.2020
//Created by Novikov Dmitry
//*********************************************************************************************************//

#include <iostream>
#include <string>
#include <fstream>
#include <functional>
#include <Windows.h>
#include "ModbusRegisterMap.h"
#include "ModbusProtocolHandler.h"
#include "IndustryDataStreamsAL.h"

int main()
{
	setlocale(LC_ALL, "en_US.UTF-8");
	std::cout << "Start program here...\n";
	
	//create modbus master device
	//experimental
	//ModbusProtocolMaster modbusMaster;
	//modbusMaster.readAllRegisters();
	
	//get exe path
	char exeName[MAX_PATH];
	GetModuleFileNameA(NULL, exeName, MAX_PATH);
	string exePath(exeName);
	exePath = exePath.substr(0, exePath.find_last_of("\\"));
	exePath += "\\";

	//read COM port name and speed
	string comPortName;
	int comPortBR;
	int deviceAddress;
	std::ifstream cfgFile(exePath + "config.ini");
	if (!cfgFile.is_open())
	{
		std::cout << "Can't open configuration file config.ini" << endl;
		std::cout << "Exit..." << endl;
		return 0;
	}
	cfgFile >> comPortName;
	cfgFile >> comPortBR;
	cfgFile >> deviceAddress;
	cfgFile.close();
	if (comPortName.empty())
	{
		std::cout << "Invalid COM port name. Exit..." << endl;
		return 0;
	}
	if (comPortBR < 600 || comPortBR > 256000)
	{
		std::cout << "Invalid COM port baud rate config, port = " << comPortName << ", BR = " << comPortBR << endl;
		std::cout << "Exit..." << endl;
		return 0;
	}
	if (deviceAddress > 255)
	{
		std::cout << "Invalid device address. Exit..." << endl;
		return 0;
	}

	//string for input text
	string inputStr("");

	//create and load registers map
	ModbusRegMap registerMap;
	if (!registerMap.LoadFromFile(exePath + "Region 2.json"))
	{
		std::cout << "Can't load registers map. Exit..." << endl;
		return 0;
	}
	std::cout << "Protocol name = " << registerMap.GetModbusProtocolName() << endl;
	std::cout << "Protocol version = " << registerMap.GetModbusProtocolVersion() << endl;
	//registerMap.SaveToFile(exePath + "SAVED.json");

	//create modbus slave device
	ModbusProtocolSlave modbusSlave;
	if (!modbusSlave.SetRegisterMap(&registerMap) || !modbusSlave.SetDeviceAddress(deviceAddress))
	{
		std::cout << "Can't configuration modbus slave device. Exit..." << endl;
		return 0;
	}

	//configuration and open COM port
	wstring comPortNameW(comPortName.begin(), comPortName.end());
	comPortNameW = L"\\\\.\\" + comPortNameW;
	std::unique_ptr<DataStreamCOM> comStream  = nullptr;
	try
	{
		comStream = std::make_unique<DataStreamCOM>(comPortNameW.c_str(), comPortBR, 8, ONESTOPBIT, NOPARITY);
		if (!comStream)
		{
			throw -1;
		}
	}
	catch (...)
	{
		std::cout << "Can't configuration and open COM port. Exit..." << endl;
		return 0;
	}	
	std::cout << "Open COM port " << comPortName << " at BR = " << comPortBR << endl;
	//set receive and send dunctions and start
	if ( (*comStream).SetDataReceiveFunc(
			std::bind(&ModbusProtocolSlave::inputPacketParse, &modbusSlave, std::placeholders::_1, std::placeholders::_2)
		) &&
		modbusSlave.SetSendDataFunc(
			std::bind(&DataStreamCOM::SendData, comStream.get(), std::placeholders::_1, std::placeholders::_2)
		))
	{
		(*comStream).StreamStart();
		while (inputStr != "exit")
		{
			std::cin >> inputStr;
		}
	}
	(*comStream).StreamStop();

	return 0;
}
