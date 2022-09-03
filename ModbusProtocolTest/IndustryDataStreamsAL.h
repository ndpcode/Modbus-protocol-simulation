//*********************************************************************************************************//
//Industry data streams header file.
//Work with serial data streams via COM port or Ethernet
//Created 01.07.2020
//Created by Novikov Dmitry
//*********************************************************************************************************//

#ifndef INDUSTRY_DATA_STREAMS_AL
#define INDUSTRY_DATA_STREAMS_AL

#include <stdint.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <windows.h>
#include <wchar.h>
#include <functional>
#include <iostream>

using std::cout;
using std::wcout;
using std::endl;
using std::string;
using std::wstring;
using std::vector;
using std::function;
using std::thread;
using std::atomic;
using std::mutex;

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* debug interface for output errors */
#define ERROR_OUTPUT_TO_CONSOLE
//#define ERROR_OUTPUT_TO_WINDLG
inline void ids_outputErrorMessageA(const char* text)
{
#ifdef ERROR_OUTPUT_TO_CONSOLE
	cout << text << endl;
#endif
#ifdef ERROR_OUTPUT_TO_WINDLG
	MessageBoxA(NULL, text, "Error", MB_RETRYCANCEL);
#endif
}

inline void ids_outputErrorMessageW(const wchar_t* text)
{
#ifdef ERROR_OUTPUT_TO_CONSOLE
	wcout << text << endl;
#endif
#ifdef ERROR_OUTPUT_TO_WINDLG
	MessageBoxW(NULL, text, L"Error", MB_RETRYCANCEL);
#endif
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* base class for industry data streams */
class IndustryDataStreamAL
{
		//pointer to receive data function, this function is called when port received data
		typedef void(*DataReceiveFunc)(uint8_t*, size_t);
		typedef function <void(uint8_t*, size_t)> DataReceiveFuncObj;

	public:
		//constructor
		IndustryDataStreamAL()
		{
		}

		//destrutor - virtual
		virtual ~IndustryDataStreamAL()
		{
		}

		//data send function - virtual
		virtual bool SendData(uint8_t* data, size_t dataLength) = 0;

		//config extrenal handler for received data
		//overload #1
		bool SetDataReceiveFunc(DataReceiveFunc receiveFunc)
		{
			if (receiveFunc)
			{
				dataReceiveFunc = receiveFunc;
				return true;
			}
			return false;
		}
		//overload #2
		bool SetDataReceiveFunc(DataReceiveFuncObj receiveFunc)
		{
			if (receiveFunc)
			{
				dataReceiveFuncObj = receiveFunc;
				return true;
			}
			return false;
		}

		//start transmit&receive function - virtual
		virtual bool StreamStart() = 0;

		//stop transmit&receive function - virtual
		virtual bool StreamStop() = 0;

		//get last transmit state
		bool GetLastTransmitState() const { return lastTransmitState; }
		bool NewTransmitReady() const { return newTransmitReady; }

	protected:
		//pointer to external handler for received data
		DataReceiveFunc dataReceiveFunc = nullptr;
		DataReceiveFuncObj dataReceiveFuncObj = nullptr;

		//send (transmit) stream state
		bool transmitStreamStarted = false;
		//get (receive) stream state
		bool receiveStreamStarted = false;

		//transmit data thread & function
		thread transmitThread;
		atomic <bool> transmitThreadWork = false;
		virtual void transmitDataThreadFunction() = 0;

		//receive data thread & function
		thread receiveThread;
		atomic <bool> receiveThreadWork = false;
		virtual void receiveDataThreadFunction() = 0;

		//flag as atomic bool to immediately terminate threads
		atomic <bool> stopThreadsFlag = false;

		//flag - last transmit state
		atomic <bool> lastTransmitState = false;
		atomic <bool> newTransmitReady = true;

		//data for transmit thread
		uint8_t* dataForTransmit = nullptr;
		size_t lenOfDataForTransmit = 0;

		//mutex for protect transmit data
		mutex mutex_transmitDataProtect;
};
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* class for COM port data stream */
class DataStreamCOM : public IndustryDataStreamAL
{
	public:
		//constructor 1
		DataStreamCOM()
		{
			//clear com ports list and get new list
			comPortsList.clear();
			GetAvailableCOMList();
		}
		//constructor 2
		DataStreamCOM(const wchar_t* comPortIn, uint32_t baudRateIn, uint8_t byteSizeIn, uint8_t stopBitsIn, uint8_t parityIn)
		{
			//check input data
			try
			{
				if (!comPortIn)
				{
					throw (string)"COMStream ERROR: NULL pointer to COM port name.";
				}
				if (wcslen(comPortIn) < 4)
				{
					throw (string)"COMStream ERROR: bad name of COM port";
				}
				if (baudRateIn == 0 || baudRateIn > 10000000)
				{
					throw (string)"COMStream ERROR:  BaudRate value wrong";
				}
				if (byteSizeIn < 4 || byteSizeIn > 8)
				{
					throw (string)"COMStream ERROR:  byte size value wrong";
				}
				if (stopBitsIn > 2)
				{
					throw (string)"COMStream ERROR:  stop bits value wrong";
				}
				if (parityIn > 4)
				{
					throw (string)"COMStream ERROR:  parity value wrong";
				}
			}
			catch (string& s)
			{
				ids_outputErrorMessageA(s.c_str());
				return;
			}
			//clear com ports list and get new list
			comPortsList.clear();
			GetAvailableCOMList();
			//accept input data
			comPortName.clear();
			comPortName = comPortIn;
			baudRate = baudRateIn;
			byteSize = byteSizeIn;
			stopBits = stopBitsIn;
			parity = parityIn;
		}

		//destructor
		~DataStreamCOM()
		{
			//clear com ports list
			comPortsList.clear();
		}

		//set & get com port name
		bool SetCOMPortName(wchar_t* comPortIn)
		{
			try
			{
				if (!comPortIn)
				{
					throw (string)"COMStream ERROR: NULL pointer to COM port name.";
				}
				if (wcslen(comPortIn) < 4)
				{
					throw (string)"COMStream ERROR: bad name of COM port";
				}
			}
			catch (string& s)
			{
				ids_outputErrorMessageA(s.c_str());
				return false;
			}
			comPortName = comPortIn;
			return true;
		}
		wstring GetCOMPortName() const { return comPortName; }

		//set & get BaudRate
		bool SetBaudRate(uint32_t baudRateIn)
		{
			if (baudRateIn == 0 || baudRateIn > 10000000)
			{
				ids_outputErrorMessageA("COMStream ERROR: BaudRate value wrong");
				return false;
			}
			baudRate = baudRateIn;
			return true;
		}
		uint32_t GetBaudRate() const { return baudRate; }

		//set & get byte size
		bool SetByteSize(uint8_t byteSizeIn)
		{
			if (byteSizeIn < 4 || byteSizeIn > 8)
			{
				ids_outputErrorMessageA("COMStream ERROR: byte size value wrong");
				return false;
			}
			byteSize = byteSizeIn;
			return true;
		}
		uint8_t GetByteSize() const { return byteSize; }

		//set & get stop bits
		bool SetStopBits(uint8_t stopBitsIn)
		{
			if (stopBitsIn > 2)
			{
				ids_outputErrorMessageA("COMStream ERROR: stop bits value wrong");
				return false;
			}
			stopBits = stopBitsIn;
			return true;
		}
		uint8_t GetStopBits() const { return stopBits; }

		//set & get parity
		bool SetParity(uint8_t parityIn)
		{
			if (parityIn > 4)
			{
				ids_outputErrorMessageA("COMStream ERROR: stop bits value wrong");
				return false;
			}
			parity = parityIn;
			return true;
		}
		uint8_t GetParity() const { return parity; }

		//start transmit&receive function
		virtual bool StreamStart() override;

		//stop transmit&receive function
		virtual bool StreamStop() override;

		//data send function - virtual
		virtual bool SendData(uint8_t* data, size_t dataLength) override;

		//function for enum available in OS Windows com ports
		const vector <wstring>* GetAvailableCOMList();

	private:
		//thread function for transmit data
		virtual void transmitDataThreadFunction() {};

		//thread function for receive data
		virtual void receiveDataThreadFunction();

		//-----------com port parameters-----------
		vector <wstring> comPortsList; //available com ports list
		HANDLE comPortHandle = INVALID_HANDLE_VALUE;
		mutex mutex_comPortHandle;
	#define inputBufferSize 1024
		char inputBuffer[inputBufferSize];
		wstring comPortName = L"NAN";
		uint32_t baudRate = 9600;
		uint8_t byteSize = 8;
		uint8_t stopBits = ONESTOPBIT;
		uint8_t parity = NOPARITY;
		const uint32_t comPortTotalTimeout = 1;
		const uint32_t comPortWriteTimeout = 1;
		const uint32_t comPortReadTimeout = 10000;
};
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* class for Ethernet port data stream */
class DataStreamEthernet : public IndustryDataStreamAL
{
	public:
		DataStreamEthernet()
		{
		}
		~DataStreamEthernet()
		{
		}
};
/*-----------------------------------------------------------------------------------------------------------------------------*/

#endif
