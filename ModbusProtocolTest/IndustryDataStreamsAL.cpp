//*********************************************************************************************************//
//Industry data streams source file.
//Work with serial data streams via COM port or Ethernet
//Created 01.07.2020
//Created by Novikov Dmitry
//*********************************************************************************************************//

#include "IndustryDataStreamsAL.h"

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* start com port data stream */
bool DataStreamCOM::StreamStart()
{
	//check input data
	if (transmitStreamStarted || receiveStreamStarted ||
		transmitThreadWork || receiveThreadWork ||
		this->transmitThread.joinable() || this->receiveThread.joinable())
	{
		return false;
	}

	//try connect to com port
	try
	{
		//access to COM port
		comPortHandle = CreateFile(comPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
		if (comPortHandle == INVALID_HANDLE_VALUE)
		{
			throw (string)"COMStream ERROR: can't open COM port.";
		}

		//set events mask on com port - trigger event
		SetCommMask(comPortHandle, EV_RXCHAR);
		//set size input and output buffers
		SetupComm(comPortHandle, 1500, 1500);

		//config timeouts
		COMMTIMEOUTS comPortTimeouts;
		comPortTimeouts.ReadIntervalTimeout = MAXDWORD;
		comPortTimeouts.ReadTotalTimeoutMultiplier = 0;
		comPortTimeouts.ReadTotalTimeoutConstant = this->comPortTotalTimeout;
		comPortTimeouts.WriteTotalTimeoutMultiplier = 0;
		comPortTimeouts.WriteTotalTimeoutConstant = this->comPortTotalTimeout;

		if (!SetCommTimeouts(comPortHandle, &comPortTimeouts))
		{
			CloseHandle(comPortHandle);
			comPortHandle = INVALID_HANDLE_VALUE;
			throw (string)"COMStream ERROR: can't config COM port timeouts.";
		}

		//config DCB
		DCB dcbCOMParams;
		memset(&dcbCOMParams, 0, sizeof(dcbCOMParams));
		dcbCOMParams.DCBlength = sizeof(DCB);
		GetCommState(comPortHandle, &dcbCOMParams);
		dcbCOMParams.BaudRate = DWORD(this->baudRate);
		dcbCOMParams.ByteSize = this->byteSize;
		dcbCOMParams.StopBits = this->stopBits;
		dcbCOMParams.Parity = this->parity;
		//dcbCOMParams.fAbortOnError = TRUE;
		//dcbCOMParams.fDtrControl = DTR_CONTROL_DISABLE;
		//dcbCOMParams.fRtsControl = RTS_CONTROL_DISABLE;
		//dcbCOMParams.fBinary = TRUE;
		//dcbCOMParams.fParity = FALSE;
		//dcbCOMParams.fInX = FALSE;
		//dcbCOMParams.fOutX = FALSE;
		//dcbCOMParams.XonChar = 0;
		//dcbCOMParams.XoffChar = (unsigned char)0xFF;
		//dcbCOMParams.fErrorChar = FALSE;
		//dcbCOMParams.fNull = FALSE;
		//dcbCOMParams.fOutxCtsFlow = FALSE;
		//dcbCOMParams.fOutxDsrFlow = FALSE;
		//dcbCOMParams.XonLim = 128;
		//dcbCOMParams.XoffLim = 128;

		if (!SetCommState(comPortHandle, &dcbCOMParams))
		{
			CloseHandle(comPortHandle);
			comPortHandle = INVALID_HANDLE_VALUE;
			throw (string)"COMStream ERROR: can't config COM port main parameters.";
		}

	}
	catch (string& s)
	{
		ids_outputErrorMessageA(s.c_str());
		return false;
	}
	catch (...)
	{
		ids_outputErrorMessageA("COMStream ERROR: unknown error during access to COM port.");
		return false;
	}

	//try create threads
	try
	{
		//receive thread
		this->receiveThread = thread(&DataStreamCOM::receiveDataThreadFunction, this);
		if (!this->receiveThread.joinable())
		{
			throw (string)"COMStream ERROR: can't start receive thread or stream.";
		}
	}
	catch (string& s)
	{
		ids_outputErrorMessageA(s.c_str());
		this->StreamStop();
		return false;
	}
	catch (...)
	{
		ids_outputErrorMessageA("COMStream ERROR: unknown error during start system thread.");
		this->StreamStop();
		return false;
	}

	//set flags about stream start
	transmitStreamStarted = true;
	receiveStreamStarted = true;

	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* stop com port data stream */
bool DataStreamCOM::StreamStop()
{
	//stop message for threads
	stopThreadsFlag = true;

	transmitStreamStarted = false;

	if (receiveThread.joinable())
	{
		receiveThread.join();
		receiveStreamStarted = false;
	}
	else if (!receiveThreadWork)
	{
		receiveStreamStarted = false;
	}

	//close COM port
	std::lock_guard <mutex> lock_comPortHandle(mutex_comPortHandle);
	CloseHandle(comPortHandle);
	comPortHandle = INVALID_HANDLE_VALUE;

	//return
	if (!receiveThread.joinable() && !transmitStreamStarted && !receiveStreamStarted)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* send data to port - asynchronous transmit */
bool DataStreamCOM::SendData(uint8_t* data, size_t dataLength)
{
	//check input
	if (!data || !dataLength)
	{
		return false;
	}

	//lock for com port handle
	std::unique_lock <mutex> lock_comPortHandle(mutex_comPortHandle, std::defer_lock);
	//lock for transmit data
	std::unique_lock <mutex> lockTransmit(mutex_transmitDataProtect, std::defer_lock);
	DWORD bytesWritten;
	DWORD waitResult;
	OVERLAPPED sync = { 0 };
	bool writeResult;

	//lock all data for transmit
	lockTransmit.lock();
	if (!this->newTransmitReady)
	{
		ids_outputErrorMessageA("ERROR COM port transmit: transmit process busy.");
		return false;
	}

	if (dataLength)
	{
		this->dataForTransmit = data;
		this->lenOfDataForTransmit = dataLength;
		this->newTransmitReady = false;
		this->lastTransmitState = false;

		try
		{
			//create synchro object
			sync.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			if (sync.hEvent == NULL)
			{
				throw (string)"ERROR COM port transmit: fail create synchronization object.";
			}

			//lock handle and write data
			lock_comPortHandle.lock();
			if (comPortHandle != INVALID_HANDLE_VALUE)
			{
				writeResult = WriteFile(comPortHandle, this->dataForTransmit, this->lenOfDataForTransmit, &bytesWritten, &sync);
				lock_comPortHandle.unlock();
				//check result
				if (!writeResult)
				{
					if (GetLastError() == ERROR_IO_PENDING)
					{
						//ok - wait operation complete
						waitResult = WaitForSingleObject(sync.hEvent, this->comPortWriteTimeout);
						if (waitResult == WAIT_OBJECT_0)
						{
							//lock handle and get result
							lock_comPortHandle.lock();
							if (!GetOverlappedResult(comPortHandle, &sync, &bytesWritten, FALSE))
							{
								//unlock immediatelly
								lock_comPortHandle.unlock();
								//error message
								throw (string)"ERROR COM port transmit: ok send data during timeout, but result of asynchronous operation = false.";
							}
							else
							{
								writeResult = true;
							}
							lock_comPortHandle.unlock();
						}
						else
						{
							throw (string)"ERROR COM port transmit: fail send data during timeout.";
						}
					}
					else
					{
						throw (string)"ERROR COM port transmit: fail write data to COM port.";
					}
				}
				else
				{
					throw (string)"ERROR COM port transmit: fail write data to COM port.";
				}

				//check result
				if (writeResult && (this->lenOfDataForTransmit == bytesWritten))
				{
					this->lastTransmitState = true;
				}
				this->newTransmitReady = true;
				this->lenOfDataForTransmit = 0;
				this->dataForTransmit = nullptr;
			}
			else
			{
				lock_comPortHandle.unlock();
				throw (string)"ERROR COM port transmit: COM port closed.";
			}

			//close event object
			CloseHandle(sync.hEvent);
		}
		catch (string& s)
		{
			if (sync.hEvent != NULL)
			{
				CloseHandle(sync.hEvent);
			}
			ids_outputErrorMessageA(s.c_str());
			this->lastTransmitState = false;
			this->newTransmitReady = true;
			this->dataForTransmit = nullptr;
			this->lenOfDataForTransmit = 0;
			return false;
		}
		catch (...)
		{
			if (sync.hEvent != NULL)
			{
				CloseHandle(sync.hEvent);
			}
			ids_outputErrorMessageA("ERROR COM port transmit: unknown error.");
			this->lastTransmitState = false;
			this->newTransmitReady = true;
			this->dataForTransmit = nullptr;
			this->lenOfDataForTransmit = 0;
			return false;
		}
	}
	//unlock all transmit data
	lockTransmit.unlock();

	return true;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* thread for asynchronous receive data */
void DataStreamCOM::receiveDataThreadFunction()
{
	OVERLAPPED sync = { 0 };
	unsigned long wait, read, state;
	std::unique_lock <mutex> lock_comPortHandle(mutex_comPortHandle, std::defer_lock);

	//change status flag
	receiveThreadWork = true;

	while (1)
	{
		//check exit message for thread
		if (stopThreadsFlag)
		{
			break;
		}

		//try receive
		try
		{
			//create synchro object
			sync.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			if (sync.hEvent == NULL)
			{
				throw (string)"ERROR COM port receive: fail create synchronization object.";
			}

			//lock COM port handle
			lock_comPortHandle.lock();
			//set mask on com port events
			if (SetCommMask(comPortHandle, EV_RXCHAR))
			{
				//reset data
				wait = 0;
				read = 0;
				state = 0;
				//connect com port object and synchro object
				WaitCommEvent(comPortHandle, &state, &sync);
				lock_comPortHandle.unlock();
				//wait data...
				wait = WaitForSingleObject(sync.hEvent, this->comPortReadTimeout);
				//wait..., if data received
				if (wait == WAIT_OBJECT_0)
				{
					lock_comPortHandle.lock();
					//read receive data
					ReadFile(comPortHandle, this->inputBuffer, inputBufferSize, &read, &sync);
					lock_comPortHandle.unlock();
					//wait end reading
					wait = WaitForSingleObject(sync.hEvent, this->comPortReadTimeout);
					//get length of received data
					if (wait == WAIT_OBJECT_0)
					{
						lock_comPortHandle.lock();
						if (GetOverlappedResult(comPortHandle, &sync, &read, FALSE))
						{
							lock_comPortHandle.unlock();
							//call external handler for data - free function
							if (this->dataReceiveFunc)
							{
								this->dataReceiveFunc((uint8_t*)this->inputBuffer, read);
							}
							//call external handler for data - function of object
							if (this->dataReceiveFuncObj)
							{
								this->dataReceiveFuncObj((uint8_t*)this->inputBuffer, read);
							}
						}
						else
						{
							lock_comPortHandle.unlock();
							throw (string)"ERROR COM port receive: ok receive data, but result of asynchronous operation = false.";
						}
					}
					else
					{
						throw (string)"ERROR COM port receive: fail receive data.";
					}
				}
				else
				{
					throw (string)"ERROR COM port receive: fail receive data during timeout.";
				}
			}
			else
			{
				lock_comPortHandle.unlock();
				throw (string)"ERROR COM port receive: fail set mask for event.";
			}
			CloseHandle(sync.hEvent);
		}
		catch (string& s)
		{
			if (sync.hEvent != NULL)
			{
				CloseHandle(sync.hEvent);
			}
			ids_outputErrorMessageA(s.c_str());
			continue;
		}
		catch (...)
		{
			if (sync.hEvent != NULL)
			{
				CloseHandle(sync.hEvent);
			}
			ids_outputErrorMessageA("ERROR COM port transmit: unknown error.");
			continue;
		}
	}

	//change status flag
	receiveThreadWork = false;
}
/*-----------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
/*  */
const vector <wstring>* DataStreamCOM::GetAvailableCOMList()
{
	HANDLE comPortHandle;
	wstring comPortName;

	//clear list
	this->comPortsList.clear();

	//iterate ports
	for (int i = 1; i < 256; i++)
	{
		comPortName = L"\\\\.\\COM" + std::to_wstring(i);
		comPortHandle = CreateFileW(comPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_READONLY, 0);
		if (comPortHandle != INVALID_HANDLE_VALUE)
		{
			(this->comPortsList).push_back(comPortName);
			CloseHandle(comPortHandle);
		}
	}

	return &(this->comPortsList);
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
