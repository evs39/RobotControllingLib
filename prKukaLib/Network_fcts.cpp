#include "KUKA.h"

int KUKA::ROBOT_KRC4::iWinSockINI()
{
	KUKA::ROBOT_KRC4::ver = MAKEWORD(2, 2);
	return WSAStartup(KUKA::ROBOT_KRC4::ver, &KUKA::ROBOT_KRC4::wsaData);
}

void KUKA::ROBOT_KRC4::vWinSockClean()
{
	WSACleanup();
}


void KUKA::ROBOT_KRC4::vSocketSettings(std::string sIP,
	unsigned short usPN,
	unsigned short usSockNumber)
{
	switch (usSockNumber)
	{
	case 0:
	{
		this->strIP_SCR = sIP;
		this->usPortNumber_SCR = usPN;
		break;
	}
	case 1:
	{
		this->strIP_SSCM = sIP;
		this->usPortNumber_SSCM = usPN;
		break;
	}
	}
}

int KUKA::ROBOT_KRC4::iStartClient(unsigned short usSockNumb)
{
	switch (usSockNumb)
	{
	case 0:
	{
		this->serverInfo[usSockNumb].sin_addr.s_addr = inet_addr(this->strIP_SCR.c_str());
		this->serverInfo[usSockNumb].sin_port = htons(this->usPortNumber_SCR);
		break;
	}
	case 1:
	{
		this->serverInfo[usSockNumb].sin_addr.s_addr = inet_addr(this->strIP_SSCM.c_str());
		this->serverInfo[usSockNumb].sin_port = htons(this->usPortNumber_SSCM);
		break;
	}
	default:
	{
		return 1;
	}
	}

	this->serverInfo[usSockNumb].sin_family = AF_INET;

	this->clientSock[usSockNumb] = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	

	if (clientSock[usSockNumb] == SOCKET_ERROR)
	{
		return (SOCKET_ERROR * usSockNumb);
	}

	if (connect(clientSock[usSockNumb], (sockaddr*)(&serverInfo[usSockNumb]), sizeof(serverInfo[usSockNumb])) == SOCKET_ERROR)
	{
		closesocket(clientSock[usSockNumb]);
		return (SOCKET_ERROR * usSockNumb);
	}

	return 0;
}

void KUKA::ROBOT_KRC4::vDataSend(unsigned short usSockNumb)
{
	iErrorOfObject = 0;

	switch (usSockNumb)
	{
	case SOCKET_FOR_RC:
	{
		iErrorOfObject = send(this->clientSock[usSockNumb], this->strControl.c_str(), this->strControl.size(), 0);

		if (iErrorOfObject == SOCKET_ERROR)
		{
			//std::cout << "Control server is not available\n" << std::endl;
			return;
		}
		break;
		this->strControl.clear();
	}
	case SOCKET_FOR_SCM:
	{
		while (this->ucSSCM_CurBuffer == 0xff)
		{
			Sleep(1000);

			if (send(this->clientSock[usSockNumb], "", 0, 0) < 0)
			{
				//std::cout << "interrupt\n";
				return;
			}
		}

		iErrorOfObject = send(this->clientSock[usSockNumb], this->strCommandMass.c_str(), this->strCommandMass.size(), 0);
		this->ucSSCM_CurBuffer = 0xff;
		this->strCommandMass.clear();

		if (iErrorOfObject == SOCKET_ERROR)
		{
			//std::cout << "Data server is not available\n" << std::endl;
			return;
		}
		//else std::cout << "Data sent\n";
		break;
	}
	default:
		break;
	}
}

void KUKA::ROBOT_KRC4::vDataWrite(unsigned short usSockNumb,
	std::string *strToWrite)
{
	switch (usSockNumb)
	{
	case SOCKET_FOR_RC:
	{
		this->strControl = *strToWrite;
		break;
	}
	case SOCKET_FOR_SCM:
	{
		this->strCommandMass = *strToWrite;
		break;
	}
	default:
	{
		break;
	}
	}
}

void KUKA::ROBOT_KRC4::vReceiveControlData()
{
	int iRet;
	unsigned char ucmControlData[300];

	ZeroMemory(ucmControlData, 300);

	this->strControl = "";
	this->vDataSend(SOCKET_FOR_RC);

	if (this->iErrorOfObject == 0)
		while ((iRet = recv(this->clientSock[0], (char *)ucmControlData, 300, 0)) > 0)
		{

			if ((ucmControlData[0] == 0x02) && (ucmControlData[1] == 0x03) && (iRet >= 30) && (this->iMainAxisNumber == (INT)(ucmControlData[2] >> 4)) && (this->iExtAxisNumber == (INT)(ucmControlData[2] & 0x0f)))
			{
				//std::cout << "received right\n";

				switch ((unsigned short)ucmControlData[3])
				{
				case 0:
				{
					if ((ucmControlData[28] >= 0x00) && (ucmControlData[28] <= 0x11))
					{
						this->ucSSCM_CurBuffer = (unsigned char)ucmControlData[28];
					}
					else ucSSCM_CurBuffer = 0xff;

					//std::cout << "Current buffer state " << (unsigned short)(this->ucSSCM_CurBuffer) << '\n';
					//std::cout << "Current axis number " << (unsigned short)((unsigned char)ucmControlData[2]) << '\n';
					break;
				}
				case 255:
				{
					E6POS e6pRecData;

					memcpy(&e6pRecData.X, &ucmControlData[4], ((this->iMainAxisNumber + this->iExtAxisNumber) * sizeof(REAL) + 2 * sizeof(INT)));

					for (unsigned short i = 0; i < (this->iMainAxisNumber + this->iExtAxisNumber + 2); i++)
					{
						if ((i < 6) || (i > 7))
						{
							if (this->ofsLogData.good())
							{
								std::cout << *((REAL *)&e6pRecData.X + i) << "	";
								this->ofsLogData << *((REAL *)&e6pRecData.X + i) << "	";
							}
						}
						else
						{
							if (this->ofsLogData.good())
							{
								std::cout << *((INT *)&e6pRecData.X + i) << "	";
								this->ofsLogData << *((INT *)&e6pRecData.X + i) << "	";
							}
						}
					}
					//std::cout << '\n';
					this->ofsLogData << '\n';
					break;
				}
				default:
				{
					//std::cout << "Void command\n";
				}
				}
			}
			else
			{
				//std::cout << "Received wrong message or wrong robot config\n";
			}
		}
}

void KUKA::ROBOT_KRC4::vCloseSocket(unsigned short usSockNumb)
{
	switch (usSockNumb)
	{
	case 0:
	{
		closesocket(this->clientSock[usSockNumb]);
		break;
	}
	case 1:
		closesocket(this->clientSock[usSockNumb]);
		break;
	default:
		this->iErrorOfObject = 14;
		return;
	}

	this->iErrorOfObject = 0;
}