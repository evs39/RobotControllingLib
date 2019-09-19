#include "KUKA.h"
#include "Definitions_Main.h"
#include "Definitions_Add.h"

KUKA::ROBOT_KRC4::ROBOT_KRC4(std::string strName,
	INT iMAN,
	INT iEAN)
	:strRoboName(strName), iMainAxisNumber(iMAN), iExtAxisNumber(iEAN)
{
	/*Object constructor(конструктор объекта)
	*/
	
	if ((iMAN < 1) | (iMAN > 6) | (iEAN < 0) | (iEAN > 6))//checking of axes number (проверка числа осей)
	{
		this->iErrorOfObject = 1;//wrong (неправильное)
	}
	else this->iErrorOfObject = 0;//right (правильное)

	this->iCommandLength = 36 + 4 * iEAN;//message length (расчет длины сообщения)

	thr[0] = NULL;
	thr[1] = NULL;
}

KUKA::ROBOT_KRC4::~ROBOT_KRC4()
{
	if (this->ifsData.is_open()) this->ifsData.close();
	if (this-ofsLogData.is_open()) this->ofsLogData.close();

	vWinSockClean();

	if ((this->thr[0] != NULL) && ((*this->thr[0]).joinable())) (*this->thr[0]).join();
	if ((this->thr[1] != NULL) && ((*this->thr[1]).joinable())) (*this->thr[1]).join();
}


bool KUKA::ROBOT_KRC4::bCheckMassLength()
{
	if (this->strCommandMass.size() > 3420) return false;
	else return true;
}

char KUKA::ROBOT_KRC4::cCRC8()
{
	char cCRC = 0x00;

	for (int i = 0; i < (iCommandLength - 1); i++) cCRC ^= cmCommand[i];

	return cCRC;
}

void KUKA::ROBOT_KRC4::vBaseOrTool()
{

}

void KUKA::ROBOT_KRC4::vCommand()
{
	ZeroMemory(this->cmCommand, 120);

	switch (this->tcdData)
	{
	//Setting of the robot parameters
	case KUKA_TYPE_COMMAND_PARAM:
	{
		break;
	}
	//Setting of the robot base
	case KUKA_TYPE_COMMAND_BASE:
	{
		this->cmCommand[0] = '\x02';
		memcpy(&this->cmCommand[1], &this->frBASE.X, 24);
		break;
	}
	//Setting of the robot tool
	case KUKA_TYPE_COMMAND_TOOL:
	{
		this->cmCommand[0] = '\x03';
		memcpy(&this->cmCommand[1], &this->frTOOL.X, 24);
		break;
	}
	//Setting of the robot axes acceleration
	case KUKA_TYPE_COMMAND_AXACC:
	{
		this->cmCommand[0] = '\x04';
		memcpy(&(this->cmCommand[1]), &(this->aaData.A1), sizeof(REAL) * (6 + this->iExtAxisNumber));
		break;
	}
	//Setting of the robot axes velocity
	case KUKA_TYPE_COMMAND_AXVEL:
	{
		this->cmCommand[0] = '\x05';
		memcpy(&(this->cmCommand[1]), &(this->avData.A1), sizeof(REAL) * (6 + this->iExtAxisNumber));
		break;
	}
	//Setting of the robot CP acceletations and velocities
	case KUKA_TYPE_COMMAND_LINAVEXTAXV:
	{
		this->cmCommand[0] = '\x06';
		memcpy(&(this->cmCommand[1]), &(this->lavevData.ACC_CP), sizeof(REAL) * (6 + this->iExtAxisNumber));
		break;
	}
	//Setting of the robot PTP or LIN movement
	case KUKA_TYPE_COMMAND_ONEPOINTMOVEMENT:
	{
		this->cmCommand[0] = '\x07';

		switch (tpData)
		{
		//A target point is e6axis
		case KUKA_TYPE_POINT_AXIS:
		{
			memcpy(&(this->cmCommand[1]), &e6aData.A1, 24);

			if (this->iExtAxisNumber > 0) memcpy(&(this->cmCommand[25]), &e6aData.E1, iExtAxisNumber * 4);

			cmCommand[24 + iExtAxisNumber * 4 + 3] = (unsigned char)topmData;
			cmCommand[24 + iExtAxisNumber * 4 + 4] = (unsigned char)tpData;
			break;
		}
		//a target point is e6pos
		case KUKA_TYPE_POINT_POS:
		{
			memcpy(&(this->cmCommand[1]), &e6pData.X, 24);

			if (this->iExtAxisNumber > 0) memcpy(&(this->cmCommand[25]), &e6pData.E1, iExtAxisNumber * 4);

			cmCommand[24 + iExtAxisNumber * 4 + 1] = e6pData.S ^ toData;
			cmCommand[24 + iExtAxisNumber * 4 + 2] = e6pData.T;
			cmCommand[24 + iExtAxisNumber * 4 + 3] = (unsigned char)topmData;
			cmCommand[24 + iExtAxisNumber * 4 + 4] = (unsigned char)tpData;
			break;
		}
		}

		break;
	}
	case KUKA_TYPE_COMMAND_AUTOCALIBR:
	{
		this->cmCommand[0] = '\xfe';
		memcpy(&(this->cmCommand[1]), &e6pData.X, 24);

		if (this->iExtAxisNumber > 0) memcpy(&(this->cmCommand[25]), &e6pData.E1, iExtAxisNumber * 4);

		cmCommand[24 + iExtAxisNumber * 4 + 1] = e6pData.S ^ toData;
		cmCommand[24 + iExtAxisNumber * 4 + 2] = e6pData.T;
		cmCommand[24 + iExtAxisNumber * 4 + 3] = (unsigned char)topmData;
		cmCommand[24 + iExtAxisNumber * 4 + 4] = (unsigned char)tpData;
		break;
	}
	default:
	{
		iErrorOfObject = 16;
		return;
		break; 
	}
	}

	this->iErrorOfObject = 0;
	this->cmCommand[iCommandLength - 1] = this->cCRC8();
	
	this->strCommandMass.append((const char*)(this->cmCommand), this->iCommandLength);
}

void KUKA::ROBOT_KRC4::vShowRobot()
{
	//std::cout << "RobotName: " << strRoboName.c_str()
	//	<< "	Main axes number: " << iMainAxisNumber
	//	<< "	External axes number: " << iExtAxisNumber << '\n';
}

void KUKA::ROBOT_KRC4::vStartMassForming()
{
	strCommandMass.clear();
	strCommandMass += '\x02';
}

void KUKA::ROBOT_KRC4::vEndMassForming()
{
	strCommandMass += "\x03\xff\xff\xff";
}





template <>
void vDataAddition<std::string>(std::string *strData,
	const std::string *strInput)
{
	for (unsigned int i = 0; i < strInput->size(); i++)
	{
		//std::cout << "got\n";
		*strData += (*strInput)[i];
	}
}

template <>
void vDataAddition<int>(std::string *strData,
	const int *strInput)
{
	//std::cout << "Hi from another version_!!!\n";
	char cmDataToAdd[4];
	memcpy(&cmDataToAdd[0], strInput, 4);
	for (int i = 0; i < 4; i++) *strData += cmDataToAdd[i];
}