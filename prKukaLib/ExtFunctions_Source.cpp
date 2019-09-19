#include "KUKA.h"
#include "Definitions_Add.h"
#include "Definitions_Main.h"

int KUKA::ROBOT_KRC4::iChooseExtFunction()
{
	switch (this->tcdData)
	{
	case KUKA_TYPE_COMMAND_AUTOCALIBR:
	{
		this->ifsData >> this->tpData;
		this->tpData = KUKA_TYPE_POINT_POS;

		this->ifsData >> this->topmData;
		this->topmData = KUKA_ONEPOINTMOT_LIN;

		REAL *ptrRPos = &(this->e6pData.X);

		for (unsigned short i = 0; i < 6; i++) this->ifsData >> *(ptrRPos + i);

		ptrRPos = &(this->e6pData.E1);

		for (unsigned short i = 0; i < this->iExtAxisNumber; i++) this->ifsData >> *(ptrRPos + i);

		this->ifsData >> this->e6pData.S;
		this->ifsData >> this->e6pData.T;
		break;
	}
	default:
		break;
	}

	return 0;
}

int KUKA::ROBOT_KRC4_EXT::iChooseExtFunction()
{
	std::cout << "peregruz\n"; 
	return 0;
}