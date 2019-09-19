#include "KUKA.h"
#include "Definitions_Main.h"

int KUKA::ROBOT_KRC4::iOpenFileToReadCommands(const std::string strFileName)
{
	std::string strRead;
	unsigned short usMainAxNumberFromFile, usExtAxNumberFromFile;
	this->ifsData.open(strFileName);

	if (ifsData.good())
	{
		//std::cout << "The file is opened\n";
		ifsData >> strRead >> usMainAxNumberFromFile >> usExtAxNumberFromFile;

		//if (strRead == "Start") std::cout << "Right file format\n";

		if ((usMainAxNumberFromFile == this->iMainAxisNumber) && (usExtAxNumberFromFile == this->iExtAxisNumber))
		{
			//std::cout << "Commands are availible to the robot\n";
			return 0;
		}
		else
		{
			//std::cout << "Commands aren't availible to the robot\n";
			ifsData.close();
			return 3;
		}
	}
	return 1;
}

int KUKA::ROBOT_KRC4::iOpenFileToWriteLog(const std::string strFileName)
{
	this->ofsLogData.open(strFileName);

	if (this->ofsLogData.good())
	{
		//std::cout << "The log file is opened\n";
		return 0;
	}
	return 1;
}

int KUKA::ROBOT_KRC4::iReadCommandsFromFile()
{
	std::string strCommand;

	if (ifsData.is_open())
	{
		this->vStartMassForming();
		while ((!ifsData.eof()) && (this->bCheckMassLength()))
		{
			ifsData >> strCommand;

			if (strCommand == "Command")
			{
				ifsData >> this->tcdData;

				switch (this->tcdData)
				{
				case 1:
				{
					break;
				}
				case 2:
				{
					this->ifsData >> this->frBASE.X >> this->frBASE.Y >> this->frBASE.Z
						>> this->frBASE.A >> this->frBASE.B >> this->frBASE.C;
					break;
				}
				case 3:
				{
					this->ifsData >> this->frTOOL.X >> this->frTOOL.Y >> this->frTOOL.Z
						>> this->frTOOL.A >> this->frTOOL.B >> this->frTOOL.C;
					break;
				}
				case 4:
				{
					REAL *ptrAxis = &(this->aaData.A1);

					for (unsigned short i = 0; i < (6 + this->iExtAxisNumber); i++) ifsData >> *(ptrAxis + i);

					break;
				}
				case 5:
				{
					REAL *ptrAxis = &(this->avData.A1);

					for (unsigned short i = 0; i < (6 + this->iExtAxisNumber); i++) ifsData >> *(ptrAxis + i);

					break;
				}
				case 6:
				{
					REAL *ptrAxis = &(this->lavevData.ACC_CP);

					for (unsigned short i = 0; i < (6 + this->iExtAxisNumber); i++) ifsData >> *(ptrAxis + i);

					break;
				}
				case 7:
				{
					ifsData >> this->tpData;

					switch (this->tpData)
					{
					case 1:
					{
						ifsData >> this->topmData;

						REAL *ptrRAxis = &(this->e6aData.A1);

						for (unsigned short i = 0; i < (6 + this->iExtAxisNumber); i++) ifsData >> *(ptrRAxis + i);

						break;
					}
					case 2:
					{
						ifsData >> this->topmData;

						REAL *ptrRPos = &(this->e6pData.X);

						for (unsigned short i = 0; i < 6; i++) ifsData >> *(ptrRPos + i);

						ptrRPos = &(this->e6pData.E1);

						for (unsigned short i = 0; i < this->iExtAxisNumber; i++) ifsData >> *(ptrRPos + i);

						ifsData >> this->e6pData.S;
						ifsData >> this->e6pData.T;
						//std::cout << "i'mhere" << std::endl;
						break;
					}
					}
					break;
				}
				case 254:
				{
					ifsData >> this->tpData;
					this->tpData = KUKA_TYPE_POINT_POS;

					ifsData >> this->topmData;
					this->topmData = KUKA_ONEPOINTMOT_LIN;

					REAL *ptrRPos = &(this->e6pData.X);

					for (unsigned short i = 0; i < 6; i++) ifsData >> *(ptrRPos + i);

					ptrRPos = &(this->e6pData.E1);

					for (unsigned short i = 0; i < this->iExtAxisNumber; i++) ifsData >> *(ptrRPos + i);

					ifsData >> this->e6pData.S;
					ifsData >> this->e6pData.T;
					break;
				}
				default:
				{
					this->iChooseExtFunction();
					break;
				}
				}

				ifsData >> strCommand;

				if (strCommand != "End")
				{
					this->strCommandMass.clear();
					return 1;
				}

				//this->vCommand();
			}
			else
			{
				break;
			}
		}

		this->vEndMassForming();
	}

	return 0;
}

int KUKA::ROBOT_KRC4::iReadCommandsFromFileAndSend()
{
	std::string strCommand;

	if (ifsData.is_open())
	{
		this->vStartMassForming();
		while (!ifsData.eof())
		{
			while ((!ifsData.eof()) && (this->bCheckMassLength()))
			{
				ifsData >> strCommand;

				if (strCommand == "Command")
				{
					ifsData >> this->tcdData;

					switch (this->tcdData)
					{
					case 1:
					{
						break;
					}
					case 2:
					{
						this->ifsData >> this->frBASE.X >> this->frBASE.Y >> this->frBASE.Z
							>> this->frBASE.A >> this->frBASE.B >> this->frBASE.C;
						break;
					}
					case 3:
					{
						this->ifsData >> this->frTOOL.X >> this->frTOOL.Y >> this->frTOOL.Z
							>> this->frTOOL.A >> this->frTOOL.B >> this->frTOOL.C;
						break;
					}
					case 4:
					{
						REAL *ptrAxis = &(this->aaData.A1);

						for (unsigned short i = 0; i < (6 + this->iExtAxisNumber); i++) ifsData >> *(ptrAxis + i);

						break;
					}
					case 5:
					{
						REAL *ptrAxis = &(this->avData.A1);

						for (unsigned short i = 0; i < (6 + this->iExtAxisNumber); i++) ifsData >> *(ptrAxis + i);

						break;
					}
					case 6:
					{
						REAL *ptrAxis = &(this->lavevData.ACC_CP);

						for (unsigned short i = 0; i < (6 + this->iExtAxisNumber); i++) ifsData >> *(ptrAxis + i);

						break;
					}
					case 7:
					{
						ifsData >> this->tpData;

						switch (this->tpData)
						{
						case 1:
						{
							ifsData >> this->topmData;

							REAL *ptrRAxis = &(this->e6aData.A1);

							for (unsigned short i = 0; i < (6 + this->iExtAxisNumber); i++) ifsData >> *(ptrRAxis + i);

							break;
						}
						case 2:
						{
							ifsData >> this->topmData;

							REAL *ptrRPos = &(this->e6pData.X);

							for (unsigned short i = 0; i < 6; i++) ifsData >> *(ptrRPos + i);

							ptrRPos = &(this->e6pData.E1);

							for (unsigned short i = 0; i < this->iExtAxisNumber; i++) ifsData >> *(ptrRPos + i);

							ifsData >> this->e6pData.S;
							ifsData >> this->e6pData.T;
							//std::cout << "i'mhere" << std::endl;
							break;
						}
						}
						break;
					}
					case 254:
					{
						ifsData >> this->tpData;
						this->tpData = KUKA_TYPE_POINT_POS;

						ifsData >> this->topmData;
						this->topmData = KUKA_ONEPOINTMOT_LIN;

						REAL *ptrRPos = &(this->e6pData.X);

						for (unsigned short i = 0; i < 6; i++) ifsData >> *(ptrRPos + i);

						ptrRPos = &(this->e6pData.E1);

						for (unsigned short i = 0; i < this->iExtAxisNumber; i++) ifsData >> *(ptrRPos + i);

						ifsData >> this->e6pData.S;
						ifsData >> this->e6pData.T;
						break;
					}
					default:
					{
						this->iChooseExtFunction();
						break;
					}
					}

					ifsData >> strCommand;

					if (strCommand != "End")
					{
						this->strCommandMass.clear();
						return 1;
					}

					this->vCommand();
				}
				else
				{
					break;
				}
			}

			this->vEndMassForming();
			this->vDataSend(SOCKET_FOR_SCM);
		}
	}

	return 0;
}

int KUKA::ROBOT_KRC4::iCheckFStreamOfOBject(unsigned short usFstreamNumb)
{
	switch (usFstreamNumb)
	{
	case FILE_CM:
	{
		if (this->ifsData.good() != true) return 1;
		break;
	}
	case FILE_LG:
	{
		if (this->ofsLogData.good() != true) return 1;
		break;
	}
	}

	return 0;
}

void KUKA::ROBOT_KRC4::vCloseFStream(unsigned short usFstreamNumb)
{
	switch (usFstreamNumb)
	{
	case FILE_CM:
	{
		this->ifsData.close();
	}
	case FILE_LG:
	{
		this->ofsLogData.close();
	}
	}
}