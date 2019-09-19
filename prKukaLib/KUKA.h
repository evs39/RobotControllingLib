#pragma once

#ifndef _KUKA_OPT_BY_ALIR_
#define _KUKA_OPT_BY_ALIR_

#include <cstring>
#include <fstream>
#include <iostream>
#include <locale>
#include <thread>
#include <string>
#include <string.h>
#include <wchar.h>
#include <WinSock2.h>
#include "Definitions_Add.h"
#include "Definitions_Main.h"

#pragma warning(disable:4996)
#pragma comment(lib, "WSock32.Lib")

#define SOCKET_FOR_RC 0
#define SOCKET_FOR_SCM 1

#define FILE_CM 0
#define FILE_LG 1

namespace KUKA
{
#pragma region functions
#pragma endregion

	typedef float REAL;
	typedef int INT;
	typedef unsigned char CHAR;

	struct AXES_A
	{
		REAL A1, A2, A3, A4, A5, A6,
			E1, E2, E3, E4, E5, E6;
	};
	struct AXES_V
	{
		REAL A1, A2, A3, A4, A5, A6,
			E1, E2, E3, E4, E5, E6;
	};
	struct E6AXIS
	{
		REAL A1, A2, A3, A4, A5, A6,
			E1, E2, E3, E4, E5, E6;
	};
	struct E6POS
	{
		REAL X, Y, Z, A, B, C;
		INT S, T;
		REAL E1, E2, E3, E4, E5, E6;
	};
	struct FRAME
	{
		REAL X, Y, Z, A, B, C;
	};
	struct LIN_AV_EXTAX_V
	{
		REAL ACC_CP, ACC_ORI1, ACC_ORI2, VEL_CP, VEL_ORI1, VEL_ORI2,
			E1, E2, E3, E4, E5, E6;
	};

	class ROBOT_KRC4
	{
	protected:
		CHAR cmCommand[120];
		INT iMainAxisNumber;
		INT iExtAxisNumber;
		INT iCommandLength;

		std::ifstream ifsData;
		std::ofstream ofsLogData;
		SOCKADDR_IN serverInfo[2];
		SOCKET clientSock[2];

		std::string strRoboName;
		std::string strIP_SCR, strIP_SSCM;
		unsigned char ucSSCM_CurBuffer = 0xff;
		unsigned short usPortNumber_SCR, usPortNumber_SSCM;

		static WORD ver;
		static WSADATA wsaData;

#pragma region privateFunctions
#pragma endregion

	public:

		std::thread *thr[2];

		AXES_A aaData;
		AXES_V avData;
		E6AXIS e6aData;
		E6POS e6pData;
		FRAME frTOOL, frBASE;
		INT iErrorOfObject;
		LIN_AV_EXTAX_V lavevData;
		std::string strControl, strCommandMass;
		unsigned short tcdData;
		unsigned short topmData;
		unsigned short toData;
		unsigned short tpData;
#pragma region publicFunstions

		ROBOT_KRC4(std::string,
			INT,
			INT);

		~ROBOT_KRC4();
		
		static int iWinSockINI();
		static void vWinSockClean();
		
		bool bCheckMassLength();
		
		char cCRC8();

		void vBaseOrTool();
		void vCommand();
		void vCloseSocket(unsigned short);
		void vDataSend(unsigned short);
		void vDataWrite(unsigned short, std::string *);
		void vEndMassForming();
		int iOpenFileToReadCommands(const std::string);
		int iReadCommandsFromFileAndSend();
		int iOpenFileToWriteLog(const std::string);
		int iReadCommandsFromFile();
		void vReceiveControlData();
		void vShowRobot();
		void vSocketSettings(std::string,
			unsigned short,
			unsigned short);
		void vCloseFStream(unsigned short);
		int iStartClient(unsigned short);
		int iCheckFStreamOfOBject(unsigned short);
		virtual int iChooseExtFunction();
		void vStartMassForming();

#pragma endregion

	};
	class ROBOT_KRC4_EXT : public ROBOT_KRC4
	{
	public:
		ROBOT_KRC4_EXT(std::string strName,
			INT iMainAx,
			INT iExtAx) : ROBOT_KRC4(strName, iMainAx, iExtAx) {};
		int iChooseExtFunction();
	};
};

#endif





template <typename T>
void CAST_TO(std::string *strData,
	const T* t) 
{
	vDataAddition(strData, t);
};

template <typename First, typename... Rest>
void CAST_TO(std::string *strData,
	const First* first,
	const Rest*... rest)
{
	vDataAddition(strData, first);
	CAST_TO(strData, rest...);
}

template <typename T> 
void vDataAddition(std::string *strData,
	const T* t)
{
	std::cout << "Hi from another version\n";
	char cmDataToAdd[sizeof(*t)];
	memcpy(&cmDataToAdd[0], t, sizeof(*t));
	for (int i = 0; i < sizeof(*t); i++) *strData += cmDataToAdd[i];
};

template <>
void vDataAddition<std::string>(std::string *strData,
	const std::string *strInput);

template <>
void vDataAddition<int>(std::string *strData,
	const int *strInput);