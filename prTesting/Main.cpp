#include <iostream>
#include "KUKA.h"
#include <wchar.h>

using namespace std;
using namespace KUKA;


WORD ROBOT_KRC4::ver;
WSADATA ROBOT_KRC4::wsaData;

void vCloseCon(KUKA::ROBOT_KRC4_EXT *);

int main()
{

	ROBOT_KRC4_EXT* Agilus = new ROBOT_KRC4_EXT("Agilus KR6", 6, 1);
	ROBOT_KRC4_EXT* KR120 = new ROBOT_KRC4_EXT("KR120", 6, 1);
	//KR120->iChooseExtFunction();
	delete KR120;
	cout << "Address of Agilus: " << Agilus << endl;

	KUKA::ROBOT_KRC4::iWinSockINI();
	cout << KUKA_TYPE_COMMAND_EMPTY << "\n";

	if (Agilus->iErrorOfObject != 0)
	{
		cout << "Wrong object parameters\n";
		delete Agilus;
		Agilus = NULL;
	}
	else
	{ 
		Agilus->vShowRobot();
		
		string ip1, ip2;
		ip2 = "172.31.1.147";
		//ip2 = "127.0.0.1";

		Agilus->vSocketSettings(ip2, 54600, SOCKET_FOR_RC);
		Agilus->vSocketSettings(ip2, 54601, SOCKET_FOR_SCM);

		Agilus->iStartClient(0);
		Agilus->iStartClient(1);

		Agilus->iOpenFileToReadCommands("fileToRead_.txt");
		Agilus->iOpenFileToWriteLog("fileLog.txt");
		cout << "Address of Agilus: " << Agilus << endl;
		
		
		E6POS e6pTest;
		//TYPE_POINT ptTest = AXIS;
		//TYPE_ONEPOINTMOT opmtTest = PTP;
		//TYPE_ORI otTest = VAR;

		cout << "READING FILE " << Agilus->iReadCommandsFromFile() << endl;

		Agilus->thr[1] = NULL;
		//std::cout << Agilus->thr[0];
		//std::cout << Agilus->thr[1];
		Agilus->thr[1] = new thread (&ROBOT_KRC4_EXT::vReceiveControlData,ref(Agilus));
		//std::cout << Agilus->thr[1];
		thread thr2(vCloseCon, ref(Agilus));

		//Agilus->vDataSend(SOCKET_FOR_SCM);
		Agilus->vDataSend(SOCKET_FOR_SCM);
		cout << "thread created\n";
		if ((*Agilus->thr[1]).joinable()) (*Agilus->thr[1]).join();
		//if ((*Agilus->thr[0]).joinable()) (*Agilus->thr[0]).join();
		//cout << "threads	" << (*(Agilus->thr[0])).get_id();
		if (thr2.joinable()) thr2.join();
	}
	//delete Agilus;
	//delete Agilus;
	KUKA::ROBOT_KRC4::vWinSockClean();

	system("PAUSE");
	return 0;
}

void vCloseCon(KUKA::ROBOT_KRC4_EXT *rKRC_4)
{
	string strQuit = "";
	while (strQuit != "quit")
	{
		Sleep(1000);
		cin >> strQuit;
	}
	delete rKRC_4;
	cout << "quit\n";
}