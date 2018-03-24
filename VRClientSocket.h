#pragma once
#include "UDT_IRDlg.h"

class CUDT_IRDlg;

// CUDTClientSocket 명령 대상입니다.

class CVRClientSocket : public CSocket
{
public:
	CVRClientSocket(CUDT_IRDlg* pVRControllerDlg);
	virtual ~CVRClientSocket();
	virtual void OnClose(int nErrorCode);
	CUDT_IRDlg* m_pVRControllerDlg;
};
