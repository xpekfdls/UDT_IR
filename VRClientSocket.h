#pragma once
#include "UDT_IRDlg.h"

class CUDT_IRDlg;

// CUDTClientSocket ��� ����Դϴ�.

class CVRClientSocket : public CSocket
{
public:
	CVRClientSocket(CUDT_IRDlg* pVRControllerDlg);
	virtual ~CVRClientSocket();
	virtual void OnClose(int nErrorCode);
	CUDT_IRDlg* m_pVRControllerDlg;
};
