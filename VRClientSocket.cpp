// VRClientSocket.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "VRClientSocket.h"
class CUDT_IRDlg;

// CVRClientSocket

CVRClientSocket::CVRClientSocket(CUDT_IRDlg* pVRControllerDlg)
: m_pVRControllerDlg(NULL)
{
	m_pVRControllerDlg = pVRControllerDlg;
}

CVRClientSocket::~CVRClientSocket()
{
}
// CVRClientSocket ��� �Լ�


void CVRClientSocket::OnClose(int nErrorCode)
{
	// TODO: ���⿡ Ư��ȭ�� �ڵ带 �߰� ��/�Ǵ� �⺻ Ŭ������ ȣ���մϴ�.
	m_pVRControllerDlg->ProcessClose();

	CSocket::OnClose(nErrorCode);
}
