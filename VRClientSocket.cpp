// VRClientSocket.cpp : 구현 파일입니다.
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
// CVRClientSocket 멤버 함수


void CVRClientSocket::OnClose(int nErrorCode)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	m_pVRControllerDlg->ProcessClose();

	CSocket::OnClose(nErrorCode);
}
