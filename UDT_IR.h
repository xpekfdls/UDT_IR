
// UDT_IR.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CUDT_IRApp:
// �� Ŭ������ ������ ���ؼ��� UDT_IR.cpp�� �����Ͻʽÿ�.
//

class CUDT_IRApp : public CWinApp
{
public:
	CUDT_IRApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CUDT_IRApp theApp;