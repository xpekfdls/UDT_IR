// ModeDlg.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "UDT_IR.h"
#include "ModeDlg.h"
#include "afxdialogex.h"


// CModeDlg 대화 상자입니다.

IMPLEMENT_DYNAMIC(CModeDlg, CDialogEx)

CModeDlg::CModeDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CModeDlg::IDD, pParent)
	, m_nMode(0)
{

}

CModeDlg::~CModeDlg()
{
}

void CModeDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CModeDlg, CDialogEx)
	ON_BN_CLICKED(ID_BTN_Passive1, &CModeDlg::OnBnClickedBtnPassive1)
	ON_BN_CLICKED(ID_BTN_Active1, &CModeDlg::OnBnClickedBtnActive1)
	//ON_BN_CLICKED(IDC_Perturbation, &CModeDlg::OnBnClickedPerturbation)
	ON_BN_CLICKED(ID_BTN_Active2, &CModeDlg::OnBnClickedBtnDualtask)
END_MESSAGE_MAP()


// CModeDlg 메시지 처리기입니다.


void CModeDlg::OnBnClickedBtnPassive1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_nMode = 7;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}
/*
void CModeDlg::OnBnClickedBtnPassive2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_nMode = 4;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}*/


void CModeDlg::OnBnClickedBtnActive1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_nMode = 8;
	//m_nMode = 4;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}
/*
void CModeDlg::OnBnClickedBtnActive2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_nMode = 6;
	UpdateData(FALSE);
	CDialogEx::OnOK();
	}*/


void CModeDlg::OnBnClickedPerturbation()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	//m_nMode = 9;
	//m_nMode = 8;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}


void CModeDlg::OnBnClickedBtnDualtask()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_nMode = 9;
	//m_nMode = 4;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}
