// ModeDlg.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "UDT_IR.h"
#include "ModeDlg.h"
#include "afxdialogex.h"


// CModeDlg ��ȭ �����Դϴ�.

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


// CModeDlg �޽��� ó�����Դϴ�.


void CModeDlg::OnBnClickedBtnPassive1()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_nMode = 7;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}
/*
void CModeDlg::OnBnClickedBtnPassive2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_nMode = 4;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}*/


void CModeDlg::OnBnClickedBtnActive1()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_nMode = 8;
	//m_nMode = 4;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}
/*
void CModeDlg::OnBnClickedBtnActive2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_nMode = 6;
	UpdateData(FALSE);
	CDialogEx::OnOK();
	}*/


void CModeDlg::OnBnClickedPerturbation()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	//m_nMode = 9;
	//m_nMode = 8;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}


void CModeDlg::OnBnClickedBtnDualtask()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_nMode = 9;
	//m_nMode = 4;
	UpdateData(FALSE);
	CDialogEx::OnOK();
}
