#pragma once


// CModeDlg ��ȭ �����Դϴ�.

class CModeDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CModeDlg)

public:
	CModeDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CModeDlg();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_DIALOG_MODE };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()

public:
	int m_nMode;
	afx_msg void OnBnClickedBtnPassive1();
	afx_msg void OnBnClickedBtnActive1();
	afx_msg void OnBnClickedPerturbation();
	afx_msg void OnBnClickedBtnDualtask();
};
