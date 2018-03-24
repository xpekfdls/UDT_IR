#pragma once


// CModeDlg 대화 상자입니다.

class CModeDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CModeDlg)

public:
	CModeDlg(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CModeDlg();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_DIALOG_MODE };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()

public:
	int m_nMode;
	afx_msg void OnBnClickedBtnPassive1();
	afx_msg void OnBnClickedBtnActive1();
	afx_msg void OnBnClickedPerturbation();
	afx_msg void OnBnClickedBtnDualtask();
};
