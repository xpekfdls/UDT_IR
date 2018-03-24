
// UDT_IRDlg.h : 헤더 파일
//

#pragma once
#include "afxwin.h"
#include "afxcmn.h"
#include <winsock2.h>
#include "DXRenderer.h"
#include "VRClientSocket.h"
#include "CommThread.h"
#include <Eigen/Dense>
#include "resource.h"
#include <NIDAQmx.h>
#include "ChartViewer.h"
#include <mmsystem.h>

class CVRClientSocket;

#define InfraredSourceValueMaximum static_cast<float>(USHRT_MAX)

#define InfraredOutputValueMinimum (0.01f)

#define InfraredOutputValueMaximum (1.0f)

#define InfraredSceneValueAverage (0.08f)

#define InfraredSceneStandardDeviations (3.0f)

#define CalcRatio(x) (max(InfraredOutputValueMinimum,min(InfraredOutputValueMaximum,x / InfraredSourceValueMaximum / InfraredSceneValueAverage / InfraredSceneStandardDeviations)))

#define PELVIS	0
#define ANKLE_R	1
#define ANKLE_L	2
#define WRIST_R	3
#define WRIST_L	4
#define HEAD 5
#define HIP_R 6
#define HIP_L 7

const int sampleSize = 240;

// CUDT_IRDlg 대화 상자
class CUDT_IRDlg : public CDialogEx
{
	// 생성입니다.
public:
	CUDT_IRDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

	static void TimerProc(UINT uiID, UINT uiMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2);
	static void TimerProc_Kinect(UINT uiID, UINT uiMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2);

	// 대화 상자 데이터입니다.
	enum { IDD = IDD_UDT_IR_DIALOG };


protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


	// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL			OnInitDialog();
	afx_msg void			OnPaint();
	afx_msg HCURSOR			OnQueryDragIcon();
	afx_msg void			OnTimer(UINT_PTR nIDEvent);
	DECLARE_MESSAGE_MAP()
public:
	// GUI Controls
	CEdit					m_editTime;
	CEdit					m_editRate;
	CButton					m_btnStart;
	CSliderCtrl				m_sldrGain;
	/*
	CSpinButtonCtrl			m_spinK1;
	CSpinButtonCtrl			m_spinK2;
	CSpinButtonCtrl			m_spinK3;
	CSpinButtonCtrl			m_spinBeta;
	CSpinButtonCtrl			m_spinXref;
	*/
	//

	// Kinect SDK
	HRESULT					InitializeDefaultSensor();
	IKinectSensor*			m_pKinectSensor;
	ICoordinateMapper*		m_pCoordinateMapper;
	IMultiSourceFrameReader*m_pMultiSourceFrameReader;
	//

	// DirectX Rendering
	DXRenderer*				m_pDXRenderer;
	ID2D1Factory*			m_pD2DFactory;
	RGBQUAD					m_pOutputRGBX[DATA_LENGTH];
	//

	// Flags
	bool					m_bFlag;
	bool					m_bStart;
	//

	//File handle
	HANDLE					m_hFile;
	HANDLE					m_hFile_FSR;
	//

	// Marker Extraction
	float					m_fGain;
	CameraSpacePoint		m_pMarkers[25];
	//

	// Time Variables
	float					m_fTime;
	float					m_fFPS;
	double					m_fFPS_data;
	INT64					m_nStartTime;

	//


	// passive walking mode variables
	//float					m_fStartTime;     //start time when walking in fast or slow speed
	//

	// Speed Variables
	float					m_fPelSpd;
	float					m_fPelAcc;
	//

	// Displayed Variables
	float					m_fSpeed;
	double					m_fHeadAngle;
	float					m_fPelPos;
	float					m_fA1;
	float					m_fA2;
	float					m_fA3;
	float					m_fA4;
	float					m_fA5;
	float					m_fA6;
	float					m_fA7;
	//
	double					slope; // slope variable of subject


	// TCP
	CVRClientSocket*		m_pVRClientSocket;


	// Control Values
	float					m_fXref;
	float					m_fK1;
	float					m_fK2;
	float					m_fK3;
	float					m_fBeta;
	//

	// Windows Messages
	afx_msg void			OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void			OnEnChangeEditGain();
	afx_msg void			OnBnClickedBtnStart();
	//afx_msg void			OnDeltaposSpinXref(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void			OnEnChangeEditXref();
	afx_msg void			OnEnChangeEditK1();
	//afx_msg void			OnDeltaposSpinK1(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void			OnEnChangeEditK2();
	//afx_msg void			m_dataSeriesA7 *pNMHDR, LRESULT *pResult);
	afx_msg void			OnEnChangeEditK3();
	//afx_msg void			OnDeltaposSpinK3(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void			OnEnChangeEditBeta();
	//afx_msg void			OnDeltaposSpinBeta(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void			OnEnChangeEditComfSPD();
	afx_msg void			OnEnChangeEditRepeatTime();
	afx_msg void			OnClose();
	virtual BOOL			PreTranslateMessage(MSG* pMsg);
	//

	// Methods
	HRESULT					GetFrame(IMultiSourceFrame* pMultiSourceFrame, IInfraredFrame** ppIRFrame, IBodyFrame** ppBodyFrame);
	HRESULT					ProcessFrame(INT64 nTime, const UINT16* pBuffer, UINT nWidth, UINT nHeight);
	void					ProcessClose();
	void					DataSave(float fTime);
	void					DataSave_FSR(float fTime, double FSR1, double FSR2, double FSR3, double FSR4, double FSR5, double FSR6, double trigger);
	void					SaveSettings();
	void					LoadSettings();
	void					CalcFeedBack();
	float					CalcVwObv();
	int						CalcArm(Joint* Joints);
	//
	//CString					m_strFilename;
	afx_msg void			OnEnChangeEditFilename();
	CCommThread				m_Comm;
	LRESULT					OnReceive(WPARAM wParam, LPARAM lParam);
	void					PrintStr(CString* pString);
	CListBox				m_listMsg;
	CComboBox				m_comboComPort;
	double					m_fSensors[9];
	double					KalmanAngle(double* sensor);

	Eigen::Matrix2d			P;	// Kalman
	Eigen::Matrix2d			K;	// Kalman Gain
	Eigen::Vector2d			x;	// Kalman state vector
	TaskHandle				m_hAOTask;
	TaskHandle				m_hAITask;
	TaskHandle				m_hCITask;
	char*					m_strAO;
	char*					m_strCI;
	char*					m_strAI;
	afx_msg void			OnEnChangeEditYoffset();
	afx_msg void			OnEnChangeEditZoffset();
	int						m_nYoff;
	int						m_nZoff;
	//CSpinButtonCtrl			m_spinYoff;
	//CSpinButtonCtrl			m_spinZoff;
	//afx_msg void			OnDeltaposSpinYoffset(NMHDR *pNMHDR, LRESULT *pResult);
	//afx_msg void			OnDeltaposSpinZoffset(NMHDR *pNMHDR, LRESULT *pResult);
	CEdit					m_editYoff;
	CEdit					m_editZoff;
	afx_msg void			OnEnChangeEditAcclmt();
	//afx_msg void			OnDeltaposSpinAcclmt(NMHDR *pNMHDR, LRESULT *pResult);
	CEdit					m_editAccLmt;
	//CSpinButtonCtrl			m_spinAccLmt;
	float					m_fAccLmt;
	CIPAddressCtrl			m_ipAddress;
	CString					m_strIP;
	afx_msg void			OnIpnFieldchangedIpAddress(NMHDR *pNMHDR, LRESULT *pResult);
	float					m_fSpdAct;
	CEdit m_editFilename;
	double HDR(double gyro);
	float m_fXrpos;
	afx_msg void OnBnClickedBtnMode();
	afx_msg void OnEnChangeEditXoffset();
	//afx_msg void OnDeltaposSpinXoffset(NMHDR *pNMHDR, LRESULT *pResult);
	CEdit m_editXoff;
	//CSpinButtonCtrl m_spinXoff;
	int m_nXoff;
	afx_msg void OnEnChangeEditDeclmt();
	//afx_msg void OnDeltaposSpinDeclmt(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnEnChangeEditSpdlmt();
	//afx_msg void OnDeltaposSpinSpdlmt(NMHDR *pNMHDR, LRESULT *pResult);
	float m_fDecLmt;
	float m_fSpdLmt;
	float m_fComfSpd;
	CEdit m_editComfSpd;
	float m_fFast;
	float m_fSlow;
	float fSpeed_p;
	float fSpeed_pp;
	int m_switch;
	int m_count;
	int m_fRepeatTime;
	CEdit m_editRepeatTime;
	float m_fSteady_Time_Interval;
	CEdit m_editSteadyTimeInterval;
	float m_fPertinterval;
	CEdit m_editPertinterval;
	//CSpinButtonCtrl m_spinDecLmt;
	//CSpinButtonCtrl m_spinSpdLmt;
	CEdit m_editDecLmt;
	CEdit m_editSpdLmt;
	CEdit m_editXref;
	CEdit m_editAnalogCheck;
	float CalcVwSwing();
	float PassiveMode();
	float Perturbation();
	float Calslope(); // calculate the  slope of subject
	bool CalcEmergency(Joint* joints);
	int m_nMode;
	CButton m_btnMode;
	void Wait(DWORD dwMilliSecond);
	float m_fLambda;
	afx_msg void OnEnChangeEditRate();
	afx_msg void OnEnChangeEditRepeattime();
	afx_msg void OnEnChangeEditSteadytimeinterval();
	afx_msg void OnBnClickedMove();
	afx_msg void OnCbnSelchangeComboComport();
	afx_msg void OnEnChangeEditK4();
	afx_msg void OnStnClickedPcChartdir1();
	CStatic m_ValueA1;
	CStatic m_ValueA2;
	CStatic m_ValueA3;
	CStatic m_ValueA4;
	CStatic m_ValueA5;
	CStatic m_ValueA6;
	afx_msg void OnStnClickedChart1();
	CChartViewer m_ChartViewer;

	double global_current_time;
	double global_system_time;


	double m_timeStamps[sampleSize];	// The timestamps for the data series
	double m_dataSeriesA1[sampleSize];	// The values for the data series A1
	double m_dataSeriesA2[sampleSize];	// The values for the data series A2
	double m_dataSeriesA3[sampleSize];	// The values for the data series A3
	double m_dataSeriesA4[sampleSize];	// The values for the data series A4
	double m_dataSeriesA5[sampleSize];	// The values for the data series A5
	double m_dataSeriesA6[sampleSize];	// The values for the data series A6
	double m_dataSeriesA7[sampleSize];	// The values for the data series A7

	// The index of the array position to which new data values are added.
	int m_currentIndex;

	double m_nextDataTime;	// Used by the random number generator to generate real time data.
	int m_extBgColor;		// The default background color.

	// Shift new data values into the real time data series 
	void getData();
	void kinectUpdate();
	void chartUpdate();
	// Draw chart
	void drawChart(CChartViewer *viewer);
	void trackLineLegend(XYChart *c, int mouseX);
	// utility to get default background color
	int getDefaultBgColor();
	// utility to load icon resource to a button
	void loadButtonIcon(int buttonId, int iconId, int width, int height);

	afx_msg void OnBnClickedRunpb();
	afx_msg void OnRunPB();
	afx_msg void OnFreezePB();
	afx_msg void OnViewPortChanged();
	afx_msg void OnMouseMovePlotArea();
	CButton m_RunPB;
};