// UDT_IRDlg.cpp : 구현 파일
//


#include "stdafx.h"
#include "UDT_IR.h"
#include "UDT_IRDlg.h"
#include "VRClientSocket.h"
#include "DXRenderer.h"
#include "afxdialogex.h"
#include <winsock2.h>
#include <Windows.h>
#include "ChartViewer.h"
#include <vector>
#include <sstream>
#include <time.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

static const int DataRateTimer = 1;
static const int ChartUpdateTimer = 2;
static const int KinectTimer = 3;
static const int DataInterval = 250;

// CUDT_IRDlg 대화 상자
CUDT_IRDlg::CUDT_IRDlg(CWnd* pParent /*=NULL*/) : CDialogEx(CUDT_IRDlg::IDD, pParent)
, m_pKinectSensor(NULL)				// Kinect Sensor Pointer
, m_pCoordinateMapper(NULL)			// Coordinate Mapper for BodyToScreen()
, m_pMultiSourceFrameReader(NULL)	// MultiSource(IR & Body)
, m_pDXRenderer(NULL)				// DirectX Renderer
, m_pD2DFactory(NULL)				// Direct2D Factory
, m_bStart(false)					// Started Flag
, m_nStartTime(0)					// Time When Started
, m_fGain(6.5)						// IR Display Gain
, m_fTime(0)						// Elapsed Time from m_nStartTime
, m_fFPS(0)							// Framerate
, m_fHeadAngle(0)					// Head Rotation Angle
, m_fPelPos(0)						// Pelvis Position
, m_hFile(NULL)						// Data File handle
, m_hFile_FSR(NULL)
, m_pVRClientSocket(NULL)			// Socket For VR Progaram(Unity)
, m_fK1(0.0F)						// Control Variable(Acceleration)
, m_fK2(0.0F)						// Control Variable(Speed)
, m_fK3(0.0F)						// Control Variable(X_diff)
, m_fBeta(0.0F)						// Control Variable(State Observer)
, m_fSpeed(0)						// Command Speed
, m_fPelSpd(0)						// Pelvis Speed
, m_fPelAcc(0)						// Pelvis Acceration
, m_fXref(0)						// Pelvis Reference Position
, m_nYoff(0)						// Offset for Magnetometer's Y axis(right)
, m_nZoff(0)						// Offset for Magnetometer's Z axis(forward)
, m_fAccLmt(0)						// Acceleration Limit
, m_hAOTask(NULL)
, m_hAITask(NULL)
, m_hCITask(NULL)
, m_fSpdAct(0.0F)
, m_fXrpos(0)
, m_nXoff(0)
, m_fDecLmt(0)
, m_fSpdLmt(0)
, m_nMode(0)
, m_fLambda(0)
, m_fFast(0)
, m_fSlow(0)
, m_switch(0)
, m_count(0)
, m_fRepeatTime(0)
, m_fSteady_Time_Interval(0)
, fSpeed_p(0)
, slope(0)
, m_fPertinterval(0)
, fSpeed_pp(0)
, global_current_time(0)
, global_system_time(0)
{

	m_strAO = "Dev3/ao0";
	m_strCI = "Dev3/ctr0";
	m_strAI = "Dev3/ai1:7";
	// Matrix and Vectors for Kalman Filter
	P = Eigen::Matrix2d::Zero();
	K = Eigen::Matrix2d::Zero();
	x = Eigen::Vector2d::Zero();
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CUDT_IRDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	// to Display Time and Rate
	DDX_Control(pDX, IDC_EDIT_TIME, m_editTime);
	DDX_Control(pDX, IDC_EDIT_RATE, m_editRate);

	// to Control IR Display Gain
	DDX_Control(pDX, IDC_SLDR_GAIN, m_sldrGain);
	DDX_Text(pDX, IDC_EDIT_GAIN, m_fGain);
	DDV_MinMaxFloat(pDX, m_fGain, 0.0F, 10.0F);

	// to Control Control Variables
	DDX_Text(pDX, IDC_EDIT_K1, m_fK1);		// K1(acc)
	DDX_Text(pDX, IDC_EDIT_K2, m_fK2);		// K2(spd)
	DDX_Text(pDX, IDC_EDIT_K3, m_fK3);		// K3(pos)

	DDX_Text(pDX, IDC_EDIT_BETA, m_fBeta);	// Beta(obv)
	DDX_Text(pDX, IDC_EDIT_XREF, m_fXref);	// Ref. Pos.
	DDX_Control(pDX, IDC_EDIT_XREF, m_editXref);
	/*
	DDX_Control(pDX, IDC_SPIN_XREF, m_spinXref);
	DDX_Control(pDX, IDC_SPIN_K1, m_spinK1);
	DDX_Control(pDX, IDC_SPIN_K2, m_spinK2);
	DDX_Control(pDX, IDC_SPIN_K3, m_spinK3);
	DDX_Control(pDX, IDC_SPIN_BETA, m_spinBeta);
	DDX_Control(pDX, IDC_SPIN_ACCLMT, m_spinAccLmt);
	DDX_Control(pDX, IDC_SPIN_DECLMT, m_spinDecLmt);
	DDX_Control(pDX, IDC_SPIN_SPDLMT, m_spinSpdLmt);
	*/
	DDX_Text(pDX, IDC_EDIT_ACCLMT, m_fAccLmt);
	DDX_Control(pDX, IDC_EDIT_ACCLMT, m_editAccLmt);

	DDX_Text(pDX, IDC_EDIT_DECLMT, m_fDecLmt);
	DDX_Control(pDX, IDC_EDIT_DECLMT, m_editDecLmt);

	DDX_Text(pDX, IDC_EDIT_SPDLMT, m_fSpdLmt);
	DDX_Control(pDX, IDC_EDIT_SPDLMT, m_editSpdLmt);


	// to Control Magnetometer Offsets
	//DDX_Control(pDX, IDC_SPIN_YOFFSET, m_spinYoff);
	//DDX_Control(pDX, IDC_SPIN_ZOFFSET, m_spinZoff);
	//DDX_Control(pDX, IDC_SPIN_XOFFSET, m_spinXoff);

	DDX_Text(pDX, IDC_EDIT_YOFFSET, m_nYoff);
	DDX_Control(pDX, IDC_EDIT_YOFFSET, m_editYoff);
	DDX_Text(pDX, IDC_EDIT_ZOFFSET, m_nZoff);
	DDX_Control(pDX, IDC_EDIT_ZOFFSET, m_editZoff);
	DDX_Text(pDX, IDC_EDIT_XOFFSET, m_nXoff);
	DDX_Control(pDX, IDC_EDIT_XOFFSET, m_editXoff);

	DDX_Text(pDX, IDC_EDIT_RepeatTime, m_fRepeatTime);
	DDX_Control(pDX, IDC_EDIT_RepeatTime, m_editRepeatTime);
	DDX_Text(pDX, IDC_EDIT_SteadyTimeInterval, m_fSteady_Time_Interval);
	DDX_Control(pDX, IDC_EDIT_SteadyTimeInterval, m_editSteadyTimeInterval);
	DDX_Text(pDX, IDC_EDIT_ComfSPD, m_fComfSpd);
	DDX_Control(pDX, IDC_EDIT_ComfSPD, m_editComfSpd);

	DDX_Control(pDX, IDC_EDIT_ANALOGCHECK, m_editAnalogCheck);

	// to Control GUI
	DDX_Control(pDX, IDC_BTN_START, m_btnStart);
	DDX_Control(pDX, IDC_LIST_MSG, m_listMsg);
	DDX_Control(pDX, IDC_COMBO_COMPORT, m_comboComPort);
	DDX_Control(pDX, IDC_IP_ADDRESS, m_ipAddress);
	DDX_Control(pDX, IDC_EDIT_FILENAME, m_editFilename);
	DDX_Control(pDX, IDC_BTN_MODE, m_btnMode);
	DDX_Control(pDX, IDC_A1, m_ValueA1);
	DDX_Control(pDX, IDC_A2, m_ValueA2);
	DDX_Control(pDX, IDC_A3, m_ValueA3);
	DDX_Control(pDX, IDC_A4, m_ValueA4);
	DDX_Control(pDX, IDC_A5, m_ValueA5);
	DDX_Control(pDX, IDC_A6, m_ValueA6);
	DDX_Control(pDX, IDC_Chart1, m_ChartViewer);
	DDX_Control(pDX, IDC_RunPB, m_RunPB);
}

BEGIN_MESSAGE_MAP(CUDT_IRDlg, CDialogEx)
	// Default
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	// for Slider Control
	ON_WM_HSCROLL()
	// for Internal Timer
	ON_WM_TIMER()
	// for Start Button
	ON_BN_CLICKED(IDC_BTN_START, &CUDT_IRDlg::OnBnClickedBtnStart)
	// for Gain Edit
	ON_EN_CHANGE(IDC_EDIT_GAIN, &CUDT_IRDlg::OnEnChangeEditGain)
	// for Control Variables
	ON_EN_CHANGE(IDC_EDIT_K1, &CUDT_IRDlg::OnEnChangeEditK1)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_K1, &CUDT_IRDlg::OnDeltaposSpinK1)
	ON_EN_CHANGE(IDC_EDIT_K2, &CUDT_IRDlg::OnEnChangeEditK2)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_K2, &CUDT_IRDlg::OnDeltaposSpinK2)
	ON_EN_CHANGE(IDC_EDIT_K3, &CUDT_IRDlg::OnEnChangeEditK3)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_K3, &CUDT_IRDlg::OnDeltaposSpinK3)
	ON_EN_CHANGE(IDC_EDIT_BETA, &CUDT_IRDlg::OnEnChangeEditBeta)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_BETA, &CUDT_IRDlg::OnDeltaposSpinBeta)
	ON_EN_CHANGE(IDC_EDIT_XREF, &CUDT_IRDlg::OnEnChangeEditXref)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_XREF, &CUDT_IRDlg::OnDeltaposSpinXref)
	// for Magnetometer Offsets
	ON_EN_CHANGE(IDC_EDIT_YOFFSET, &CUDT_IRDlg::OnEnChangeEditYoffset)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_YOFFSET, &CUDT_IRDlg::OnDeltaposSpinYoffset)
	ON_EN_CHANGE(IDC_EDIT_ZOFFSET, &CUDT_IRDlg::OnEnChangeEditZoffset)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_ZOFFSET, &CUDT_IRDlg::OnDeltaposSpinZoffset)
	// for Serial Communication
	ON_MESSAGE(WM_COMM_READ, &CUDT_IRDlg::OnReceive)
	// to Close the Window
	ON_WM_CLOSE()
	ON_EN_CHANGE(IDC_EDIT_ACCLMT, &CUDT_IRDlg::OnEnChangeEditAcclmt)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_ACCLMT, &CUDT_IRDlg::OnDeltaposSpinAcclmt)
	ON_NOTIFY(IPN_FIELDCHANGED, IDC_IP_ADDRESS, &CUDT_IRDlg::OnIpnFieldchangedIpAddress)
	ON_BN_CLICKED(IDC_BTN_MODE, &CUDT_IRDlg::OnBnClickedBtnMode)
	ON_EN_CHANGE(IDC_EDIT_XOFFSET, &CUDT_IRDlg::OnEnChangeEditXoffset)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_XOFFSET, &CUDT_IRDlg::OnDeltaposSpinXoffset)
	ON_EN_CHANGE(IDC_EDIT_DECLMT, &CUDT_IRDlg::OnEnChangeEditDeclmt)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DECLMT, &CUDT_IRDlg::OnDeltaposSpinDeclmt)
	ON_EN_CHANGE(IDC_EDIT_SPDLMT, &CUDT_IRDlg::OnEnChangeEditSpdlmt)
	//ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_SPDLMT, &CUDT_IRDlg::OnDeltaposSpinSpdlmt)
	ON_EN_CHANGE(IDC_EDIT_ComfSPD, &CUDT_IRDlg::OnEnChangeEditComfSPD)
	ON_EN_CHANGE(IDC_EDIT_RepeatTime, &CUDT_IRDlg::OnEnChangeEditRepeattime)
	ON_EN_CHANGE(IDC_EDIT_SteadyTimeInterval, &CUDT_IRDlg::OnEnChangeEditSteadytimeinterval)
	ON_BN_CLICKED(IDC_MOVE, &CUDT_IRDlg::OnBnClickedMove)
	ON_CBN_SELCHANGE(IDC_COMBO_COMPORT, &CUDT_IRDlg::OnCbnSelchangeComboComport)
	ON_STN_CLICKED(IDC_PC_CHARTDIR1, &CUDT_IRDlg::OnStnClickedPcChartdir1)

	ON_CONTROL(CVN_ViewPortChanged, IDC_Chart1, OnViewPortChanged)
	ON_CONTROL(CVN_MouseMovePlotArea, IDC_Chart1, OnMouseMovePlotArea)

	ON_STN_CLICKED(IDC_Chart1, &CUDT_IRDlg::OnStnClickedChart1)


	ON_BN_CLICKED(IDC_RunPB, OnRunPB)
	ON_BN_CLICKED(IDC_FreezePB, OnFreezePB)
END_MESSAGE_MAP()

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CUDT_IRDlg::OnPaint()
{
	//remove kinect

	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}

}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CUDT_IRDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

BOOL CUDT_IRDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	// Block Closing the Window by ESC & ENTER Key
	if (pMsg->wParam == VK_RETURN || pMsg->wParam == VK_ESCAPE) return TRUE;

	return CDialogEx::PreTranslateMessage(pMsg);
}

// CUDT_IRDlg 메시지 처리기
BOOL CUDT_IRDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	// GUI Initializing (Slider & Spin can use only unsigned integer)
	m_sldrGain.SetRange(100, 1000);	// Set Range 1 ~ 10
	m_sldrGain.SetPageSize(10);		// Set Size for PgUp & PgDn

	/////////////////////////////////////

	m_extBgColor = getDefaultBgColor();     // Default background color

	// Clear data arrays to Chart::NoValue
	for (int i = 0; i < sampleSize; ++i)
		m_timeStamps[i] = m_dataSeriesA1[i] = m_dataSeriesA2[i] = m_dataSeriesA3[i] = Chart::NoValue;
	m_currentIndex = 0;

	// Set m_nextDataTime to the current time. It is used by the real time random number 
	// generator so it knows what timestamp should be used for the next data point.
	SYSTEMTIME st;
	GetLocalTime(&st);
	m_nextDataTime = Chart::chartTime(st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute,
		st.wSecond) + st.wMilliseconds / 1000.0;
	////////////////////////////////////

	// Load Last Settings
	LoadSettings();
	m_fXrpos = m_fXref;
	m_sldrGain.SetPos((int)(m_fGain * 100));	// Set Position in Integer

	// Automatic Com Port Recognition
	CStringArray* ComPorts = m_Comm.GetPorts();

	m_comboComPort.ResetContent();
	for (int i = 0; i<ComPorts->GetSize(); i++)
		m_comboComPort.AddString(ComPorts->GetAt(i));
	m_comboComPort.SetCurSel(0);

	m_Comm.hCommWnd = this->m_hWnd;

	m_ipAddress.SetWindowTextW(m_strIP);


	// Init Direct2D
	D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

	// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
	// We'll use this to draw the data we receive from the Kinect to the screen
	m_pDXRenderer = new DXRenderer();
	CWnd* Video = GetDlgItem(IDC_PIC_VIDEO);
	HRESULT hr = m_pDXRenderer->Initialize(Video->m_hWnd, m_pD2DFactory, WIDTH, HEIGHT, WIDTH * sizeof(RGBQUAD));

	InitializeDefaultSensor();

	TIMECAPS tc;
	MMRESULT m_Timer;
	MMRESULT m_Timer_Kinect;
	timeGetDevCaps(&tc, sizeof(TIMECAPS));
	unsigned int Resolution = min(max(tc.wPeriodMin, 0), tc.wPeriodMax);
	timeBeginPeriod(Resolution);

	m_Timer_Kinect = timeSetEvent(10, Resolution, (LPTIMECALLBACK)TimerProc_Kinect, (DWORD_PTR)this, TIME_PERIODIC);

	OnRunPB();

	char* AITask = "AnalogInput";

	if (DAQmxFailed(DAQmxCreateTask(AITask, &m_hAITask)))
		PrintStr(&CString(_T("Error : AI/DAQmxCreateTask is not working")));
	else if (DAQmxFailed(DAQmxCreateAIVoltageChan(m_hAITask, m_strAI, AITask, DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, "")))
		PrintStr(&CString(_T("Error : AI/DAQmxCreateAIVoltageChan is not working")));
	else if (DAQmxFailed(DAQmxStartTask(m_hAITask)))
		PrintStr(&CString(_T("Error : AI/DAQmxStartTask is not working")));
	else
	{
		CString strTemp(m_strAI);
		PrintStr(&CString(_T("Analog Input : ") + strTemp));
	}
	global_system_time = timeGetTime();
	global_current_time = 0;
	m_btnStart.EnableWindow(FALSE);
	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CUDT_IRDlg::TimerProc_Kinect(UINT uiID, UINT uiMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{
	CUDT_IRDlg * p = (CUDT_IRDlg*)dwUser;
	p->global_current_time = (timeGetTime() - p->global_system_time)/1000;
	p->getData();
	p->kinectUpdate();
	//p->chartUpdate();
}
void CUDT_IRDlg::TimerProc(UINT uiID, UINT uiMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{
	CUDT_IRDlg * p = (CUDT_IRDlg*)dwUser;
}

#define PROJECT_NAME _T("UDT_IR")
void CUDT_IRDlg::LoadSettings()
{
	CWinApp *pApp = AfxGetApp();

	m_fGain = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("GAIN"), 45) / 10.0);

	m_fXref = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("XREF"), 30) / 10.0F);

	m_fK1 = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("K1"), 1) / 1000.0F);
	m_fK2 = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("K2"), 1001) / 1000.0F);
	m_fK3 = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("K3"), 2000) / 1000.0F);

	m_fBeta = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("BETA"), 11) / 1000.0F);

	m_fAccLmt = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("ACCLMT"), 50) / 100.0F);
	m_fDecLmt = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("DECLMT"), 50) / 100.0F);
	m_fSpdLmt = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("SPDLMT"), 50) / 100.0F);

	m_nXoff = (int)(pApp->GetProfileInt(PROJECT_NAME, _T("XOFF"), 0)) - 200;
	m_nYoff = (int)(pApp->GetProfileInt(PROJECT_NAME, _T("YOFF"), 0)) - 200;
	m_nZoff = (int)(pApp->GetProfileInt(PROJECT_NAME, _T("ZOFF"), 0)) - 200;


	m_fComfSpd = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("ComfSPD"), 100) / 10.0F);
	m_fRepeatTime = (int)(pApp->GetProfileInt(PROJECT_NAME, _T("RepeatTime"), 0));
	m_fSteady_Time_Interval = (float)(pApp->GetProfileInt(PROJECT_NAME, _T("SteadyTimeInterval"), 0));

	m_strIP = pApp->GetProfileStringW(PROJECT_NAME, _T("IP"), _T("127.0.0.1"));

	UpdateData(FALSE);
}

#include "ModeDlg.h"
void CUDT_IRDlg::OnBnClickedBtnMode()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CModeDlg cModeDlg;
	cModeDlg.DoModal();
	m_nMode = cModeDlg.m_nMode;
	switch (m_nMode)
	{
	case 7:
		PrintStr(&CString("Passive Treadmill Mode Selected"));
		break;
	case 8:
		PrintStr(&CString("User Driven Treadmill Mode Selected"));
		break;
	}
	// Initializing TCP port
	m_pVRClientSocket = new CVRClientSocket(this);
	// Try Socket Open
	if (m_pVRClientSocket == NULL)	// Getting Socket Failed
	{
		PrintStr(&CString(_T("Failed to allocate the socket")));	// Message to ListBox
		return;	// Terminate Function
	}
	else if (!m_pVRClientSocket->Create())	// Socket Create Failed
	{
		PrintStr(&CString(_T("Failed to create the socket")));	// Message to ListBox
		ProcessClose();	// Close All
		return;	// Terminate Function
	}

	if (!m_pVRClientSocket->Connect(m_strIP, 9992))
	{
		PrintStr(&CString(_T("Failed to connect to VR server")));	// Message to ListBox
		ProcessClose();	// Close All
		return;	// Terminate Function
	}

	char strSend2[27] = { 0, };
	for (UINT i = 0; i < 100; i++)
	{
		sprintf_s(strSend2, "%d\n", cModeDlg.m_nMode);
		m_pVRClientSocket->Send(strSend2, static_cast<int>(strlen(strSend2)));
	}

	if (m_hFile)
	{
		CloseHandle(m_hFile);
		m_hFile = NULL;
	}
	if (m_hFile_FSR)
	{
		CloseHandle(m_hFile_FSR);
		m_hFile_FSR = NULL;
	}
	ProcessClose();
	m_btnStart.EnableWindow(TRUE);
	m_btnMode.EnableWindow(FALSE);
	Wait(1000);
}

void CUDT_IRDlg::OnBnClickedBtnStart()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (!m_bStart)	// Start
	{
		//if (m_nMode == 0)//이부분 수정(161030)
		//{
		//	// Initializing Serial Port
		//	CString strPortNum;
		//	m_comboComPort.GetWindowTextW(strPortNum);	// Get Port Number from ComboBox

		//	if (m_Comm.OpenPort(_T("\\\\.\\") + strPortNum, _T("38400"), _T("8"), _T("NO"), _T("NO"), _T("1")))// Port Open
		//		PrintStr(&CString(_T("Connected to ") + strPortNum));
		//	else//Port Open Failed
		//		PrintStr(&CString(_T("Could not connect to ") + strPortNum));
		//}

		// Initializing TCP port
		m_pVRClientSocket = new CVRClientSocket(this);
		// Try Socket Open
		if (m_pVRClientSocket == NULL)	// Getting Socket Failed
		{
			PrintStr(&CString(_T("Failed to allocate the socket")));	// Message to ListBox
			return;	// Terminate Function
		}
		else if (!m_pVRClientSocket->Create())	// Socket Create Failed
		{
			PrintStr(&CString(_T("Failed to create the socket")));	// Message to ListBox
			ProcessClose();	// Close All
			return;	// Terminate Function
		}

		if (!m_pVRClientSocket->Connect(m_strIP, 9992))
		{
			PrintStr(&CString(_T("Failed to connect to VR server")));	// Message to ListBox
			ProcessClose();	// Close All
			return;	// Terminate Function
		}

		// DAQmx initializing
		char* AOTask = "AnalogOutput";

		if (DAQmxFailed(DAQmxCreateTask(AOTask, &m_hAOTask)))
			PrintStr(&CString(_T("Check connection with Device...2")));
		else if (DAQmxFailed(DAQmxCreateAOVoltageChan(m_hAOTask, m_strAO, AOTask, -10.0, 10.0, DAQmx_Val_Volts, "")))
			PrintStr(&CString(_T("Check connection with Device...3")));
		else if (DAQmxFailed(DAQmxStartTask(m_hAOTask)))
			PrintStr(&CString(_T("Check connection with Device...4")));
		else
		{
			CString strTemp(m_strAO);
			PrintStr(&CString(_T("Analog Output : ") + strTemp));
		}

		char* CITask = "CounterInput";

		if (DAQmxFailed(DAQmxCreateTask(CITask, &m_hCITask)))
			PrintStr(&CString(_T("Check connection with Device...5")));
		else if (DAQmxFailed(DAQmxCreateCIAngEncoderChan(m_hCITask, m_strCI, CITask, DAQmx_Val_X4, FALSE, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Radians, 2000, 0.0, "")))
			PrintStr(&CString(_T("Check connection with Device...6")));
		else if (DAQmxFailed(DAQmxStartTask(m_hCITask)))
			PrintStr(&CString(_T("Check connection with Device...7")));
		else
		{
			CString strTemp(m_strCI);
			PrintStr(&CString(_T("Counter Input : ") + strTemp));
		}


		// Set Zero Joint Position
		memset(m_pMarkers, 0, sizeof(CameraSpacePoint)* 25);
		// Set Zero Timestamp
		m_nStartTime = 0;
		// Disable ComboBox
		m_comboComPort.EnableWindow(FALSE);
		// Change Button Name
		m_btnStart.SetWindowTextW(_T("End"));
		m_bStart = true;

	}
	else	// End
	{
		if (m_Comm.m_bConnected)
		{
			m_Comm.ClosePort();
			PrintStr(&CString("Port Closed"));
		}
		if (m_pVRClientSocket)
		{
			m_pVRClientSocket->Close();
			delete m_pVRClientSocket;
			m_pVRClientSocket = NULL;
			PrintStr(&CString(_T("Socket Closed")));
		}

		if (m_hAOTask)
		{
			while (m_fSpeed > 0)
			{
				m_fSpeed -= 0.6f / m_fFPS;
				float64 data[1] = { -m_fSpeed / 0.0505f / 2 / 3.141592f * 60 * 60 / 28 / 100 }; //gray treadmill
				//float64 data[1] = { m_fSpeed / 0.0475f * 25 / 7 * 30 / 3.141592f / 100 }; // yellow treadmill
				if (DAQmxFailed(DAQmxWriteAnalogF64(m_hAOTask, 1, TRUE, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL)))
					PrintStr(&CString(_T("Check connection with Device...1")));
				Sleep(33);
			}

			DAQmxStopTask(m_hAOTask);
			DAQmxClearTask(m_hAOTask);
			m_hAOTask = NULL;
			PrintStr(&CString("Analog Output Closed"));
		}

		if (m_hCITask)
		{
			DAQmxStopTask(m_hCITask);
			DAQmxClearTask(m_hCITask);
			m_hCITask = NULL;
			PrintStr(&CString("Counter Input Closed"));
		}

		m_nMode = 0;
		m_switch = 0;
		m_count = 0;
		fSpeed_p = 0;
		m_bStart = false;

		m_comboComPort.EnableWindow(TRUE);
		m_btnStart.SetWindowTextW(_T("Start"));
		m_btnStart.EnableWindow(FALSE);
		m_btnMode.EnableWindow(TRUE);
		if (m_hFile)
		{
			CloseHandle(m_hFile);
			m_hFile = NULL;
		}
		if (m_hFile_FSR)
		{
			CloseHandle(m_hFile_FSR);
			m_hFile_FSR = NULL;
		}
	}
}

HRESULT CUDT_IRDlg::InitializeDefaultSensor()
{

	HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		if (SUCCEEDED(hr))
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Infrared
				| FrameSourceTypes::FrameSourceTypes_Body,
				&m_pMultiSourceFrameReader);
		}
	}

	if (!m_pKinectSensor || FAILED(hr)){
		PrintStr(&CString(_T("Kinect Conection Failed...2")));
		return E_FAIL;
	}

	return hr;

}


/////////////////////////////////////////////////

//
// User clicks on the Run pushbutton
//
void CUDT_IRDlg::OnRunPB()
{
}

//
// User clicks on the Freeze pushbutton
//
void CUDT_IRDlg::OnFreezePB()
{
}

//////////////////////////////////////////////////////////

static void shiftData(double *data, int len, double newValue)
{
	memmove(data, data + 1, sizeof(*data) * (len - 1));
	data[len - 1] = newValue;
}

//
// The data acquisition routine. In this demo, this is invoked every 250ms.
//
void CUDT_IRDlg::kinectUpdate()
{
	if (!m_pMultiSourceFrameReader){	// Terminate if There's not FrameReader
		PrintStr(&CString(_T("Kinect Conection Failed...2")));
		return;
	}

	// Allocate IR & Body Frames
	IMultiSourceFrame* pMultiSourceFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);
	IInfraredFrame* pIRFrame = NULL;
	IBodyFrame*		pBodyFrame = NULL;

	if (SUCCEEDED(hr))	// Get Frames
		hr = GetFrame(pMultiSourceFrame, &pIRFrame, &pBodyFrame);

	if (SUCCEEDED(hr))
	{

		// Get Time Data
		INT64 nTime = 0;			// Timestamp(usec)
		static INT64 nTime_p = 0;	// Timestamp_past(usec)
		CString str = _T("");		// Temporary String to Display Time & Rate

		hr = pIRFrame->get_RelativeTime(&nTime);	// Get Timestamp

		if (m_nStartTime == 0)		// If Just Started
			m_nStartTime = nTime;	// Set StartTime to Present

		// Elapsed Time(second)
		m_fTime = (float)((nTime - m_nStartTime) / 10000000.0f);
		// Display Time to EditBox
		str.Format(_T("%4.3f"), global_current_time);
		m_editTime.SetWindowTextW(str);

		// FrameRate(Hz)
		m_fFPS = (float)(10000000.0f / (float)(nTime - nTime_p));
		// Display Framerate to EditBox
		str.Format(_T("%2d"), (int)floor(m_fFPS + 0.5));
		m_editRate.SetWindowTextW(str);

		// Update Timestamp_past
		nTime_p = nTime;

		// get IR frame data
		UINT nIRBufferSize = 0;		// IR BufferSize
		UINT16 *pIRBuffer = NULL;	// IR Buffer

		if (SUCCEEDED(hr))	// Get Frame Buffer
			hr = GetBuffer(pIRFrame, &nIRBufferSize, &pIRBuffer);

		// Process Output Frame
		if (SUCCEEDED(hr))
			hr = ProcessFrame(nTime, pIRBuffer, WIDTH, HEIGHT);

		// Set All markers to zero
		memset(m_pMarkers, 0, sizeof(CameraSpacePoint)* 25);

		// Get Body Data from Kinect //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// data initializing
		IBody* ppBodies[NUM_BODIES] = { NULL, };
		IBody* pBody = NULL;
		BOOLEAN bTracked = FALSE;
		Joint Joints[JointType_Count];
		memset(Joints, 0, JointType_Count*sizeof(Joint));

		// Joint index array for convinience
		int JointIndex[8] = { JointType_SpineBase, JointType_AnkleRight, JointType_AnkleLeft, JointType_WristRight, JointType_WristLeft, JointType_Head, JointType_HipRight, JointType_HipLeft };

		if (SUCCEEDED(hr))
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < NUM_BODIES; i++)
			{
				if (pBody = ppBodies[i])
					hr = pBody->get_IsTracked(&bTracked);

				if (SUCCEEDED(hr) && bTracked)
				{
					D2D1_POINT_2F jointPoints[JointType_Count];
					hr = pBody->GetJoints(JointType_Count, Joints);
					if (SUCCEEDED(hr))
					{
						for (int j = 0; j < JointType_Count; j++)
						{
							jointPoints[j] = m_pDXRenderer->BodyToScreen(m_pCoordinateMapper, Joints[j].Position, WIDTH, HEIGHT);

							if (m_bStart
								&& abs(Joints[JointType_SpineBase].Position.X) < 0.4)
							{
								for (UINT k = 0; k < 8; k++)
								{
									if (j == JointIndex[k])
										memcpy(&m_pMarkers[k], &Joints[j].Position, sizeof(CameraSpacePoint));
								}
							}
						}
						m_pDXRenderer->DrawBody(Joints, jointPoints);
					}
				}
			}
		}

		if (m_bStart)
		{
			BYTE c = 'Q';
			m_Comm.WriteComm(&c, 1);
			m_fHeadAngle = KalmanAngle(m_fSensors);
			CalcFeedBack();
			static float fSpeed_p = 0;

			if (m_nMode == 8) // active mode
			{
				if (m_pMarkers[PELVIS].Z)
				{
					float Vw = CalcVwSwing();
					m_fXrpos = m_fXref + Vw * 0.3f / m_fSpdLmt;
					if (m_fPelPos < 0)
						m_fSpeed = 0;
					else
					{
						m_fSpeed = (float)(m_fK1 * m_fPelAcc + (m_fK2 - 1) * m_fPelSpd + m_fK3 * (m_fPelPos - m_fXrpos)) + Vw;
						m_fSpeed = m_fSpeed+2.5;
						CString check;
						check.Format(_T("%0.3f"), m_fSpeed);
						PrintStr(&CString(_T("m_fSpeed") + check));
					}


				}
				else
				{
					static float m_fStartTime = 0;

					PrintStr(&CString(_T("Error : Pelvis Position is not detected")));
					//m_fSpeed -= 0.3f / m_fFPS;
					m_fSpeed -= 0.1f / m_fFPS;
					m_fLambda = 0.0f;
				}

			}

			//passive mode			
			else if (m_nMode == 7)
			{
				PassiveMode();

			}


			if (m_fSpeed < 0)
				m_fSpeed = 0;
			if (m_fSpeed >= m_fSpdLmt)
				m_fSpeed = m_fSpdLmt;
			fSpeed_p = m_fSpeed;

			if (m_hAOTask)
			{
				//float64 data[1] = { -m_fSpeed / 0.0505f * 60 / 28 / (2 * 3.141592f) * 60 / 100 }; //gray treadmill
				float64 data[1] = { m_fSpeed / 0.0475f * 25 / 7 / (2 * 3.141592f) * 60 / 100 }; // yellow treadmill
				//                    Speed / R * r2 / r1 * (toRPM) / 100(1000RPM to 10V)
				if (DAQmxFailed(DAQmxWriteAnalogF64(m_hAOTask, 1, TRUE, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL)))
					PrintStr(&CString(_T("Error : DAQmxWriteAnalogF64/A0 is not working")));
			}

			if (m_hCITask)
			{
				int32 read;
				float64 data[1];
				static float64 data_p = 0.0;
				if (DAQmxFailed(DAQmxReadCounterF64(m_hCITask, 1, 10.0, data, 1, &read, 0)))
					PrintStr(&CString(_T("Error : DAQmxReadCounterF64/Cl is not working")));
				else
				{
					m_fSpdAct = (float)(data_p - data[0])*m_fFPS;
					data_p = data[0];
				}
			}

			if (m_pVRClientSocket)
			{
				char strSend2[27] = { 0, };

				sprintf_s(strSend2, "%.3f %.2f %d\n", m_fSpeed, /*m_fHeadAngle,*/ m_fComfSpd /*,CalcArm(Joints)*/);//,m_fRepeatTime, m_fSteady_Time_Interval);
				m_pVRClientSocket->Send(strSend2, static_cast<int>(strlen(strSend2)));
			}

			DataSave(global_current_time);

			CString str;
			str.Format(_T("%.2f"), m_fHeadAngle);
			SetDlgItemTextW(IDC_EDIT_HEADANGLE, str);
			str.Format(_T("%.3f"), m_fSpeed);
			SetDlgItemTextW(IDC_EDIT_SPEED, str);
		}
	}


	SafeRelease(pIRFrame);
	SafeRelease(pBodyFrame);
	SafeRelease(pMultiSourceFrame);


	CWnd* Video = GetDlgItem(IDC_PIC_VIDEO);
	CRect rect;
	Video->GetClientRect(&rect);
	InvalidateRect(&rect, 0);
}

void CUDT_IRDlg::chartUpdate()
{
	m_ChartViewer.updateViewPort(true, false);
}

void CUDT_IRDlg::getData()
{
	if (m_hAITask)
	{
		int32 read;
		float64 data[5000];
		if (DAQmxFailed(DAQmxReadAnalogF64(m_hAITask, -1, 10.0, DAQmx_Val_GroupByChannel, data, 5000, &read, NULL)))
			PrintStr(&CString(_T("Error : AI/DAQmxReadAnalogF64 is not working")));
		else
		{
			m_fA1 = data[0];
			m_fA2 = data[1];
			m_fA3 = data[2];
			m_fA4 = data[3];
			m_fA5 = data[4];
			m_fA6 = data[5];
			m_fA7 = data[6];
		}
	}
	SYSTEMTIME st;
	GetLocalTime(&st);
	double now = Chart::chartTime(st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute,
		st.wSecond) + st.wMilliseconds / 1000.0;

	// This is our formula for the random number generator
	do
	{
		// Get a data sample
		double p = m_nextDataTime * 4;
		double dataA1 = m_fA1 * 100;
		double dataA2 = m_fA2 * 100;
		double dataA3 = m_fA3 * 100;
		double dataA4 = m_fA4 * 100 + 300;
		double dataA5 = m_fA5 * 100 + 300;
		double dataA6 = m_fA6 * 100 + 300;
		double dataA7 = m_fA7;

		// After obtaining the new values, we need to update the data arrays.
		if (m_currentIndex < sampleSize)
		{
			// Store the new values in the current index position, and increment the index.
			m_dataSeriesA1[m_currentIndex] = dataA1;
			m_dataSeriesA2[m_currentIndex] = dataA2;
			m_dataSeriesA3[m_currentIndex] = dataA3;
			m_dataSeriesA4[m_currentIndex] = dataA4;
			m_dataSeriesA5[m_currentIndex] = dataA5;
			m_dataSeriesA6[m_currentIndex] = dataA6;
			m_dataSeriesA7[m_currentIndex] = dataA7;
			m_timeStamps[m_currentIndex] = m_nextDataTime;
			++m_currentIndex;
		}
		else
		{
			// The data arrays are full. Shift the arrays and store the values at the end.
			shiftData(m_dataSeriesA1, sampleSize, dataA1);
			shiftData(m_dataSeriesA2, sampleSize, dataA2);
			shiftData(m_dataSeriesA3, sampleSize, dataA3);
			shiftData(m_dataSeriesA4, sampleSize, dataA4);
			shiftData(m_dataSeriesA5, sampleSize, dataA5);
			shiftData(m_dataSeriesA6, sampleSize, dataA6);
			shiftData(m_dataSeriesA7, sampleSize, dataA7);
			shiftData(m_timeStamps, sampleSize, m_nextDataTime);
		}

		m_nextDataTime += DataInterval / 1000.0;
	} while (m_nextDataTime < now);


	if (m_nMode == 7 || m_nMode == 8)
	{
		DataSave_FSR(global_current_time, m_dataSeriesA1[m_currentIndex - 1], m_dataSeriesA2[m_currentIndex - 1], m_dataSeriesA3[m_currentIndex - 1], m_dataSeriesA4[m_currentIndex - 1], m_dataSeriesA5[m_currentIndex - 1], m_dataSeriesA6[m_currentIndex - 1], m_dataSeriesA7[m_currentIndex - 1]);

	}
	//
	// We provide some visual feedback to the latest numbers generated, so you can see the
	// data being generated.
	//

	char buffer[1024];

	sprintf_s(buffer, "%.2f", m_dataSeriesA1[m_currentIndex - 1]);
	m_ValueA1.SetWindowText(CString(buffer));

	sprintf_s(buffer, "%.2f", m_dataSeriesA2[m_currentIndex - 1]);
	m_ValueA2.SetWindowText(CString(buffer));

	sprintf_s(buffer, "%.2f", m_dataSeriesA3[m_currentIndex - 1]);
	m_ValueA3.SetWindowText(CString(buffer));

	sprintf_s(buffer, "%.2f", m_dataSeriesA4[m_currentIndex - 1]);
	m_ValueA4.SetWindowText(CString(buffer));

	sprintf_s(buffer, "%.2f", m_dataSeriesA5[m_currentIndex - 1]);
	m_ValueA5.SetWindowText(CString(buffer));

	sprintf_s(buffer, "%.2f", m_dataSeriesA6[m_currentIndex - 1]);
	m_ValueA6.SetWindowText(CString(buffer));


}



//
// Draw the chart and display it in the given viewer
//
void CUDT_IRDlg::drawChart(CChartViewer *viewer)
{
	// Create an XYChart object 600 x 270 pixels in size, with light grey (f4f4f4) 
	// background, black (000000) border, 1 pixel raised effect, and with a rounded frame.
	XYChart *c = new XYChart(600, 270, 0xf4f4f4, 0x000000, 1);
	c->setRoundedFrame(m_extBgColor);

	// Set the plotarea at (55, 55) and of size 520 x 185 pixels. Use white (ffffff) 
	// background. Enable both horizontal and vertical grids by setting their colors to 
	// grey (cccccc). Set clipping mode to clip the data lines to the plot area.
	c->setPlotArea(55, 55, 520, 185, 0xffffff, -1, -1, 0xcccccc, 0xcccccc);
	c->setClipping();

	// Add a title to the chart using 15 pts Times New Roman Bold Italic font, with a light
	// grey (dddddd) background, black (000000) border, and a glass like raised effect.
	c->addTitle("Force sensing resistors", "timesbi.ttf", 15
		)->setBackground(0xdddddd, 0x000000, Chart::glassEffect());

	// Set the reference font size of the legend box
	c->getLegend()->setFontSize(8);

	// Configure the y-axis with a 10pts Arial Bold axis title
	c->yAxis()->setTitle("Force (mV)", "arialbd.ttf", 10);

	// Configure the x-axis to auto-scale with at least 75 pixels between major tick and 
	// 15  pixels between minor ticks. This shows more minor grid lines on the chart.
	c->xAxis()->setTickDensity(75, 15);

	// Set the axes width to 2 pixels
	c->xAxis()->setWidth(2);
	c->yAxis()->setWidth(2);

	// Now we add the data to the chart. 
	double firstTime = m_timeStamps[0];
	if (firstTime != Chart::NoValue)
	{
		// Set up the x-axis to show the time range in the data buffer
		c->xAxis()->setDateScale(firstTime, firstTime + DataInterval * sampleSize / 1000);

		// Set the x-axis label format
		c->xAxis()->setLabelFormat("{value|hh:nn:ss}");

		// Create a line layer to plot the lines
		LineLayer *layer = c->addLineLayer();

		// The x-coordinates are the timeStamps.
		layer->setXData(DoubleArray(m_timeStamps, sampleSize));

		// The 3 data series are used to draw 3 lines.
		layer->addDataSet(DoubleArray(m_dataSeriesA1, sampleSize), 0xff0000, "A1");
		layer->addDataSet(DoubleArray(m_dataSeriesA2, sampleSize), 0x00cc00, "A2");
		layer->addDataSet(DoubleArray(m_dataSeriesA3, sampleSize), 0x0000ff, "A3");
		layer->addDataSet(DoubleArray(m_dataSeriesA4, sampleSize), 0xff0000, "A4");
		layer->addDataSet(DoubleArray(m_dataSeriesA5, sampleSize), 0x00cc00, "A5");
		layer->addDataSet(DoubleArray(m_dataSeriesA6, sampleSize), 0x0000ff, "A6");
		layer->addDataSet(DoubleArray(m_dataSeriesA7, sampleSize), 0x0000ff, "A7");
	}

	// Set the chart image to the WinChartViewer
	delete viewer->getChart();
	viewer->setChart(c);
}


int CUDT_IRDlg::getDefaultBgColor()
{
	LOGBRUSH LogBrush;
	HBRUSH hBrush = (HBRUSH)SendMessage(WM_CTLCOLORDLG, (WPARAM)CClientDC(this).m_hDC,
		(LPARAM)m_hWnd);
	::GetObject(hBrush, sizeof(LOGBRUSH), &LogBrush);
	int ret = LogBrush.lbColor;
	return ((ret & 0xff) << 16) | (ret & 0xff00) | ((ret & 0xff0000) >> 16);
}
//
// View port changed event
//
void CUDT_IRDlg::OnViewPortChanged()
{
	drawChart(&m_ChartViewer);
}

void CUDT_IRDlg::OnMouseMovePlotArea()
{
	m_ChartViewer.updateDisplay();
}

//////////////////////////////////////////////////////////

////////////////////////////////////////////////


void CUDT_IRDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.




	CDialogEx::OnTimer(nIDEvent);

}


HRESULT CUDT_IRDlg::GetFrame(IMultiSourceFrame* pMultiSourceFrame, IInfraredFrame** ppIRFrame, IBodyFrame** ppBodyFrame)
{


	IInfraredFrameReference* pIRFrameReference = NULL;

	HRESULT hr = pMultiSourceFrame->get_InfraredFrameReference(&pIRFrameReference);
	if (SUCCEEDED(hr))
		hr = pIRFrameReference->AcquireFrame(ppIRFrame);
	SafeRelease(pIRFrameReference);
	if (SUCCEEDED(hr))
	{
		IBodyFrameReference* pBodyFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))
			hr = pBodyFrameReference->AcquireFrame(ppBodyFrame);
		SafeRelease(pBodyFrameReference);
	}
	return hr;

}

void CUDT_IRDlg::CalcFeedBack()
{
	static float fPelPos_p = 0;
	static float fPelSpd_p = 0;
	static UINT nError = 0;
	if (m_pMarkers[PELVIS].Z != 0)
	{
		/*m_fPelPos = m_pMarkers[PELVIS].Z;

		m_fPelSpd = (m_fPelPos - fPelPos_p) * m_fFPS / (nError + 1);
		m_fPelAcc = (m_fPelSpd - fPelSpd_p) * m_fFPS / (nError + 1);*/

		m_fPelPos = m_fXref - m_pMarkers[PELVIS].Z; // modified on 17.08.05
		m_fPelSpd = -(m_fPelPos - fPelPos_p) * m_fFPS / (nError + 1);
		m_fPelAcc = -(m_fPelSpd - fPelSpd_p) * m_fFPS / (nError + 1);

		fPelPos_p = m_fPelPos;
		fPelSpd_p = m_fPelSpd;
		nError = 0;
	}

	else
		nError++;

	CString str;
	str.Format(_T("%.3f"), m_fPelPos);
	SetDlgItemTextW(IDC_EDIT_PELPOS, str);
}

float CUDT_IRDlg::CalcVwObv()
{
	float lambda_ = -m_fBeta * m_fLambda - m_fSpeed + m_fBeta * m_fPelPos;
	m_fLambda += lambda_ / m_fFPS;

	return m_fBeta * (m_fPelPos - m_fLambda);
}


int CUDT_IRDlg::CalcArm(Joint* Joints)
{
	/*static float WL_p = 0;
	static float WR_p = 0;

	float WL = Joints[JointType_WristLeft].Position.Z;
	float WR = Joints[JointType_WristRight].Position.Z;
	int ret = 0;

	if (WL != 0 && WR != 0)
	{
	if ((WL_p - WL) >= 1.2 && (WR_p - WR) >= 1.2)
	ret = 1;
	}

	WL_p = WL;
	WR_p = WR;

	return ret;*/

	if (m_pMarkers[WRIST_R].Z != 0 && m_pMarkers[WRIST_L].Z != 0)
	{
		if (abs(m_pMarkers[WRIST_R].Z - m_pMarkers[PELVIS].Z >= 0.2)
			&& abs(m_pMarkers[WRIST_L].Z - m_pMarkers[PELVIS].Z >= 0.2))
			return 1;
		else
			return 0;
	}
	else
		return 0;
}


double CUDT_IRDlg::KalmanAngle(double* sensor)
{
	double dt = 1.0 / m_fFPS;
	Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
	R(0, 0) = 0.01;
	R(1, 1) = 0.01;
	Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
	A(0, 1) = dt;
	Eigen::Matrix2d Q;
	Q << pow(dt, 4) / 4, pow(dt, 3) / 2,
		pow(dt, 3) / 2, pow(dt, 2);
	Eigen::Vector2d xp = A*x;
	Eigen::Matrix2d Pp = A*P*A.transpose() + Q;
	Eigen::Matrix2d temp = Pp + R;
	K = Pp*temp.inverse();

	/*double roll = atan2(-sensor[1], sensor[2]) * 1.5707;
	double pitch = atan2(-sensor[1], sensor[0]) * 1.5707;

	double X = sensor[7] * cos(roll) + sensor[6] * sin(pitch) * sin(roll) + sensor[8] * cos(pitch) * sin(roll);
	double Y = sensor[7] * cos(pitch) - sensor[8] * sin(roll);*/

	Eigen::Vector2d v(atan2(sensor[8], sensor[7]) * 180 / 3.141592, -HDR(sensor[4]) * 1000);
	x = xp + K*(v - xp);
	P = Pp - K*Pp;

	return x(0);
}

#define TH (0.01)
double CUDT_IRDlg::HDR(double gyro)
{
	static double I = 0;
	double i_c = 1e-5;
	static double r = 1;
	double c1 = 0.1;
	double c2 = 0.15;
	double hdr = gyro + I;
	static double gyro_p = 0.0;

	if ((gyro > 0 && gyro_p > 0) || ((gyro < 0 && gyro_p < 0)))
		r++;
	else
		r = 1;

	double R = (1 + c1) / (1 + c1 * pow(r, c2));

	if (-(TH) < gyro_p && gyro_p < 0)
		I = I + R*i_c;
	else if (0 < gyro_p && gyro_p < TH)
		I = I - R*i_c;

	return hdr;
}

void CUDT_IRDlg::DataSave(float fTime)
{
	if (m_hFile == NULL && m_bStart)
	{
		PrintStr(&CString(_T("File Created")));
		CString Filename;
		CString Filepath;
		CString strFile;

		GetCurrentDirectory(sizeof(Filepath), (LPWSTR)(LPCWSTR)Filepath);

		Filepath.Format(_T("TreadmillData"));

		m_editFilename.GetWindowTextW(Filename);

		if (Filename.IsEmpty())
		{
			SYSTEMTIME time;
			GetLocalTime(&time);
			Filename.Format(_T("%d-%02d-%02d_h%02dm%02ds%02d.txt"), time.wYear, time.wMonth, time.wDay, time.wHour, time.wMinute, time.wSecond);
		}
		else{
			Filename = Filename + _T(".txt");
		}

		PathCombine((LPWSTR)(LPCWSTR)strFile, Filepath, Filename);

		m_hFile = CreateFile(strFile, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, 0, NULL);


		DWORD nWritten;
		char strData[73] = { 0 };
		sprintf_s(strData, "%% Time\t");
		WriteFile(m_hFile, strData, (DWORD)(sizeof(char)* strlen(strData)), &nWritten, NULL);
		sprintf_s(strData, "SBX\tSBY\tSBZ\tFRX\tFRY\tFRZ\tFLX\tFLY\tFLZ\tWRX\tWRY\tWRZ\tWLX\tWLY\tWLZ\t");
		// add head, hip left & right on 17.09.21
		WriteFile(m_hFile, strData, (DWORD)(sizeof(char)* strlen(strData)), &nWritten, NULL);
		sprintf_s(strData, "HDX\tHDY\tHDZ\tHRX\tHRY\tHRZ\tHRLX\tHLY\tHLZ\t");
		WriteFile(m_hFile, strData, (DWORD)(sizeof(char)* strlen(strData)), &nWritten, NULL);
		//sprintf_s(strData, "SpdCom\tax\tay\taz\tgx\tgy\tgz\tmx\tmy\tmz\tAngle");
		sprintf_s(strData, "SpdCom\tax\tay\taz\tgx\tgy\tgz\tmx\tmy\tmz\tAngle");
		WriteFile(m_hFile, strData, (DWORD)(sizeof(char)* strlen(strData)), &nWritten, NULL);
		WriteFile(m_hFile, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);
	}
	else if (m_bStart)
	{
		DWORD nWritten;
		char strData[10] = { 0, };
		sprintf_s(strData, "%.3f\t", fTime);
		WriteFile(m_hFile, strData, (DWORD)(sizeof(char)* strlen(strData)), &nWritten, NULL);
		for (UINT i = 0; i < 8; i++)
		{
			char str[30] = { 0, };
			sprintf_s(str, "%.4f\t%.4f\t%.4f\t", m_pMarkers[i].X, m_pMarkers[i].Y, m_pMarkers[i].Z);
			WriteFile(m_hFile, str, (DWORD)(sizeof(char)* strlen(str)), &nWritten, NULL);
		}

		sprintf_s(strData, "%.3f\t", m_fSpeed);
		WriteFile(m_hFile, strData, (DWORD)(sizeof(char)* strlen(strData)), &nWritten, NULL);
		WriteFile(m_hFile, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);
	}
}


void CUDT_IRDlg::DataSave_FSR(float fTime, double FSR1, double FSR2, double FSR3, double FSR4, double FSR5, double FSR6, double trigger)
{
	if (m_hFile_FSR == NULL)
	{
		CString Filename;
		CString Filepath;
		CString strFile;

		GetCurrentDirectory(sizeof(Filepath), (LPWSTR)(LPCWSTR)Filepath);

		Filepath.Format(_T("TreadmillData"));

		m_editFilename.GetWindowTextW(Filename);

		if (Filename.IsEmpty())
		{
			SYSTEMTIME time;
			GetLocalTime(&time);
			Filename.Format(_T("%d-%02d-%02d_h%02dm%02ds%02d-FSR.txt"), time.wYear, time.wMonth, time.wDay, time.wHour, time.wMinute, time.wSecond);
		}
		else{

			Filename = Filename + _T("-FSR.txt");
		}

		PathCombine((LPWSTR)(LPCWSTR)strFile, Filepath, Filename);

		m_hFile_FSR = CreateFile(strFile, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, 0, NULL);


		DWORD nWritten;
		char index[100] = { 0 };

		sprintf_s(index, "%% ref_X :%.4f", m_fXref);
		WriteFile(m_hFile_FSR, index, (DWORD)(sizeof(char)* strlen(index)), &nWritten, NULL);
		WriteFile(m_hFile_FSR, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);

		sprintf_s(index, "%% max_spd :%.4f", m_fSpdLmt);
		WriteFile(m_hFile_FSR, index, (DWORD)(sizeof(char)* strlen(index)), &nWritten, NULL);
		WriteFile(m_hFile_FSR, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);

		sprintf_s(index, "%% com_spd :%.4f", m_fComfSpd);
		WriteFile(m_hFile_FSR, index, (DWORD)(sizeof(char)* strlen(index)), &nWritten, NULL);
		WriteFile(m_hFile_FSR, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);

		sprintf_s(index, "%% repeat time :%.4f", m_fRepeatTime);
		WriteFile(m_hFile_FSR, index, (DWORD)(sizeof(char)* strlen(index)), &nWritten, NULL);
		WriteFile(m_hFile_FSR, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);

		sprintf_s(index, "%% steady_interval :%.4f", m_fSteady_Time_Interval);
		WriteFile(m_hFile_FSR, index, (DWORD)(sizeof(char)* strlen(index)), &nWritten, NULL);
		WriteFile(m_hFile_FSR, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);

		char strData[73] = { 0 };
		sprintf_s(strData, "%% Time\tA1\tA2\tA3\tA4\tA5\tA6\tTrigger\tSpeed\t");
		WriteFile(m_hFile_FSR, strData, (DWORD)(sizeof(char)* strlen(strData)), &nWritten, NULL);
		WriteFile(m_hFile_FSR, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);
	}
	else if (m_nMode == 7 || m_nMode == 8)
	{
		DWORD nWritten;
		char strData[10] = { 0, };
		sprintf_s(strData, "%.3f\t", fTime);
		WriteFile(m_hFile_FSR, strData, (DWORD)(sizeof(char)* strlen(strData)), &nWritten, NULL);

		char str[100] = { 0, };
		sprintf_s(str, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t", FSR1, FSR2, FSR3, FSR4 - 300, FSR5 - 300, FSR6 - 300, trigger, m_fSpeed);
		WriteFile(m_hFile_FSR, str, (DWORD)(sizeof(char)* strlen(str)), &nWritten, NULL);
		WriteFile(m_hFile_FSR, "\r\n", (DWORD)(sizeof(char)* 2), &nWritten, NULL);

	}
}


HRESULT CUDT_IRDlg::ProcessFrame(INT64 nTime, const UINT16* pBuffer, UINT nWidth, UINT nHeight)
{
	// Make sure we've received valid data
	if (m_pCoordinateMapper && m_pOutputRGBX && pBuffer)
	{
		for (UINT32 index = 0; index < DATA_LENGTH; index++)
		{
			byte intensity = static_cast<byte>(min(255.0f, CalcRatio(static_cast<float>(pBuffer[index])) * m_fGain * 255.0f));

			//(m_pOutputRGBX + index)->rgbRed = intensity;
			//(m_pOutputRGBX + index)->rgbGreen = intensity;
			//(m_pOutputRGBX + index)->rgbBlue = intensity;

			// 좌우 반전!!
			UINT x = index % WIDTH;
			UINT y = index / WIDTH;
			(m_pOutputRGBX + (y + 1) * WIDTH - x - 1)->rgbRed = intensity;
			(m_pOutputRGBX + (y + 1) * WIDTH - x - 1)->rgbGreen = intensity;
			(m_pOutputRGBX + (y + 1) * WIDTH - x - 1)->rgbBlue = intensity;
		}// for (UINT32 index = 0; index < (nIRWidth * nIRHeight); index++)

		// Draw the data with Direct2D
		m_pDXRenderer->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), nWidth * nHeight * sizeof(RGBQUAD));

		return S_OK;
	}//if - Make sure we've received valid data
	else
		return E_FAIL;
}

void CUDT_IRDlg::OnClose()
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	ProcessClose();	// Close All
	SaveSettings();	// Save Variables
	CDialogEx::OnClose();
}

void CUDT_IRDlg::ProcessClose()
{
	if (m_Comm.m_bConnected)
	{
		m_Comm.ClosePort();
		PrintStr(&CString("Port Closed"));
	}
	if (m_pVRClientSocket)
	{
		m_pVRClientSocket->Close();
		delete m_pVRClientSocket;
		m_pVRClientSocket = NULL;
		PrintStr(&CString(_T("Socket Closed")));
	}

	if (m_hAOTask)
	{
		while (m_fSpeed > 0)
		{
			m_fSpeed -= 0.6f / m_fFPS;
			//float64 data[1] = { -m_fSpeed / 0.0505f / 2 / 3.141592f * 60 * 60 / 28 / 100 }; //gray treadmill
			float64 data[1] = { m_fSpeed / 0.0475f * 25 / 7 * 30 / 3.141592f / 100 }; // yellow treadmill
			if (DAQmxFailed(DAQmxWriteAnalogF64(m_hAOTask, 1, TRUE, 10.0, DAQmx_Val_GroupByChannel, data, NULL, NULL)))
				PrintStr(&CString(_T("Check connection with Device...1")));
			Sleep(33);
		}

		DAQmxStopTask(m_hAOTask);
		DAQmxClearTask(m_hAOTask);
		m_hAOTask = NULL;
		PrintStr(&CString("Analog Output Closed"));
	}

	if (m_hCITask)
	{
		DAQmxStopTask(m_hCITask);
		DAQmxClearTask(m_hCITask);
		m_hCITask = NULL;
		PrintStr(&CString("Counter Input Closed"));
	}

	m_switch = 0;
	m_count = 0;
	fSpeed_p = 0;
	m_bStart = false;
	m_comboComPort.EnableWindow(TRUE);
	m_btnStart.SetWindowTextW(_T("Start"));
	m_btnMode.EnableWindow(TRUE);

}

void CUDT_IRDlg::SaveSettings()
{
	CWinApp *pApp = AfxGetApp();

	pApp->WriteProfileStringW(PROJECT_NAME, _T("IP"), m_strIP);

	pApp->WriteProfileInt(PROJECT_NAME, _T("GAIN"), (int)(m_fGain * 10));

	pApp->WriteProfileInt(PROJECT_NAME, _T("XREF"), (int)(m_fXref * 10));

	pApp->WriteProfileInt(PROJECT_NAME, _T("K1"), (int)(m_fK1 * 1000));
	pApp->WriteProfileInt(PROJECT_NAME, _T("K2"), (int)(m_fK2 * 1000));
	pApp->WriteProfileInt(PROJECT_NAME, _T("K3"), (int)(m_fK3 * 1000));

	pApp->WriteProfileInt(PROJECT_NAME, _T("BETA"), (int)(m_fBeta * 1000));

	pApp->WriteProfileInt(PROJECT_NAME, _T("ACCLMT"), (int)(m_fAccLmt * 100));
	pApp->WriteProfileInt(PROJECT_NAME, _T("DECLMT"), (int)(m_fDecLmt * 100));
	pApp->WriteProfileInt(PROJECT_NAME, _T("SPDLMT"), (int)(m_fSpdLmt * 100));

	pApp->WriteProfileInt(PROJECT_NAME, _T("XOFF"), m_nXoff + 200);
	pApp->WriteProfileInt(PROJECT_NAME, _T("YOFF"), m_nYoff + 200);
	pApp->WriteProfileInt(PROJECT_NAME, _T("ZOFF"), m_nZoff + 200);

	pApp->WriteProfileInt(PROJECT_NAME, _T("ComfSPD"), (int)(m_fComfSpd * 10));
	pApp->WriteProfileInt(PROJECT_NAME, _T("RepeatTime"), m_fRepeatTime);
	pApp->WriteProfileInt(PROJECT_NAME, _T("SteadyTimeInterval"), m_fSteady_Time_Interval);

}


LRESULT CUDT_IRDlg::OnReceive(WPARAM wParam, LPARAM lParam)
{
	BYTE aByte;
	m_Comm.m_bReserveMsg = false;
	static CString strReceive;
	int size = (m_Comm.m_QueueRead).GetSize();
	if (size == 0) return 0;

	for (int i = 0; i<size; i++)
	{
		(m_Comm.m_QueueRead).GetByte(&aByte);
		//aByte 이용 여기서 데이터 처리

		if (aByte != '\0' && aByte != '\n')
			strReceive += aByte;
		else
		{
			CString temp[45];
			CString strTok;
			int cnt = 0;
			while (AfxExtractSubString(strTok, strReceive, cnt, _T(' ')))  // 문자를 잘라준다. (AfxExtractSubString = strTok)
				temp[cnt++] = strTok;
			int j = 0;
			for (int i = 0; i < cnt; i++)
			{
				if (!temp[i].IsEmpty())
					m_fSensors[j++] = _ttof(temp[i]);
			}
			m_fSensors[6] -= m_nXoff;
			m_fSensors[7] -= m_nYoff;
			m_fSensors[8] -= m_nZoff;
			strReceive.Empty();
		}
	}
	return S_OK;
}


void CUDT_IRDlg::PrintStr(CString* pString)
{
	m_listMsg.AddString(*pString);
	m_listMsg.SetCurSel(m_listMsg.GetCount() - 1);
}


void CUDT_IRDlg::OnIpnFieldchangedIpAddress(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMIPADDRESS pIPAddr = reinterpret_cast<LPNMIPADDRESS>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_ipAddress.GetWindowTextW(m_strIP);
	*pResult = 0;
}


void CUDT_IRDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	if (m_sldrGain.GetSafeHwnd() == pScrollBar->GetSafeHwnd())
	{
		m_fGain = ((float)m_sldrGain.GetPos() / 100.0f);
		CString str;
		str.Format(_T("%.2f"), m_fGain);
		SetDlgItemText(IDC_EDIT_GAIN, str);
	}

	CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}


void CUDT_IRDlg::OnEnChangeEditGain()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_GAIN, str);
	m_fGain = (float)_ttof(str);
	m_sldrGain.SetPos((int)(m_fGain * 100));
}

/*
void CUDT_IRDlg::OnDeltaposSpinXref(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_fXref -= pNMUpDown->iDelta * 0.1f;
CString str;
str.Format(_T("%.1f"), m_fXref);
SetDlgItemText(IDC_EDIT_XREF,str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditXref()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_XREF, str);
	m_fXref = (float)_ttof(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinK1(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_fK1 -= pNMUpDown->iDelta * 0.001f;
CString str;
str.Format(_T("%.3f"), m_fK1);
SetDlgItemText(IDC_EDIT_K1, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditK1()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_K1, str);
	m_fK1 = (float)_ttof(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinK2(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_fK2 -= pNMUpDown->iDelta * 0.001f;
CString str;
str.Format(_T("%.3f"), m_fK2);
SetDlgItemText(IDC_EDIT_K2, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditK2()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_K2, str);
	m_fK2 = (float)_ttof(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinK3(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_fK3 -= pNMUpDown->iDelta * 0.001f;
CString str;
str.Format(_T("%.3f"), m_fK3);
SetDlgItemText(IDC_EDIT_K3, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditK3()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_K3, str);
	m_fK3 = (float)_ttof(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinBeta(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_fBeta -= pNMUpDown->iDelta * 0.001f;
CString str;
str.Format(_T("%.3f"), m_fBeta);
SetDlgItemText(IDC_EDIT_BETA, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditBeta()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_BETA, str);
	m_fBeta = (float)_ttof(str);
}


void CUDT_IRDlg::OnEnChangeEditXoffset()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_XOFFSET, str);
	m_nXoff = _ttoi(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinXoffset(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_nXoff -= pNMUpDown->iDelta;
CString str;
str.Format(_T("%d"), m_nXoff);
SetDlgItemText(IDC_EDIT_XOFFSET, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/
void CUDT_IRDlg::OnEnChangeEditYoffset()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_YOFFSET, str);
	m_nYoff = _ttoi(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinYoffset(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_nYoff -= pNMUpDown->iDelta;
CString str;
str.Format(_T("%d"), m_nYoff);
SetDlgItemText(IDC_EDIT_YOFFSET, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditZoffset()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_ZOFFSET, str);
	m_nZoff = _ttoi(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinZoffset(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_nZoff -= pNMUpDown->iDelta;
CString str;
str.Format(_T("%d"), m_nZoff);
SetDlgItemText(IDC_EDIT_ZOFFSET, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditAcclmt()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_ACCLMT, str);
	m_fAccLmt = (float)_ttof(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinAcclmt(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_fAccLmt -= pNMUpDown->iDelta * 0.01f;
CString str;
str.Format(_T("%.2f"), m_fAccLmt);
SetDlgItemText(IDC_EDIT_ACCLMT, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/


void CUDT_IRDlg::OnEnChangeEditDeclmt()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_DECLMT, str);
	m_fDecLmt = (float)_ttof(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinDeclmt(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_fDecLmt -= pNMUpDown->iDelta * 0.01f;
CString str;
str.Format(_T("%.2f"), m_fDecLmt);
SetDlgItemText(IDC_EDIT_DECLMT, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditSpdlmt()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_SPDLMT, str);
	m_fSpdLmt = (float)_ttof(str);
}

/*
void CUDT_IRDlg::OnDeltaposSpinSpdlmt(NMHDR *pNMHDR, LRESULT *pResult)
{
LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
m_fSpdLmt -= pNMUpDown->iDelta * 0.01f;
CString str;
str.Format(_T("%.2f"), m_fSpdLmt);
SetDlgItemText(IDC_EDIT_SPDLMT, str);
pNMUpDown->iDelta = 0;
*pResult = 1;
}
*/

void CUDT_IRDlg::OnEnChangeEditRepeattime()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_RepeatTime, str);
	m_fRepeatTime = (int)_ttof(str);
}


void CUDT_IRDlg::OnEnChangeEditSteadytimeinterval()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_SteadyTimeInterval, str);
	m_fSteady_Time_Interval = (float)_ttof(str);
}
typedef enum _foot { LEFT, RIGHT } foot;

float CUDT_IRDlg::CalcVwSwing()
{
	static float Vw = 0.0f;
	static float PZ_p = 0.0f;
	static float FL_p = 0.0f;
	static float FR_p = 0.0f;

	static float Lv_max = 0.0f;
	static float Rv_max = 0.0f;
	static float Sw_max = 0.0f;
	static float Sw_max_p = 0.0f;

	static foot swing_p = foot::LEFT;
	static foot swing = foot::LEFT;

	static int state = 0;

	/*float Pv = (PZ_p - m_pMarkers[PELVIS].Z) * m_fFPS;
	float Lv = (FL_p - m_pMarkers[ANKLE_R].Z) * m_fFPS - Pv;
	float Rv = (FR_p - m_pMarkers[ANKLE_L].Z) * m_fFPS - Pv;*/
	float Pv = (m_pMarkers[PELVIS].Z - PZ_p) * m_fFPS;
	float Lv = (m_pMarkers[ANKLE_L].Z - FL_p) * m_fFPS - Pv;
	float Rv = (m_pMarkers[ANKLE_R].Z - FR_p) * m_fFPS - Pv;


	if (Lv - Rv > 1)
		swing = foot::LEFT;
	else if (Rv - Lv > 1)
		swing = foot::RIGHT;

	if (Lv_max < Lv)
		Lv_max = Lv;

	if (Rv_max < Rv)
		Rv_max = Rv;

	if (swing_p != swing)
	{
		Sw_max_p = Sw_max;
		switch (swing)
		{
		case foot::RIGHT:
			Sw_max = Rv_max;
			Lv_max = 0;
			break;
		case foot::LEFT:
			Sw_max = Lv_max;
			Rv_max = 0;
			break;
		}
	}

	float Vw_o = CalcVwObv();

	if ((Sw_max - Sw_max_p) > 5)
		//if ((Sw_max - Sw_max_p) > 0.1)
		state = 1;
	else if ((Sw_max - Sw_max_p) < -5)
		//else if ((Sw_max - Sw_max_p) < -0.1)
		state = 2;
	else if ((Vw_o - Vw) < 0.01)
		state = 0;

	if (m_pMarkers[PELVIS].Z == 0)
		state = 2;

	switch (state)
	{
	case 0:
		Vw = Vw_o;
		break;
	case 1:
		Vw += m_fAccLmt / m_fFPS;
		break;
	case 2:
		Vw -= m_fDecLmt / m_fFPS;
		break;
	}
	if (Vw < 0)
		Vw = 0;

	PZ_p = m_pMarkers[PELVIS].Z;
	FL_p = m_pMarkers[ANKLE_L].Z;
	FR_p = m_pMarkers[ANKLE_R].Z;

	swing_p = swing;

	return Vw;
}


bool CUDT_IRDlg::CalcEmergency(Joint* joints)
{
	bool ret = false;
	/*
	if ((joints[JointType_WristLeft].Position.Y - joints[JointType_ShoulderLeft].Position.Y) > 0.15)
	ret = true;
	*/
	return ret;
}


void CUDT_IRDlg::Wait(DWORD dwMilliSecond)
{
	MSG msg;
	DWORD dwStart;
	dwStart = GetTickCount();

	while (GetTickCount() - dwStart < dwMilliSecond)
	{
		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}
}


void CUDT_IRDlg::OnEnChangeEditComfSPD()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_ComfSPD, str);
	m_fComfSpd = (float)_ttof(str);
}

float CUDT_IRDlg::PassiveMode()
{

	//static int m_switch = 0;	// 1 : velocity decrease, 2: velocity increase
	//static int m_count = 0;		// counting number for stop the treadmill
	static float m_fStartTime = 0; //start time when speed is fixed(fast or slow)
	static float m_fDiffTime = 0;  //to check 20 sec from start time

	m_fFast = m_fComfSpd * 1.25f;
	m_fSlow = m_fComfSpd * 0.75f;


	if (fSpeed_p < m_fSlow && m_switch == 0)
		m_fSpeed = fSpeed_p + 0.1f / m_fFPS;
	else if (fSpeed_p >= m_fSlow && m_switch == 0)
	{
		m_fSpeed = m_fSlow;
	}

	//this part is problem
	//StartTime change for checking 20sec with same speed
	if (fSpeed_p >= m_fSlow && m_switch == 0 && m_fSlow != 0)
	{
		m_fStartTime = m_fTime;
		m_switch = 1;
		PrintStr(&CString(_T("initial mode")));
	}
	//else if (m_switch == 2 && abs(fSpeed_p-m_fSlow)<0.011)
	else if (m_switch == 2 && abs(fSpeed_p - m_fSlow)<0.0001)
	{
		m_fStartTime = m_fTime;
		m_switch = 1;
		PrintStr(&CString(_T("slow mode")));
		//m_count++;
	}
	//else if (abs(fSpeed_p- m_fFast)<0.01 && m_switch == 1)
	else if (abs(fSpeed_p - m_fFast)<0.0001 && m_switch == 1)
	{
		m_fStartTime = m_fTime;
		m_switch = 2;
		m_count++;
		PrintStr(&CString(_T("fast mode")));
	}
	m_fDiffTime = m_fTime - m_fStartTime;


	//for stop sequence
	if (m_count == m_fRepeatTime)		//m_count = n : nth repeat time is last sequence
		m_switch = 3;

	if (m_switch == 3)//don't change about m_switch
	{
		//m_fSpeed = m_fFast;
		m_fSpeed = m_fFast;
		//if (m_fDiffTime >= 20)//const time interval is 20 sec
		if (m_fDiffTime >= m_fSteady_Time_Interval) //const time interval is 20 sec
			m_count++;
	}
	if (m_count >= m_fRepeatTime + 1) //m_count >= n+1
		m_fSpeed = fSpeed_p - 0.1f / m_fFPS;

	//velocity change
	if (m_switch == 1)
	{
		//time interval control(acceleration0
		//if (m_fDiffTime >= 20 && m_fDiffTime < 25) //const time interval is 20 sec, transitioin time is within 5 sec(btw 20 to 25)
		if (m_fDiffTime >= m_fSteady_Time_Interval && m_fDiffTime < (m_fSteady_Time_Interval + 10)) //const time interval is 20 sec, transitioin time is within 5 sec(btw 20 to 25)
		{
			CString FSpeed_p;
			FSpeed_p.Format(_T("%f"), m_fDiffTime);
			PrintStr(&CString(_T("m_fDiffTime is : ") + FSpeed_p));
			if (fSpeed_p < m_fFast)
				//if (m_fFast - fSpeed_p>0.01)
				m_fSpeed = fSpeed_p + 0.1f / m_fFPS;
			else
			{
				m_fSpeed = m_fFast;

			}

		}
	}
	if (m_switch == 2)
	{
		//time interval control(decceleration)
		//if (m_fDiffTime >= 20 && m_fDiffTime < 25) //const time interval is 20 sec, transitioin time is within 5 sec(btw 20 to 25)
		if (m_fDiffTime >= m_fSteady_Time_Interval && m_fDiffTime < (m_fSteady_Time_Interval + 10)) //const time interval is 20 sec, transitioin time is within 5 sec(btw 20 to 25)
		{
			if (fSpeed_p > m_fSlow)
				m_fSpeed = fSpeed_p - 0.1f / m_fFPS;
			else
				m_fSpeed = m_fSlow;
		}
	}


	fSpeed_p = m_fSpeed;

	/*CString FSpeed_p;
	FSpeed_p.Format(_T("%f"), m_fFast);
	PrintStr(&CString(_T("fSpeed_p is : ") + FSpeed_p));*/

	return m_fSpeed;
}

float CUDT_IRDlg::Calslope()
{
	float head_z = m_pMarkers[HEAD].Z;
	float hip_lz = m_pMarkers[HIP_L].Z;
	float hip_rz = m_pMarkers[HIP_R].Z;

	float head_y = m_pMarkers[HEAD].Y;
	float hip_ly = m_pMarkers[HIP_L].Y;
	float hip_ry = m_pMarkers[HIP_R].Y;

	static float diff_z = abs(head_z - (hip_lz + hip_rz) / 2);
	static float diff_y = abs(head_y - (hip_ly + hip_ry) / 2);

	slope = atan2(diff_y, diff_z) * 180 / 3.141592;

	return slope;
}


void CUDT_IRDlg::OnBnClickedMove()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_switch = 2;
	m_btnMode.EnableWindow(FALSE);
}


void CUDT_IRDlg::OnCbnSelchangeComboComport()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CUDT_IRDlg::OnEnChangeEditK4()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CUDT_IRDlg::OnStnClickedPcChartdir1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CUDT_IRDlg::OnStnClickedChart1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}
