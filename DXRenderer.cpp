//------------------------------------------------------------------------------
// <copyright file="ImageRenderer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include "DXRenderer.h"

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;

/// <summary>
/// Constructor
/// </summary>
DXRenderer::DXRenderer() :
m_hWnd(0),
m_sourceWidth(0),
m_sourceHeight(0),
m_sourceStride(0),
m_pD2DFactory(NULL),
m_pRenderTarget(NULL),
m_pBitmap(0),
m_pBrushJointTracked(NULL),
m_pBrushJointInferred(NULL),
m_pBrushBoneTracked(NULL),
m_pBrushBoneInferred(NULL),
m_pBrushHandClosed(NULL),
m_pBrushHandOpen(NULL),
m_pBrushHandLasso(NULL)
{
}

/// <summary>
/// Destructor
/// </summary>
DXRenderer::~DXRenderer()
{
	DiscardResources();
	SafeRelease(m_pD2DFactory);
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT DXRenderer::EnsureResources()
{
	HRESULT hr = S_OK;

	if (NULL == m_pRenderTarget)
	{
		D2D1_SIZE_U size = D2D1::SizeU(m_sourceWidth, m_sourceHeight);

		D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
		rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
		rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

		// Create a hWnd render target, in order to render to the window set in initialize
		hr = m_pD2DFactory->CreateHwndRenderTarget(
			rtProps,
			D2D1::HwndRenderTargetProperties(m_hWnd, size),
			&m_pRenderTarget
			);

		if (FAILED(hr))
		{
			return hr;
		}

		// Create a bitmap that we can copy image data into and then render to the target
		hr = m_pRenderTarget->CreateBitmap(
			size,
			D2D1::BitmapProperties(D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE)),
			&m_pBitmap
			);

		if (FAILED(hr))
		{
			SafeRelease(m_pRenderTarget);
			return hr;
		}
	}

	// light green
	m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

	m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
	m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
	m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

	return hr;
}

/// <summary>
/// Dispose of Direct2d resources 
/// </summary>
void DXRenderer::DiscardResources()
{
	SafeRelease(m_pRenderTarget);
	SafeRelease(m_pBitmap);
}

/// <summary>
/// Set the window to draw to as well as the video format
/// Implied bits per pixel is 32
/// </summary>
/// <param name="hWnd">window to draw to</param>
/// <param name="pD2DFactory">already created D2D factory object</param>
/// <param name="sourceWidth">width (in pixels) of image data to be drawn</param>
/// <param name="sourceHeight">height (in pixels) of image data to be drawn</param>
/// <param name="sourceStride">length (in bytes) of a single scanline</param>
/// <returns>indicates success or failure</returns>
HRESULT DXRenderer::Initialize(HWND hWnd, ID2D1Factory* pD2DFactory, int sourceWidth, int sourceHeight, int sourceStride)
{
	if (NULL == pD2DFactory)
	{
		return E_INVALIDARG;
	}

	m_hWnd = hWnd;

	// One factory for the entire application so save a pointer here
	m_pD2DFactory = pD2DFactory;

	m_pD2DFactory->AddRef();

	// Get the frame size
	m_sourceWidth = sourceWidth;
	m_sourceHeight = sourceHeight;
	m_sourceStride = sourceStride;

	return S_OK;
}

/// <summary>
/// Draws a 32 bit per pixel image of previously specified width, height, and stride to the associated hwnd
/// </summary>
/// <param name="pImage">image data in RGBX format</param>
/// <param name="cbImage">size of image data in bytes</param>
/// <returns>indicates success or failure</returns>
HRESULT DXRenderer::Draw(BYTE* pImage, unsigned long cbImage)
{
	// incorrectly sized image data passed in
	if (cbImage < ((m_sourceHeight - 1) * m_sourceStride) + (m_sourceWidth * 4))
	{
		return E_INVALIDARG;
	}

	// create the resources for this draw device
	// they will be recreated if previously lost
	HRESULT hr = EnsureResources();

	if (FAILED(hr))
	{
		return hr;
	}

	// Copy the image that was passed in into the direct2d bitmap
	hr = m_pBitmap->CopyFromMemory(NULL, pImage, m_sourceStride);

	if (FAILED(hr))
	{
		return hr;
	}

	m_pRenderTarget->BeginDraw();

	// Draw the bitmap stretched to the size of the window
	m_pRenderTarget->DrawBitmap(m_pBitmap);

	hr = m_pRenderTarget->EndDraw();

	// Device lost, need to recreate the render target
	// We'll dispose it now and retry drawing
	if (hr == D2DERR_RECREATE_TARGET)
	{
		hr = S_OK;
		DiscardResources();
	}

	return hr;
}

D2D1_POINT_2F DXRenderer::BodyToScreen(ICoordinateMapper* pCoordinateMapper, const CameraSpacePoint& bodyPoint, int width, int height)
{// Calculate the body's position on the screen
	DepthSpacePoint depthPoint = { 0 };
	pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / WIDTH;
	float screenPointY = static_cast<float>(depthPoint.Y * height) / HEIGHT;

	//return D2D1::Point2F(screenPointX, screenPointY);

	//ÁÂ¿ì ¹ÝÀü!!
	return D2D1::Point2F(WIDTH - screenPointX + 1, screenPointY);
}


void DXRenderer::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
	// Draw the bones

	// create the resources for this draw device
	// they will be recreated if previously lost
	HRESULT hr = EnsureResources();

	if (FAILED(hr))
		return;
	else
		m_pRenderTarget->BeginDraw();

	// Torso
	DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
	DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineBase);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);
	//Right upper arm
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
	DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_WristRight);

	//Left upper arm
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
	DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_WristLeft);

	// Right Leg
	DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_AnkleRight);

	// Left Leg
	DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_AnkleLeft);

	// Draw the joints
	for (int i = 0; i < JointType_Count; ++i)
	{
		D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

		switch (i)
		{
		case JointType_SpineMid:
		case JointType_ElbowRight:
		case JointType_HandRight:
		case JointType_ElbowLeft:
		case JointType_HandLeft:
		case JointType_ThumbRight:
		case JointType_ThumbLeft:
		case JointType_HandTipRight:
		case JointType_HandTipLeft:
		case JointType_FootRight:
		case JointType_FootLeft:
		case JointType_KneeRight:
		case JointType_KneeLeft:
			break;
		default:
			if (pJoints[i].TrackingState == TrackingState_Inferred)
				m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
			else if (pJoints[i].TrackingState == TrackingState_Tracked)
				m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
			break;
		}
	}

	hr = m_pRenderTarget->EndDraw();

	// Device lost, need to recreate the render target
	// We'll dispose it now and retry drawing
	if (hr == D2DERR_RECREATE_TARGET)
		DiscardResources();
}


void DXRenderer::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
		m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
	else
		m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
}
