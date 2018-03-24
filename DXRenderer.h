//------------------------------------------------------------------------------
// <copyright file="ImageRenderer.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// Manages the drawing of image data

#pragma once

#include <d2d1.h>
#include <Kinect.h>

class DXRenderer
{
public:
	/// <summary>
	/// Constructor
	/// </summary>
	DXRenderer();

	/// <summary>
	/// Destructor
	/// </summary>
	virtual ~DXRenderer();

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
	HRESULT Initialize(HWND hwnd, ID2D1Factory* pD2DFactory, int sourceWidth, int sourceHeight, int sourceStride);

	/// <summary>
	/// Draws a 32 bit per pixel image of previously specified width, height, and stride to the associated hwnd
	/// </summary>
	/// <param name="pImage">image data in RGBX format</param>
	/// <param name="cbImage">size of image data in bytes</param>
	/// <returns>indicates success or failure</returns>
	HRESULT Draw(BYTE* pImage, unsigned long cbImage);

private:
	HWND                     m_hWnd;

	// Format information
	UINT                     m_sourceHeight;
	UINT                     m_sourceWidth;
	LONG                     m_sourceStride;

	// Direct2D 
	ID2D1Factory*            m_pD2DFactory;
	ID2D1HwndRenderTarget*   m_pRenderTarget;
	ID2D1Bitmap*             m_pBitmap;

	// Body/Hand drawing
	ID2D1SolidColorBrush*   m_pBrushJointTracked;
	ID2D1SolidColorBrush*   m_pBrushJointInferred;
	ID2D1SolidColorBrush*   m_pBrushBoneTracked;
	ID2D1SolidColorBrush*   m_pBrushBoneInferred;
	ID2D1SolidColorBrush*   m_pBrushHandClosed;
	ID2D1SolidColorBrush*   m_pBrushHandOpen;
	ID2D1SolidColorBrush*   m_pBrushHandLasso;

	/// <summary>
	/// Ensure necessary Direct2d resources are created
	/// </summary>
	/// <returns>indicates success or failure</returns>
	HRESULT EnsureResources();

	/// <summary>
	/// Dispose of Direct2d resources 
	/// </summary>
	void DiscardResources();
public:
	D2D1_POINT_2F BodyToScreen(ICoordinateMapper* pCoordinateMapper, const CameraSpacePoint& bodyPoint, int width, int height);
	void DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);
	void DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);
};

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

template<class Interface>
inline HRESULT GetBuffer(Interface *& pFrame, UINT* pnBufferSize, UINT16** ppBuffer)
{
	IFrameDescription* pFrameDescription = NULL;
	int nWidth = 0;
	int nHeight = 0;;

	HRESULT hr = pFrame->get_FrameDescription(&pFrameDescription);

	if (SUCCEEDED(hr))
		hr = pFrameDescription->get_Width(&nWidth);

	if (SUCCEEDED(hr))
		hr = pFrameDescription->get_Height(&nHeight);

	if (SUCCEEDED(hr))
		hr = pFrame->AccessUnderlyingBuffer(pnBufferSize, ppBuffer);

	return hr;
}