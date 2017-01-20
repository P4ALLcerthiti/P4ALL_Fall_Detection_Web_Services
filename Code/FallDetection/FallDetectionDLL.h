// FallDetectionDLL.h : main header file for the FallDetectionDLL DLL
//

#pragma once

//#ifndef __AFXWIN_H__
//	#error "include 'stdafx.h' before including this file for PCH"
//#endif

#include "resource.h"		// main symbols


// CFallDetectionDLLApp
// See FallDetectionDLL.cpp for the implementation of this class
//

class CFallDetectionDLLApp : public CWinApp
{
public:
	CFallDetectionDLLApp();

// Overrides
public:
	virtual BOOL InitInstance();

	DECLARE_MESSAGE_MAP()
};
