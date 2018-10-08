// Copyright (c) 2014 Cyberith GmbH
#pragma once
#include "CVirtDevice.h"
#include <string>


typedef void* IntPtr;


/**
*
*/
class CVirt
{
private:
	// VARIABLES
	static IntPtr DLLHandle;
	// FUNCTIONS
	static IntPtr GetDLLHandle();
	static IntPtr GetDLLProcHandle(std::string _procName);

public:
	static IntPtr CybSDK_VirtDevice_FindDevice();
	static IntPtr CybSDK_VirtDevice_CreateDeviceMockupXInput();
	//
	static bool CybSDK_VirtDevice_Open(IntPtr device);
	static bool CybSDK_VirtDevice_IsOpen(IntPtr device);
	static bool CybSDK_VirtDevice_Close(IntPtr device);
	//
	static float CybSDK_VirtDevice_GetPlayerHeight(IntPtr device);
	static void CybSDK_VirtDevice_ResetPlayerHeight(IntPtr device);
	static float CybSDK_VirtDevice_GetPlayerOrientation(IntPtr device);
	static float CybSDK_VirtDevice_GetMovementSpeed(IntPtr device);
	static float CybSDK_VirtDevice_GetMovementDirection(IntPtr device);
	static void CybSDK_VirtDevice_ResetPlayerOrientation(IntPtr device);
	//
	static bool CybSDK_VirtDevice_HasHaptic(IntPtr device);
	static void CybSDK_VirtDevice_SetHapticBaseplate(IntPtr device, float value);
	//
	static CVirtDevice* FindDevice();
	static CVirtDevice* CreateDeviceMockupXInput();
};