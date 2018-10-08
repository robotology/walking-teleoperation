// Copyright (c) 2014 Cyberith GmbH
#include "CVirtDeviceNative.h"
#include "math.h"
#include "stdafx.h"

CVirtDeviceNative::CVirtDeviceNative(IntPtr devicePtr)
{
	this->devicePtr = devicePtr;
}

CVirtDeviceNative::~CVirtDeviceNative()
{
}

bool CVirtDeviceNative::Open()
{
	return CVirt::CybSDK_VirtDevice_Open(this->devicePtr);
}

bool CVirtDeviceNative::IsOpen()
{
	return CVirt::CybSDK_VirtDevice_IsOpen(this->devicePtr);
}

bool CVirtDeviceNative::Close()
{
	return CVirt::CybSDK_VirtDevice_Close(this->devicePtr);
}

float CVirtDeviceNative::GetPlayerHeight()
{
	return CVirt::CybSDK_VirtDevice_GetPlayerHeight(this->devicePtr);
}

void CVirtDeviceNative::ResetPlayerHeight()
{
	return CVirt::CybSDK_VirtDevice_ResetPlayerHeight(this->devicePtr);
}

float CVirtDeviceNative::GetPlayerOrientation()
{
	float playerOrient = CVirt::CybSDK_VirtDevice_GetPlayerOrientation(this->devicePtr);
	//
	return playerOrient;
}

float CVirtDeviceNative::GetMovementSpeed()
{
	return CVirt::CybSDK_VirtDevice_GetMovementSpeed(this->devicePtr);
}

float CVirtDeviceNative::GetMovementDirection()
{
	float movDir = CVirt::CybSDK_VirtDevice_GetMovementDirection(this->devicePtr);
	//
	return movDir;
}

void CVirtDeviceNative::ResetPlayerOrientation()
{
	return CVirt::CybSDK_VirtDevice_ResetPlayerOrientation(this->devicePtr);
}

bool CVirtDeviceNative::IsPtrNull()
{
	return (this->devicePtr == nullptr);
}