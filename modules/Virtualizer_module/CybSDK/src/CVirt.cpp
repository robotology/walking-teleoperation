// Copyright (c) 2014 Cyberith GmbH
#include "CVirt.h"
#include "CVirtDeviceNative.h"
#include "WinBase.h"
#include "stdafx.h"
#include "windows.h"

//////////////////////////////////
// Shortcuts to DLL function calls
//////////////////////////////////
typedef IntPtr (*_CybSDK_VirtDevice_FindDevice)(); // Declare the DLL function.
typedef IntPtr (*_CybSDK_VirtDevice_CreateDeviceMockupXInput)();
//
typedef bool (*_CybSDK_VirtDevice_Open)(IntPtr device);
typedef bool (*_CybSDK_VirtDevice_IsOpen)(IntPtr device);
typedef bool (*_CybSDK_VirtDevice_Close)(IntPtr device);
//
typedef float (*_CybSDK_VirtDevice_GetPlayerHeight)(IntPtr device);
typedef void (*_CybSDK_VirtDevice_ResetPlayerHeight)(IntPtr device);
typedef float (*_CybSDK_VirtDevice_GetPlayerOrientation)(IntPtr device);
typedef float (*_CybSDK_VirtDevice_GetMovementSpeed)(IntPtr device);
typedef float (*_CybSDK_VirtDevice_GetMovementDirection)(IntPtr device);
typedef void (*_CybSDK_VirtDevice_ResetPlayerOrientation)(IntPtr device);

//
////////////////
// Dll functions;
////////////////
_CybSDK_VirtDevice_FindDevice DLLCybSDK_VirtDevice_FindDevice = NULL;
_CybSDK_VirtDevice_CreateDeviceMockupXInput DLLCybSDK_VirtDevice_CreateDeviceMockupXInput = NULL;
//
_CybSDK_VirtDevice_Open DLLCybSDK_VirtDevice_Open = NULL;
_CybSDK_VirtDevice_IsOpen DLLCybSDK_VirtDevice_IsOpen = NULL;
_CybSDK_VirtDevice_Close DLLCybSDK_VirtDevice_Close = NULL;
//
_CybSDK_VirtDevice_GetPlayerHeight DLLCybSDK_VirtDevice_GetPlayerHeight = NULL;
_CybSDK_VirtDevice_ResetPlayerHeight DLLCybSDK_VirtDevice_ResetPlayerHeight = NULL;
_CybSDK_VirtDevice_GetPlayerOrientation DLLCybSDK_VirtDevice_GetPlayerOrientation = NULL;
_CybSDK_VirtDevice_GetMovementSpeed DLLCybSDK_VirtDevice_GetMovementSpeed = NULL;
_CybSDK_VirtDevice_GetMovementDirection DLLCybSDK_VirtDevice_GetMovementDirection = NULL;
_CybSDK_VirtDevice_ResetPlayerOrientation DLLCybSDK_VirtDevice_ResetPlayerOrientation = NULL;

////////
//  Init
////////
IntPtr CVirt::DLLHandle = nullptr;

/////////////
//  Functions
/////////////
IntPtr CVirt::GetDLLHandle()
{
    // std::cout << "[CYBERITH] GET DLL" << std::endl;
    if (DLLHandle == NULL)
    {
        // std::cout << "[CYBERITH] TRY GET DLL" << std::endl;
        // string > LPCWSTR
#if WIN64
        std::string CybSDKLib = "CybSDK.dll";
#else
        std::string CybSDKLib = "CybSDK.dll";
#endif
        std::wstring stemp = std::wstring(CybSDKLib.begin(), CybSDKLib.end());
        LPCWSTR sw = stemp.c_str();

        // Try get the handle
        HINSTANCE hGetProcIDDLL = LoadLibrary(sw);
        DLLHandle = hGetProcIDDLL;
        if (DLLHandle == nullptr)
        {
            // std::cout << "[CYBERITH] TRY GET DLL FAILED" << std::endl;
            return nullptr;
        }
    }

    return DLLHandle;
}

IntPtr CVirt::GetDLLProcHandle(std::string _procName)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        IntPtr procHandle = GetProcAddress((HMODULE)CVirt::DLLHandle, _procName.c_str());
        if (procHandle != nullptr)
        {
            return procHandle;
        }
    }

    return nullptr;
}

//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
IntPtr CVirt::CybSDK_VirtDevice_FindDevice()
{
    // std::cout << "[CYBERITH]  CybSDK_VirtDevice_FindDevice" << std::endl;
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        // std::cout << "[CYBERITH] CybSDK_VirtDevice_FindDevice > try > got it" << std::endl;
        if (DLLCybSDK_VirtDevice_FindDevice == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_FindDevice"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_FindDevice
                = (_CybSDK_VirtDevice_FindDevice)CVirt::GetDLLProcHandle(
                    procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_FindDevice != NULL)
        {
            return DLLCybSDK_VirtDevice_FindDevice();
        }
    }

    return nullptr;
}

IntPtr CVirt::CybSDK_VirtDevice_CreateDeviceMockupXInput()
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_CreateDeviceMockupXInput == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_CreateDeviceMockupXInput"; // The exact name of the DLL
                                                                // function.
            DLLCybSDK_VirtDevice_CreateDeviceMockupXInput
                = (_CybSDK_VirtDevice_CreateDeviceMockupXInput)CVirt::GetDLLProcHandle(
                    procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_CreateDeviceMockupXInput != NULL)
        {
            return DLLCybSDK_VirtDevice_CreateDeviceMockupXInput();
        }
    }

    return nullptr;
}
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
bool CVirt::CybSDK_VirtDevice_Open(IntPtr device)
{
    // std::cout << "[CYBERITH] CybSDK_VirtDevice_Open" << std::endl;
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        // std::cout << "[CYBERITH] CybSDK_VirtDevice_Open > try" << std::endl;
        if (DLLCybSDK_VirtDevice_Open == NULL)
        {
            // std::cout << "[CYBERITH] CybSDK_VirtDevice_Open > try > got it" << std::endl;
            std::string procName = "CybSDK_VirtDevice_Open"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_Open = (_CybSDK_VirtDevice_Open)CVirt::GetDLLProcHandle(
                procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_Open != NULL)
        {
            return DLLCybSDK_VirtDevice_Open(device);
        }
    }

    return false;
}

bool CVirt::CybSDK_VirtDevice_IsOpen(IntPtr device)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_IsOpen == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_IsOpen"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_IsOpen = (_CybSDK_VirtDevice_IsOpen)CVirt::GetDLLProcHandle(
                procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_IsOpen != NULL)
        {
            return DLLCybSDK_VirtDevice_IsOpen(device);
        }
    }

    return false;
}

bool CVirt::CybSDK_VirtDevice_Close(IntPtr device)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_Close == NULL)
        {
            std::string procName = "CybSDK_VirtDevice_Close"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_Close = (_CybSDK_VirtDevice_Close)CVirt::GetDLLProcHandle(
                procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_Close != NULL)
        {
            return DLLCybSDK_VirtDevice_Close(device);
        }
    }

    return false;
}
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
float CVirt::CybSDK_VirtDevice_GetPlayerHeight(IntPtr device)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_GetPlayerHeight == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_GetPlayerHeight"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_GetPlayerHeight
                = (_CybSDK_VirtDevice_GetPlayerHeight)CVirt::GetDLLProcHandle(
                    procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_GetPlayerHeight != NULL)
        {
            return DLLCybSDK_VirtDevice_GetPlayerHeight(device);
        }
    }

    return 0.0F;
}

void CVirt::CybSDK_VirtDevice_ResetPlayerHeight(IntPtr device)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_ResetPlayerHeight == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_ResetPlayerHeight"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_ResetPlayerHeight
                = (_CybSDK_VirtDevice_ResetPlayerHeight)CVirt::GetDLLProcHandle(
                    procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_ResetPlayerHeight != NULL)
        {
            return DLLCybSDK_VirtDevice_ResetPlayerHeight(device);
        }
    }

    return;
}

float CVirt::CybSDK_VirtDevice_GetPlayerOrientation(IntPtr device)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_GetPlayerOrientation == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_GetPlayerOrientation"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_GetPlayerOrientation
                = (_CybSDK_VirtDevice_GetPlayerOrientation)CVirt::GetDLLProcHandle(
                    procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_GetPlayerOrientation != NULL)
        {
            return DLLCybSDK_VirtDevice_GetPlayerOrientation(device);
        }
    }

    return 0.0F;
}

float CVirt::CybSDK_VirtDevice_GetMovementSpeed(IntPtr device)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_GetMovementSpeed == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_GetMovementSpeed"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_GetMovementSpeed
                = (_CybSDK_VirtDevice_GetMovementSpeed)CVirt::GetDLLProcHandle(
                    procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_GetMovementSpeed != NULL)
        {
            return DLLCybSDK_VirtDevice_GetMovementSpeed(device);
        }
    }

    return 0.0F;
}

float CVirt::CybSDK_VirtDevice_GetMovementDirection(IntPtr device)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_GetMovementDirection == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_GetMovementDirection"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_GetMovementDirection
                = (_CybSDK_VirtDevice_GetMovementDirection)CVirt::GetDLLProcHandle(
                    procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_GetMovementDirection != NULL)
        {
            return DLLCybSDK_VirtDevice_GetMovementDirection(device);
        }
    }

    return 0.0F;
}

void CVirt::CybSDK_VirtDevice_ResetPlayerOrientation(IntPtr device)
{
    if (CVirt::DLLHandle == nullptr)
        CVirt::GetDLLHandle();

    if (CVirt::DLLHandle != nullptr)
    {
        if (DLLCybSDK_VirtDevice_ResetPlayerOrientation == NULL)
        {
            std::string procName
                = "CybSDK_VirtDevice_ResetPlayerOrientation"; // The exact name of the DLL function.
            DLLCybSDK_VirtDevice_ResetPlayerOrientation
                = (_CybSDK_VirtDevice_ResetPlayerOrientation)CVirt::GetDLLProcHandle(
                    procName); // Export the DLL function.
        }

        if (DLLCybSDK_VirtDevice_ResetPlayerOrientation != NULL)
        {
            return DLLCybSDK_VirtDevice_ResetPlayerOrientation(device);
        }
    }

    return;
}

//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
CVirtDevice* CVirt::FindDevice()
{
    IntPtr dev = CVirt::CybSDK_VirtDevice_FindDevice();
    if (dev != nullptr)
    {
        return new CVirtDeviceNative(dev);
    } else
    {
        return nullptr;
    }
}

CVirtDevice* CVirt::CreateDeviceMockupXInput()
{
    IntPtr dev = CVirt::CybSDK_VirtDevice_CreateDeviceMockupXInput();
    if (dev != nullptr)
    {
        return new CVirtDeviceNative(dev);
    } else
    {
        return nullptr;
    }
}