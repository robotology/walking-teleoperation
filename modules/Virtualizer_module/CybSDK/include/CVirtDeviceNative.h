// Copyright (c) 2014 Cyberith GmbH
#pragma once
#include "CVirt.h"
#include "CVirtDevice.h"

/**
 *
 */
class CVirtDeviceNative : public CVirtDevice
{
private:
    IntPtr devicePtr;

public:
    CVirtDeviceNative(IntPtr devicePtr);
    ~CVirtDeviceNative();

    bool Open() override;
    bool IsOpen() override;
    bool Close() override;

    float GetPlayerHeight() override;
    void ResetPlayerHeight() override;
    float GetPlayerOrientation() override;
    float GetMovementSpeed() override;
    float GetMovementDirection() override;
    virtual void ResetPlayerOrientation() override;

    bool IsPtrNull();
};